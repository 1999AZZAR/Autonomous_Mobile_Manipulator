"""
AI Decision Engine — continuous brain loop for autonomous navigation.

Three modes:
  REPLAY  — navigate via saved waypoints + IMU (camera OFF, no AI)
  IFTTT   — sensor threshold rules (camera OFF, no AI)
  AI      — camera + YOLOv8/API reasoning (camera ON)

Camera is only activated when AI mode is active. Decision engine
auto-selects the best mode based on task context.
"""

import os
import json
import time
import base64
import threading
import logging
from datetime import datetime
from typing import Optional, Dict, Any, List

from prisma import Json
from automation_engine import _run_async

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """You are the brain of an autonomous mobile manipulator robot.

ROBOT CAPABILITIES (output as JSON actions):
- move: forward/backward/left/right
- turn: left/right/angle_<degrees>
- gripper: open/close
- tilt: up/down/center/angle_<0-180>
- stop: emergency stop
- speed: 0-100
- patrol: start/stop

SAFETY RULES:
- Never move toward obstacles closer than 20cm
- Stop immediately if uncertain (confidence < 0.4)
- Prefer slow, deliberate movements
- If stuck, try turning 90 degrees

OUTPUT FORMAT (JSON only, no markdown):
{
  "reasoning": "brief description of what you see and plan",
  "actions": [
    {"type": "move", "value": "forward", "delay_ms": 500}
  ],
  "confidence": 0.85,
  "continue": true
}

confidence: 0.0-1.0 (how sure you are about this decision)
continue: true if robot should keep thinking, false if task is complete"""


class DecisionMode:
    REPLAY = "replay"
    IFTTT = "ifttt"
    AI = "ai"
    OFFLINE_AI = "offline_ai"


class AIDecisionEngine:
    """
    Continuous decision loop. Selects between REPLAY/IFTTT/AI modes.
    Camera only activates in AI mode.
    """

    def __init__(self, camera_service, automation_engine, mega_interface=None,
                 sensor_manager=None, waypoint_memory=None):
        self.camera = camera_service
        self.automation = automation_engine
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.waypoints = waypoint_memory
        self.db = None  # Prisma client

        # Decision loop state
        self.running = False
        self.loop_interval = 3.0  # seconds between decisions
        self.task_goal = ""
        self.current_mode = DecisionMode.IFTTT
        self.last_decision = None
        self.decision_history = []  # last N decisions for context
        self.max_history = 5

        # Human guidance (injected into next prompt)
        self.human_guidance = ""

        # AI backend config
        self.backend = os.environ.get('AI_BACKEND', 'hybrid')  # local, api, hybrid, offline_ai
        self.api_model = os.environ.get('AI_MODEL', 'gpt-4o')
        self.api_key = os.environ.get('OPENAI_API_KEY', '')
        self.yolo_model = None  # loaded lazily

        # Offline MLP engine (used when backend=offline_ai)
        self.offline_engine = None

        # Thread control
        self._thread = None
        self._lock = threading.Lock()

    def initialize(self, db_client):
        """Set Prisma client and initialize offline engine."""
        self.db = db_client
        try:
            import sys, os
            _scripts_dir = os.path.join(os.path.dirname(__file__))
            if _scripts_dir not in sys.path:
                sys.path.insert(0, _scripts_dir)
            from ml.inference_engine import OfflineDecisionEngine
            self.offline_engine = OfflineDecisionEngine(
                camera_service=self.camera,
                automation_engine=self.automation,
                mega_interface=self.mega,
                sensor_manager=self.sensors,
                waypoint_memory=self.waypoints,
            )
            self.offline_engine.initialize(db_client)
        except ImportError as e:
            logger.debug(f"Offline engine not available: {e}")
        except Exception as e:
            logger.error(f"Failed to init offline engine: {e}")

    def start(self, task_goal: str = "", interval: float = None) -> dict:
        """Start the decision loop."""
        if self.running:
            return {'success': False, 'error': 'Already running'}

        self.task_goal = task_goal
        if interval:
            self.loop_interval = max(1.0, interval)

        self.running = True
        self._thread = threading.Thread(
            target=self._decision_loop,
            daemon=True,
            name="ai-decision-loop"
        )
        self._thread.start()
        logger.info(f"AI decision loop started (goal: '{task_goal}', interval: {self.loop_interval}s)")
        return {'success': True}

    def stop(self) -> dict:
        """Stop the decision loop and deactivate camera."""
        if not self.running:
            return {'success': False, 'error': 'Not running'}

        self.running = False
        self.camera.deactivate()
        logger.info("AI decision loop stopped")
        return {'success': True}

    def analyze_once(self, task_goal: str = None) -> dict:
        """Run a single analysis cycle without starting the loop."""
        goal = task_goal or self.task_goal or "Analyze the current situation"
        return self._run_analysis(goal)

    def set_human_guidance(self, guidance: str):
        """Inject human guidance into the next AI decision."""
        self.human_guidance = guidance
        logger.info(f"Human guidance set: {guidance[:100]}")

    def get_status(self) -> dict:
        """Get current decision engine status."""
        return {
            'running': self.running,
            'mode': self.current_mode,
            'task_goal': self.task_goal,
            'loop_interval': self.loop_interval,
            'backend': self.backend,
            'camera': self.camera.get_state(),
            'last_decision': self.last_decision,
            'history_count': len(self.decision_history),
            'human_guidance': self.human_guidance[:200] if self.human_guidance else None,
        }

    def get_decisions(self, limit: int = 20) -> List[dict]:
        """Get recent decision history from database."""
        if not self.db:
            return []
        try:
            logs = _run_async(self.db.aidecision.find_many(
                order={'createdAt': 'desc'},
                take=limit
            ))
            return [
                {
                    'id': d.id,
                    'taskGoal': d.taskGoal,
                    'mode': d.mode,
                    'aiResponse': d.aiResponse,
                    'actionsExecuted': d.actionsExecuted,
                    'confidence': d.confidence,
                    'backend': d.backend,
                    'modelUsed': d.modelUsed,
                    'latencyMs': d.latencyMs,
                    'createdAt': d.createdAt.isoformat() if d.createdAt else None,
                } for d in logs
            ]
        except Exception as e:
            logger.error(f"Failed to fetch decisions: {e}")
            return []

    def _decision_loop(self):
        """Main decision loop — runs in background thread."""
        while self.running:
            try:
                result = self._run_analysis(self.task_goal)
                self.last_decision = result

                # Manage camera power based on mode
                if self.current_mode == DecisionMode.AI:
                    if self.camera.state.value != 'active':
                        self.camera.activate()
                else:
                    if self.camera.state.value == 'active':
                        self.camera.deactivate()

                time.sleep(self.loop_interval)

            except Exception as e:
                logger.error(f"Decision loop error: {e}")
                # Fallback: switch to IFTTT mode, deactivate camera
                self.current_mode = DecisionMode.IFTTT
                self.camera.deactivate()
                time.sleep(5.0)

    def _run_analysis(self, task_goal: str) -> dict:
        """Run a single analysis cycle. Returns decision result."""
        start_time = time.time()

        # 1. Read sensor data
        sensor_data = self._read_sensors()

        # 2. Decide mode
        mode = self._select_mode(sensor_data, task_goal)
        self.current_mode = mode

        # 3. Execute based on mode
        if mode == DecisionMode.REPLAY:
            result = self._execute_replay(sensor_data, task_goal)
        elif mode == DecisionMode.AI:
            result = self._execute_ai(sensor_data, task_goal)
        elif mode == DecisionMode.OFFLINE_AI:
            result = self._execute_offline_ai(sensor_data, task_goal)
        else:
            result = self._execute_ifttt(sensor_data, task_goal)

        # 4. Execute actions if any
        if result.get('actions') and self.mega:
            self._execute_actions(result['actions'])

        # 5. Log decision
        latency_ms = int((time.time() - start_time) * 1000)
        result['latency_ms'] = latency_ms
        result['mode'] = mode
        result['timestamp'] = datetime.now().isoformat()

        # 6. Save to database
        self._log_decision(result, sensor_data, task_goal)

        # 7. Update history
        self.decision_history.append(result)
        if len(self.decision_history) > self.max_history:
            self.decision_history.pop(0)

        return result

    def _select_mode(self, sensor_data: dict, task_goal: str) -> str:
        """Select the best navigation mode based on context."""
        # If backend is explicitly offline_ai, prefer it
        if self.backend == 'offline_ai' and self.offline_engine and self.offline_engine.model_loaded:
            return DecisionMode.OFFLINE_AI

        # If there's a saved path and task matches → REPLAY
        if self.waypoints and task_goal:
            paths = self.waypoints.list_paths()
            for p in paths:
                if p['name'].lower() in task_goal.lower():
                    return DecisionMode.REPLAY

        # If task goal mentions visual reasoning or object detection → AI
        visual_keywords = ['see', 'look', 'find', 'detect', 'identify', 'object', 'person', 'color', 'where']
        if any(kw in task_goal.lower() for kw in visual_keywords):
            if self.offline_engine and self.offline_engine.model_loaded:
                return DecisionMode.OFFLINE_AI
            return DecisionMode.AI

        # If no task goal or simple navigation → IFTTT or offline_ai
        if not task_goal:
            if self.offline_engine and self.offline_engine.model_loaded:
                return DecisionMode.OFFLINE_AI
            return DecisionMode.IFTTT

        # Default: offline_ai if available, else API AI
        if self.offline_engine and self.offline_engine.model_loaded:
            return DecisionMode.OFFLINE_AI
        return DecisionMode.AI

    def _execute_replay(self, sensor_data: dict, task_goal: str) -> dict:
        """Execute waypoint replay mode."""
        if not self.waypoints or not self.waypoints.replaying:
            # Try to find and start a matching path
            paths = self.waypoints.list_paths() if self.waypoints else []
            for p in paths:
                if p['name'].lower() in task_goal.lower():
                    self.waypoints.start_replay(p['id'])
                    return {
                        'reasoning': f"Replaying saved path '{p['name']}' via IMU navigation",
                        'actions': [],
                        'confidence': 0.9,
                        'continue': True,
                    }

        return {
            'reasoning': 'REPLAY mode: navigating via saved waypoints and IMU',
            'actions': [],
            'confidence': 0.9,
            'continue': self.waypoints.replaying if self.waypoints else False,
        }

    def _execute_ai(self, sensor_data: dict, task_goal: str) -> dict:
        """Execute AI mode: capture frame, run inference, return actions."""
        # Capture camera frame
        frame_b64 = self.camera.capture_base64()

        # Run local YOLO detection if available
        yolo_detections = self._run_yolo(frame_b64)

        # Build context for API call
        context = self._build_context(sensor_data, task_goal, yolo_detections, frame_b64)

        # Run API inference
        ai_response = self._call_ai_api(context)

        if ai_response:
            # Parse and validate
            parsed = self._parse_response(ai_response)
            if parsed:
                # Execute actions
                self._execute_actions(parsed.get('actions', []))
                return parsed

        # Fallback: if AI fails, use YOLO detections with rule-based logic
        if yolo_detections:
            return self._rule_based_from_yolo(yolo_detections, sensor_data, task_goal)

        return {
            'reasoning': 'AI analysis complete but no actionable response',
            'actions': [],
            'confidence': 0.3,
            'continue': True,
        }

    def _execute_ifttt(self, sensor_data: dict, task_goal: str) -> dict:
        """Execute IFTTT mode: let existing automation engine handle it."""
        return {
            'reasoning': 'IFTTT mode: sensor-based automation handling navigation',
            'actions': [],
            'confidence': 0.8,
            'continue': True,
        }

    def _execute_offline_ai(self, sensor_data: dict, task_goal: str) -> dict:
        """Execute offline MLP inference mode. Fully local, no API call."""
        if not self.offline_engine:
            return self._execute_ifttt(sensor_data, task_goal)

        camera_frame = None
        if task_goal and self.camera:
            try:
                camera_frame = self.camera.capture_base64()
            except Exception:
                pass

        result = self.offline_engine.analyze(
            sensor_data=sensor_data,
            camera_frame=camera_frame,
            task_goal=task_goal or '',
        )
        return result

    def _read_sensors(self) -> dict:
        """Read current sensor data from cache."""
        if not self.sensors:
            return {}
        try:
            return self.sensors.read_all_sensors()
        except Exception:
            return {}

    def _run_yolo(self, frame_b64: Optional[str]) -> List[dict]:
        """Run YOLOv8 local detection. Returns list of detections."""
        if not frame_b64:
            return []

        try:
            if self.yolo_model is None:
                from ultralytics import YOLO
                self.yolo_model = YOLO('yolov8n.pt')  # nano model, fast

            import cv2
            import numpy as np

            # Decode base64 to image
            img_bytes = base64.b64decode(frame_b64)
            nparr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Run detection
            results = self.yolo_model(img, verbose=False)

            detections = []
            for r in results:
                for box in r.boxes:
                    detections.append({
                        'class': r.names[int(box.cls)],
                        'confidence': float(box.conf),
                        'bbox': box.xyxy.tolist()[0],
                    })

            return detections

        except ImportError:
            logger.debug("ultralytics not installed, skipping YOLO")
            return []
        except Exception as e:
            logger.error(f"YOLO detection error: {e}")
            return []

    def _build_context(self, sensor_data: dict, task_goal: str,
                       yolo_detections: List[dict], frame_b64: Optional[str]) -> str:
        """Build the prompt context for AI API call."""
        parts = []

        # Sensor data summary
        if sensor_data:
            sensor_summary = {}
            for key in ['laser_left_front', 'laser_right_front', 'laser_back_left',
                         'ultra_front_left', 'ultra_front_right', 'imu_heading']:
                val = sensor_data.get(key)
                if val is not None:
                    sensor_summary[key] = round(val, 2)
            if sensor_summary:
                parts.append(f"CURRENT SENSORS: {json.dumps(sensor_summary)}")

        # YOLO detections
        if yolo_detections:
            det_str = ", ".join([
                f"{d['class']} ({d['confidence']:.0%}) at {self._bbox_center(d['bbox'])}"
                for d in yolo_detections[:5]
            ])
            parts.append(f"DETECTED OBJECTS: {det_str}")

        # Task goal
        if task_goal:
            parts.append(f"TASK GOAL: {task_goal}")

        # Human guidance
        if self.human_guidance:
            parts.append(f"HUMAN GUIDANCE: {self.human_guidance}")

        # Recent history
        if self.decision_history:
            history_lines = []
            for h in self.decision_history[-3:]:
                reasoning = h.get('reasoning', 'unknown')[:80]
                conf = h.get('confidence', 0)
                history_lines.append(f"- {reasoning} (confidence: {conf:.0%})")
            parts.append(f"RECENT HISTORY:\n" + "\n".join(history_lines))

        return "\n".join(parts)

    def _call_ai_api(self, context: str) -> Optional[str]:
        """Call AI API (OpenAI GPT-4V or Gemini)."""
        if not self.api_key:
            logger.debug("No API key configured, skipping API call")
            return None

        try:
            import httpx

            headers = {
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            }

            messages = [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': context},
            ]

            payload = {
                'model': self.api_model,
                'messages': messages,
                'temperature': 0.3,
                'max_tokens': 500,
            }

            response = httpx.post(
                'https://api.openai.com/v1/chat/completions',
                headers=headers,
                json=payload,
                timeout=15.0,
            )

            if response.status_code == 200:
                data = response.json()
                return data['choices'][0]['message']['content']
            else:
                logger.error(f"API error {response.status_code}: {response.text[:200]}")
                return None

        except ImportError:
            logger.debug("httpx not installed")
            return None
        except Exception as e:
            logger.error(f"API call failed: {e}")
            return None

    def _parse_response(self, text: str) -> Optional[dict]:
        """Parse AI JSON response."""
        try:
            # Try direct JSON parse
            result = json.loads(text)
            if 'reasoning' in result and 'actions' in result:
                return result
        except json.JSONDecodeError:
            pass

        # Try to extract JSON from markdown code block
        try:
            if '```json' in text:
                json_str = text.split('```json')[1].split('```')[0].strip()
            elif '```' in text:
                json_str = text.split('```')[1].split('```')[0].strip()
            else:
                # Try to find JSON object in text
                start = text.index('{')
                end = text.rindex('}') + 1
                json_str = text[start:end]

            result = json.loads(json_str)
            if 'reasoning' in result and 'actions' in result:
                return result
        except (ValueError, IndexError, json.JSONDecodeError):
            pass

        logger.warning(f"Failed to parse AI response: {text[:200]}")
        return None

    def _execute_actions(self, actions: list):
        """Execute a list of AI-decided actions via mega interface."""
        if not self.mega or not actions:
            return

        for action in actions:
            action_type = action.get('type', '')
            value = action.get('value', '')
            delay_ms = action.get('delay_ms', 0)

            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)

            cmd_map = {
                'move': {'forward': 'f', 'backward': 'b', 'forward-left': 'q', 'forward-right': 'e', 'backward-left': 'z', 'backward-right': 'x'},
                'turn': {'left': 't', 'right': 'y'},
                'gripper': {'open': 'no', 'close': 'nc'},
                'tilt': {'up': 'tu', 'down': 'td', 'center': 'tt'},
                'stop': {'': 's'},
            }

            if action_type in cmd_map:
                cmd = cmd_map[action_type].get(value)
                if cmd:
                    self.mega.send_command(cmd)
                    logger.debug(f"AI action: {action_type}={value} → '{cmd}'")
            elif action_type == 'speed':
                try:
                    speed_val = int(float(value))
                    self.mega.send_command(f'sp{max(0, min(100, speed_val))}')
                except (ValueError, TypeError):
                    pass
            elif action_type == 'turn' and value.startswith('angle_'):
                angle = value.replace('angle_', '')
                self.mega.send_command(f'ta{angle}')

    def _rule_based_from_yolo(self, detections: List[dict], sensor_data: dict, task_goal: str) -> dict:
        """Fallback: map YOLO detections to actions using simple rules."""
        actions = []
        reasoning_parts = []

        for det in detections:
            cls = det['class']
            conf = det['confidence']

            if conf < 0.4:
                continue

            if cls in ('person', 'cat', 'dog'):
                actions.append({'type': 'stop', 'value': '', 'delay_ms': 0})
                reasoning_parts.append(f"Living thing detected ({cls}), stopping")
            elif cls in ('chair', 'table', 'bench'):
                actions.append({'type': 'turn', 'value': 'right', 'delay_ms': 500})
                reasoning_parts.append(f"Furniture detected ({cls}), turning right to avoid")

        if not actions:
            actions.append({'type': 'move', 'value': 'forward', 'delay_ms': 300})
            reasoning_parts.append("No significant objects, moving forward")

        return {
            'reasoning': "; ".join(reasoning_parts) or "Rule-based decision from YOLO detections",
            'actions': actions,
            'confidence': 0.6,
            'continue': True,
        }

    def _bbox_center(self, bbox: list) -> str:
        """Get human-readable center position of bounding box."""
        x_center = (bbox[0] + bbox[2]) / 2
        if x_center < 0.33:
            return "left"
        elif x_center > 0.66:
            return "right"
        return "center"

    def _log_decision(self, result: dict, sensor_data: dict, task_goal: str):
        """Log decision to database."""
        if not self.db:
            return
        try:
            create_data = {
                'taskGoal': task_goal,
                'mode': result.get('mode', 'unknown'),
                'aiResponse': Json(result),
                'actionsExecuted': len(result.get('actions', [])),
                'backend': self.backend,
            }
            if sensor_data:
                create_data['sensorSnapshot'] = Json(sensor_data)
            if result.get('confidence') is not None:
                create_data['confidence'] = result.get('confidence')
            if self.current_mode == DecisionMode.AI and self.api_model:
                create_data['modelUsed'] = self.api_model
            if result.get('latency_ms') is not None:
                create_data['latencyMs'] = result.get('latency_ms')

            _run_async(self.db.aidecision.create(data=create_data))
        except Exception as e:
            logger.error(f"Failed to log decision: {e}")

    def cleanup(self):
        """Stop loop and release resources."""
        self.running = False
        self.camera.deactivate()
