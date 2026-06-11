import os
import time
import json
import logging
from typing import Optional, List, Dict
from datetime import datetime

from .feature_encoder import FeatureEncoder
from .mlp_model import MLPDecisionModel, COMMANDS
from .training_collector import TrainingCollector

logger = logging.getLogger(__name__)

MODELS_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'models')


class OfflineDecisionEngine:
    MODES = ['replay', 'ifttt', 'offline_ai']

    def __init__(self, camera_service=None, automation_engine=None,
                 mega_interface=None, sensor_manager=None,
                 waypoint_memory=None,
                 model_name: str = 'mlp_decision'):
        self.camera = camera_service
        self.automation = automation_engine
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.waypoints = waypoint_memory
        self.db = None

        self.running = False
        self.current_mode = 'offline_ai'
        self.task_goal = ''
        self.last_decision = None
        self.decision_history = []
        self.max_history = 5

        self.yolo_model = None

        self.encoder = FeatureEncoder(window_size=5)
        self.model = MLPDecisionModel()

        self.collector = TrainingCollector()
        self._session_id: Optional[str] = None

        model_path = os.path.join(MODELS_DIR, f'{model_name}.pt')
        config_path = os.path.join(MODELS_DIR, f'{model_name}_config.json')
        self.model_path = model_path
        self.config_path = config_path
        self.model_loaded = False
        self.model_accuracy = 0.0
        self.training_date = None

    def initialize(self, db_client=None):
        self.db = db_client
        self._try_load_model()

    def start(self, task_goal: str = '', interval: float = None) -> dict:
        if self.running:
            return {'success': False, 'error': 'Already running'}
        self.task_goal = task_goal
        self.running = True
        self._session_id = self.collector.new_session(
            mode='offline_ai',
            notes=f'task: {task_goal[:100]}'
        )
        return {'success': True}

    def stop(self) -> dict:
        self.running = False
        if self.camera:
            self.camera.deactivate()
        return {'success': True}

    def analyze(self, sensor_data: dict, camera_frame: Optional[str] = None,
                task_goal: str = '') -> dict:
        start_time = time.time()

        yolo_detections = self._run_yolo(camera_frame) if camera_frame else []

        feature_vector = self.encoder.encode(
            sensor_data, yolo_detections,
            self.last_decision.get('actions', [{}])[0].get('value') if self.last_decision else None
        )

        command, confidence, probs = self.model.predict(feature_vector)

        actions = self._apply_overrides(sensor_data, yolo_detections, command, confidence)

        reward = 0.0
        self.collector.record_frame(
            features=feature_vector,
            command=actions[0]['value'],
            reward=reward,
            confidence=confidence,
            mode='offline_ai',
            sensor_snapshot=sensor_data,
            yolo_snapshot={'detections': yolo_detections},
            session_id=self._session_id,
        )

        latency_ms = int((time.time() - start_time) * 1000)

        result = {
            'reasoning': self._build_reasoning(yolo_detections, sensor_data,
                                                command, confidence),
            'actions': actions,
            'confidence': confidence,
            'continue': True,
            'mode': 'offline_ai',
            'latency_ms': latency_ms,
            'timestamp': datetime.now().isoformat(),
            'model_loaded': self.model_loaded,
        }

        self.last_decision = result
        self.decision_history.append(result)
        if len(self.decision_history) > self.max_history:
            self.decision_history.pop(0)

        self._log_decision(result, sensor_data, task_goal)

        return result

    def get_status(self) -> dict:
        return {
            'running': self.running,
            'mode': self.current_mode,
            'task_goal': self.task_goal,
            'model_loaded': self.model_loaded,
            'model_accuracy': self.model_accuracy,
            'training_date': self.training_date,
            'last_decision': self.last_decision,
            'history_count': len(self.decision_history),
            'encoder_window': len(self.encoder.buffer),
        }

    def get_model_info(self) -> dict:
        return {
            'loaded': self.model_loaded,
            'accuracy': self.model_accuracy,
            'training_date': self.training_date,
            'commands': COMMANDS,
            'input_dim': self.model.net[0].in_features,
            'model_path': self.model_path,
        }

    def load_model(self, model_path: str = None, config_path: str = None):
        path = model_path or self.model_path
        cfg = config_path or self.config_path

        if os.path.exists(cfg):
            config = MLPDecisionModel.load_config(cfg)
            if 'norm_params' in config and config['norm_params']:
                self.encoder.load_params(cfg)

        if os.path.exists(path):
            self.model.load_model(path)
            self.model_loaded = True
            logger.info(f"MLP model loaded from {path}")
        else:
            logger.warning(f"Model file not found: {path}")

    def _try_load_model(self):
        self.load_model()

    def _run_yolo(self, frame_b64: str) -> List[dict]:
        if not frame_b64:
            return []
        try:
            if self.yolo_model is None:
                from ultralytics import YOLO
                self.yolo_model = YOLO('yolov8n.pt')

            import cv2
            import numpy as np
            import base64

            img_bytes = base64.b64decode(frame_b64)
            nparr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
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
            return []
        except Exception as e:
            logger.error(f"YOLO error: {e}")
            return []

    def _apply_overrides(self, sensor_data: dict, detections: List[dict],
                         mlp_command: str, mlp_confidence: float) -> List[dict]:
        has_living = any(
            d.get('class') in ('person', 'cat', 'dog', 'bird')
            and d.get('confidence', 0) > 0.4
            for d in detections
        )
        if has_living:
            return [{'type': 'stop', 'value': 's', 'delay_ms': 0}]

        critical_sensors = [
            sensor_data.get('laser_left_front', 1500),
            sensor_data.get('laser_left_back', 1500),
            sensor_data.get('laser_right_front', 1500),
            sensor_data.get('laser_right_back', 1500),
            sensor_data.get('ultra_front_left', 4000),
            sensor_data.get('ultra_front_right', 4000),
        ]
        if any(v is not None and v < 200 for v in critical_sensors):
            return [{'type': 'stop', 'value': 's', 'delay_ms': 0}]

        if mlp_confidence < 0.4:
            return self._rule_based_fallback(sensor_data, detections)

        return [{'type': 'move', 'value': mlp_command, 'delay_ms': 300}]

    def _rule_based_fallback(self, sensor_data: dict,
                             detections: List[dict]) -> List[dict]:
        front_clear = min(
            sensor_data.get('laser_left_front', 1500),
            sensor_data.get('laser_right_front', 1500),
        )
        left_clear = sensor_data.get('laser_left_back', 1500)
        right_clear = sensor_data.get('laser_right_back', 1500)

        if detections:
            for d in detections:
                if d['class'] in ('person', 'cat', 'dog') and d['confidence'] > 0.4:
                    return [{'type': 'stop', 'value': 's', 'delay_ms': 0}]

        if front_clear < 500:
            if left_clear > right_clear:
                return [{'type': 'move', 'value': 'q', 'delay_ms': 300}]
            else:
                return [{'type': 'move', 'value': 'e', 'delay_ms': 300}]
        elif front_clear > 800:
            return [{'type': 'move', 'value': 'f', 'delay_ms': 300}]
        else:
            return [{'type': 'move', 'value': 's', 'delay_ms': 0}]

    def _build_reasoning(self, detections: List[dict], sensor_data: dict,
                         command: str, confidence: float) -> str:
        parts = []
        if detections:
            det_str = ', '.join(
                f"{d['class']} ({d['confidence']:.0%})"
                for d in detections[:3]
            )
            parts.append(f"detected: {det_str}")

        front = min(
            sensor_data.get('laser_left_front', 1500),
            sensor_data.get('laser_right_front', 1500),
        )
        parts.append(f"front_clear: {int(front)}mm")

        cmd_labels = {
            'f': 'forward', 'b': 'backward',
            'q': 'fwd-left', 'e': 'fwd-right',
            'z': 'bwd-left', 'x': 'bwd-right',
            't': 'turn-left', 'y': 'turn-right', 's': 'stop',
        }
        parts.append(f"cmd: {cmd_labels.get(command, command)} ({confidence:.0%})")

        return ' | '.join(parts)

    def _log_decision(self, result: dict, sensor_data: dict, task_goal: str):
        if not self.db:
            return
        try:
            from prisma import Json
            from automation_engine import _run_async
            create_data = {
                'taskGoal': task_goal,
                'mode': 'offline_ai',
                'aiResponse': Json(result),
                'actionsExecuted': len(result.get('actions', [])),
                'backend': 'offline_mlp',
            }
            if result.get('confidence') is not None:
                create_data['confidence'] = result['confidence']
            if result.get('latency_ms') is not None:
                create_data['latencyMs'] = result['latency_ms']
            _run_async(self.db.aidecision.create(data=create_data))
        except Exception as e:
            logger.error(f"Failed to log decision: {e}")

    def cleanup(self):
        self.running = False
        if self.camera:
            self.camera.deactivate()
        self.encoder.reset()
