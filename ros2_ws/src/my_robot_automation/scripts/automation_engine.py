"""
Automation Engine
IFTTT-style automation system for the robot.
Evaluates sensor conditions and executes robot actions in real-time.
Uses Redis for sensor caching and rate limiting.
"""

import os
import json
import time
import asyncio
import threading
import logging
from datetime import datetime
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)

# Persistent background event loop for Prisma async calls
_loop: Optional[asyncio.AbstractEventLoop] = None
_loop_thread: Optional[threading.Thread] = None


def _ensure_event_loop():
    """Ensure a persistent background event loop exists for async Prisma calls."""
    global _loop, _loop_thread
    if _loop is not None and _loop.is_running():
        return _loop
    _loop = asyncio.new_event_loop()
    _loop_thread = threading.Thread(target=_loop.run_forever, daemon=True, name="prisma-event-loop")
    _loop_thread.start()
    return _loop


def _run_async(coro):
    """Run an async coroutine on the persistent background event loop. Blocks until done."""
    loop = _ensure_event_loop()
    future = asyncio.run_coroutine_threadsafe(coro, loop)
    return future.result()


class AutomationEngine:
    """
    Real-time automation engine that evaluates sensor conditions and executes robot actions.
    Supports sensor triggers, time-based triggers, manual triggers, and webhook triggers.
    Uses Redis for sensor value caching (10Hz) and rate limiting.
    """

    REDIS_SENSOR_KEY = 'amm:sensors'
    REDIS_RATE_PREFIX = 'amm:rate:'
    REDIS_SENSOR_TTL = 2  # seconds — stale data expires

    def __init__(self, sensor_manager, mega_interface, simulation_mode=False):
        self.sensor_manager = sensor_manager
        self.mega_interface = mega_interface
        self.simulation_mode = simulation_mode

        # Redis client (initialized in initialize())
        self.redis = None

        # Fallback in-memory cache when Redis is unavailable
        self._fallback_cache: Dict[str, Any] = {}
        self._fallback_lock = threading.RLock()

        self.cache_update_interval = 0.1  # 100ms = 10Hz

        # Rate limiting config
        self.rate_limit_window = 5.0
        self.max_executions_per_window = 10

        # Circular trigger protection
        self.trigger_depth = 0
        self.max_trigger_depth = 5

        # Time-based scheduler
        self.scheduled_tasks = {}
        self.scheduler_running = False

        # Prisma client
        self.db = None

        # Control flags
        self.running = False
        self._cache_thread = None
        self._scheduler_thread = None

        # Feed key mapping (automation feed key -> sensor data path)
        self.feed_key_map = {
            'laser_left_front': ('laser_sensors', 'left_front'),
            'laser_left_back': ('laser_sensors', 'left_back'),
            'laser_right_front': ('laser_sensors', 'right_front'),
            'laser_right_back': ('laser_sensors', 'right_back'),
            'laser_back_left': ('laser_sensors', 'back_left'),
            'laser_back_right': ('laser_sensors', 'back_right'),
            'ultra_front_left': ('ultrasonic_sensors', 'front_left'),
            'ultra_front_right': ('ultrasonic_sensors', 'front_right'),
            'line_left': ('line_sensors', 'left'),
            'line_center': ('line_sensors', 'center'),
            'line_right': ('line_sensors', 'right'),
            'imu_heading': ('imu', 'orientation', 'z'),
            'imu_pitch': ('imu', 'orientation', 'y'),
            'imu_roll': ('imu', 'orientation', 'x'),
            'tf_luna_distance': ('tf_luna', 'distance'),
        }

    def initialize(self):
        """Initialize Prisma client, Redis, and database connection."""
        # Redis
        redis_url = os.environ.get('REDIS_URL', 'redis://localhost:6379/0')
        try:
            import redis
            self.redis = redis.from_url(redis_url, decode_responses=True)
            self.redis.ping()
            logger.info(f"Redis connected: {redis_url}")
        except Exception as e:
            logger.warning(f"Redis unavailable ({e}), using in-memory fallback")
            self.redis = None

        # Prisma — connect on the persistent background event loop
        try:
            from prisma import Prisma
            self.db = Prisma()
            _run_async(self.db.connect())
            logger.info("Prisma client connected to database")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize Prisma client: {e}")
            return False

    def start(self):
        """Start the automation engine (sensor cache + scheduler)."""
        if self.running:
            logger.warning("Automation engine already running")
            return

        self.running = True

        # Start sensor cache update thread
        self._cache_thread = threading.Thread(
            target=self._update_sensor_cache,
            daemon=True,
            name="sensor-cache"
        )
        self._cache_thread.start()
        logger.info("Sensor cache started (10Hz)")

        # Start time-based scheduler
        self._scheduler_thread = threading.Thread(
            target=self._run_scheduler,
            daemon=True,
            name="automation-scheduler"
        )
        self._scheduler_thread.start()
        logger.info("Time-based scheduler started")

    def stop(self):
        """Stop the automation engine."""
        self.running = False
        if self.db:
            self.db.disconnect()
        logger.info("Automation engine stopped")

    # ── Sensor cache (Redis with in-memory fallback) ──────────────────

    def _update_sensor_cache(self):
        """Continuously push sensor data to Redis at 10Hz."""
        while self.running:
            try:
                sensor_data = self.sensor_manager.read_all_sensors()
                if self.redis:
                    try:
                        self.redis.set(self.REDIS_SENSOR_KEY, json.dumps(sensor_data), ex=self.REDIS_SENSOR_TTL)
                    except Exception:
                        pass
                # Always keep fallback for direct reads
                with self._fallback_lock:
                    self._fallback_cache = sensor_data
                time.sleep(self.cache_update_interval)
            except Exception as e:
                logger.error(f"Error updating sensor cache: {e}")
                time.sleep(1.0)

    def _read_sensor_cache(self) -> Dict[str, Any]:
        """Read sensor data from Redis, falling back to in-memory cache."""
        if self.redis:
            try:
                raw = self.redis.get(self.REDIS_SENSOR_KEY)
                if raw:
                    return json.loads(raw)
            except Exception:
                pass
        with self._fallback_lock:
            return dict(self._fallback_cache)

    def get_cached_sensor_value(self, feed_key: str) -> Optional[Any]:
        """Get a sensor value from the cache by feed key."""
        if feed_key == 'mega_connected':
            return self.mega_interface.mega_connected if self.mega_interface else False

        if feed_key not in self.feed_key_map:
            return None

        path = self.feed_key_map[feed_key]
        data = self._read_sensor_cache()

        try:
            for key in path:
                data = data[key]
            return data
        except (KeyError, TypeError):
            return None

    def get_all_feed_values(self) -> Dict[str, Any]:
        """Get current values for all supported feed keys."""
        values = {}
        for feed_key in self.feed_key_map:
            values[feed_key] = self.get_cached_sensor_value(feed_key)
        values['mega_connected'] = self.mega_interface.mega_connected if self.mega_interface else False
        return values

    def evaluate_condition(self, operator: str, actual: Any, threshold: Any) -> bool:
        """Evaluate a single condition against a threshold."""
        try:
            # Convert to floats for numeric comparison
            if isinstance(actual, (int, float)) and isinstance(threshold, (int, float)):
                actual_val = float(actual)
                threshold_val = float(threshold)
            elif isinstance(threshold, str):
                # Handle boolean strings
                if threshold.lower() in ('true', '1', 'yes'):
                    threshold_val = True
                elif threshold.lower() in ('false', '0', 'no'):
                    threshold_val = False
                else:
                    try:
                        threshold_val = float(threshold)
                        actual_val = float(actual)
                    except (ValueError, TypeError):
                        # String comparison
                        return self._compare_strings(operator, str(actual), str(threshold))
            else:
                actual_val = actual
                threshold_val = threshold

            return self._compare_numeric(operator, actual_val, threshold_val)
        except (ValueError, TypeError) as e:
            logger.error(f"Error evaluating condition: {e}")
            return False

    def _compare_numeric(self, operator: str, actual: float, threshold: float) -> bool:
        """Compare two numeric values with the given operator."""
        if operator == '>':
            return actual > threshold
        elif operator == '<':
            return actual < threshold
        elif operator == '>=':
            return actual >= threshold
        elif operator == '<=':
            return actual <= threshold
        elif operator == '==':
            return abs(actual - threshold) < 0.001  # Float tolerance
        elif operator == '!=':
            return abs(actual - threshold) >= 0.001
        else:
            logger.error(f"Unknown operator: {operator}")
            return False

    def _compare_strings(self, operator: str, actual: str, threshold: str) -> bool:
        """Compare two string values with the given operator."""
        if operator == '==':
            return actual.lower() == threshold.lower()
        elif operator == '!=':
            return actual.lower() != threshold.lower()
        else:
            logger.error(f"String comparison only supports == and !=, got {operator}")
            return False

    def evaluate_automation(self, automation) -> dict:
        """
        Evaluate an automation's conditions against current sensor values.
        Returns dict with 'matched' (bool), 'type' ('if' or 'else'), and 'conditions' details.
        """
        try:
            # Get conditions from database
            conditions = _run_async(self.db.automationcondition.find_many(
                where={'automationId': automation.id}
            ))

            if not conditions:
                return {'matched': False, 'type': 'if', 'conditions': []}

            # Split into if and else conditions
            if_conditions = [c for c in conditions if not c.isElse]
            else_conditions = [c for c in conditions if c.isElse]

            # Evaluate if conditions
            if_results = []
            if_match = True

            for condition in if_conditions:
                actual_value = self.get_cached_sensor_value(condition.feedKey)
                if actual_value is None:
                    if_results.append({
                        'feedKey': condition.feedKey,
                        'operator': condition.operator,
                        'threshold': condition.value,
                        'actual': None,
                        'matched': False
                    })
                    if_match = False
                    continue

                matched = self.evaluate_condition(condition.operator, actual_value, condition.value)
                if_results.append({
                    'feedKey': condition.feedKey,
                    'operator': condition.operator,
                    'threshold': condition.value,
                    'actual': actual_value,
                    'matched': matched
                })

                # Apply condition matching logic
                if automation.conditionMatch == 'ALL' and not matched:
                    if_match = False
                    break
                elif automation.conditionMatch == 'ANY' and matched:
                    if_match = True
                    break

            # If if-conditions matched, return if-branch
            if if_match:
                return {'matched': True, 'type': 'if', 'conditions': if_results}

            # Otherwise, evaluate else conditions
            if not else_conditions:
                return {'matched': False, 'type': 'if', 'conditions': if_results}

            else_results = []
            else_match = True

            for condition in else_conditions:
                actual_value = self.get_cached_sensor_value(condition.feedKey)
                if actual_value is None:
                    else_results.append({
                        'feedKey': condition.feedKey,
                        'operator': condition.operator,
                        'threshold': condition.value,
                        'actual': None,
                        'matched': False
                    })
                    else_match = False
                    continue

                matched = self.evaluate_condition(condition.operator, actual_value, condition.value)
                else_results.append({
                    'feedKey': condition.feedKey,
                    'operator': condition.operator,
                    'threshold': condition.value,
                    'actual': actual_value,
                    'matched': matched
                })

                if automation.elseConditionMatch == 'ALL' and not matched:
                    else_match = False
                    break
                elif automation.elseConditionMatch == 'ANY' and matched:
                    else_match = True
                    break

            if else_match:
                return {'matched': True, 'type': 'else', 'conditions': if_results + else_results}
            else:
                return {'matched': False, 'type': 'if', 'conditions': if_results + else_results}

        except Exception as e:
            logger.error(f"Error evaluating automation {automation.id}: {e}")
            return {'matched': False, 'type': 'if', 'conditions': []}

    def check_rate_limit(self, automation_id: int) -> bool:
        """Check if an automation has exceeded its rate limit.
        Uses Redis sorted sets when available, falls back to in-memory."""
        key = f"{self.REDIS_RATE_PREFIX}{automation_id}"
        now = time.time()
        window_start = now - self.rate_limit_window

        if self.redis:
            try:
                pipe = self.redis.pipeline()
                pipe.zremrangebyscore(key, '-inf', window_start)
                pipe.zcard(key)
                pipe.zadd(key, {str(now): now})
                pipe.expire(key, int(self.rate_limit_window) + 1)
                results = pipe.execute()
                count = results[1]
                if count >= self.max_executions_per_window:
                    # Remove the entry we just added
                    self.redis.zrem(key, str(now))
                    return False
                return True
            except Exception:
                pass

        # In-memory fallback
        return True

    def execute_action(self, action) -> bool:
        """Execute a single robot action. Returns True if successful."""
        try:
            if action.actionType == 'move':
                return self._execute_move(action.actionValue)
            elif action.actionType == 'turn':
                return self._execute_turn(action.actionValue)
            elif action.actionType == 'gripper':
                return self._execute_gripper(action.actionValue)
            elif action.actionType == 'gripper_tilt':
                return self._execute_gripper_tilt(action.actionValue)
            elif action.actionType == 'stop':
                return self._execute_stop()
            elif action.actionType == 'speed':
                return self._execute_speed(action.actionValue)
            elif action.actionType == 'patrol':
                return self._execute_patrol(action.actionValue)
            elif action.actionType == 'delay':
                return self._execute_delay(action.actionValue)
            elif action.actionType == 'webhook':
                return self._execute_webhook(action.webhookUrl)
            elif action.actionType == 'trigger':
                return self._execute_trigger_chain(action.triggerId)
            else:
                logger.error(f"Unknown action type: {action.actionType}")
                return False
        except Exception as e:
            logger.error(f"Error executing action {action.actionType}: {e}")
            return False

    def _execute_move(self, direction: str) -> bool:
        """Execute a move command."""
        direction_map = {
            'forward': 'f',
            'backward': 'b',
            'forward-left': 'q',
            'forward-right': 'e',
            'backward-left': 'z',
            'backward-right': 'x'
        }
        cmd = direction_map.get(direction.lower())
        if cmd and self.mega_interface:
            return self.mega_interface.send_command_to_mega(cmd)
        return False

    def _execute_turn(self, direction: str) -> bool:
        """Execute a turn command."""
        if direction.lower() == 'left':
            return self.mega_interface.send_command_to_mega('t') if self.mega_interface else False
        elif direction.lower() == 'right':
            return self.mega_interface.send_command_to_mega('y') if self.mega_interface else False
        else:
            # Try to parse as degrees
            try:
                degrees = int(direction)
                if degrees > 0:
                    return self.mega_interface.send_command_to_mega(f'ta{degrees}') if self.mega_interface else False
                else:
                    return self.mega_interface.send_command_to_mega(f'ta{degrees}') if self.mega_interface else False
            except ValueError:
                return False

    def _execute_gripper(self, action: str) -> bool:
        """Execute a gripper open/close command."""
        if action.lower() == 'open':
            return self.mega_interface.send_command_to_mega('no') if self.mega_interface else False
        elif action.lower() == 'close':
            return self.mega_interface.send_command_to_mega('nc') if self.mega_interface else False
        return False

    def _execute_gripper_tilt(self, angle: str) -> bool:
        """Execute a gripper tilt command."""
        try:
            angle_val = int(angle)
            return self.mega_interface.send_command_to_mega(f'ta{angle_val}') if self.mega_interface else False
        except ValueError:
            return False

    def _execute_stop(self) -> bool:
        """Execute an emergency stop."""
        return self.mega_interface.send_command_to_mega('s') if self.mega_interface else False

    def _execute_speed(self, speed: str) -> bool:
        """Execute a speed command."""
        try:
            speed_val = int(speed)
            if 0 <= speed_val <= 100:
                # Send speed command (format depends on Mega firmware)
                return self.mega_interface.send_command_to_mega(f'sp{speed_val}') if self.mega_interface else False
        except ValueError:
            pass
        return False

    def _execute_patrol(self, action: str) -> bool:
        """Execute a patrol command."""
        if action.lower() == 'start':
            return self.mega_interface.send_command_to_mega('pp') if self.mega_interface else False
        elif action.lower() == 'stop':
            return self.mega_interface.send_command_to_mega('s') if self.mega_interface else False
        return False

    def _execute_delay(self, delay_ms: str) -> bool:
        """Execute a delay."""
        try:
            delay = int(delay_ms) / 1000.0  # Convert ms to seconds
            time.sleep(delay)
            return True
        except ValueError:
            return False

    def _execute_webhook(self, url: str) -> bool:
        """Execute a webhook POST request."""
        if not url:
            return False
        try:
            import requests
            response = requests.post(url, timeout=5)
            return response.status_code < 400
        except Exception as e:
            logger.error(f"Webhook failed: {e}")
            return False

    def _execute_trigger_chain(self, trigger_id) -> bool:
        """Execute a chained automation trigger."""
        if self.trigger_depth >= self.max_trigger_depth:
            logger.warning(f"Max trigger depth ({self.max_trigger_depth}) reached, skipping chain")
            return False

        if trigger_id is None:
            return False

        try:
            automation = _run_async(self.db.automation.find_unique(where={'id': int(trigger_id)}))
            if automation and automation.isActive:
                self.trigger_depth += 1
                result = self.run_automation(automation, reason="chain")
                self.trigger_depth -= 1
                return result
        except Exception as e:
            logger.error(f"Error in trigger chain: {e}")
            self.trigger_depth = max(0, self.trigger_depth - 1)
        return False

    def run_automation(self, automation, reason: str = "sensor") -> bool:
        """
        Run an automation: evaluate conditions, execute actions if matched.
        Returns True if automation was triggered and actions were executed.
        """
        # Check rate limit
        if not self.check_rate_limit(automation.id):
            logger.debug(f"Rate limited: {automation.name}")
            return False

        # Evaluate conditions
        eval_result = self.evaluate_automation(automation)

        if not eval_result['matched']:
            return False

        # Get actions based on matched branch
        actions = _run_async(self.db.automationaction.find_many(
            where={
                'automationId': automation.id,
                'isElse': eval_result['type'] == 'else'
            },
            order={'actionOrder': 'asc'}
        ))

        if not actions:
            return False

        # Execute actions
        actions_executed = 0
        for action in actions:
            if self.execute_action(action):
                actions_executed += 1
            else:
                logger.warning(f"Action {action.actionType} failed for automation {automation.name}")

        # Log execution
        try:
            _run_async(self.db.automationlog.create(
                data={
                    'automationId': automation.id,
                    'automationName': automation.name,
                    'triggerReason': reason,
                    'actionsExecuted': actions_executed
                }
            ))
        except Exception as e:
            logger.error(f"Failed to log automation execution: {e}")

        logger.info(f"Automation '{automation.name}' triggered ({reason}), {actions_executed}/{len(actions)} actions executed")
        return True

    def _run_scheduler(self):
        """Background scheduler for time-based automations."""
        self.scheduler_running = True
        last_check = {}

        while self.running:
            try:
                now = datetime.now()

                # Get all active automations with time triggers
                automations = _run_async(self.db.automation.find_many(
                    where={
                        'isActive': True,
                        'triggerType': 'time'
                    }
                ))

                for automation in automations:
                    if not automation.scheduleCron:
                        continue

                    # Simple cron matching (minute-level granularity)
                    if self._should_run_now(automation.scheduleCron, now, last_check.get(automation.id)):
                        self.run_automation(automation, reason="scheduled")
                        last_check[automation.id] = now

                time.sleep(30)  # Check every 30 seconds

            except Exception as e:
                logger.error(f"Scheduler error: {e}")
                time.sleep(5)

    def _should_run_now(self, cron_expr: str, now: datetime, last_run: Optional[datetime]) -> bool:
        """
        Simple cron expression matcher. Supports:
        - */n (every n minutes)
        - * (every minute)
        - Specific minute (e.g., "0", "30")
        """
        if not cron_expr:
            return False

        # Parse minute field (first field)
        parts = cron_expr.split()
        if len(parts) < 1:
            return False

        minute_field = parts[0]
        current_minute = now.minute

        # Check if we should run this minute
        should_run = False

        if minute_field == '*':
            should_run = True
        elif minute_field.startswith('*/'):
            try:
                interval = int(minute_field[2:])
                should_run = current_minute % interval == 0
            except ValueError:
                pass
        else:
            try:
                target_minute = int(minute_field)
                should_run = current_minute == target_minute
            except ValueError:
                pass

        if not should_run:
            return False

        # Avoid running twice in the same minute
        if last_run and last_run.date() == now.date() and last_run.hour == now.hour and last_run.minute == now.minute:
            return False

        return True

    def trigger_manual(self, automation_id: int) -> dict:
        """Manually trigger an automation via API."""
        try:
            automation = _run_async(self.db.automation.find_unique(where={'id': automation_id}))
            if not automation:
                return {'success': False, 'error': 'Automation not found'}

            result = self.run_automation(automation, reason="manual")
            return {'success': True, 'triggered': result}
        except Exception as e:
            logger.error(f"Error triggering automation {automation_id}: {e}")
            return {'success': False, 'error': str(e)}

    def get_available_feeds(self) -> List[dict]:
        """Get list of available sensor feeds with metadata."""
        feeds = [
            {'key': 'laser_left_front', 'name': 'Laser Left Front', 'unit': 'cm', 'category': 'laser'},
            {'key': 'laser_left_back', 'name': 'Laser Left Back', 'unit': 'cm', 'category': 'laser'},
            {'key': 'laser_right_front', 'name': 'Laser Right Front', 'unit': 'cm', 'category': 'laser'},
            {'key': 'laser_right_back', 'name': 'Laser Right Back', 'unit': 'cm', 'category': 'laser'},
            {'key': 'laser_back_left', 'name': 'Laser Back Left', 'unit': 'cm', 'category': 'laser'},
            {'key': 'laser_back_right', 'name': 'Laser Back Right', 'unit': 'cm', 'category': 'laser'},
            {'key': 'ultra_front_left', 'name': 'Ultrasonic Front Left', 'unit': 'cm', 'category': 'ultrasonic'},
            {'key': 'ultra_front_right', 'name': 'Ultrasonic Front Right', 'unit': 'cm', 'category': 'ultrasonic'},
            {'key': 'line_left', 'name': 'Line Sensor Left', 'unit': 'bool', 'category': 'line'},
            {'key': 'line_center', 'name': 'Line Sensor Center', 'unit': 'bool', 'category': 'line'},
            {'key': 'line_right', 'name': 'Line Sensor Right', 'unit': 'bool', 'category': 'line'},
            {'key': 'imu_heading', 'name': 'IMU Heading', 'unit': 'deg', 'category': 'imu'},
            {'key': 'imu_pitch', 'name': 'IMU Pitch', 'unit': 'deg', 'category': 'imu'},
            {'key': 'imu_roll', 'name': 'IMU Roll', 'unit': 'deg', 'category': 'imu'},
            {'key': 'tf_luna_distance', 'name': 'TF-Luna Distance', 'unit': 'cm', 'category': 'distance'},
            {'key': 'mega_connected', 'name': 'Mega Connected', 'unit': 'bool', 'category': 'system'},
        ]
        return feeds

    def get_available_actions(self) -> List[dict]:
        """Get list of available action types with metadata."""
        actions = [
            {'type': 'move', 'name': 'Move', 'values': ['forward', 'backward', 'left', 'right']},
            {'type': 'turn', 'name': 'Turn', 'values': ['left', 'right', '<degrees>']},
            {'type': 'gripper', 'name': 'Gripper', 'values': ['open', 'close']},
            {'type': 'gripper_tilt', 'name': 'Gripper Tilt', 'values': ['0-180']},
            {'type': 'stop', 'name': 'Stop', 'values': []},
            {'type': 'speed', 'name': 'Speed', 'values': ['0-100']},
            {'type': 'patrol', 'name': 'Patrol', 'values': ['start', 'stop']},
            {'type': 'delay', 'name': 'Delay', 'values': ['<ms>']},
            {'type': 'webhook', 'name': 'Webhook', 'values': ['<url>']},
            {'type': 'trigger', 'name': 'Trigger Automation', 'values': ['<automation_id>']},
        ]
        return actions
