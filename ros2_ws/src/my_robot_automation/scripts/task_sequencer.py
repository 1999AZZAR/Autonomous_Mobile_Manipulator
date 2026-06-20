"""
Task Sequencer — chain multi-step missions with conditions and actions.
"""

import time
import logging
from typing import Any, Callable, Optional
from enum import Enum

logger = logging.getLogger(__name__)


class ConditionType(Enum):
    SENSOR = "sensor"
    TIMEOUT = "timeout"
    POSITION = "position"
    ALWAYS = "always"


class ActionType(Enum):
    MOVE = "move"
    TURN = "turn"
    GRIPPER = "gripper"
    TILT = "tilt"
    LIFTER = "lifter"
    WAIT = "wait"
    GOTO_WAYPOINT = "goto_waypoint"
    FOLLOW_LINE = "follow_line"
    ROTATE = "rotate"
    CALL_SUB = "call_sub"


class StepCondition:
    def __init__(self, ctype: ConditionType, params: dict[str, Any] | None = None):
        self.ctype = ctype
        self.params = params or {}

    def evaluate(self, context: dict[str, Any]) -> bool:
        if self.ctype == ConditionType.ALWAYS:
            return True
        if self.ctype == ConditionType.TIMEOUT:
            elapsed = time.time() - context.get("step_start", time.time())
            return elapsed >= self.params.get("seconds", 0)
        if self.ctype == ConditionType.SENSOR:
            val = context.get("sensors", {}).get(self.params.get("key", ""), 0)
            threshold = self.params.get("threshold", 300)
            operator = self.params.get("operator", "<")
            if operator == "<":
                return val < threshold
            if operator == ">":
                return val > threshold
            return val == threshold
        if self.ctype == ConditionType.POSITION:
            pos = context.get("position", (0, 0))
            tx, ty = self.params.get("x", 0), self.params.get("y", 0)
            tol = self.params.get("tolerance", 0.2)
            return ((pos[0] - tx)**2 + (pos[1] - ty)**2)**0.5 <= tol
        return False


class TaskStep:
    def __init__(self, name: str, action_type: ActionType,
                 action_params: dict[str, Any] | None = None,
                 condition: StepCondition | None = None,
                 retry_count: int = 0):
        self.name = name
        self.action_type = action_type
        self.action_params = action_params or {}
        self.condition = condition or StepCondition(ConditionType.ALWAYS)
        self.retry_count = retry_count
        self._attempts = 0

    @property
    def exhausted(self) -> bool:
        return self.retry_count > 0 and self._attempts > self.retry_count


class TaskSequence:
    def __init__(self, name: str, steps: list[TaskStep] | None = None):
        self.name = name
        self.steps = steps or []
        self._current = 0

    @property
    def current_step(self) -> Optional[TaskStep]:
        if 0 <= self._current < len(self.steps):
            return self.steps[self._current]
        return None

    @property
    def done(self) -> bool:
        return self._current >= len(self.steps)


class TaskSequencer:
    """Executes multi-step task sequences with conditions, retry, and error handling."""

    def __init__(self, send_command: Callable[[str], None],
                 get_context: Callable[[], dict[str, Any]]):
        self.send_command = send_command
        self.get_context = get_context
        self._queue: list[TaskSequence] = []
        self._current: Optional[TaskSequence] = None
        self._paused = False
        self._context: dict[str, Any] = {}
        self._step_start = 0.0

    @property
    def running(self) -> bool:
        return self._current is not None and not self._paused

    def enqueue(self, seq: TaskSequence):
        self._queue.append(seq)
        logger.info("TaskSequencer: enqueued '%s' (%d steps)", seq.name, len(seq.steps))

    def start(self, seq: TaskSequence) -> bool:
        if self._current and not self._current.done:
            logger.warning("TaskSequencer: already running '%s'", self._current.name)
            return False
        self._current = seq
        self._paused = False
        logger.info("TaskSequencer: started '%s' (%d steps)", seq.name, len(seq.steps))
        return True

    def pause(self):
        self._paused = True
        self.send_command('s')

    def resume(self):
        self._paused = False

    def stop(self):
        self._current = None
        self._queue.clear()
        self.send_command('s')

    def abort(self):
        self._current = None
        self.send_command('s')
        logger.warning("TaskSequencer: ABORTED")

    def tick(self):
        if self._paused or not self._current:
            self._try_dequeue()
            return

        seq = self._current
        if seq.done:
            logger.info("TaskSequencer: '%s' completed", seq.name)
            self._current = None
            self._try_dequeue()
            return

        step = seq.current_step
        if step is None:
            seq._current += 1
            return

        if step._attempts == 0:
            self._step_start = time.time()
            self._execute_step(step)
            step._attempts = 1
        else:
            self._context = self.get_context()
            if step.condition.evaluate(self._context):
                logger.info("TaskSequencer: step '%s' condition met", step.name)
                seq._current += 1
            elif step.exhausted:
                logger.warning("TaskSequencer: step '%s' exhausted retries", step.name)
                seq._current += 1
            else:
                elapsed = time.time() - self._step_start
                if elapsed >= 5.0:
                    self._execute_step(step)
                    step._attempts += 1
                    self._step_start = time.time()

    def _execute_step(self, step: TaskStep):
        params = step.action_params
        logger.debug("TaskSequencer: executing '%s' → %s %s",
                     step.name, step.action_type.value, params)

        try:
            if step.action_type == ActionType.MOVE:
                direction = params.get("direction", "f")
                duration = params.get("duration", 1.0)
                self.send_command(direction)
                if duration > 0:
                    time.sleep(duration)
                    self.send_command('s')

            elif step.action_type == ActionType.TURN:
                angle = params.get("angle", 90)
                self.send_command(f'ta{int(angle)}')

            elif step.action_type == ActionType.GRIPPER:
                action = params.get("action", "close")
                cmd_map = {"close": "nc", "open": "no", "half": "nh"}
                self.send_command(cmd_map.get(action, "nc"))

            elif step.action_type == ActionType.TILT:
                angle = params.get("angle", 90)
                self.send_command(f'ta{int(angle)}')

            elif step.action_type == ActionType.LIFTER:
                direction = params.get("direction", "up")
                self.send_command('u' if direction == "up" else 'd')
                time.sleep(params.get("duration", 1.0))
                self.send_command('s')

            elif step.action_type == ActionType.WAIT:
                seconds = params.get("seconds", 1.0)
                time.sleep(seconds)

            elif step.action_type == ActionType.GOTO_WAYPOINT:
                pass

            elif step.action_type == ActionType.FOLLOW_LINE:
                pass

            elif step.action_type == ActionType.ROTATE:
                angle = params.get("angle", 360)
                direction = "y" if angle > 0 else "t"
                duration = min(abs(angle) / 45.0, 4.0)
                self.send_command(direction)
                time.sleep(duration)
                self.send_command('s')

            elif step.action_type == ActionType.CALL_SUB:
                pass

        except Exception as exc:
            logger.error("TaskSequencer: step '%s' error: %s", step.name, exc)
            self.send_command('s')

    def _try_dequeue(self):
        if self._queue and not self._current:
            nxt = self._queue.pop(0)
            self.start(nxt)
