"""
Finite State Machine for Autonomous Mobile Manipulator.

Priority hierarchy (highest → lowest):
  ESTOP > LINE_FOLLOW > TASK_SEQ > IFTTT > AI_VISION > WAYPOINT > MANUAL > IDLE
"""

import enum
import time
import logging
from typing import Optional, Callable

logger = logging.getLogger(__name__)


class State(enum.Enum):
    IDLE = 0
    MANUAL = 1
    WAYPOINT = 2
    AI_VISION = 3
    IFTTT = 4
    TASK_SEQ = 5
    LINE_FOLLOW = 6
    CALIBRATE = 7
    ESTOP = 8


_STATE_PRIORITY = {
    State.ESTOP: 100,
    State.LINE_FOLLOW: 80,
    State.TASK_SEQ: 70,
    State.IFTTT: 60,
    State.AI_VISION: 50,
    State.WAYPOINT: 40,
    State.MANUAL: 30,
    State.CALIBRATE: 20,
    State.IDLE: 10,
}


def can_preempt(current: State, requested: State) -> bool:
    return _STATE_PRIORITY[requested] > _STATE_PRIORITY[current]


TransitionHook = Callable[[State, State], None]


class RobotFSM:
    def __init__(self, initial: State = State.IDLE):
        self._state = initial
        self._prev: Optional[State] = None
        self._transition_hooks: list[TransitionHook] = []
        self._lock = False  # prevent re-entrant transitions
        self._enter_times: dict[State, float] = {}
        self._line_engaged = False
        logger.info("FSM initialised: %s", initial.name)

    @property
    def state(self) -> State:
        return self._state

    @property
    def previous(self) -> Optional[State]:
        return self._prev

    def time_in_state(self) -> float:
        t0 = self._enter_times.get(self._state)
        return time.time() - t0 if t0 else 0.0

    def on_transition(self, hook: TransitionHook):
        self._transition_hooks.append(hook)

    def request(self, new: State, reason: str = "") -> bool:
        if self._lock:
            logger.debug("FSM locked, ignoring %s request", new.name)
            return False
        if new == self._state:
            return True

        if not can_preempt(self._state, new):
            logger.info(
                "FSM deny %s → %s: priority too low (reason: %s)",
                self._state.name, new.name, reason or "none"
            )
            return False

        self._lock = True
        try:
            old = self._state
            self._prev = old
            self._state = new
            self._enter_times[new] = time.time()
            logger.info(
                "FSM transition: %s → %s [%s]",
                old.name, new.name, reason or "no reason"
            )
            for hook in self._transition_hooks:
                try:
                    hook(old, new)
                except Exception as exc:
                    logger.error("FSM hook error: %s", exc)
        finally:
            self._lock = False
        return True

    def estop(self, reason: str = "emergency"):
        self.request(State.ESTOP, reason)

    def release_estop(self) -> bool:
        if self._state != State.ESTOP:
            return True
        prev = self._prev or State.IDLE
        logger.info("ESTOP released → %s", prev.name)
        return self.request(prev, "estop-release")

    def tick_line_sensors(self, line_left: bool, line_center: bool, line_right: bool):
        confidence = 0
        if line_center:
            confidence += 2
        if line_left:
            confidence += 1
        if line_right:
            confidence += 1

        on_line = confidence >= 2

        if on_line and not self._line_engaged:
            self._line_engaged = True
            if can_preempt(self._state, State.LINE_FOLLOW):
                self.request(State.LINE_FOLLOW, "line-detected")
        elif not on_line and self._line_engaged:
            self._line_engaged = False
            if self._state == State.LINE_FOLLOW:
                prev = self._prev if self._prev and self._prev != State.LINE_FOLLOW else State.IDLE
                self.request(prev, "line-lost")
