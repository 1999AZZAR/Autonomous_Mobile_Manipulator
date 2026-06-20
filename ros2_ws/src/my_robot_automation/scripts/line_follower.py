"""
Line Follower Subsystem — PID controller + line detection + auto-transition.
"""

import time
import logging
from typing import Callable

logger = logging.getLogger(__name__)


class PIDController:
    def __init__(self, kp: float = 1.5, ki: float = 0.05, kd: float = 0.8,
                 integral_limit: float = 10.0, output_limit: float = 100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()

    def compute(self, error: float) -> float:
        now = time.time()
        dt = now - self._last_time
        if dt <= 0:
            dt = 0.01
        self._last_time = now

        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


class LineFollower:
    """Line tracking using 3-line sensor + PID → movement commands.

    Command mapping:
      error ~ 0              → 'f'   (straight)
      error small positive   → 'e'   (forward-right curve)
      error small negative   → 'q'   (forward-left curve)
      error large            → 'y'/'t' (hard turn CW/CCW)
    """

    def __init__(self, send_command: Callable[[str], None]):
        self.send_command = send_command
        self.pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
        self.active = False
        self._last_cmd_time = 0.0
        self._cmd_interval = 0.08

    def engage(self):
        if not self.active:
            self.active = True
            self.pid.reset()
            logger.info("Line follower ENGAGED")

    def disengage(self):
        if self.active:
            self.active = False
            self.send_command('s')
            logger.info("Line follower DISENGAGED")

    @staticmethod
    def compute_error(line_left: bool, line_center: bool, line_right: bool) -> float:
        if line_center:
            if line_left and not line_right:
                return -1.0
            if line_right and not line_left:
                return 1.0
            return 0.0
        if line_left:
            return -2.0
        if line_right:
            return 2.0
        return 0.0

    def tick(self, line_left: bool, line_center: bool, line_right: bool):
        if not self.active:
            return

        now = time.time()
        if now - self._last_cmd_time < self._cmd_interval:
            return
        self._last_cmd_time = now

        error = self.compute_error(line_left, line_center, line_right)

        if error == 0 and not line_center:
            self.send_command('s')
            return

        pid_out = self.pid.compute(error)

        if abs(pid_out) > 60:
            cmd = 'y' if pid_out > 0 else 't'
        elif abs(pid_out) > 20:
            cmd = 'e' if pid_out > 0 else 'q'
        else:
            cmd = 'f'

        self.send_command(cmd)
