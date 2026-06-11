import math
import random
import time
import logging
from typing import List, Dict, Optional, Tuple

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

if not logger.handlers:
    logger.addHandler(logging.StreamHandler())

COMMANDS = ['f', 'b', 'q', 'e', 'z', 'x', 't', 'y', 's']

# Robot-local velocities (m/s) for each command
CMD_VEL = {
    'f': (0, 0.3), 'b': (0, -0.3),
    'q': (0.21, 0.21), 'e': (-0.21, 0.21),
    'z': (0.21, -0.21), 'x': (-0.21, -0.21),
    't': (0, 0), 'y': (0, 0), 's': (0, 0),
}
CMD_OMEGA = {'t': 60, 'y': -60}  # deg/s

SENSOR_ANGLES = {
    'laser_left_front': 45,
    'laser_left_back': 135,
    'laser_right_front': -45,
    'laser_right_back': -135,
    'laser_back_left': 90,
    'laser_back_right': -90,
    'ultra_front_left': 20,
    'ultra_front_right': -20,
}

LASER_MAX = 1500
ULTRA_MAX = 4000
SPEED = 0.3
OBSTACLE_CLEARANCE = 0.35


class BackendSimulation:
    def __init__(self):
        self.reset()

    def reset(self, obstacles: Optional[List[Dict]] = None):
        self.x = 0.0
        self.y = -2.0
        self.heading = 0.0
        self._heading_offset = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.total_distance = 0.0
        self.collisions = 0
        self.steps = 0

        self.navigation_goal: Optional[Tuple[float, float]] = None
        self.goal_reached = False
        self._stuck_counter = 0
        self._last_pos = (0.0, -2.0)
        self._last_cmd = 's'
        self._obstacle_avoid_mode = False
        self._avoid_timer = 0
        self._avoid_direction = 1

        self.obstacles = obstacles or [
            {'x': 0.8, 'y': 0.0, 'w': 0.4, 'h': 0.4},
            {'x': -0.6, 'y': 1.2, 'w': 0.4, 'h': 0.4},
            {'x': 0.5, 'y': 2.5, 'w': 0.4, 'h': 0.4},
            {'x': -0.8, 'y': 3.5, 'w': 0.4, 'h': 0.4},
        ]

        self.bounds = {'x_min': -2.0, 'x_max': 2.0, 'y_min': -2.5, 'y_max': 5.0}
        self.last_command = 's'
        self._time = 0.0

    def set_obstacles(self, obstacles: List[Dict]):
        self.obstacles = [dict(o) for o in obstacles]

    def calibrate_heading(self):
        zero_heading = self.heading
        self._heading_offset = zero_heading
        logger.info(f"BackendSim heading calibrated: {zero_heading:.1f}° → 0°")

    def get_heading(self) -> float:
        raw = self.heading
        calibrated = raw - self._heading_offset
        while calibrated > 180:
            calibrated -= 360
        while calibrated < -180:
            calibrated += 360
        return calibrated

    def navigate_to(self, x: float, y: float):
        self.navigation_goal = (float(x), float(y))
        self.goal_reached = False
        self._last_pos = (self.x, self.y)
        self._stuck_counter = 0
        self._obstacle_avoid_mode = False
        logger.info(f"Navigation goal set: ({x:.3f}, {y:.3f})")

    def clear_goal(self):
        self.navigation_goal = None
        self.goal_reached = False
        self._obstacle_avoid_mode = False

    def step(self, command: str, dt: float = 1.0 / 30.0):
        self.last_command = command
        self.steps += 1
        self._time += dt

        vx, vy = CMD_VEL.get(command, (0, 0))
        omega = CMD_OMEGA.get(command, 0)

        heading_rad = math.radians(self.heading)
        dx = (vx * math.cos(heading_rad) - vy * math.sin(heading_rad))
        dy = (vx * math.sin(heading_rad) + vy * math.cos(heading_rad))

        new_x = self.x + dx
        new_y = self.y + dy
        new_heading = (self.heading + omega * dt) % 360

        collided = self._check_collision(new_x, new_y)
        if collided:
            self.collisions += 1
        else:
            self.x = new_x
            self.y = new_y
            self.total_distance += math.sqrt(dx * dx + dy * dy)

        self.x = max(self.bounds['x_min'], min(self.bounds['x_max'], self.x))
        self.y = max(self.bounds['y_min'], min(self.bounds['y_max'], self.y))
        self.heading = new_heading
        self.vx = vx
        self.vy = vy
        self.omega = omega

        if self.navigation_goal and not self.goal_reached:
            gx, gy = self.navigation_goal
            dist = math.sqrt((gx - self.x) ** 2 + (gy - self.y) ** 2)
            if dist < 0.3:
                self.goal_reached = True
                logger.info(f"Goal reached at ({self.x:.3f}, {self.y:.3f})")

    def _check_collision(self, x: float, y: float) -> bool:
        robot_r = 0.15
        for obs in self.obstacles:
            ox, oy = obs['x'], obs['y']
            ow = obs.get('w', 0.3) / 2
            oh = obs.get('h', 0.3) / 2
            closest_x = max(ox - ow, min(x, ox + ow))
            closest_y = max(oy - oh, min(y, oy + oh))
            dist = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
            if dist < robot_r:
                return True
        if x < self.bounds['x_min'] or x > self.bounds['x_max']:
            return True
        if y < self.bounds['y_min'] or y > self.bounds['y_max']:
            return True
        return False

    def get_sensors(self) -> Dict:
        sensors = {}
        for name, angle in SENSOR_ANGLES.items():
            absolute = (self.heading + angle) % 360
            dist = self._raycast(absolute)
            if 'laser' in name:
                dist = min(dist, LASER_MAX)
                noise = random.gauss(0, 15)
                dist = max(0, dist + noise)
                if random.random() < 0.02:
                    dist = LASER_MAX
            else:
                dist = min(dist, ULTRA_MAX)
                noise = random.gauss(0, 30)
                dist = max(0, dist + noise)

            sensors[name] = dist

        front_dist = min(
            sensors.get('ultra_front_left', ULTRA_MAX),
            sensors.get('ultra_front_right', ULTRA_MAX),
        )
        if front_dist < 200:
            for u in ['ultra_front_left', 'ultra_front_right']:
                if random.random() < 0.3:
                    sensors[u] += random.uniform(100, 300)

        calibrated_heading = self.get_heading()
        sensors['line_left'] = 0
        sensors['line_center'] = 1
        sensors['line_right'] = 0
        sensors['imu_heading'] = calibrated_heading + random.gauss(0, 2)
        sensors['imu_pitch'] = math.sin(self._time * 0.1) * 3.0
        sensors['imu_roll'] = math.cos(self._time * 0.1) * 5.0

        return sensors

    def _raycast(self, angle_deg: float) -> float:
        angle_rad = math.radians(angle_deg)
        max_dist = ULTRA_MAX / 1000.0
        step = 0.05
        for d in [i * step for i in range(int(max_dist / step))]:
            px = self.x + d * math.cos(angle_rad)
            py = self.y + d * math.sin(angle_rad)
            for obs in self.obstacles:
                ox, oy = obs['x'], obs['y']
                ow = obs.get('w', 0.3) / 2
                oh = obs.get('h', 0.3) / 2
                if (ox - ow <= px <= ox + ow) and (oy - oh <= py <= oy + oh):
                    return d * 1000.0
        return max_dist * 1000.0

    def compute_command(self) -> str:
        sensors = self.get_sensors()

        if self.goal_reached or self.navigation_goal is None:
            return 's'

        gx, gy = self.navigation_goal
        dx = gx - self.x
        dy = gy - self.y
        dist_to_goal = math.sqrt(dx * dx + dy * dy)

        if dist_to_goal < 0.3:
            self.goal_reached = True
            return 's'

        desired_heading = math.degrees(math.atan2(dy, dx)) % 360
        heading_error = (desired_heading - self.heading + 180) % 360 - 180

        current_pos = (self.x, self.y)
        pos_delta = math.sqrt(
            (current_pos[0] - self._last_pos[0]) ** 2
            + (current_pos[1] - self._last_pos[1]) ** 2
        )
        self._last_pos = current_pos

        if pos_delta < 0.002 and self._last_cmd in ['f', 'q', 'e']:
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0

        min_front = min(
            sensors.get('laser_left_front', LASER_MAX),
            sensors.get('laser_right_front', LASER_MAX),
            sensors.get('ultra_front_left', ULTRA_MAX),
            sensors.get('ultra_front_right', ULTRA_MAX),
        )

        min_left = min(
            sensors.get('laser_left_front', LASER_MAX),
            sensors.get('laser_left_back', LASER_MAX),
        )
        min_right = min(
            sensors.get('laser_right_front', LASER_MAX),
            sensors.get('laser_right_back', LASER_MAX),
        )

        if self._obstacle_avoid_mode:
            self._avoid_timer -= 1
            if self._avoid_timer <= 0:
                self._obstacle_avoid_mode = False
            if self._avoid_direction > 0:
                return 'q'
            return 'e'

        if min_front < OBSTACLE_CLEARANCE * 1000 or self._stuck_counter > 5:
            self._obstacle_avoid_mode = True
            self._avoid_timer = random.randint(5, 12)

            if dist_to_goal < 1.0:
                self._avoid_direction = 1 if heading_error >= 0 else -1
            elif min_left > min_right:
                self._avoid_direction = 1
            else:
                self._avoid_direction = -1

            if self._avoid_direction > 0:
                return 'q'
            return 'e'

        if abs(heading_error) > 5:
            if heading_error > 0:
                self._last_cmd = 't'
                return 't'
            self._last_cmd = 'y'
            return 'y'

        if min_front < 0.5 * 1000:
            self._last_cmd = 'f'
            return 'f'

        self._last_cmd = 's'
        return 's'

    def get_state(self) -> Dict:
        return {
            'x': round(self.x, 3),
            'y': round(self.y, 3),
            'heading': round(self.heading, 1),
            'vx': round(self.vx, 3),
            'vy': round(self.vy, 3),
            'omega': round(self.omega, 1),
            'total_distance': round(self.total_distance, 2),
            'collisions': self.collisions,
            'steps': self.steps,
            'obstacles': self.obstacles,
            'bounds': self.bounds,
        }


_sim_instance: Optional[BackendSimulation] = None


def get_backend_sim() -> BackendSimulation:
    global _sim_instance
    if _sim_instance is None:
        _sim_instance = BackendSimulation()
    return _sim_instance


def reset_backend_sim(obstacles: Optional[List[Dict]] = None):
    global _sim_instance
    _sim_instance = BackendSimulation()
    if obstacles:
        _sim_instance.reset(obstacles)
