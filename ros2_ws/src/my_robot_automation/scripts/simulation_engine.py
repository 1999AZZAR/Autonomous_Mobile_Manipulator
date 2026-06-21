import math
import random
import logging
from typing import List, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

COMMANDS = ['f', 'b', 'q', 'e', 'z', 'x', 't', 'y', 's']
CMD_VEL = {
    'f': (0, 0.3), 'b': (0, -0.3),
    'q': (0.21, 0.21), 'e': (-0.21, 0.21),
    'z': (0.21, -0.21), 'x': (-0.21, -0.21),
    't': (0, 0), 'y': (0, 0), 's': (0, 0),
}
CMD_OMEGA = {'t': 60, 'y': -60}

SENSOR_GEOMETRY = {
    'laser': {
        'left_front':  {'x': 0.19, 'y': 0.10, 'angle': 45},
        'left_back':   {'x': 0.19, 'y': -0.10, 'angle': 135},
        'right_front': {'x': -0.19, 'y': 0.10, 'angle': -45},
        'right_back':  {'x': -0.19, 'y': -0.10, 'angle': -135},
        'back_left':   {'x': 0.09, 'y': -0.18, 'angle': 90},
        'back_right':  {'x': -0.09, 'y': -0.18, 'angle': -90},
    },
    'ultra': {
        'front_left':  {'x': 0.10, 'y': 0.20, 'angle': 20},
        'front_right': {'x': -0.10, 'y': 0.20, 'angle': -20},
    },
}

LINE_SENSOR_X = [-0.048, 0, 0.048]
LINE_SENSOR_Y = 0.15

LASER_MAX_MM = 1500
ULTRA_MAX_MM = 4000

OBSTACLE_CLEARANCE = 0.35

DEFAULT_OBSTACLES: List[Dict] = [
    {'x': 0.8, 'y': 0.0, 'w': 0.4, 'h': 0.4},
    {'x': -0.6, 'y': 1.2, 'w': 0.4, 'h': 0.4},
    {'x': 0.5, 'y': 2.5, 'w': 0.4, 'h': 0.4},
    {'x': -0.8, 'y': 3.5, 'w': 0.4, 'h': 0.4},
]

DEFAULT_BOUNDS = {'x_min': -2.0, 'x_max': 2.0, 'y_min': -2.5, 'y_max': 5.0}


class SimulationEngine:
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
        self._time = 0.0

        self.navigation_goal: Optional[Tuple[float, float]] = None
        self.goal_reached = False
        self._stuck_counter = 0
        self._last_pos = (0.0, -2.0)
        self._last_cmd = 's'
        self._obstacle_avoid_mode = False
        self._avoid_timer = 0
        self._avoid_direction = 1

        self.obstacles = [dict(o) for o in (obstacles or DEFAULT_OBSTACLES)]
        self.bounds = dict(DEFAULT_BOUNDS)

    def set_obstacles(self, obstacles: List[Dict]):
        self.obstacles = [dict(o) for o in obstacles]

    def get_obstacles(self) -> List[Dict]:
        return list(self.obstacles)

    def calibrate_heading(self):
        self._heading_offset = self.heading
        logger.info(f"sim heading calibrated: {self.heading:.1f}° -> 0°")

    def get_heading(self) -> float:
        raw = self.heading - self._heading_offset
        while raw > 180:
            raw -= 360
        while raw < -180:
            raw += 360
        return raw

    def set_position(self, x: float, y: float, heading: float):
        self.x = x
        self.y = y
        self.heading = heading

    def navigate_to(self, x: float, y: float):
        self.navigation_goal = (float(x), float(y))
        self.goal_reached = False
        self._last_pos = (self.x, self.y)
        self._stuck_counter = 0
        self._obstacle_avoid_mode = False
        logger.info(f"nav goal: ({x:.3f}, {y:.3f})")

    def clear_goal(self):
        self.navigation_goal = None
        self.goal_reached = False
        self._obstacle_avoid_mode = False

    def step(self, command: str, dt: float = 1.0 / 30.0):
        self._last_cmd = command
        self.steps += 1
        self._time += dt

        vx, vy = CMD_VEL.get(command, (0, 0))
        omega = CMD_OMEGA.get(command, 0)

        h_rad = math.radians(self.heading)
        dx = vx * math.cos(h_rad) + vy * math.sin(h_rad)
        dy = -vx * math.sin(h_rad) + vy * math.cos(h_rad)

        nx = self.x + dx
        ny = self.y + dy
        nh = (self.heading + omega * dt) % 360

        if self._check_collision(nx, ny):
            self.collisions += 1
        else:
            self.x = nx
            self.y = ny
            self.total_distance += math.sqrt(dx * dx + dy * dy)

        self.x = max(self.bounds['x_min'], min(self.bounds['x_max'], self.x))
        self.y = max(self.bounds['y_min'], min(self.bounds['y_max'], self.y))
        self.heading = nh
        self.vx = vx
        self.vy = vy
        self.omega = omega

        if self.navigation_goal and not self.goal_reached:
            gx, gy = self.navigation_goal
            dist = math.hypot(gx - self.x, gy - self.y)
            if dist < 0.3:
                self.goal_reached = True
                logger.info(f"goal reached ({self.x:.3f}, {self.y:.3f})")

    def _check_collision(self, x: float, y: float) -> bool:
        rr = 0.15
        for o in self.obstacles:
            ox, oy = o['x'], o['y']
            ow = o.get('w', 0.3) / 2
            oh = o.get('h', 0.3) / 2
            cx = max(ox - ow, min(x, ox + ow))
            cy = max(oy - oh, min(y, oy + oh))
            if math.hypot(x - cx, y - cy) < rr:
                return True
        if x < self.bounds['x_min'] or x > self.bounds['x_max']:
            return True
        if y < self.bounds['y_min'] or y > self.bounds['y_max']:
            return True
        return False

    def get_sensors(self) -> Dict:
        sensors = {}
        for name, cfg in SENSOR_GEOMETRY['laser'].items():
            angle = (self.heading + cfg['angle']) % 360
            dist = self._raycast(angle)
            dist = min(dist, LASER_MAX_MM)
            dist = max(0, dist + random.gauss(0, 15))
            if random.random() < 0.02:
                dist = LASER_MAX_MM
            sensors[f'laser_{name}'] = round(dist)

        for name, cfg in SENSOR_GEOMETRY['ultra'].items():
            angle = (self.heading + cfg['angle']) % 360
            dist = self._raycast(angle)
            dist = min(dist, ULTRA_MAX_MM)
            dist = max(0, dist + random.gauss(0, 30))
            sensors[f'ultra_{name}'] = round(dist)

        front_dist = min(
            sensors.get('ultra_front_left', ULTRA_MAX_MM),
            sensors.get('ultra_front_right', ULTRA_MAX_MM),
        )
        if front_dist < 200:
            for u in ['ultra_front_left', 'ultra_front_right']:
                if random.random() < 0.3:
                    sensors[u] = round(sensors.get(u, ULTRA_MAX_MM) + random.uniform(100, 300))

        lx, ly, lz = 0, 0, 0
        h_rad = math.radians(self.heading)
        for i, offset_x in enumerate(LINE_SENSOR_X):
            wx = self.x + (offset_x * math.cos(h_rad) - LINE_SENSOR_Y * math.sin(h_rad))
            wy = self.y + (offset_x * math.sin(h_rad) + LINE_SENSOR_Y * math.cos(h_rad))
            on_floor = (
                self.bounds['x_min'] <= wx <= self.bounds['x_max']
                and self.bounds['y_min'] <= wy <= self.bounds['y_max']
            )
            on_obstacle = self._point_in_obstacle(wx, wy)
            if on_obstacle:
                sensors['line_left' if i == 0 else 'line_center' if i == 1 else 'line_right'] = 0
            elif on_floor:
                sensors['line_left' if i == 0 else 'line_center' if i == 1 else 'line_right'] = random.choice([0, 1023])
            else:
                sensors['line_left' if i == 0 else 'line_center' if i == 1 else 'line_right'] = 1023

        sensors['line_center'] = 1

        ch = self.get_heading()
        sensors['imu_heading'] = round(ch + random.gauss(0, 2), 1)
        sensors['imu_pitch'] = round(math.sin(self._time * 0.1) * 3.0, 1)
        sensors['imu_roll'] = round(math.cos(self._time * 0.1) * 5.0, 1)

        sensors['tf_luna_distance'] = round(
            min(self._raycast(self.heading), 1500) + random.gauss(0, 10)
        )

        return sensors

    def _point_in_obstacle(self, x: float, y: float) -> bool:
        for o in self.obstacles:
            ox, oy = o['x'], o['y']
            ow = o.get('w', 0.3) / 2
            oh = o.get('h', 0.3) / 2
            if (ox - ow <= x <= ox + ow) and (oy - oh <= y <= oy + oh):
                return True
        return False

    def _raycast(self, angle_deg: float) -> float:
        a_rad = math.radians(angle_deg)
        max_m = ULTRA_MAX_MM / 1000.0
        step = 0.05
        for d in [i * step for i in range(int(max_m / step))]:
            px = self.x + d * math.cos(a_rad)
            py = self.y + d * math.sin(a_rad)
            if self._point_in_obstacle(px, py):
                return d * 1000.0
        return max_m * 1000.0

    def compute_command(self) -> str:
        sensors = self.get_sensors()

        if self.goal_reached or self.navigation_goal is None:
            return 's'

        gx, gy = self.navigation_goal
        dx = gx - self.x
        dy = gy - self.y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < 0.3:
            self.goal_reached = True
            return 's'

        desired_heading = (90 - math.degrees(math.atan2(dy, dx)) + 360) % 360
        heading_error = (desired_heading - self.heading + 180) % 360 - 180

        pos_delta = math.hypot(self.x - self._last_pos[0], self.y - self._last_pos[1])
        self._last_pos = (self.x, self.y)

        if pos_delta < 0.002 and self._last_cmd in ('f', 'q', 'e'):
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0

        min_front = min(
            sensors.get('laser_left_front', LASER_MAX_MM),
            sensors.get('laser_right_front', LASER_MAX_MM),
            sensors.get('ultra_front_left', ULTRA_MAX_MM),
            sensors.get('ultra_front_right', ULTRA_MAX_MM),
        )
        min_left = min(
            sensors.get('laser_left_front', LASER_MAX_MM),
            sensors.get('laser_left_back', LASER_MAX_MM),
        )
        min_right = min(
            sensors.get('laser_right_front', LASER_MAX_MM),
            sensors.get('laser_right_back', LASER_MAX_MM),
        )

        if self._obstacle_avoid_mode:
            self._avoid_timer -= 1
            if self._avoid_timer <= 0:
                self._obstacle_avoid_mode = False
            return 'q' if self._avoid_direction > 0 else 'e'

        if min_front < OBSTACLE_CLEARANCE * 1000 or self._stuck_counter > 5:
            self._obstacle_avoid_mode = True
            self._avoid_timer = random.randint(5, 12)
            if dist_to_goal < 1.0:
                self._avoid_direction = 1 if heading_error >= 0 else -1
            elif min_left > min_right:
                self._avoid_direction = 1
            else:
                self._avoid_direction = -1
            return 'q' if self._avoid_direction > 0 else 'e'

        if abs(heading_error) > 5:
            return 't' if heading_error > 0 else 'y'

        if min_front < 0.5 * 1000:
            return 'f'

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
            'navigation_goal': self.navigation_goal,
            'goal_reached': self.goal_reached,
        }


_inst: Optional[SimulationEngine] = None


def get_engine() -> SimulationEngine:
    global _inst
    if _inst is None:
        _inst = SimulationEngine()
    return _inst


def reset_engine(obstacles: Optional[List[Dict]] = None):
    global _inst
    _inst = SimulationEngine()
    if obstacles:
        _inst.reset(obstacles)
