import math
import random
import time
import numpy as np
from typing import List, Dict, Optional, Tuple

COMMANDS = ['f', 'b', 'q', 'e', 'z', 'x', 't', 'y', 's']

CMD_VEL = {
    'f': (0, 1), 'b': (0, -1),
    'q': (0.7, 0.7), 'e': (-0.7, 0.7),
    'z': (0.7, -0.7), 'x': (-0.7, -0.7),
    't': (0, 0), 'y': (0, 0), 's': (0, 0),
}
CMD_OMEGA = {'t': 60, 'y': -60}

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
SPEED = 0.15


class BackendSimulation:
    def __init__(self):
        self.reset()

    def reset(self, obstacles: Optional[List[Dict]] = None):
        self.x = 0.0
        self.y = -2.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.total_distance = 0.0
        self.collisions = 0
        self.steps = 0

        self.obstacles = obstacles or [
            {'x': 0.8, 'y': 0.0, 'w': 0.4, 'h': 0.4},
            {'x': -0.6, 'y': 1.2, 'w': 0.4, 'h': 0.4},
            {'x': 0.5, 'y': 2.5, 'w': 0.4, 'h': 0.4},
            {'x': -0.8, 'y': 3.5, 'w': 0.4, 'h': 0.4},
        ]

        self.bounds = {'x_min': -2.0, 'x_max': 2.0, 'y_min': -2.5, 'y_max': 5.0}
        self.last_command = 's'
        self._time = 0.0

    def step(self, command: str) -> Dict:
        self.last_command = command
        self.steps += 1
        self._time += 1.0 / 30.0

        vx, vy = CMD_VEL.get(command, (0, 0))
        omega = CMD_OMEGA.get(command, 0)

        heading_rad = math.radians(self.heading)
        dx = (vx * math.cos(heading_rad) - vy * math.sin(heading_rad)) * SPEED
        dy = (vx * math.sin(heading_rad) + vy * math.cos(heading_rad)) * SPEED

        new_x = self.x + dx
        new_y = self.y + dy
        new_heading = (self.heading + omega * (1.0 / 30.0)) % 360

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
        self.vx = vx * SPEED
        self.vy = vy * SPEED
        self.omega = omega

        return self.get_sensors()

    def _check_collision(self, x: float, y: float) -> bool:
        robot_r = 0.15
        for obs in self.obstacles:
            ox, oy = obs['x'], obs['y']
            ow = obs.get('w', 0.3)
            oh = obs.get('h', 0.3)
            closest_x = max(ox - ow / 2, min(x, ox + ow / 2))
            closest_y = max(oy - oh / 2, min(y, oy + oh / 2))
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

        front_dist = min(sensors.get('ultra_front_left', ULTRA_MAX),
                         sensors.get('ultra_front_right', ULTRA_MAX))
        if front_dist < 200:
            for u in ['ultra_front_left', 'ultra_front_right']:
                if random.random() < 0.3:
                    sensors[u] += random.uniform(100, 300)

        sensors['line_left'] = 0
        sensors['line_center'] = 1
        sensors['line_right'] = 0
        sensors['imu_heading'] = self.heading + random.gauss(0, 2)
        sensors['imu_pitch'] = math.sin(self._time * 0.1) * 3.0
        sensors['imu_roll'] = math.cos(self._time * 0.1) * 5.0

        return sensors

    def _raycast(self, angle_deg: float) -> float:
        angle_rad = math.radians(angle_deg)
        max_dist = ULTRA_MAX / 1000.0
        step = 0.05
        for d in np.arange(0, max_dist, step):
            px = self.x + d * math.cos(angle_rad)
            py = self.y + d * math.sin(angle_rad)
            for obs in self.obstacles:
                ox, oy = obs['x'], obs['y']
                ow = obs.get('w', 0.3) / 2
                oh = obs.get('h', 0.3) / 2
                if (ox - ow <= px <= ox + ow) and (oy - oh <= py <= oy + oh):
                    return d * 1000.0
        return max_dist * 1000.0

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
