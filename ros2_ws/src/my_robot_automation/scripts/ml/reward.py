from typing import Optional, List


CRITICAL_MM = 200
WARNING_MM = 500


class RewardCalculator:
    def compute(self, sensor_before: dict, sensor_after: dict,
                command: str, yolo_detections: Optional[List[dict]] = None,
                position_delta: float = 0.0) -> float:
        reward = 0.0

        reward += self._collision_penalty(sensor_after)
        reward += self._progress_reward(position_delta)
        reward += self._stagnation_penalty(sensor_before, sensor_after)
        reward += self._obstacle_avoidance(sensor_before, sensor_after, command)
        reward += self._object_interaction(yolo_detections, command)
        reward += self._line_following(sensor_after)

        return max(-2.0, min(2.0, reward))

    def _collision_penalty(self, sensors: dict) -> float:
        laser_keys = ['laser_left_front', 'laser_left_back',
                      'laser_right_front', 'laser_right_back',
                      'laser_back_left', 'laser_back_right']
        ultra_keys = ['ultra_front_left', 'ultra_front_right']

        for key in laser_keys:
            val = sensors.get(key, 1500)
            if val is not None and val < CRITICAL_MM:
                return -2.0
            if val is not None and val < WARNING_MM:
                return -1.0
        for key in ultra_keys:
            val = sensors.get(key, 4000)
            if val is not None and val < CRITICAL_MM:
                return -1.5

        return 0.0

    def _progress_reward(self, position_delta: float) -> float:
        if position_delta > 10:
            return min(1.0, position_delta / 100.0)
        return 0.0

    def _stagnation_penalty(self, before: dict, after: dict) -> float:
        return 0.0

    def _obstacle_avoidance(self, before: dict, after: dict, command: str) -> float:
        turning = command in ('t', 'y', 'q', 'e', 'z', 'x')
        front_clear_before = min(
            before.get('laser_left_front', 1500),
            before.get('laser_right_front', 1500),
            before.get('ultra_front_left', 4000),
            before.get('ultra_front_right', 4000),
        )
        if turning and front_clear_before < WARNING_MM:
            return 0.5
        return 0.0

    def _object_interaction(self, detections: Optional[List[dict]],
                            command: str) -> float:
        if not detections:
            return 0.0

        has_target = any(
            d.get('class') in ('person', 'cat', 'dog', 'bird',
                               'chair', 'table', 'bench')
            and d.get('confidence', 0) > 0.5
            for d in detections
        )
        if not has_target:
            return 0.0

        approach_or_interact = command in ('f', 'q', 'e')
        stop_or_avoid = command in ('t', 'y', 's')
        if has_target and stop_or_avoid:
            return 0.5
        if has_target and approach_or_interact:
            return 0.3
        return 0.0

    def _line_following(self, sensors: dict) -> float:
        line = sensors.get('line_center', 0)
        return 0.3 if line else 0.0
