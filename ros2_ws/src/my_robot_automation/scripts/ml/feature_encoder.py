import json
import numpy as np
from collections import deque
from typing import Optional, List


class FeatureEncoder:
    SENSOR_KEYS = [
        'laser_left_front', 'laser_left_back', 'laser_right_front',
        'laser_right_back', 'laser_back_left', 'laser_back_right',
        'ultra_front_left', 'ultra_front_right',
        'line_left', 'line_center', 'line_right',
        'imu_heading', 'imu_pitch', 'imu_roll',
    ]
    SENSOR_DIM = len(SENSOR_KEYS)  # 14

    YOLO_CLASSES = ['person', 'animal', 'obstacle', 'object_of_interest']
    YOLO_CLASS_DIM = len(YOLO_CLASSES)  # 4
    MAX_YOLO_OBJECTS = 4
    YOLO_PER_OBJECT = YOLO_CLASS_DIM + 2  # onehot(4) + conf + dist_estimate
    YOLO_DIM = MAX_YOLO_OBJECTS * YOLO_PER_OBJECT  # 24

    STATE_DIM = 3  # vx, vy, omega

    FRAME_DIM = SENSOR_DIM + YOLO_DIM + STATE_DIM  # 41

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self._reset_normalization()

    def _reset_normalization(self):
        self.norm_params = {
            'laser_max': 1500.0,
            'ultra_max': 4000.0,
            'imu_max': 180.0,
            'vel_max': 500.0,
            'yolo_conf_max': 1.0,
            'yolo_dist_max': 4000.0,
        }

    def encode(self, sensor_data: dict, yolo_detections: List[dict],
               last_command: Optional[str] = None) -> np.ndarray:
        frame = np.concatenate([
            self._encode_sensors(sensor_data),
            self._encode_yolo(yolo_detections),
            self._encode_state(last_command),
        ])
        self.buffer.append(frame)
        return self._build_window()

    def _encode_sensors(self, data: dict) -> np.ndarray:
        raw = np.zeros(self.SENSOR_DIM, dtype=np.float32)
        for i, key in enumerate(self.SENSOR_KEYS):
            val = data.get(key, 0)
            if isinstance(val, bool):
                val = 1.0 if val else 0.0
            raw[i] = float(val) if val is not None else 0.0

        norm = raw.copy()
        norm[0:6] /= self.norm_params['laser_max']
        norm[6:8] /= self.norm_params['ultra_max']
        norm[8:11] = np.clip(norm[8:11], 0, 1)
        norm[11] /= self.norm_params['imu_max']
        norm[12] /= self.norm_params['imu_max']
        norm[13] /= self.norm_params['imu_max']
        return np.clip(norm, 0, 1)

    def _class_to_onehot(self, class_name: str) -> np.ndarray:
        vec = np.zeros(self.YOLO_CLASS_DIM, dtype=np.float32)
        if class_name in ('person', 'cat', 'dog', 'bird'):
            vec[0] = 1.0
        elif class_name in ('chair', 'table', 'bench', 'couch', 'book',
                            'bottle', 'cup', 'vase', 'potted plant'):
            vec[2] = 1.0
        elif class_name in ('car', 'truck', 'bicycle', 'motorcycle'):
            vec[2] = 1.0
        else:
            vec[3] = 1.0
        return vec

    def _estimate_distance(self, bbox: List[float], img_w: int = 640,
                           img_h: int = 480) -> float:
        _, _, w, h = bbox
        bbox_area = (w / img_w) * (h / img_h)
        if bbox_area < 0.01:
            return self.norm_params['yolo_dist_max']
        return max(100.0, 4000.0 * (0.01 / max(bbox_area, 0.001)))

    def _encode_yolo(self, detections: List[dict]) -> np.ndarray:
        flat = np.zeros(self.YOLO_DIM, dtype=np.float32)
        for idx, det in enumerate(detections[:self.MAX_YOLO_OBJECTS]):
            base = idx * self.YOLO_PER_OBJECT
            onehot = self._class_to_onehot(det.get('class', ''))
            conf = float(det.get('confidence', 0))
            bbox = det.get('bbox', [0, 0, 0, 0])
            if isinstance(bbox, list) and len(bbox) == 4:
                dist = self._estimate_distance(bbox)
            else:
                dist = self.norm_params['yolo_dist_max']
            flat[base:base+4] = onehot
            flat[base+4] = conf
            flat[base+5] = dist / self.norm_params['yolo_dist_max']
        return np.clip(flat, 0, 1)

    def _encode_state(self, command: Optional[str]) -> np.ndarray:
        cmd_vel = {
            'f': (1, 0, 0), 'b': (-1, 0, 0),
            'q': (0.7, 0.7, 0), 'e': (0.7, -0.7, 0),
            'z': (-0.7, 0.7, 0), 'x': (-0.7, -0.7, 0),
            't': (0, 0, 1), 'y': (0, 0, -1),
            's': (0, 0, 0),
        }
        vx, vy, omega = cmd_vel.get(command, (0, 0, 0))
        return np.array([
            vx / self.norm_params['vel_max'],
            vy / self.norm_params['vel_max'],
            omega,
        ], dtype=np.float32)

    def _build_window(self) -> np.ndarray:
        frames = list(self.buffer)
        if len(frames) < self.window_size:
            padding = [np.zeros(self.FRAME_DIM, dtype=np.float32)
                       for _ in range(self.window_size - len(frames))]
            frames = padding + frames
        return np.concatenate(frames).astype(np.float32)

    def save_params(self, path: str):
        with open(path, 'w') as f:
            json.dump(self.norm_params, f)

    def load_params(self, path: str):
        with open(path) as f:
            self.norm_params.update(json.load(f))

    def reset(self):
        self.buffer.clear()
