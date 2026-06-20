"""
Camera Service — power-aware USB camera capture.

Camera is OFF by default. Only activates when AI decision engine requests it.
Supports OpenCV USB cameras and saves frames as JPEG/base64.
"""

import time
import base64
import threading
import logging
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)


class CameraState(Enum):
    OFF = "off"
    WARMING_UP = "warming_up"
    ACTIVE = "active"
    ERROR = "error"


class CameraService:
    """
    Power-aware camera service. Camera stays OFF until explicitly activated.
    Thread-safe frame capture with configurable resolution and FPS.
    """

    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.state = CameraState.OFF
        self._cap = None
        self._lock = threading.Lock()
        self._last_frame = None
        self._last_frame_time = 0.0
        self._frame_count = 0
        self._error_count = 0
        self._max_errors = 5

    def activate(self) -> bool:
        """Activate camera. Returns True if successful."""
        with self._lock:
            if self.state == CameraState.ACTIVE:
                return True

            self.state = CameraState.WARMING_UP
            logger.info(f"Activating camera {self.camera_id}")

            try:
                import cv2
                self._cap = cv2.VideoCapture(self.camera_id)
                if not self._cap.isOpened():
                    logger.error(f"Failed to open camera {self.camera_id}")
                    self.state = CameraState.ERROR
                    return False

                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                # Warm up — read and discard first frames
                for _ in range(3):
                    self._cap.read()
                    time.sleep(0.05)

                self.state = CameraState.ACTIVE
                self._error_count = 0
                logger.info(f"Camera {self.camera_id} activated ({self.width}x{self.height})")
                return True

            except ImportError:
                logger.error("OpenCV not installed (opencv-python-headless)")
                self.state = CameraState.ERROR
                return False
            except Exception as e:
                logger.error(f"Camera activation failed: {e}")
                self.state = CameraState.ERROR
                return False

    def deactivate(self):
        """Deactivate camera to save power."""
        with self._lock:
            if self.state == CameraState.OFF:
                return

            if self._cap:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

            self.state = CameraState.OFF
            self._last_frame = None
            logger.info(f"Camera {self.camera_id} deactivated")

    def capture_frame(self) -> Optional[bytes]:
        """Capture a single JPEG frame. Returns None if camera is off or error."""
        if self.state != CameraState.ACTIVE:
            return None

        with self._lock:
            if not self._cap or not self._cap.isOpened():
                self.state = CameraState.ERROR
                return None

            try:
                import cv2
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    self._error_count += 1
                    if self._error_count >= self._max_errors:
                        logger.error("Too many capture errors, deactivating camera")
                        self.state = CameraState.ERROR
                    return None

                # Encode as JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg_bytes = buffer.tobytes()

                self._last_frame = jpeg_bytes
                self._last_frame_time = time.time()
                self._frame_count += 1
                self._error_count = 0

                return jpeg_bytes

            except Exception as e:
                self._error_count += 1
                logger.error(f"Frame capture error: {e}")
                if self._error_count >= self._max_errors:
                    self.state = CameraState.ERROR
                return None

    def capture_base64(self) -> Optional[str]:
        """Capture frame and return as base64 string."""
        jpeg = self.capture_frame()
        if jpeg is None:
            return None
        return base64.b64encode(jpeg).decode('utf-8')

    def get_last_frame_base64(self) -> Optional[str]:
        """Get last captured frame as base64 (no new capture)."""
        if self._last_frame is None:
            return None
        return base64.b64encode(self._last_frame).decode('utf-8')

    def get_state(self) -> dict:
        """Get current camera state."""
        return {
            'state': self.state.value,
            'camera_id': self.camera_id,
            'resolution': f'{self.width}x{self.height}',
            'frame_count': self._frame_count,
            'error_count': self._error_count,
            'last_frame_time': self._last_frame_time,
        }

    def cleanup(self):
        """Release all resources."""
        self.deactivate()
