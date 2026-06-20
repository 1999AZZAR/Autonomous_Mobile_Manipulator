"""
Waypoint Memory — save, load, and replay waypoints for IMU-based navigation.

Training phase: record waypoints while driving manually.
Competition phase: replay saved paths using IMU dead reckoning.
No camera or AI needed for replay — pure IMU + motor control.
"""

import time
import math
import threading
import logging
from typing import Optional, List

from prisma import Json
from automation_engine import _run_async

logger = logging.getLogger(__name__)


class WaypointMemory:
    """
    Manages saved paths and waypoints. Handles recording during training
    and replay during autonomous execution using IMU navigation.
    """

    def __init__(self, mega_interface=None, sensor_manager=None):
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.db = None  # Prisma client, set in initialize()

        # Recording state
        self.recording = False
        self.current_path_id = None
        self.recorded_waypoints = []
        self.last_record_time = 0.0
        self.record_interval = 0.5  # seconds between auto-recorded waypoints

        # Replay state
        self.replaying = False
        self.current_replay_path_id = None
        self.replay_index = 0
        self.replay_thread = None

        # Navigation state (set by main.py's IMU tracking)
        self.get_position = None  # callable returning [x, y, z]
        self.get_orientation = None  # callable returning [roll, pitch, yaw]

    def initialize(self, db_client):
        """Set Prisma client."""
        self.db = db_client

    def start_recording(self, name: str, description: str = None) -> dict:
        """Start recording a new path. Creates SavedPath and begins capturing waypoints."""
        if self.recording:
            return {'success': False, 'error': 'Already recording'}

        try:
            path = _run_async(self.db.savedpath.create(
                data={'name': name, 'description': description}
            ))
            self.current_path_id = path.id
            self.recorded_waypoints = []
            self.recording = True
            self.last_record_time = 0.0
            logger.info(f"Started recording path '{name}' (id={path.id})")
            return {'success': True, 'path_id': path.id, 'name': name}

        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            return {'success': False, 'error': str(e)}

    def record_waypoint(self, actions: dict = None) -> dict:
        """Record a single waypoint at current position. Call manually or via auto-record."""
        if not self.recording:
            return {'success': False, 'error': 'Not recording'}

        now = time.time()
        if now - self.last_record_time < self.record_interval:
            return {'success': False, 'error': 'Too soon since last record'}

        position = self.get_position() if self.get_position else [0.0, 0.0, 0.0]
        orientation = self.get_orientation() if self.get_orientation else [0.0, 0.0, 0.0]

        # Get sensor snapshot
        sensor_snapshot = None
        if self.sensors:
            try:
                sensor_snapshot = self.sensors.read_all_sensors()
            except Exception:
                pass

        order = len(self.recorded_waypoints)
        waypoint_data = {
            'x': position[0],
            'y': position[1],
            'heading': orientation[2],  # yaw
        }
        if actions:
            waypoint_data['actions'] = actions
        if sensor_snapshot:
            waypoint_data['sensorSnapshot'] = sensor_snapshot

        try:
            create_data = {
                'pathId': self.current_path_id,
                'order': order,
                'x': waypoint_data['x'],
                'y': waypoint_data['y'],
                'heading': waypoint_data['heading'],
            }
            if 'actions' in waypoint_data:
                create_data['actions'] = Json(waypoint_data['actions'])
            if 'sensorSnapshot' in waypoint_data:
                create_data['sensorSnapshot'] = Json(waypoint_data['sensorSnapshot'])

            wp = _run_async(self.db.waypoint.create(data=create_data))
            self.recorded_waypoints.append(waypoint_data)
            self.last_record_time = now
            logger.debug(f"Recorded waypoint #{order}: ({waypoint_data['x']:.2f}, {waypoint_data['y']:.2f}), heading={waypoint_data['heading']:.1f}")
            return {'success': True, 'waypoint_id': wp.id, 'order': order}

        except Exception as e:
            logger.error(f"Failed to record waypoint: {e}")
            return {'success': False, 'error': str(e)}

    def stop_recording(self) -> dict:
        """Stop recording and return summary."""
        if not self.recording:
            return {'success': False, 'error': 'Not recording'}

        self.recording = False
        count = len(self.recorded_waypoints)
        path_id = self.current_path_id
        self.current_path_id = None
        self.recorded_waypoints = []

        logger.info(f"Stopped recording: {count} waypoints saved (path_id={path_id})")
        return {'success': True, 'path_id': path_id, 'waypoint_count': count}

    def list_paths(self) -> List[dict]:
        """List all saved paths."""
        try:
            paths = _run_async(self.db.savedpath.find_many(order={'createdAt': 'desc'}))
            result = []
            for p in paths:
                wp_count = _run_async(self.db.waypoint.count(where={'pathId': p.id}))
                result.append({
                    'id': p.id,
                    'name': p.name,
                    'description': p.description,
                    'waypoint_count': wp_count,
                    'created_at': p.createdAt.isoformat() if p.createdAt else None,
                })
            return result
        except Exception as e:
            logger.error(f"Failed to list paths: {e}")
            return []

    def get_path(self, path_id: int) -> Optional[dict]:
        """Get a path with all its waypoints."""
        try:
            path = _run_async(self.db.savedpath.find_unique(where={'id': path_id}))
            if not path:
                return None

            waypoints = _run_async(self.db.waypoint.find_many(
                where={'pathId': path_id},
                order={'order': 'asc'}
            ))

            return {
                'id': path.id,
                'name': path.name,
                'description': path.description,
                'waypoints': [
                    {
                        'id': w.id,
                        'order': w.order,
                        'x': w.x,
                        'y': w.y,
                        'heading': w.heading,
                        'actions': w.actions,
                        'sensorSnapshot': w.sensorSnapshot,
                    } for w in waypoints
                ],
                'created_at': path.createdAt.isoformat() if path.createdAt else None,
            }
        except Exception as e:
            logger.error(f"Failed to get path {path_id}: {e}")
            return None

    def delete_path(self, path_id: int) -> dict:
        """Delete a path and all its waypoints."""
        try:
            _run_async(self.db.waypoint.delete_many(where={'pathId': path_id}))
            _run_async(self.db.savedpath.delete(where={'id': path_id}))
            logger.info(f"Deleted path {path_id}")
            return {'success': True}
        except Exception as e:
            logger.error(f"Failed to delete path {path_id}: {e}")
            return {'success': False, 'error': str(e)}

    def start_replay(self, path_id: int) -> dict:
        """Start replaying a saved path using IMU navigation."""
        if self.replaying:
            return {'success': False, 'error': 'Already replaying'}

        path_data = self.get_path(path_id)
        if not path_data or not path_data['waypoints']:
            return {'success': False, 'error': 'Path not found or empty'}

        self.replaying = True
        self.current_replay_path_id = path_id
        self.replay_index = 0

        self.replay_thread = threading.Thread(
            target=self._replay_loop,
            args=(path_data['waypoints'],),
            daemon=True,
            name="waypoint-replay"
        )
        self.replay_thread.start()
        logger.info(f"Started replaying path '{path_data['name']}' ({len(path_data['waypoints'])} waypoints)")
        return {'success': True, 'path_id': path_id, 'waypoint_count': len(path_data['waypoints'])}

    def stop_replay(self):
        """Stop the current replay."""
        if not self.replaying:
            return
        self.replaying = False
        self.current_replay_path_id = None
        self.replay_index = 0
        logger.info("Replay stopped")

    def _replay_loop(self, waypoints: List[dict]):
        """Navigate through waypoints using IMU dead reckoning."""
        for i, wp in enumerate(waypoints):
            if not self.replaying:
                break

            target_x = wp['x']
            target_y = wp['y']
            target_heading = wp['heading']

            logger.info(f"Replay waypoint #{i}: ({target_x:.2f}, {target_y:.2f}), heading={target_heading:.1f}")

            # Navigate to waypoint
            self._navigate_to_waypoint(target_x, target_y, target_heading)

            # Execute actions at waypoint if any
            if wp.get('actions') and self.mega:
                self._execute_waypoint_actions(wp['actions'])

            self.replay_index = i + 1
            time.sleep(0.1)

        self.replaying = False
        logger.info("Replay completed")

    def _navigate_to_waypoint(self, target_x: float, target_y: float, target_heading: float):
        """Navigate to a single waypoint using IMU dead reckoning."""
        if not self.get_position or not self.mega:
            logger.warning("Cannot navigate: no position callback or mega interface")
            return

        max_attempts = 3
        position_tolerance = 0.15  # meters
        heading_tolerance = 10.0  # degrees

        for attempt in range(max_attempts):
            if not self.replaying:
                return

            pos = self.get_position()
            orient = self.get_orientation()
            current_x, current_y = pos[0], pos[1]
            current_heading = orient[2]

            # Calculate distance and angle to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < position_tolerance:
                logger.debug(f"Reached waypoint (distance={distance:.3f}m)")
                return

            # Calculate desired heading
            desired_heading = math.degrees(math.atan2(dy, dx))
            heading_error = desired_heading - current_heading

            # Normalize to -180 to 180
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360

            # Turn if heading is off
            if abs(heading_error) > heading_tolerance:
                direction = 't' if heading_error > 0 else 'y'  # t=turn left CCW, y=turn right CW
                self.mega.send_command(direction)
                time.sleep(abs(heading_error) / 90.0)  # Rough time estimate
                self.mega.send_command('s')
                time.sleep(0.2)
                continue

            # Move forward
            self.mega.send_command('f')
            # Move for estimated time based on distance
            move_time = min(distance / 0.2, 2.0)  # ~0.2 m/s, max 2s
            time.sleep(move_time)
            self.mega.send_command('s')
            time.sleep(0.2)

        logger.warning(f"Waypoint navigation: max attempts reached for ({target_x:.2f}, {target_y:.2f})")

    def _execute_waypoint_actions(self, actions):
        """Execute actions stored at a waypoint."""
        if not isinstance(actions, dict) or not self.mega:
            return

        action_type = actions.get('type')
        action_value = actions.get('value')

        if action_type == 'gripper':
            cmd = 'no' if action_value == 'open' else 'nc'
            self.mega.send_command(cmd)
        elif action_type == 'tilt':
            if action_value in ('up', 'down', 'center'):
                cmd = {'up': 'tu', 'down': 'td', 'center': 'tt'}[action_value]
                self.mega.send_command(cmd)
            elif action_value and action_value.startswith('angle_'):
                angle = action_value.replace('angle_', '')
                self.mega.send_command(f'ta{angle}')
        elif action_type == 'move':
            direction_map = {'forward': 'f', 'backward': 'b', 'forward-left': 'q', 'forward-right': 'e', 'backward-left': 'z', 'backward-right': 'x'}
            cmd = direction_map.get(action_value)
            if cmd:
                self.mega.send_command(cmd)
                time.sleep(1.0)
                self.mega.send_command('s')

    def cleanup(self):
        """Stop recording/replaying."""
        self.recording = False
        self.replaying = False
