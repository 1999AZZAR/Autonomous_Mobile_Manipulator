"""
REST API backend for Autonomous Mobile Manipulator control.
Serves JSON only — frontend is a separate TypeScript app.
"""

import time
import threading
import logging
import math
import json
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG, DEFAULT_SIMULATION_MODE
from mega_interface import MegaInterface
from sensor_manager import SensorManager
from gpio_controller import GPIOController

# ROS2 interface is optional (only available when ROS2 is installed)
try:
    from ros2_interface import ROS2Interface
    ROS2_AVAILABLE = True
except ImportError:
    ROS2Interface = None
    ROS2_AVAILABLE = False

logger = logging.getLogger(__name__)


class FlaskApp:
    """Main Flask application for robot control"""

    def __init__(self, mega_interface=None, sensor_manager=None, ros2_interface=None, simulation_mode=False, main_app=None):
        logger.info("Initializing Flask application...")
        self.app = Flask(__name__)
        CORS(self.app, resources={r"/api/*": {"origins": "*"}})
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.ros2 = ros2_interface
        self.simulation_mode = simulation_mode
        self.main_app = main_app
        self.safety_bypass = False
        self._ws_clients: list = []
        self._ws_lock = threading.Lock()

        logger.info(f"Mega interface: {type(mega_interface)}")
        logger.info(f"Sensor manager: {type(sensor_manager)}")
        logger.info(f"ROS2 interface: {type(ros2_interface)}")

        try:
            self._setup_routes()
            logger.info("Routes setup completed")
        except Exception as e:
            logger.error(f"Failed to setup routes: {e}")
            raise

        logger.info("Flask application initialized")

    def _setup_routes(self):
        """Setup Flask routes — pure JSON API, no HTML rendering"""

        @self.app.route('/')
        def api_root():
            return jsonify({
                'service': 'amm-control-api',
                'version': '1.0.0',
                'docs': '/api/status',
            })

        @self.app.route('/health')
        def health():
            return jsonify({
                'status': 'healthy',
                'timestamp': time.time(),
                'simulation_mode': self.simulation_mode,
            })

        # Status and diagnostics
        @self.app.route('/api/robot/sensors')
        def get_sensors():
            return self._get_sensors()

        @self.app.route('/api/robot/sensors/diagnostics')
        def get_sensor_diagnostics():
            return self._get_sensor_diagnostics()

        # Movement control
        @self.app.route('/api/robot/speed', methods=['POST'])
        def set_speed():
            return self._set_speed()

        @self.app.route('/api/robot/turbo', methods=['POST'])
        def toggle_turbo():
            return self._toggle_turbo()

        @self.app.route('/api/robot/move', methods=['POST'])
        def move_robot():
            return self._move_robot()

        @self.app.route('/api/robot/turn', methods=['POST'])
        def turn_robot():
            return self._turn_robot()

        @self.app.route('/api/robot/stop', methods=['POST'])
        def stop_robot():
            return self._stop_robot()

        # Wheel control
        @self.app.route('/api/robot/wheels/<int:wheel_id>', methods=['POST'])
        def control_wheel(wheel_id):
            return self._control_wheel(wheel_id)

        @self.app.route('/api/robot/wheels/stop', methods=['POST'])
        def stop_all_wheels():
            return self._stop_all_wheels()

        # Manipulation
        @self.app.route('/api/robot/picker/gripper', methods=['POST'])
        def control_gripper():
            return self._control_gripper()

        @self.app.route('/api/robot/picker/gripper_tilt', methods=['POST'])
        def set_gripper_tilt():
            return self._set_gripper_tilt()

        # Serial communication
        @self.app.route('/api/serial/send', methods=['POST'])
        def send_serial_command():
            return self._send_serial_command()

        # Emergency controls
        @self.app.route('/api/robot/emergency-stop', methods=['POST'])
        def emergency_stop():
            return self._emergency_stop()

        # Path Planning
        @self.app.route('/api/robot/sequences/execute', methods=['POST'])
        def execute_sequence():
            return self._execute_sequence()

        @self.app.route('/api/robot/sequences/save', methods=['POST'])
        def save_sequence():
            return self._save_sequence()

        @self.app.route('/api/robot/sequences/load/<name>', methods=['GET'])
        def load_sequence(name):
            return self._load_sequence(name)

        @self.app.route('/api/robot/sequences/list', methods=['GET'])
        def list_sequences():
            return self._list_sequences()

        @self.app.route('/api/robot/waypoints/navigate', methods=['POST'])
        def navigate_waypoints():
            return self._navigate_waypoints()

        @self.app.route('/api/robot/safety/bypass', methods=['POST'])
        def toggle_safety_bypass():
            return self._toggle_safety_bypass()

        @self.app.route('/api/debug/sensor-test', methods=['GET'])
        def sensor_test():
            """Debug endpoint to test sensor functionality"""
            if not self.sensors:
                return jsonify({'error': 'No sensor manager available'}), 500

            sensor_data = self.sensors.read_all_sensors()
            return jsonify({
                'timestamp': time.time(),
                'sensor_data': sensor_data,
                'mega_connected': self.mega.mega_connected if self.mega else False,
                'safety_bypass': self.safety_bypass
            })

        @self.app.route('/api/robot/position', methods=['GET'])
        def get_current_position():
            return self._get_current_position()

        @self.app.route('/api/map/canvas', methods=['GET'])
        def get_map_canvas():
            return self._get_map_canvas(), 200, {'Content-Type': 'text/html'}

        @self.app.route('/api/serial/monitor', methods=['GET'])
        def get_serial_monitor_data():
            return self._get_serial_monitor_data()

        @self.app.route('/api/mega/status', methods=['GET'])
        def get_mega_status():
            return self._get_mega_status()

        @self.app.route('/api/mega/reconnect', methods=['POST'])
        def reconnect_mega():
            return self._reconnect_mega()

        # --- Endpoints expected by TypeScript frontend ---

        @self.app.route('/api/sensors')
        def get_sensors_flat():
            """Flat sensor readings for the TS dashboard"""
            if self.sensors:
                data = self.sensors.read_all_sensors()
                data['mega_connected'] = self.mega.mega_connected if self.mega else False
                return jsonify(data)
            return jsonify({'error': 'sensors unavailable'}), 503

        @self.app.route('/api/status')
        def get_status_flat():
            """System status for the TS frontend (flat format)"""
            mega_connected = self.mega.mega_connected if self.mega else False
            ros2_available = self.ros2 is not None and ROS2_AVAILABLE
            return jsonify({
                'mega_connected': mega_connected,
                'ros2_available': ros2_available,
                'simulation_mode': self.simulation_mode,
                'uptime': time.time(),
            })

        @self.app.route('/api/command', methods=['POST'])
        def send_command():
            """Single-char command from frontend D-Pad / keyboard"""
            data = request.get_json(silent=True) or {}
            cmd = data.get('command', '')
            if not cmd:
                return jsonify({'error': 'command required'}), 400
            if self.mega:
                self.mega.send_command_to_mega(cmd)
                return jsonify({'ok': True, 'command': cmd})
            return jsonify({'error': 'mega not connected'}), 503

        @self.app.route('/api/feeds')
        def get_all_feeds():
            """All sensor feed values"""
            if self.sensors:
                return jsonify(self.sensors.read_all_sensors())
            return jsonify({})

        @self.app.route('/api/feeds/<key>')
        def get_feed_value(key):
            """Single feed value"""
            if self.sensors:
                data = self.sensors.read_all_sensors()
                return jsonify({'key': key, 'value': data.get(key)})
            return jsonify({'key': key, 'value': None})

        # --- WebSocket endpoint (SSE fallback for simple clients) ---

        @self.app.route('/ws/sensors')
        def ws_sensors():
            """Server-Sent Events stream for real-time sensor data.
            The TS frontend connects via WebSocket; this SSE endpoint
            is a fallback for simple HTTP clients."""
            def stream():
                while True:
                    if self.sensors:
                        data = self.sensors.read_all_sensors()
                        yield f"data: {json.dumps(data)}\n\n"
                    time.sleep(0.5)
            return Response(stream(), mimetype='text/event-stream')

        # End of route setup

    def _get_status(self):
        """Get system status"""
        mega_connected = self.mega.mega_connected if self.mega else False
        ros2_services = self.ros2.actuator_clients_created if self.ros2 else False

        return jsonify({
            'success': True,
            'data': {
                'simulation_mode': self.simulation_mode,
                'mega_connected': mega_connected,
                'ros2_services_available': ros2_services,
                'actuators_available': True,  # GPIO always available
                'timestamp': time.time()
            }
        })

    def _get_sensors(self):
        """Get sensor data"""
        if self.sensors:
            sensor_data = self.sensors.read_all_sensors()
            return jsonify({
                'success': True,
                'data': sensor_data,
                'timestamp': time.time()
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Sensor manager not available',
                'timestamp': time.time()
            }), 500

    def _get_sensor_diagnostics(self):
        """Get sensor diagnostics"""
        if self.sensors:
            health = self.sensors.get_sensor_health()
            return jsonify({
                'success': True,
                'data': {
                    'simulation_mode': self.simulation_mode,
                    'sensor_health': health,
                    'adc_config': {
                        'vref': 3.3,
                        'resolution': 1024,
                        'spi_speed': 1350000
                    }
                },
                'timestamp': time.time()
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Sensor manager not available',
                'timestamp': time.time()
            }), 500

    def _set_speed(self):
        """Set robot speed"""
        try:
            data = request.get_json()
            if not data or 'speed' not in data:
                return jsonify({'success': False, 'error': 'Speed value required', 'timestamp': time.time()}), 400

            speed_percent = int(data['speed'])
            if not (0 <= speed_percent <= 100):
                return jsonify({'success': False, 'error': 'Speed must be 0-100', 'timestamp': time.time()}), 400

            if self.mega:
                success = self.mega.set_speed(speed_percent)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Speed set to {speed_percent}%',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to set speed', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Speed control error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _toggle_turbo(self):
        """Toggle turbo mode"""
        try:
            if self.mega:
                success = self.mega.toggle_turbo()
                if success:
                    return jsonify({
                        'success': True,
                        'message': 'Turbo mode toggled',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to toggle turbo', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Turbo toggle error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _move_robot(self):
        """Move robot in specified direction with sensor-based safety checks"""
        try:
            data = request.get_json()
            direction = data.get('direction')
            speed = data.get('speed', 0.5)
            duration = data.get('duration', 0.0)

            if not direction:
                return jsonify({'success': False, 'error': 'Direction required', 'timestamp': time.time()}), 400

            # Sensor-based safety check before movement
            if not self._check_movement_safety(direction):
                return jsonify({
                    'success': False,
                    'error': f'Obstacle detected in {direction} direction. Movement blocked for safety.',
                    'timestamp': time.time()
                }), 403

            # Try Mega first, then ROS2, then GPIO
            success = False
            if self.mega:
                success = self.mega.move_robot(direction, speed)
            if not success and self.ros2:
                success = self.ros2.call_move_robot(direction, speed, duration)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Robot moved {direction}',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to move robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Move robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _turn_robot(self):
        """Turn robot in specified direction"""
        try:
            data = request.get_json()
            direction = data.get('direction')
            speed = data.get('speed', 0.5)

            if not direction:
                return jsonify({'success': False, 'error': 'Direction required', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.turn_robot(direction, speed)
            if not success and self.ros2:
                # ROS2 turn would use move_robot with turn commands
                pass

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Robot turned {direction}',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to turn robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Turn robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _stop_robot(self):
        """Stop robot movement"""
        try:
            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.stop_robot()
            if not success and self.ros2:
                # ROS2 stop would use move_robot with stop command
                pass

            if success:
                return jsonify({
                    'success': True,
                    'message': 'Robot stopped',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to stop robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Stop robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _control_wheel(self, wheel_id):
        """Control individual wheel"""
        try:
            data = request.get_json()
            speed = data.get('speed', 0.0)

            if self.mega:
                success = self.mega.control_wheel(wheel_id, speed)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Wheel {wheel_id} set to speed {speed}',
                        'wheel': wheel_id,
                        'speed': speed,
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': f'Failed to control wheel {wheel_id}', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Wheel control error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _stop_all_wheels(self):
        """Stop all wheels"""
        try:
            if self.mega:
                success = self.mega.stop_all_wheels()
                if success:
                    return jsonify({
                        'success': True,
                        'action': 'stop_all_wheels',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to stop wheels', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Wheel stop error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _control_gripper(self):
        """Control gripper"""
        try:
            data = request.get_json()
            command = data.get('command')

            if not command:
                return jsonify({'success': False, 'error': 'Command required', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.control_gripper(command)
            if not success and self.ros2:
                success = self.ros2.call_control_gripper(command)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Gripper {command} executed',
                    'command': command,
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': f'Failed to execute gripper command {command}', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Gripper control error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _set_gripper_tilt(self):
        """Set gripper tilt angle"""
        try:
            data = request.get_json()
            angle = data.get('angle', 90.0)

            if not (0 <= angle <= 180):
                return jsonify({'success': False, 'error': 'Angle must be 0-180', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.set_gripper_tilt(angle)
            if not success and self.ros2:
                success = self.ros2.call_set_gripper_tilt(angle)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Gripper tilt set to {angle}°',
                    'angle': angle,
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': f'Failed to set gripper tilt to {angle}°', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Gripper tilt error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _send_serial_command(self):
        """Send direct serial command to Mega"""
        try:
            data = request.get_json()
            if not data or 'command' not in data:
                return jsonify({'success': False, 'error': 'Command required', 'timestamp': time.time()}), 400

            command = data['command'].strip().lower()
            if len(command) < 1:
                return jsonify({'success': False, 'error': 'Command cannot be empty', 'timestamp': time.time()}), 400

            # Validate command - complete set from COMMANDS.md
            valid_commands = [
                # Movement commands (all cardinal, diagonal, rotation, turning)
                'f', 'b', 'l', 'r', 'q', 'e', 'z', 'x', 'c', 'w', 't', 'y', 'a', 'j', 's',
                # Control commands
                'p', 'v', 'o',
                # Speed control (50%-100%)
                '5', '6', '7', '8', '9', '0',
                # Lifter commands
                'u', 'd',
                # Testing commands
                '1', '2', '3', '4', 'g', 'h',
                # Servo commands
                'mu', 'md', 'mc', 'no', 'nc', 'nh',
                # Sensor commands
                'sr', 'ls',
                # Safety commands
                'se', 'sd',
                # Publishing commands
                'spe', 'spd'
            ]

            # Check for angle commands
            is_valid = command in valid_commands
            if not is_valid:
                if command.startswith('ta') and len(command) >= 3:
                    try:
                        angle = int(command[2:])
                        is_valid = (0 <= angle <= 180)
                    except ValueError:
                        is_valid = False
                elif command.startswith('ga') and len(command) >= 3:
                    try:
                        angle = int(command[2:])
                        is_valid = (0 <= angle <= 180)
                    except ValueError:
                        is_valid = False
                elif command.startswith('w') and len(command) >= 2:
                    # Wheel commands: w[0-3][-100-100]
                    try:
                        parts = command.split('w')
                        if len(parts) == 2 and parts[0] == '':
                            wheel_part, speed_part = parts[1].split('[') if '[' in parts[1] else (parts[1], '')
                            if not speed_part:
                                # w[0-3] format
                                wheel_id = int(wheel_part)
                                speed = 50  # default
                            else:
                                # w[0-3][speed] format
                                wheel_id = int(wheel_part)
                                speed = int(speed_part.rstrip(']'))
                            is_valid = (0 <= wheel_id <= 3) and (-100 <= speed <= 100)
                        elif parts[1] == 'stop':
                            is_valid = True  # wstop command
                    except (ValueError, IndexError):
                        is_valid = False

            if not is_valid:
                return jsonify({'success': False, 'error': f'Invalid command. Valid: {", ".join(valid_commands)}, ta<angle>, ga<angle>, w[0-3]<speed>, wstop', 'timestamp': time.time()}), 400

            if self.mega:
                success = self.mega.send_command_to_mega(command)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Command sent: {command}',
                        'command': command,
                        'mega_connected': self.mega.mega_connected,
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': f'Failed to send command: {command}', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Direct serial command error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _emergency_stop(self):
        """Emergency stop all systems"""
        try:
            # Stop all movement
            if self.mega:
                self.mega.stop_all_wheels()
                self.mega.stop_robot()

            return jsonify({
                'success': True,
                'message': 'Emergency stop executed',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Emergency stop error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _check_movement_safety(self, direction):
        """Check if movement in given direction is safe using sensor data"""
        try:
            # Safety bypass for testing/debugging
            if self.safety_bypass:
                logger.info(f"Safety bypass enabled - allowing {direction} movement")
                return True

            if not self.sensors:
                logger.warning("No sensor manager available for safety checks")
                return True  # Allow movement if no sensors available

            sensor_data = self.sensors.read_all_sensors()
            laser_sensors = sensor_data.get('laser_sensors', {})
            ultrasonic_sensors = sensor_data.get('ultrasonic_sensors', {})

            # Safety thresholds (mm)
            OBSTACLE_THRESHOLD = 300  # 30cm safety distance

            # Check sensors based on movement direction
            if direction == 'forward':
                # Check front sensors
                front_left_ir = laser_sensors.get('left_front', float('inf'))
                front_right_ir = laser_sensors.get('right_front', float('inf'))
                front_left_us = ultrasonic_sensors.get('front_left', float('inf'))
                front_right_us = ultrasonic_sensors.get('front_right', float('inf'))

                obstacles = []
                if front_left_ir and front_left_ir < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Left IR: {front_left_ir}mm")
                if front_right_ir and front_right_ir < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Right IR: {front_right_ir}mm")
                if front_left_us and front_left_us < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Left Ultrasonic: {front_left_us}mm")
                if front_right_us and front_right_us < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Right Ultrasonic: {front_right_us}mm")

            elif direction == 'backward':
                # Check back sensors
                back_left = laser_sensors.get('back_left', float('inf'))
                back_right = laser_sensors.get('back_right', float('inf'))

                obstacles = []
                if back_left and back_left < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Back Left IR: {back_left}mm")
                if back_right and back_right < OBSTACLE_THRESHOLD:
                    obstacles.append(f"Back Right IR: {back_right}mm")

            elif direction in ['left', 'right']:
                # Check side sensors for lateral movement
                left_sensors = [laser_sensors.get('left_front', float('inf')),
                               laser_sensors.get('left_back', float('inf'))]
                right_sensors = [laser_sensors.get('right_front', float('inf')),
                                laser_sensors.get('right_back', float('inf'))]

                obstacles = []
                if direction == 'left':
                    for i, sensor_val in enumerate(left_sensors):
                        if sensor_val and sensor_val < OBSTACLE_THRESHOLD:
                            obstacles.append(f"Left sensor {i+1}: {sensor_val}mm")
                else:  # right
                    for i, sensor_val in enumerate(right_sensors):
                        if sensor_val and sensor_val < OBSTACLE_THRESHOLD:
                            obstacles.append(f"Right sensor {i+1}: {sensor_val}mm")

            if obstacles:
                logger.warning(f"Safety check failed for {direction} movement. Obstacles detected: {', '.join(obstacles)}")
                return False

            return True

        except Exception as e:
            logger.error(f"Error in movement safety check: {str(e)}")
            return True  # Allow movement if safety check fails

    def _execute_sequence(self):
        """Execute a movement sequence"""
        try:
            data = request.get_json()
            if not data or 'sequence' not in data:
                return jsonify({'success': False, 'error': 'Sequence required', 'timestamp': time.time()}), 400

            sequence = data['sequence']
            if not isinstance(sequence, list) or len(sequence) == 0:
                return jsonify({'success': False, 'error': 'Valid sequence list required', 'timestamp': time.time()}), 400

            # Execute sequence in background
            threading.Thread(target=self._execute_sequence_async, args=(sequence,), daemon=True).start()

            return jsonify({
                'success': True,
                'message': f'Sequence execution started ({len(sequence)} commands)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Sequence execution error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _execute_sequence_async(self, sequence):
        """Execute sequence asynchronously"""
        try:
            for command in sequence:
                if self.mega:
                    self.mega.send_command_to_mega(command)
                # Add delay between commands
                time.sleep(0.5)

            logger.info(f'Sequence execution completed: {len(sequence)} commands')

        except Exception as e:
            logger.error(f'Async sequence execution error: {str(e)}')

    def _save_sequence(self):
        """Save a movement sequence"""
        try:
            data = request.get_json()
            if not data or 'name' not in data or 'sequence' not in data:
                return jsonify({'success': False, 'error': 'Name and sequence required', 'timestamp': time.time()}), 400

            name = data['name']
            sequence = data['sequence']

            if not isinstance(sequence, list):
                return jsonify({'success': False, 'error': 'Sequence must be a list', 'timestamp': time.time()}), 400

            # In a real implementation, this would save to a database
            # For now, we'll just acknowledge the save
            logger.info(f'Sequence saved: {name} ({len(sequence)} commands)')

            return jsonify({
                'success': True,
                'message': f'Sequence "{name}" saved ({len(sequence)} commands)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Save sequence error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _load_sequence(self, name):
        """Load a movement sequence"""
        try:
            # In a real implementation, this would load from a database
            # For now, return a placeholder
            return jsonify({
                'success': True,
                'sequence': ['f', 's'],  # Placeholder sequence
                'name': name,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Load sequence error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _list_sequences(self):
        """List saved sequences"""
        try:
            # In a real implementation, this would query a database
            # For now, return placeholder data
            sequences = {
                'lawn_mower': {'commands': 10, 'description': 'Systematic coverage pattern'},
                'spiral_search': {'commands': 8, 'description': 'Expanding spiral search'},
                'boundary_follow': {'commands': 12, 'description': 'Follow perimeter boundary'}
            }

            return jsonify({
                'success': True,
                'sequences': sequences,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'List sequences error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _navigate_waypoints(self):
        """Navigate through waypoints"""
        try:
            data = request.get_json()
            if not data or 'waypoints' not in data:
                return jsonify({'success': False, 'error': 'Waypoints required', 'timestamp': time.time()}), 400

            waypoints = data['waypoints']
            if not isinstance(waypoints, list) or len(waypoints) < 2:
                return jsonify({'success': False, 'error': 'At least 2 waypoints required', 'timestamp': time.time()}), 400

            # Execute waypoint navigation in background
            threading.Thread(target=self._navigate_waypoints_async, args=(waypoints,), daemon=True).start()

            return jsonify({
                'success': True,
                'message': f'Waypoint navigation started ({len(waypoints)} waypoints)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Waypoint navigation error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _navigate_waypoints_async(self, waypoints):
        """Navigate waypoints asynchronously using IMU-based dead reckoning with sensor obstacle avoidance"""
        try:
            logger.info(f'Starting sensor-aware IMU-based waypoint navigation with {len(waypoints)} waypoints')

            # Access the main app's IMU position tracking
            main_app = None
            if hasattr(self, 'main_app'):
                main_app = self.main_app

            # Reset position to origin for waypoint navigation
            if main_app:
                main_app.reset_position()
                logger.info("Position reset to origin (0,0,0) for waypoint navigation")
            else:
                logger.warning("No main app reference - position tracking may not work correctly")

            for i, waypoint in enumerate(waypoints):
                # Handle waypoint data structure (could be dict with action or tuple)
                if isinstance(waypoint, dict):
                    target_x, target_y, target_z = waypoint['x'], waypoint['y'], waypoint.get('z', 0)
                    action = waypoint.get('action')
                else:
                    target_x, target_y, target_z = waypoint[:3]
                    action = None

                if i == 0:
                    logger.info('Starting from first waypoint (origin)')
                    # Execute action at starting waypoint if specified
                    if action:
                        self._execute_waypoint_action(action)
                    continue  # Skip navigation for first waypoint

                logger.info(f'Navigating to waypoint {i + 1}: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})')

                # Navigate to waypoint using IMU-based position tracking with sensor safety
                success = self._navigate_to_coordinate_imu_sensor_safe(target_x, target_y, main_app)

                if success:
                    logger.info(f'Successfully reached waypoint {i + 1}')
                    # Execute action at waypoint if specified
                    if action:
                        self._execute_waypoint_action(action)
                else:
                    logger.warning(f'Failed to reach waypoint {i + 1} within timeout or due to obstacles, continuing to next waypoint')

                # Brief pause at waypoint
                time.sleep(2)

            logger.info(f'IMU-based waypoint navigation completed: {len(waypoints)} waypoints')

        except Exception as e:
            logger.error(f'IMU waypoint navigation error: {str(e)}')

    def _navigate_to_coordinate_imu(self, target_x, target_y, main_app=None):
        """Navigate to specific coordinate using IMU dead reckoning"""
        try:
            timeout = 30.0  # 30 second timeout per waypoint
            start_time = time.time()
            tolerance = 0.2  # 20cm tolerance

            while time.time() - start_time < timeout:
                # Get current position from IMU tracking
                if main_app and hasattr(main_app, 'get_current_position'):
                    current_state = main_app.get_current_position()
                    if not current_state['initialized']:
                        logger.warning("IMU position not initialized, waiting...")
                        time.sleep(1)
                        continue

                    current_pos = current_state['position']
                    current_heading = current_state['orientation'][2]  # yaw
                else:
                    # Fallback to simple position tracking
                    current_pos = [0.0, 0.0, 0.0]
                    current_heading = 0.0

                # Calculate distance and bearing to target
                dx = target_x - current_pos[0]
                dy = target_y - current_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)

                logger.debug(f'Current: ({current_pos[0]:.2f}, {current_pos[1]:.2f}) '
                           f'Heading: {current_heading:.1f}°, '
                           f'Target: ({target_x:.1f}, {target_y:.1f}), '
                           f'Distance: {distance:.2f}m')

                # Check if we've reached the waypoint
                if distance <= tolerance:
                    logger.info(f'Waypoint reached! Distance: {distance:.2f}m')
                    if self.mega:
                        self.mega.send_command_to_mega('s')  # Stop
                    return True

                # Calculate required bearing to target
                target_bearing = math.degrees(math.atan2(dy, dx))

                # Calculate turn angle
                turn_angle = target_bearing - current_heading

                # Normalize turn angle to -180 to 180 degrees
                while turn_angle > 180:
                    turn_angle -= 360
                while turn_angle < -180:
                    turn_angle += 360

                # Execute turn if needed (5 degree tolerance)
                if abs(turn_angle) > 5:
                    turn_command = 'e' if turn_angle > 0 else 'q'  # e=right turn, q=left turn
                    turn_time = min(abs(turn_angle) / 45.0, 2.0)  # Max 2 seconds turn

                    if self.mega:
                        logger.debug(f'Turning {turn_angle:.1f}° for {turn_time:.1f}s')
                        self.mega.send_command_to_mega(turn_command)
                        time.sleep(turn_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                        # Update heading based on turn
                        if main_app:
                            actual_turn = turn_angle * (turn_time / (abs(turn_angle) / 45.0))  # Estimate actual turn
                            main_app.current_orientation[2] += actual_turn
                            # Keep heading in -180 to 180 range
                            while main_app.current_orientation[2] > 180:
                                main_app.current_orientation[2] -= 360
                            while main_app.current_orientation[2] < -180:
                                main_app.current_orientation[2] += 360
                            logger.debug(f'Updated heading: {main_app.current_orientation[2]:.1f}°')

                # Move forward (proportional to remaining distance)
                if distance > tolerance:
                    # Speed based on distance (slower when close)
                    speed_factor = min(distance / 2.0, 1.0)  # Max speed at 2m distance
                    move_time = min(distance / 0.3 * speed_factor, 5.0)  # Max 5 seconds

                    if self.mega:
                        logger.debug(f'Moving forward for {move_time:.1f}s (distance: {distance:.2f}m)')
                        self.mega.send_command_to_mega('f')
                        time.sleep(move_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                        # Update position estimate based on movement
                        if main_app:
                            heading_rad = main_app.current_orientation[2] * 3.14159 / 180.0
                            move_distance = min(distance, 0.3 * move_time * speed_factor)  # Estimate actual movement
                            main_app.current_position[0] += move_distance * math.cos(heading_rad)
                            main_app.current_position[1] += move_distance * math.sin(heading_rad)
                            logger.debug(f'Updated position: ({main_app.current_position[0]:.2f}, {main_app.current_position[1]:.2f})')

                time.sleep(0.2)  # Small delay between navigation iterations

            logger.warning(f'Waypoint navigation timeout after {timeout}s')
            return False

        except Exception as e:
            logger.error(f'IMU coordinate navigation error: {str(e)}')
            return False

    def _navigate_to_coordinate_imu_sensor_safe(self, target_x, target_y, main_app=None):
        """Navigate to specific coordinate using IMU dead reckoning with sensor-based obstacle avoidance"""
        try:
            timeout = 30.0  # 30 second timeout per waypoint
            start_time = time.time()
            tolerance = 0.2  # 20cm tolerance

            while time.time() - start_time < timeout:
                # Get current position from IMU tracking
                if main_app and hasattr(main_app, 'get_current_position'):
                    current_state = main_app.get_current_position()
                    if not current_state['initialized']:
                        logger.warning("IMU position not initialized, waiting...")
                        time.sleep(1)
                        continue

                    current_pos = current_state['position']
                    current_heading = current_state['orientation'][2]  # yaw
                else:
                    # Fallback to simple position tracking
                    current_pos = [0.0, 0.0, 0.0]
                    current_heading = 0.0

                # Calculate distance and bearing to target
                dx = target_x - current_pos[0]
                dy = target_y - current_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)

                logger.debug(f'Current: ({current_pos[0]:.2f}, {current_pos[1]:.2f}) '
                           f'Heading: {current_heading:.1f}°, '
                           f'Target: ({target_x:.1f}, {target_y:.1f}), '
                           f'Distance: {distance:.2f}m')

                # Check if we've reached the waypoint
                if distance <= tolerance:
                    logger.info(f'Waypoint reached! Distance: {distance:.2f}m')
                    if self.mega:
                        self.mega.send_command_to_mega('s')  # Stop
                    return True

                # Sensor safety check before movement
                if not self._check_movement_safety('forward'):
                    logger.warning("Obstacle detected during navigation - stopping for safety")
                    if self.mega:
                        self.mega.send_command_to_mega('s')  # Emergency stop
                    return False  # Cannot continue navigation

                # Calculate required bearing to target
                target_bearing = math.degrees(math.atan2(dy, dx))

                # Calculate turn angle
                turn_angle = target_bearing - current_heading

                # Normalize turn angle to -180 to 180 degrees
                while turn_angle > 180:
                    turn_angle -= 360
                while turn_angle < -180:
                    turn_angle += 360

                # Execute turn if needed (5 degree tolerance)
                if abs(turn_angle) > 5:
                    # Check if turn direction is safe
                    turn_direction = 'right' if turn_angle > 0 else 'left'
                    if not self._check_movement_safety(turn_direction):
                        logger.warning(f"Obstacle detected in {turn_direction} during turn - cannot navigate")
                        if self.mega:
                            self.mega.send_command_to_mega('s')
                        return False

                    turn_command = 'e' if turn_angle > 0 else 'q'  # e=right turn, q=left turn
                    turn_time = min(abs(turn_angle) / 45.0, 2.0)  # Max 2 seconds turn

                    if self.mega:
                        logger.debug(f'Turning {turn_angle:.1f}° for {turn_time:.1f}s')
                        self.mega.send_command_to_mega(turn_command)
                        time.sleep(turn_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                        # Update heading based on turn
                        if main_app:
                            actual_turn = turn_angle * (turn_time / (abs(turn_angle) / 45.0))  # Estimate actual turn
                            main_app.current_orientation[2] += actual_turn
                            # Keep heading in -180 to 180 range
                            while main_app.current_orientation[2] > 180:
                                main_app.current_orientation[2] -= 360
                            while main_app.current_orientation[2] < -180:
                                main_app.current_orientation[2] += 360
                            logger.debug(f'Updated heading: {main_app.current_orientation[2]:.1f}°')

                # Move forward (proportional to remaining distance)
                if distance > tolerance:
                    # Speed based on distance (slower when close)
                    speed_factor = min(distance / 2.0, 1.0)  # Max speed at 2m distance
                    move_time = min(distance / 0.3 * speed_factor, 5.0)  # Max 5 seconds

                    # Final safety check before moving
                    if not self._check_movement_safety('forward'):
                        logger.warning("Obstacle detected right before movement - stopping")
                        if self.mega:
                            self.mega.send_command_to_mega('s')
                        return False

                    if self.mega:
                        logger.debug(f'Moving forward for {move_time:.1f}s (distance: {distance:.2f}m)')
                        self.mega.send_command_to_mega('f')
                        time.sleep(move_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                        # Update position estimate based on movement
                        if main_app:
                            heading_rad = main_app.current_orientation[2] * 3.14159 / 180.0
                            move_distance = min(distance, 0.3 * move_time * speed_factor)  # Estimate actual movement
                            main_app.current_position[0] += move_distance * math.cos(heading_rad)
                            main_app.current_position[1] += move_distance * math.sin(heading_rad)
                            logger.debug(f'Updated position: ({main_app.current_position[0]:.2f}, {main_app.current_position[1]:.2f})')

                        # Update position estimate based on movement
                        if main_app:
                            heading_rad = main_app.current_orientation[2] * 3.14159 / 180.0
                            move_distance = min(distance, 0.3 * move_time * speed_factor)  # Estimate actual movement
                            main_app.current_position[0] += move_distance * math.cos(heading_rad)
                            main_app.current_position[1] += move_distance * math.sin(heading_rad)
                            logger.debug(f'Updated position: ({main_app.current_position[0]:.2f}, {main_app.current_position[1]:.2f})')

                time.sleep(0.2)  # Small delay between navigation iterations

            logger.warning(f'Waypoint navigation timeout after {timeout}s')
            return False

        except Exception as e:
            logger.error(f'Sensor-safe IMU waypoint navigation error: {str(e)}')
            # Emergency stop on error
            if self.mega:
                self.mega.send_command_to_mega('s')
            return False

    def _execute_waypoint_action(self, action):
        """Execute action at waypoint (speed, gripper, lifter, tilt)"""
        try:
            logger.info(f'Executing waypoint action: {action}')

            if not self.mega:
                logger.warning(f'No Mega interface available, skipping action: {action}')
                return

            if action.startswith('speed_'):
                # Set motor speed (e.g., speed_80 for 80%)
                try:
                    speed = int(action.split('_')[1])
                    if 50 <= speed <= 100:
                        # Map speed percentage to Mega command (5=50%, 6=60%, 7=70%, 8=80%, 9=90%, 0=100%)
                        if speed == 100:
                            speed_command = '0'
                        else:
                            speed_command = str(speed // 10)

                        self.mega.send_command_to_mega(speed_command)
                        logger.info(f'Motor speed set to {speed}%')
                        time.sleep(0.5)  # Allow speed change to take effect
                    else:
                        logger.warning(f'Invalid speed value: {speed}, must be 50-100')
                except (ValueError, IndexError) as e:
                    logger.error(f'Invalid speed action format: {action}')

            elif action == 'gripper_open':
                self.mega.send_command_to_mega('no')  # Gripper open
                logger.info('Gripper opened')
                time.sleep(1.5)  # Allow time for servo movement

            elif action == 'gripper_close':
                self.mega.send_command_to_mega('nc')  # Gripper close
                logger.info('Gripper closed')
                time.sleep(1.5)  # Allow time for servo movement

            elif action == 'gripper_half':
                self.mega.send_command_to_mega('nh')  # Gripper half-open
                logger.info('Gripper half-opened')
                time.sleep(1.5)  # Allow time for servo movement

            elif action == 'lifter_up':
                self.mega.send_command_to_mega('u')  # Lifter up
                logger.info('Lifter moving up')
                time.sleep(4.0)  # Allow time for lifter movement (increased for safety)

            elif action == 'lifter_down':
                self.mega.send_command_to_mega('d')  # Lifter down
                logger.info('Lifter moving down')
                time.sleep(4.0)  # Allow time for lifter movement (increased for safety)

            elif action == 'tilt_up':
                self.mega.send_command_to_mega('mu')  # Tilt up
                logger.info('Camera tilted up')
                time.sleep(1.0)  # Allow time for servo movement

            elif action == 'tilt_down':
                self.mega.send_command_to_mega('md')  # Tilt down
                logger.info('Camera tilted down')
                time.sleep(1.0)  # Allow time for servo movement

            elif action == 'tilt_center':
                self.mega.send_command_to_mega('mc')  # Tilt center
                logger.info('Camera centered')
                time.sleep(1.0)  # Allow time for servo movement

            elif action.startswith('wait_'):
                # Wait for specified seconds (e.g., wait_2.5 for 2.5 seconds)
                try:
                    wait_time = float(action.split('_')[1])
                    if 0 < wait_time <= 10:  # Max 10 seconds
                        logger.info(f'Waiting {wait_time}s at waypoint')
                        time.sleep(wait_time)
                    else:
                        logger.warning(f'Invalid wait time: {wait_time}, must be 0-10 seconds')
                except (ValueError, IndexError) as e:
                    logger.error(f'Invalid wait action format: {action}')

            elif action.startswith('tilt_angle_'):
                # Set tilt to specific angle (e.g., tilt_angle_45)
                try:
                    angle = int(action.split('_')[2])
                    if 0 <= angle <= 180:
                        self.mega.send_command_to_mega(f'ta{angle}')
                        logger.info(f'Camera tilted to {angle}°')
                        time.sleep(1.0)
                    else:
                        logger.warning(f'Invalid tilt angle: {angle}, must be 0-180')
                except (ValueError, IndexError) as e:
                    logger.error(f'Invalid tilt angle action format: {action}')

            elif action.startswith('gripper_angle_'):
                # Set gripper to specific angle (e.g., gripper_angle_90)
                try:
                    angle = int(action.split('_')[2])
                    if 0 <= angle <= 180:
                        self.mega.send_command_to_mega(f'ga{angle}')
                        logger.info(f'Gripper set to {angle}°')
                        time.sleep(1.0)
                    else:
                        logger.warning(f'Invalid gripper angle: {angle}, must be 0-180')
                except (ValueError, IndexError) as e:
                    logger.error(f'Invalid gripper angle action format: {action}')

            else:
                logger.warning(f'Unknown waypoint action: {action}')

        except Exception as e:
            logger.error(f'Error executing waypoint action {action}: {str(e)}')
            # Emergency stop on action error
            if self.mega:
                try:
                    self.mega.send_command_to_mega('s')
                    logger.info('Emergency stop sent due to action error')
                except:
                    pass

    def _toggle_safety_bypass(self):
        """Toggle safety bypass for testing"""
        try:
            data = request.get_json()
            if data and 'enabled' in data:
                self.safety_bypass = bool(data['enabled'])
            else:
                self.safety_bypass = not self.safety_bypass

            status = "ENABLED" if self.safety_bypass else "DISABLED"
            logger.warning(f"Safety bypass {status} - USE WITH CAUTION!")

            return jsonify({
                'success': True,
                'safety_bypass': self.safety_bypass,
                'message': f'Safety bypass {status}',
                'warning': 'Safety systems are disabled - robot may collide!',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Safety bypass toggle error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _get_current_position(self):
        """Get current robot position from IMU tracking"""
        try:
            if self.main_app and hasattr(self.main_app, 'get_current_position'):
                position_data = self.main_app.get_current_position()
                return jsonify({
                    'success': True,
                    'position': {
                        'x': position_data['position'][0],
                        'y': position_data['position'][1],
                        'z': position_data['position'][2]
                    },
                    'orientation': {
                        'roll': position_data['orientation'][0],
                        'pitch': position_data['orientation'][1],
                        'yaw': position_data['orientation'][2]
                    },
                    'initialized': position_data['initialized'],
                    'timestamp': time.time()
                })
            else:
                return jsonify({
                    'success': False,
                    'error': 'IMU position tracking not available',
                    'timestamp': time.time()
                }), 503

        except Exception as e:
            logger.error(f'Get current position error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _get_map_canvas(self):
        """Return the map canvas HTML separately to avoid template truncation"""
        map_html = '''<canvas id="waypoint-canvas" width="600" height="400" style="border: 2px solid #374151; border-radius: 8px; background: #1f2937; width: 100%; max-width: 600px;"></canvas>'''
        return map_html

    def _get_serial_monitor_data(self):
        """Get raw serial data from Mega for monitoring with enhanced status"""
        try:
            data_lines = []

            if self.mega:
                # Use the enhanced read method
                data_lines = self.mega.read_available_data(max_lines=50)

                # Add connection status messages
                if not self.mega.mega_connected:
                    if len(data_lines) == 0:  # Only add if no data
                        data_lines.append('[WARNING] Arduino Mega not connected')
                        if self.mega.auto_reconnect:
                            reconnect_info = self.mega.get_connection_status()
                            attempts = reconnect_info.get('reconnect_attempts', 0)
                            max_attempts = getattr(self.mega, 'max_reconnect_attempts', 5)
                            data_lines.append(f'[INFO] Auto-reconnecting... ({attempts}/{max_attempts})')
            else:
                data_lines.append('[ERROR] Mega interface not initialized')

            # Get connection status
            connection_status = self.mega.get_connection_status() if self.mega else {}

            return jsonify({
                'success': True,
                'data': data_lines,
                'timestamp': time.time(),
                'connection': {
                    'connected': connection_status.get('connected', False),
                    'port': connection_status.get('port'),
                    'auto_reconnect': connection_status.get('auto_reconnect', False),
                    'reconnect_attempts': connection_status.get('reconnect_attempts', 0),
                    'last_activity': connection_status.get('last_activity', 0),
                    'healthy': self.mega.is_healthy() if self.mega else False
                },
                'stats': connection_status.get('stats', {})
            })

        except Exception as e:
            logger.error(f'Serial monitor data error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'data': [],
                'timestamp': time.time(),
                'connection': {
                    'connected': False,
                    'healthy': False
                },
                'stats': {}
            }), 500

    def _get_mega_status(self):
        """Get detailed Mega connection status"""
        try:
            if not self.mega:
                return jsonify({
                    'success': False,
                    'error': 'Mega interface not initialized',
                    'status': {}
                }), 503

            status = self.mega.get_connection_status()
            return jsonify({
                'success': True,
                'status': status,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Mega status error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'status': {},
                'timestamp': time.time()
            }), 500

    def _reconnect_mega(self):
        """Force Mega reconnection"""
        try:
            if not self.mega:
                return jsonify({
                    'success': False,
                    'error': 'Mega interface not initialized'
                }), 503

            logger.info('Forcing Mega reconnection via API')
            success = self.mega.force_reconnect()

            return jsonify({
                'success': success,
                'message': 'Reconnection ' + ('successful' if success else 'failed'),
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Mega reconnection error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _path_to_commands(self, path):
        """Convert path coordinates to movement commands"""
        commands = []

        for i in range(1, len(path)):
            current = path[i - 1]
            next_pos = path[i]

            # Calculate movement direction
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]

            # Determine primary direction
            if abs(dx) > abs(dy):
                command = 'r' if dx > 0 else 'l'
            else:
                command = 'f' if dy > 0 else 'b'

            commands.append(command)

        commands.append('s')  # Stop at destination
        return commands

    def run(self, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG):
        """Run the Flask application"""
        logger.info(f"Starting Flask web server on http://{host}:{port}")
        self.app.run(host=host, port=port, debug=debug)
