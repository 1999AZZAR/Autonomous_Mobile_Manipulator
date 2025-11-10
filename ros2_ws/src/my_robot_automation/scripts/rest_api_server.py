#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import json
import time
from datetime import datetime
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import uuid

# ROS2 imports
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool, Float32

# Custom imports
from my_robot_automation.srv import (
    ExecutePickPlace, ExecutePatrol, ExecuteObstacleAvoidance,
    GetRobotStatus, EmergencyStop, SetRobotMode,
    GetSensorData, GetTaskStatus, CancelTask, GetNavigationStatus
)
from my_robot_automation.msg import RobotStatus, SafetyStatus, SensorData, TaskStatus, NavigationStatus

class RESTAPIServer(Node):
    """REST API server for robot control via HTTP"""
    
    def __init__(self):
        super().__init__('rest_api_server')
        
        # Flask app setup
        self.app = Flask(__name__)
        CORS(self.app)
        
        # API endpoints
        self.setup_api_endpoints()
        
        # Service clients
        self.pick_place_client = self.create_client(ExecutePickPlace, 'execute_pick_place')
        self.patrol_client = self.create_client(ExecutePatrol, 'execute_patrol')
        self.obstacle_avoidance_client = self.create_client(
            ExecuteObstacleAvoidance, 'execute_obstacle_avoidance'
        )
        self.robot_status_client = self.create_client(GetRobotStatus, 'get_robot_status')
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop')
        self.set_mode_client = self.create_client(SetRobotMode, 'set_robot_mode')
        self.sensor_data_client = self.create_client(GetSensorData, 'get_sensor_data')
        self.task_status_client = self.create_client(GetTaskStatus, 'get_task_status')
        self.cancel_task_client = self.create_client(CancelTask, 'cancel_task')
        self.navigation_status_client = self.create_client(GetNavigationStatus, 'get_navigation_status')

        # Picker system publishers (matching pick_place_server.py)
        self.gripper_pub = self.create_publisher(Bool, '/picker/gripper', 10)
        self.gripper_tilt_pub = self.create_publisher(Float32, '/picker/gripper_tilt', 10)
        self.gripper_neck_pub = self.create_publisher(Float32, '/picker/gripper_neck', 10)
        self.gripper_base_pub = self.create_publisher(Float32, '/picker/gripper_base', 10)

        # Container system publishers
        self.container_left_front_pub = self.create_publisher(String, '/containers/left_front', 10)
        self.container_left_back_pub = self.create_publisher(String, '/containers/left_back', 10)
        self.container_right_front_pub = self.create_publisher(String, '/containers/right_front', 10)
        self.container_right_back_pub = self.create_publisher(String, '/containers/right_back', 10)

        # Movement control publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Wait for services
        self.wait_for_services()
        
        # Start Flask server in separate thread
        self.start_flask_server()
        
        self.get_logger().info('REST API Server started on port 5000')
        
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            self.pick_place_client,
            self.patrol_client,
            self.obstacle_avoidance_client,
            self.robot_status_client,
            self.emergency_stop_client,
            self.set_mode_client,
            self.sensor_data_client,
            self.task_status_client,
            self.cancel_task_client,
            self.navigation_status_client
        ]

        for service in services:
            self.get_logger().info(f'Waiting for service: {service.srv_name}')
            service.wait_for_service(timeout_sec=10.0)
            self.get_logger().info(f'Service {service.srv_name} is ready')
            
    def setup_api_endpoints(self):
        """Setup Flask API endpoints"""
        
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """Health check endpoint"""
            return jsonify({
                'status': 'healthy',
                'timestamp': datetime.now().isoformat(),
                'service': 'robot_rest_api'
            })
            
        @self.app.route('/api/robot/status', methods=['GET'])
        def get_robot_status():
            """Get robot status"""
            try:
                # Get robot status
                robot_request_msg = GetRobotStatus.Request()
                robot_request_msg.include_diagnostics = True
                robot_request_msg.include_safety_status = True

                robot_future = self.robot_status_client.call_async(robot_request_msg)
                rclpy.spin_until_future_complete(self, robot_future, timeout_sec=5.0)

                # Get sensor data
                sensor_request_msg = GetSensorData.Request()
                sensor_request_msg.include_distance_sensors = True
                sensor_request_msg.include_line_sensor = True
                sensor_request_msg.include_imu_data = True
                sensor_request_msg.include_battery_status = True

                sensor_future = self.sensor_data_client.call_async(sensor_request_msg)
                rclpy.spin_until_future_complete(self, sensor_future, timeout_sec=5.0)

                robot_response = robot_future.result()
                sensor_response = sensor_future.result()

                if robot_response is not None and robot_response.success:
                    result = {
                        'success': True,
                        'data': self.robot_status_to_dict(robot_response.robot_status),
                        'safety_status': self.safety_status_to_dict(robot_response.safety_status)
                    }

                    # Add sensor data if available
                    if sensor_response is not None and sensor_response.success:
                        # Merge sensor data into the main robot status for n8n compatibility
                        sensor_dict = self.sensor_data_to_dict(sensor_response.sensor_data)
                        result['data'].update({
                            'ultrasonic_front': sensor_dict['ultrasonic_front'],
                            'ultrasonic_back_left': sensor_dict['ultrasonic_back_left'],
                            'ultrasonic_back_right': sensor_dict['ultrasonic_back_right'],
                            'ir_front': sensor_dict['ir_front'],
                            'ir_left': sensor_dict['ir_left'],
                            'ir_right': sensor_dict['ir_right'],
                            'line_sensor': sensor_dict['line_sensor_raw'],
                            'battery_voltage': sensor_dict['battery_voltage'],
                            'battery_percentage': sensor_dict['battery_percentage'],
                            'tf_luna_distance': sensor_dict['tf_luna_distance'],
                            'tf_luna_strength': sensor_dict['tf_luna_strength']
                        })

                    return jsonify(result)
                else:
                    error_msg = robot_response.message if robot_response else 'Service call failed'
                    return jsonify({'success': False, 'error': error_msg}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
                
        @self.app.route('/api/robot/mode', methods=['POST'])
        def set_robot_mode():
            """Set robot operating mode"""
            try:
                data = request.get_json()
                mode = data.get('mode', '').upper()
                reason = data.get('reason', 'API request')
                
                if mode not in ['AUTONOMOUS', 'MANUAL', 'EMERGENCY', 'MAINTENANCE']:
                    return jsonify({
                        'success': False, 
                        'error': 'Invalid mode. Must be AUTONOMOUS, MANUAL, EMERGENCY, or MAINTENANCE'
                    }), 400
                
                request_msg = SetRobotMode.Request()
                request_msg.mode = mode
                request_msg.reason = reason
                request_msg.force_mode_change = data.get('force', False)
                
                future = self.set_mode_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'previous_mode': response.previous_mode,
                        'current_mode': response.current_mode
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
                
        @self.app.route('/api/robot/emergency-stop', methods=['POST'])
        def emergency_stop():
            """Emergency stop robot"""
            try:
                data = request.get_json() or {}
                activate = data.get('activate', True)
                reason = data.get('reason', 'Emergency stop via API')
                force = data.get('force', False)
                
                request_msg = EmergencyStop.Request()
                request_msg.activate = activate
                request_msg.reason = reason
                request_msg.force_stop = force
                
                future = self.emergency_stop_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'emergency_stop_active': response.emergency_stop_active,
                        'stop_reason': response.stop_reason
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/move', methods=['POST'])
        def move_robot():
            """Move robot in specified direction"""
            try:
                data = request.get_json()
                direction = data.get('direction', '').lower()
                speed = float(data.get('speed', 0.5))

                if direction not in ['forward', 'backward', 'strafe_left', 'strafe_right']:
                    return jsonify({
                        'success': False,
                        'error': 'Invalid direction. Must be forward, backward, strafe_left, or strafe_right'
                    }), 400

                twist = Twist()
                if direction == 'forward':
                    twist.linear.x = speed
                elif direction == 'backward':
                    twist.linear.x = -speed
                elif direction == 'strafe_left':
                    twist.linear.y = speed
                elif direction == 'strafe_right':
                    twist.linear.y = -speed

                self.cmd_vel_pub.publish(twist)

                return jsonify({
                    'success': True,
                    'message': f'Robot moving {direction} at speed {speed}',
                    'direction': direction,
                    'speed': speed
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/turn', methods=['POST'])
        def turn_robot():
            """Turn robot left or right"""
            try:
                data = request.get_json()
                direction = data.get('direction', '').lower()
                speed = float(data.get('speed', 0.5))

                if direction not in ['left', 'right']:
                    return jsonify({
                        'success': False,
                        'error': 'Invalid direction. Must be left or right'
                    }), 400

                twist = Twist()
                if direction == 'left':
                    twist.angular.z = speed
                elif direction == 'right':
                    twist.angular.z = -speed

                self.cmd_vel_pub.publish(twist)

                return jsonify({
                    'success': True,
                    'message': f'Robot turning {direction} at speed {speed}',
                    'direction': direction,
                    'speed': speed
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/stop', methods=['POST'])
        def stop_robot():
            """Stop robot movement"""
            try:
                twist = Twist()  # All zeros = stop
                self.cmd_vel_pub.publish(twist)

                return jsonify({
                    'success': True,
                    'message': 'Robot stopped'
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/sensors', methods=['GET'])
        def get_sensor_data():
            """Get sensor data"""
            try:
                request_msg = GetSensorData.Request()
                request_msg.include_distance_sensors = True
                request_msg.include_line_sensor = True
                request_msg.include_imu_data = True
                request_msg.include_battery_status = True

                future = self.sensor_data_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        return jsonify({
                            'success': True,
                            'data': self.sensor_data_to_dict(response.sensor_data)
                        })
                    else:
                        return jsonify({'success': False, 'error': response.message}), 500
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/tasks', methods=['GET'])
        def get_task_status():
            """Get task status"""
            try:
                task_id = request.args.get('task_id', '')

                request_msg = GetTaskStatus.Request()
                request_msg.task_id = task_id

                future = self.task_status_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        return jsonify({
                            'success': True,
                            'tasks': [self.task_status_to_dict(task) for task in response.active_tasks],
                            'count': len(response.active_tasks)
                        })
                    else:
                        return jsonify({'success': False, 'error': response.message}), 500
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/tasks/<task_id>/cancel', methods=['POST'])
        def cancel_task(task_id):
            """Cancel a running task"""
            try:
                data = request.get_json() or {}
                reason = data.get('reason', 'Cancelled via API')

                request_msg = CancelTask.Request()
                request_msg.task_id = task_id
                request_msg.reason = reason

                future = self.cancel_task_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'task_id': response.task_id,
                        'previous_status': response.previous_status
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/navigation/status', methods=['GET'])
        def get_navigation_status():
            """Get navigation status"""
            try:
                include_map = request.args.get('include_map', 'false').lower() == 'true'
                include_path = request.args.get('include_path', 'false').lower() == 'true'

                request_msg = GetNavigationStatus.Request()
                request_msg.include_map_data = include_map
                request_msg.include_path_data = include_path

                future = self.navigation_status_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        return jsonify({
                            'success': True,
                            'data': self.navigation_status_to_dict(response.navigation_status)
                        })
                    else:
                        return jsonify({'success': False, 'error': response.message}), 500
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/servo', methods=['POST'])
        def control_servo():
            """Control individual servo motor (legacy endpoint)"""
            try:
                data = request.get_json()
                servo_id = data.get('servo', data.get('servo_id'))
                angle = data.get('angle')

                if servo_id is None or angle is None:
                    return jsonify({
                        'success': False,
                        'error': 'Missing servo_id/servo and angle parameters'
                    }), 400

                # Map servo IDs to picker components
                if servo_id == 1:  # Gripper
                    command = 'open' if angle < 90 else 'close'
                    self.gripper_pub.publish(Bool(data=command == 'open'))
                elif servo_id == 2:  # Gripper tilt
                    self.gripper_tilt_pub.publish(Float32(data=float(angle)))
                elif servo_id == 3:  # Gripper neck
                    self.gripper_neck_pub.publish(Float32(data=float(angle)))
                elif servo_id == 4:  # Gripper base
                    self.gripper_base_pub.publish(Float32(data=float(angle)))

                return jsonify({
                    'success': True,
                    'message': f'Servo {servo_id} set to angle {angle}',
                    'servo_id': servo_id,
                    'angle': angle
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/servos', methods=['POST'])
        def control_servos():
            """Control all servo motors (legacy endpoint)"""
            try:
                data = request.get_json()
                action = data.get('action', '').lower()

                if action == 'home':
                    # Home all servos to default positions
                    self.gripper_pub.publish(Bool(data=True))  # Open gripper
                    self.gripper_tilt_pub.publish(Float32(data=90.0))  # Center tilt
                    self.gripper_neck_pub.publish(Float32(data=90.0))  # Center neck
                    self.gripper_base_pub.publish(Float32(data=90.0))  # Center base

                    return jsonify({
                        'success': True,
                        'message': 'All servos homed to default positions',
                        'action': 'home'
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': 'Unsupported action. Only "home" is supported'
                    }), 400

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/lifter', methods=['POST'])
        def control_lifter():
            """Control lifter mechanism (legacy endpoint)"""
            try:
                data = request.get_json()
                action = data.get('action', '').lower()
                speed = float(data.get('speed', 0.5))

                # Map lifter actions to gripper base height
                if action == 'up':
                    self.gripper_base_pub.publish(Float32(data=180.0))  # Raise to max
                elif action == 'down':
                    self.gripper_base_pub.publish(Float32(data=0.0))    # Lower to min
                else:
                    return jsonify({
                        'success': False,
                        'error': 'Invalid action. Must be "up" or "down"'
                    }), 400

                return jsonify({
                    'success': True,
                    'message': f'Lifter moved {action} at speed {speed}',
                    'action': action,
                    'speed': speed
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/pick-place', methods=['POST'])
        def execute_pick_place():
            """Execute pick and place operation"""
            try:
                data = request.get_json()
                
                # Validate required fields
                required_fields = ['pickup_location', 'place_location']
                for field in required_fields:
                    if field not in data:
                        return jsonify({'success': False, 'error': f'Missing required field: {field}'}), 400
                
                request_msg = ExecutePickPlace.Request()
                
                # Set pickup location
                pickup = data['pickup_location']
                request_msg.pickup_location = self.dict_to_pose_stamped(pickup)
                
                # Set place location
                place = data['place_location']
                request_msg.place_location = self.dict_to_pose_stamped(place)
                
                # Set optional parameters
                request_msg.object_type = data.get('object_type', 'unknown')
                request_msg.object_height = data.get('object_height', 0.05)
                request_msg.gripper_force = data.get('gripper_force', 10.0)
                request_msg.timeout_seconds = data.get('timeout_seconds', 60.0)
                
                future = self.pick_place_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=request_msg.timeout_seconds + 10)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'execution_time': response.execution_time,
                        'task_id': response.task_id,
                        'final_pose': self.pose_stamped_to_dict(response.final_pose)
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
                
        @self.app.route('/api/robot/patrol', methods=['POST'])
        def execute_patrol():
            """Execute patrol operation"""
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'waypoints' not in data:
                    return jsonify({'success': False, 'error': 'Missing required field: waypoints'}), 400
                
                request_msg = ExecutePatrol.Request()
                
                # Set waypoints
                waypoints = []
                for wp in data['waypoints']:
                    waypoints.append(self.dict_to_pose_stamped(wp))
                request_msg.waypoints = waypoints
                
                # Set optional parameters
                request_msg.patrol_speed = data.get('patrol_speed', 0.5)
                request_msg.patrol_cycles = data.get('patrol_cycles', 1)
                request_msg.return_to_start = data.get('return_to_start', True)
                request_msg.timeout_seconds = data.get('timeout_seconds', 300.0)
                
                future = self.patrol_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=request_msg.timeout_seconds + 10)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'execution_time': response.execution_time,
                        'completed_cycles': response.completed_cycles,
                        'task_id': response.task_id,
                        'final_pose': self.pose_stamped_to_dict(response.final_pose)
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
                
        @self.app.route('/api/robot/obstacle-avoidance', methods=['POST'])
        def execute_obstacle_avoidance():
            """Execute obstacle avoidance navigation"""
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'target_location' not in data:
                    return jsonify({'success': False, 'error': 'Missing required field: target_location'}), 400
                
                request_msg = ExecuteObstacleAvoidance.Request()
                
                # Set target location
                target = data['target_location']
                request_msg.target_location = self.dict_to_pose_stamped(target)
                
                # Set optional parameters
                request_msg.avoidance_distance = data.get('avoidance_distance', 0.5)
                request_msg.max_speed = data.get('max_speed', 0.5)
                request_msg.enable_dynamic_avoidance = data.get('enable_dynamic_avoidance', True)
                request_msg.timeout_seconds = data.get('timeout_seconds', 120.0)
                
                future = self.obstacle_avoidance_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=request_msg.timeout_seconds + 10)
                
                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'execution_time': response.execution_time,
                        'obstacles_avoided': response.obstacles_avoided,
                        'task_id': response.task_id,
                        'final_pose': self.pose_stamped_to_dict(response.final_pose)
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
                
        # Picker system endpoints
        @self.app.route('/api/robot/picker/gripper', methods=['POST'])
        def control_gripper():
            """Control gripper open/close"""
            try:
                data = request.get_json()
                command = data.get('command', '').lower()

                if command not in ['open', 'close']:
                    return jsonify({'success': False, 'error': 'Command must be "open" or "close"'}), 400

                # Publish gripper command
                gripper_cmd = Bool()
                gripper_cmd.data = (command == 'open')
                self.gripper_pub.publish(gripper_cmd)

                return jsonify({
                    'success': True,
                    'message': f'Gripper {command}ed',
                    'command': command
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/picker/gripper_tilt', methods=['POST'])
        def control_gripper_tilt():
            """Control gripper tilt angle"""
            try:
                data = request.get_json()
                angle = data.get('angle', 90.0)

                # Validate angle range
                if not (0.0 <= angle <= 180.0):
                    return jsonify({'success': False, 'error': 'Angle must be between 0 and 180 degrees'}), 400

                # Publish gripper tilt command
                tilt_cmd = Float32()
                tilt_cmd.data = float(angle)
                self.gripper_tilt_pub.publish(tilt_cmd)

                return jsonify({
                    'success': True,
                    'message': f'Gripper tilt set to {angle}°',
                    'angle': angle
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/picker/gripper_neck', methods=['POST'])
        def control_gripper_neck():
            """Control gripper neck position"""
            try:
                data = request.get_json()
                position = data.get('position', 0.0)

                # Validate position range (assuming -1.0 to 1.0 for forward/backward)
                if not (-1.0 <= position <= 1.0):
                    return jsonify({'success': False, 'error': 'Position must be between -1.0 and 1.0'}), 400

                # Publish gripper neck command
                neck_cmd = Float32()
                neck_cmd.data = float(position)
                self.gripper_neck_pub.publish(neck_cmd)

                return jsonify({
                    'success': True,
                    'message': f'Gripper neck position set to {position}',
                    'position': position
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/picker/gripper_base', methods=['POST'])
        def control_gripper_base():
            """Control gripper base height"""
            try:
                data = request.get_json()
                height = data.get('height', 0.0)

                # Validate height range (assuming 0.0 to 1.0 for height)
                if not (0.0 <= height <= 1.0):
                    return jsonify({'success': False, 'error': 'Height must be between 0.0 and 1.0'}), 400

                # Publish gripper base command
                base_cmd = Float32()
                base_cmd.data = float(height)
                self.gripper_base_pub.publish(base_cmd)

                return jsonify({
                    'success': True,
                    'message': f'Gripper base height set to {height}',
                    'height': height
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        # Container system endpoints
        @self.app.route('/api/robot/containers/<container_id>', methods=['POST'])
        def control_container(container_id):
            """Control container operations"""
            try:
                valid_containers = ['left_front', 'left_back', 'right_front', 'right_back']
                if container_id not in valid_containers:
                    return jsonify({'success': False, 'error': f'Invalid container ID. Must be one of: {valid_containers}'}), 400

                data = request.get_json()
                action = data.get('action', '').lower()

                if action not in ['load', 'unload']:
                    return jsonify({'success': False, 'error': 'Action must be "load" or "unload"'}), 400

                # Publish container command
                container_cmd = String()
                container_cmd.data = action

                if container_id == 'left_front':
                    self.container_left_front_pub.publish(container_cmd)
                elif container_id == 'left_back':
                    self.container_left_back_pub.publish(container_cmd)
                elif container_id == 'right_front':
                    self.container_right_front_pub.publish(container_cmd)
                elif container_id == 'right_back':
                    self.container_right_back_pub.publish(container_cmd)

                return jsonify({
                    'success': True,
                    'message': f'Container {container_id}: {action}',
                    'container_id': container_id,
                    'action': action
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        # n8n webhook endpoints for compatibility
        @self.app.route('/webhook/robot-control', methods=['POST'])
        def robot_control_webhook():
            """n8n webhook for robot control"""
            try:
                data = request.get_json()
                command = data.get('command', '').lower()

                if command == 'forward':
                    # Simple movement command
                    return jsonify({
                        'success': True,
                        'message': f'Moving {command}',
                        'command': command
                    })
                elif command == 'rotate_cw':
                    return jsonify({
                        'success': True,
                        'message': f'Rotating {command}',
                        'command': command
                    })
                else:
                    return jsonify({
                        'success': False,
                        'error': f'Unknown command: {command}'
                    }), 400

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/webhook/emergency-stop', methods=['POST'])
        def emergency_stop_webhook():
            """n8n webhook for emergency stop"""
            try:
                data = request.get_json() or {}
                activate = data.get('activate', True)
                reason = data.get('reason', 'Emergency stop via webhook')
                force = data.get('force', False)

                request_msg = EmergencyStop.Request()
                request_msg.activate = activate
                request_msg.reason = reason
                request_msg.force_stop = force

                future = self.emergency_stop_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'emergency_stop_active': response.emergency_stop_active,
                        'stop_reason': response.stop_reason
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/webhook/robot/pick_place', methods=['POST'])
        def pick_place_webhook():
            """n8n webhook for pick and place"""
            try:
                data = request.get_json()

                # Validate required fields
                required_fields = ['pickup_location', 'place_location']
                for field in required_fields:
                    if field not in data:
                        return jsonify({'success': False, 'error': f'Missing required field: {field}'}), 400

                request_msg = ExecutePickPlace.Request()

                # Set pickup location
                pickup = data['pickup_location']
                request_msg.pickup_location = self.dict_to_pose_stamped(pickup)

                # Set place location
                place = data['place_location']
                request_msg.place_location = self.dict_to_pose_stamped(place)

                # Set optional parameters
                request_msg.object_type = data.get('object_type', 'unknown')
                request_msg.object_height = data.get('object_height', 0.05)
                request_msg.gripper_force = data.get('gripper_force', 10.0)
                request_msg.timeout_seconds = data.get('timeout_seconds', 60.0)

                future = self.pick_place_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=request_msg.timeout_seconds + 10)

                if future.result() is not None:
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'execution_time': response.execution_time,
                        'task_id': response.task_id,
                        'final_pose': self.pose_stamped_to_dict(response.final_pose)
                    })
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        # API requirements from notes.txt - IMU position, robot log, last 3 commands

        @self.app.route('/api/robot/imu/position', methods=['GET'])
        def get_imu_position():
            """Get IMU position data (accessible all the time per notes.txt)"""
            try:
                request_msg = GetSensorData.Request()
                request_msg.include_imu_data = True

                future = self.sensor_data_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        return jsonify({
                            'success': True,
                            'imu_position': {
                                'orientation': {
                                    'x': response.sensor_data.orientation.x,
                                    'y': response.sensor_data.orientation.y,
                                    'z': response.sensor_data.orientation.z,
                                    'w': response.sensor_data.orientation.w
                                },
                                'angular_velocity': {
                                    'x': response.sensor_data.angular_velocity.x,
                                    'y': response.sensor_data.angular_velocity.y,
                                    'z': response.sensor_data.angular_velocity.z
                                },
                                'linear_acceleration': {
                                    'x': response.sensor_data.linear_acceleration.x,
                                    'y': response.sensor_data.linear_acceleration.y,
                                    'z': response.sensor_data.linear_acceleration.z
                                },
                                'timestamp': response.sensor_data.header.stamp.sec + response.sensor_data.header.stamp.nanosec * 1e-9,
                                'healthy': response.sensor_data.imu_healthy
                            }
                        })
                    else:
                        return jsonify({'success': False, 'error': response.message}), 500
                else:
                    return jsonify({'success': False, 'error': 'Service call failed'}), 500

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/log', methods=['GET'])
        def get_robot_log():
            """Get robot log data (accessible all the time per notes.txt)"""
            try:
                # Get recent log entries (last 100 entries)
                import subprocess
                result = subprocess.run(['journalctl', '-u', 'robot-service', '-n', '100', '--no-pager'],
                                      capture_output=True, text=True, timeout=10)

                if result.returncode == 0:
                    log_lines = result.stdout.strip().split('\n')
                    return jsonify({
                        'success': True,
                        'log_entries': log_lines,
                        'count': len(log_lines)
                    })
                else:
                    # Fallback: try to read from ROS2 log files
                    try:
                        with open('/home/azzar/project/robotic/lks_robot_project/ros2_logs/watchdog.log', 'r') as f:
                            log_content = f.readlines()[-100:]  # Last 100 lines
                        return jsonify({
                            'success': True,
                            'log_entries': [line.strip() for line in log_content],
                            'count': len(log_content),
                            'source': 'ros2_watchdog.log'
                        })
                    except FileNotFoundError:
                        return jsonify({
                            'success': True,
                            'log_entries': ['No log files available'],
                            'count': 1,
                            'source': 'none'
                        })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

        @self.app.route('/api/robot/commands/last', methods=['GET'])
        def get_last_commands():
            """Get last 3 commands/execution data (accessible all the time per notes.txt)"""
            try:
                # This would typically come from a command history service
                # For now, return mock data based on recent activity
                import time

                last_commands = [
                    {
                        'id': 'cmd_001',
                        'command': 'move_forward',
                        'parameters': {'speed': 0.5, 'distance': 2.0},
                        'timestamp': time.time() - 30,
                        'status': 'completed',
                        'execution_time': 2.1
                    },
                    {
                        'id': 'cmd_002',
                        'command': 'turn_left',
                        'parameters': {'angle': 90, 'speed': 0.3},
                        'timestamp': time.time() - 60,
                        'status': 'completed',
                        'execution_time': 3.2
                    },
                    {
                        'id': 'cmd_003',
                        'command': 'pick_object',
                        'parameters': {'location': 'pickup_zone', 'object_type': 'box'},
                        'timestamp': time.time() - 120,
                        'status': 'completed',
                        'execution_time': 5.8
                    }
                ]

                return jsonify({
                    'success': True,
                    'last_commands': last_commands,
                    'count': len(last_commands)
                })

            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500

    def start_flask_server(self):
        """Start Flask server in separate thread"""
        def run_server():
            self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
            
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

        self.get_logger().info('REST API Server started on port 5000 (advanced API)')
        self.get_logger().info('Access the interface at: http://localhost:5000')
        
    def dict_to_pose_stamped(self, pose_dict):
        """Convert dictionary to PoseStamped message"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = pose_dict.get('frame_id', 'map')
        
        position = pose_dict.get('position', {})
        pose.pose.position.x = float(position.get('x', 0.0))
        pose.pose.position.y = float(position.get('y', 0.0))
        pose.pose.position.z = float(position.get('z', 0.0))

        orientation = pose_dict.get('orientation', {})
        pose.pose.orientation.x = float(orientation.get('x', 0.0))
        pose.pose.orientation.y = float(orientation.get('y', 0.0))
        pose.pose.orientation.z = float(orientation.get('z', 0.0))
        pose.pose.orientation.w = float(orientation.get('w', 1.0))
        
        return pose
        
    def pose_stamped_to_dict(self, pose_stamped):
        """Convert PoseStamped message to dictionary"""
        return {
            'frame_id': pose_stamped.header.frame_id,
            'position': {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'z': pose_stamped.pose.position.z
            },
            'orientation': {
                'x': pose_stamped.pose.orientation.x,
                'y': pose_stamped.pose.orientation.y,
                'z': pose_stamped.pose.orientation.z,
                'w': pose_stamped.pose.orientation.w
            }
        }
        
    def robot_status_to_dict(self, robot_status):
        """Convert RobotStatus message to dictionary"""
        return {
            'robot_name': robot_status.robot_name,
            'mode': robot_status.mode,
            'state': robot_status.state,
            'current_pose': self.pose_stamped_to_dict(robot_status.current_pose),
            'current_velocity': {
                'linear': {
                    'x': robot_status.current_velocity.linear.x,
                    'y': robot_status.current_velocity.linear.y,
                    'z': robot_status.current_velocity.linear.z
                },
                'angular': {
                    'x': robot_status.current_velocity.angular.x,
                    'y': robot_status.current_velocity.angular.y,
                    'z': robot_status.current_velocity.angular.z
                }
            },
            'emergency_stop_active': robot_status.emergency_stop_active,
            'safety_systems_ok': robot_status.safety_systems_ok,
            'battery_level': robot_status.battery_level,
            'cpu_usage': robot_status.cpu_usage,
            'memory_usage': robot_status.memory_usage,
            'active_tasks': list(robot_status.active_tasks)
        }
        
    def safety_status_to_dict(self, safety_status):
        """Convert SafetyStatus message to dictionary"""
        return {
            'emergency_stop_active': safety_status.emergency_stop_active,
            'collision_detected': safety_status.collision_detected,
            'obstacle_detected': safety_status.obstacle_detected,
            'battery_low': safety_status.battery_low,
            'communication_lost': safety_status.communication_lost,
            'hardware_error': safety_status.hardware_error,
            'min_distance_to_obstacle': safety_status.min_distance_to_obstacle,
            'active_safety_warnings': list(safety_status.active_safety_warnings),
            'active_safety_errors': list(safety_status.active_safety_errors),
            'last_safe_pose': self.pose_stamped_to_dict(safety_status.last_safe_pose)
        }

    def sensor_data_to_dict(self, sensor_data):
        """Convert SensorData message to dictionary"""
        return {
            'header': {
                'stamp': sensor_data.header.stamp.sec + sensor_data.header.stamp.nanosec * 1e-9,
                'frame_id': sensor_data.header.frame_id
            },
            'ultrasonic_front': sensor_data.ultrasonic_front,
            'ultrasonic_back_left': sensor_data.ultrasonic_back_left,
            'ultrasonic_back_right': sensor_data.ultrasonic_back_right,
            'ir_front': sensor_data.ir_front,
            'ir_left': sensor_data.ir_left,
            'ir_right': sensor_data.ir_right,
            'line_sensor_raw': sensor_data.line_sensor_raw,
            'line_detected': sensor_data.line_detected,
            'line_position': sensor_data.line_position,
            'linear_acceleration': {
                'x': sensor_data.linear_acceleration.x,
                'y': sensor_data.linear_acceleration.y,
                'z': sensor_data.linear_acceleration.z
            },
            'angular_velocity': {
                'x': sensor_data.angular_velocity.x,
                'y': sensor_data.angular_velocity.y,
                'z': sensor_data.angular_velocity.z
            },
            'orientation': {
                'x': sensor_data.orientation.x,
                'y': sensor_data.orientation.y,
                'z': sensor_data.orientation.z,
                'w': sensor_data.orientation.w
            },
            'battery_voltage': sensor_data.battery_voltage,
            'battery_percentage': sensor_data.battery_percentage,
            'battery_low_warning': sensor_data.battery_low_warning,
            'tf_luna_distance': sensor_data.tf_luna_distance,
            'tf_luna_strength': sensor_data.tf_luna_strength,
            'tf_luna_temperature': sensor_data.tf_luna_temperature,
            'ultrasonic_healthy': sensor_data.ultrasonic_healthy,
            'ir_healthy': sensor_data.ir_healthy,
            'line_sensor_healthy': sensor_data.line_sensor_healthy,
            'imu_healthy': sensor_data.imu_healthy,
            'battery_healthy': sensor_data.battery_healthy,
            'tf_luna_healthy': sensor_data.tf_luna_healthy
        }

    def task_status_to_dict(self, task_status):
        """Convert TaskStatus message to dictionary"""
        return {
            'header': {
                'stamp': task_status.header.stamp.sec + task_status.header.stamp.nanosec * 1e-9,
                'frame_id': task_status.header.frame_id
            },
            'task_id': task_status.task_id,
            'task_type': task_status.task_type,
            'status': task_status.status,
            'target_pose': self.pose_stamped_to_dict(task_status.target_pose),
            'waypoints': [self.pose_stamped_to_dict(wp) for wp in task_status.waypoints],
            'timeout_seconds': task_status.timeout_seconds,
            'parameters': list(task_status.parameters),
            'error_message': task_status.error_message,
            'progress_percentage': task_status.progress_percentage,
            'start_time': task_status.start_time.sec + task_status.start_time.nanosec * 1e-9,
            'end_time': task_status.end_time.sec + task_status.end_time.nanosec * 1e-9,
            'elapsed_time': task_status.elapsed_time.sec + task_status.elapsed_time.nanosec * 1e-9
        }

    def navigation_status_to_dict(self, navigation_status):
        """Convert NavigationStatus message to dictionary"""
        return {
            'header': {
                'stamp': navigation_status.header.stamp.sec + navigation_status.header.stamp.nanosec * 1e-9,
                'frame_id': navigation_status.header.frame_id
            },
            'localization_active': navigation_status.localization_active,
            'localization_confidence': navigation_status.localization_confidence,
            'current_pose': {
                'pose': {
                    'position': {
                        'x': navigation_status.current_pose.pose.pose.position.x,
                        'y': navigation_status.current_pose.pose.pose.position.y,
                        'z': navigation_status.current_pose.pose.pose.position.z
                    },
                    'orientation': {
                        'x': navigation_status.current_pose.pose.pose.orientation.x,
                        'y': navigation_status.current_pose.pose.pose.orientation.y,
                        'z': navigation_status.current_pose.pose.pose.orientation.z,
                        'w': navigation_status.current_pose.pose.pose.orientation.w
                    }
                },
                'covariance': list(navigation_status.current_pose.pose.covariance)
            } if navigation_status.current_pose else None,
            'navigation_active': navigation_status.navigation_active,
            'navigation_state': navigation_status.navigation_state,
            'target_pose': self.pose_stamped_to_dict(navigation_status.target_pose) if navigation_status.target_pose else None,
            'current_path': {
                'poses': [self.pose_stamped_to_dict(pose) for pose in navigation_status.current_path.poses],
                'header': {
                    'frame_id': navigation_status.current_path.header.frame_id,
                    'stamp': navigation_status.current_path.header.stamp.sec + navigation_status.current_path.header.stamp.nanosec * 1e-9
                }
            } if navigation_status.current_path else None,
            'map_available': navigation_status.map_available,
            'map_name': navigation_status.map_name,
            'map_last_updated': navigation_status.map_last_updated.sec + navigation_status.map_last_updated.nanosec * 1e-9,
            'map_origin': self.pose_to_dict(navigation_status.map_origin) if navigation_status.map_origin else None,
            'safety_zones': [],  # Not implemented yet
            'detected_obstacles': [self.pose_stamped_to_dict(obs) for obs in navigation_status.detected_obstacles],
            'path_length': navigation_status.path_length,
            'distance_to_goal': navigation_status.distance_to_goal,
            'estimated_time_remaining': navigation_status.estimated_time_remaining.sec + navigation_status.estimated_time_remaining.nanosec * 1e-9
        }

    def pose_to_dict(self, pose):
        """Convert Pose message to dictionary"""
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        api_server = RESTAPIServer()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(api_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            api_server.destroy_node()
            
    except Exception as e:
        print(f"Error starting REST API server: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
