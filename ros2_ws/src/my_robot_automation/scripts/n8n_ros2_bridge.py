#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import json
import time
import threading
from datetime import datetime
import requests
from flask import Flask, request, jsonify
from flask_cors import CORS

# ROS2 imports
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool

# Custom imports
from my_robot_automation.srv import (
    ExecutePickPlace, ExecutePatrol, ExecuteObstacleAvoidance,
    GetRobotStatus, EmergencyStop, SetRobotMode
)
from my_robot_automation.msg import RobotStatus, SafetyStatus

class N8nROS2Bridge(Node):
    """Bridge service between n8n workflows and ROS2 automation services"""
    
    def __init__(self):
        super().__init__('n8n_ros2_bridge')
        
        # Flask app setup for webhook endpoints
        self.app = Flask(__name__)
        CORS(self.app)
        
        # n8n webhook endpoints
        self.setup_webhook_endpoints()
        
        # Service clients
        self.pick_place_client = self.create_client(ExecutePickPlace, 'execute_pick_place')
        self.patrol_client = self.create_client(ExecutePatrol, 'execute_patrol')
        self.obstacle_avoidance_client = self.create_client(
            ExecuteObstacleAvoidance, 'execute_obstacle_avoidance'
        )
        self.robot_status_client = self.create_client(GetRobotStatus, 'get_robot_status')
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop')
        self.set_mode_client = self.create_client(SetRobotMode, 'set_robot_mode')
        
        # Publishers for direct control
        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Subscribers for status monitoring
        self.robot_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.robot_status_callback, 10
        )
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, 'safety_status', self.safety_status_callback, 10
        )
        
        # Robot state
        self.robot_status = None
        self.safety_status = None
        
        # Wait for services
        self.wait_for_services()
        
        # Start Flask server
        self.start_flask_server()
        
        self.get_logger().info('n8n-ROS2 Bridge started on port 5678')
        
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            self.pick_place_client,
            self.patrol_client,
            self.obstacle_avoidance_client,
            self.robot_status_client,
            self.emergency_stop_client,
            self.set_mode_client
        ]
        
        for service in services:
            self.get_logger().info(f'Waiting for service: {service.srv_name}')
            service.wait_for_service(timeout_sec=10.0)
            self.get_logger().info(f'Service {service.srv_name} is ready')
            
    def setup_webhook_endpoints(self):
        """Setup n8n webhook endpoints"""
        
        @self.app.route('/webhook/robot-control', methods=['POST'])
        def robot_control_webhook():
            """Handle robot movement commands from n8n"""
            try:
                data = request.get_json()
                command = data.get('command', '').lower()
                speed = float(data.get('speed', 0.5))
                
                cmd_vel = Twist()
                
                if command == 'forward':
                    cmd_vel.linear.x = speed
                elif command == 'backward':
                    cmd_vel.linear.x = -speed
                elif command == 'left':
                    cmd_vel.linear.y = speed
                elif command == 'right':
                    cmd_vel.linear.y = -speed
                elif command == 'rotate_cw':
                    cmd_vel.angular.z = -speed
                elif command == 'rotate_ccw':
                    cmd_vel.angular.z = speed
                elif command == 'stop':
                    cmd_vel.linear.x = 0.0
                    cmd_vel.linear.y = 0.0
                    cmd_vel.angular.z = 0.0
                else:
                    return jsonify({
                        'success': False,
                        'error': f'Unknown command: {command}'
                    }), 400
                
                self.cmd_vel_pub.publish(cmd_vel)
                
                return jsonify({
                    'success': True,
                    'message': f'Executed command: {command}',
                    'command': command,
                    'speed': speed
                })
                
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/emergency-stop', methods=['POST'])
        def emergency_stop_webhook():
            """Handle emergency stop from n8n"""
            try:
                data = request.get_json() or {}
                reason = data.get('reason', 'Emergency stop from n8n workflow')
                activate = data.get('emergency', True)
                
                request_msg = EmergencyStop.Request()
                request_msg.activate = activate
                request_msg.reason = reason
                request_msg.force_stop = data.get('force', False)
                
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
                    return jsonify({
                        'success': False,
                        'error': 'Emergency stop service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/robot/pick_place', methods=['POST'])
        def pick_place_webhook():
            """Handle pick and place operations from n8n"""
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'pickup_location' not in data or 'place_location' not in data:
                    return jsonify({
                        'success': False,
                        'error': 'Missing required fields: pickup_location, place_location'
                    }), 400
                
                request_msg = ExecutePickPlace.Request()
                
                # Convert locations to PoseStamped
                request_msg.pickup_location = self.dict_to_pose_stamped(data['pickup_location'])
                request_msg.place_location = self.dict_to_pose_stamped(data['place_location'])
                
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
                    return jsonify({
                        'success': False,
                        'error': 'Pick and place service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/robot/patrol', methods=['POST'])
        def patrol_webhook():
            """Handle patrol operations from n8n"""
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'waypoints' not in data:
                    return jsonify({
                        'success': False,
                        'error': 'Missing required field: waypoints'
                    }), 400
                
                request_msg = ExecutePatrol.Request()
                
                # Convert waypoints
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
                    return jsonify({
                        'success': False,
                        'error': 'Patrol service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/robot/obstacle_avoidance', methods=['POST'])
        def obstacle_avoidance_webhook():
            """Handle obstacle avoidance navigation from n8n"""
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'target_location' not in data:
                    return jsonify({
                        'success': False,
                        'error': 'Missing required field: target_location'
                    }), 400
                
                request_msg = ExecuteObstacleAvoidance.Request()
                
                # Set target location
                request_msg.target_location = self.dict_to_pose_stamped(data['target_location'])
                
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
                    return jsonify({
                        'success': False,
                        'error': 'Obstacle avoidance service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/robot/status', methods=['GET'])
        def robot_status_webhook():
            """Get robot status for n8n workflows"""
            try:
                request_msg = GetRobotStatus.Request()
                request_msg.include_diagnostics = True
                request_msg.include_safety_status = True
                
                future = self.robot_status_client.call_async(request_msg)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        return jsonify({
                            'success': True,
                            'robot_status': self.robot_status_to_dict(response.robot_status),
                            'safety_status': self.safety_status_to_dict(response.safety_status)
                        })
                    else:
                        return jsonify({
                            'success': False,
                            'error': response.message
                        }), 500
                else:
                    return jsonify({
                        'success': False,
                        'error': 'Robot status service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/webhook/robot/mode', methods=['POST'])
        def robot_mode_webhook():
            """Set robot mode from n8n workflows"""
            try:
                data = request.get_json()
                mode = data.get('mode', '').upper()
                reason = data.get('reason', 'Mode change from n8n workflow')
                
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
                    return jsonify({
                        'success': False,
                        'error': 'Set robot mode service call failed'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500
                
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """Health check endpoint"""
            return jsonify({
                'status': 'healthy',
                'timestamp': datetime.now().isoformat(),
                'service': 'n8n_ros2_bridge',
                'robot_status': self.robot_status is not None,
                'safety_status': self.safety_status is not None
            })
            
    def start_flask_server(self):
        """Start Flask server in separate thread"""
        def run_server():
            self.app.run(host='0.0.0.0', port=5678, debug=False, threaded=True)
            
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
    def robot_status_callback(self, msg):
        """Update robot status"""
        self.robot_status = msg
        
    def safety_status_callback(self, msg):
        """Update safety status"""
        self.safety_status = msg
        
    def dict_to_pose_stamped(self, pose_dict):
        """Convert dictionary to PoseStamped message"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = pose_dict.get('frame_id', 'map')
        
        position = pose_dict.get('position', {})
        pose.pose.position.x = position.get('x', 0.0)
        pose.pose.position.y = position.get('y', 0.0)
        pose.pose.position.z = position.get('z', 0.0)
        
        orientation = pose_dict.get('orientation', {})
        pose.pose.orientation.x = orientation.get('x', 0.0)
        pose.pose.orientation.y = orientation.get('y', 0.0)
        pose.pose.orientation.z = orientation.get('z', 0.0)
        pose.pose.orientation.w = orientation.get('w', 1.0)
        
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

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = N8nROS2Bridge()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(bridge)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            bridge.destroy_node()
            
    except Exception as e:
        print(f"Error starting n8n-ROS2 bridge: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
