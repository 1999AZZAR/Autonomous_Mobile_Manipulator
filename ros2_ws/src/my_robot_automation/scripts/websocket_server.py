#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio
import websockets
import json
import threading
import time
from datetime import datetime
import uuid

# ROS2 imports
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, JointState, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

# Custom imports
from my_robot_automation.msg import RobotStatus, SafetyStatus, AutomationTask
from my_robot_automation.srv import (
    ExecutePickPlace, ExecutePatrol, ExecuteObstacleAvoidance,
    GetRobotStatus, EmergencyStop, SetRobotMode
)

class WebSocketServer(Node):
    """WebSocket server for real-time robot communication"""
    
    def __init__(self):
        super().__init__('websocket_server')
        
        # WebSocket server
        self.websocket_server = None
        self.clients = set()
        self.client_lock = threading.Lock()
        
        # Robot state
        self.robot_status = None
        self.safety_status = None
        self.latest_laser_scan = None
        self.latest_odometry = None
        self.latest_joint_states = None
        
        # Publishers for control commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Subscribers for robot data
        self.robot_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.robot_status_callback, 10
        )
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, 'safety_status', self.safety_status_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10
        )
        
        # Service clients
        self.pick_place_client = self.create_client(ExecutePickPlace, 'execute_pick_place')
        self.patrol_client = self.create_client(ExecutePatrol, 'execute_patrol')
        self.obstacle_avoidance_client = self.create_client(
            ExecuteObstacleAvoidance, 'execute_obstacle_avoidance'
        )
        self.robot_status_client = self.create_client(GetRobotStatus, 'get_robot_status')
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop')
        self.set_mode_client = self.create_client(SetRobotMode, 'set_robot_mode')
        
        # Wait for services
        self.wait_for_services()
        
        # Start WebSocket server
        self.start_websocket_server()
        
        # Timer for periodic updates
        self.update_timer = self.create_timer(0.1, self.broadcast_updates)
        
        self.get_logger().info('WebSocket Server started on port 8765')
        
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
            
    def start_websocket_server(self):
        """Start WebSocket server in separate thread"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.websocket_handler())
            
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
    async def websocket_handler(self):
        """WebSocket connection handler"""
        async def handle_client(websocket, path):
            """Handle individual client connection"""
            client_id = str(uuid.uuid4())
            self.get_logger().info(f'Client connected: {client_id}')
            
            with self.client_lock:
                self.clients.add(websocket)
                
            try:
                # Send initial robot status
                if self.robot_status:
                    await websocket.send(json.dumps({
                        'type': 'robot_status',
                        'data': self.robot_status_to_dict(self.robot_status)
                    }))
                    
                if self.safety_status:
                    await websocket.send(json.dumps({
                        'type': 'safety_status',
                        'data': self.safety_status_to_dict(self.safety_status)
                    }))
                
                # Handle incoming messages
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await self.handle_client_message(websocket, data)
                    except json.JSONDecodeError:
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'message': 'Invalid JSON format'
                        }))
                    except Exception as e:
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'message': str(e)
                        }))
                        
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f'Client disconnected: {client_id}')
            finally:
                with self.client_lock:
                    self.clients.discard(websocket)
                    
        self.websocket_server = await websockets.serve(
            handle_client, "0.0.0.0", 8765
        )
        await self.websocket_server.wait_closed()
        
    async def handle_client_message(self, websocket, data):
        """Handle incoming client message"""
        message_type = data.get('type', '')
        
        if message_type == 'velocity_command':
            await self.handle_velocity_command(data.get('data', {}))
            await websocket.send(json.dumps({
                'type': 'command_ack',
                'message': 'Velocity command executed'
            }))
            
        elif message_type == 'emergency_stop':
            await self.handle_emergency_stop(data.get('data', {}))
            await websocket.send(json.dumps({
                'type': 'command_ack',
                'message': 'Emergency stop command executed'
            }))
            
        elif message_type == 'pick_place':
            await self.handle_pick_place(websocket, data.get('data', {}))
            
        elif message_type == 'patrol':
            await self.handle_patrol(websocket, data.get('data', {}))
            
        elif message_type == 'obstacle_avoidance':
            await self.handle_obstacle_avoidance(websocket, data.get('data', {}))
            
        elif message_type == 'set_mode':
            await self.handle_set_mode(websocket, data.get('data', {}))
            
        elif message_type == 'get_status':
            await self.send_robot_status(websocket)
            
        else:
            await websocket.send(json.dumps({
                'type': 'error',
                'message': f'Unknown message type: {message_type}'
            }))
            
    async def handle_velocity_command(self, data):
        """Handle velocity command from client"""
        cmd_vel = Twist()
        cmd_vel.linear.x = data.get('linear_x', 0.0)
        cmd_vel.linear.y = data.get('linear_y', 0.0)
        cmd_vel.angular.z = data.get('angular_z', 0.0)
        
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Published velocity command: {cmd_vel}')
        
    async def handle_emergency_stop(self, data):
        """Handle emergency stop command"""
        activate = data.get('activate', True)
        reason = data.get('reason', 'WebSocket emergency stop')
        
        request = EmergencyStop.Request()
        request.activate = activate
        request.reason = reason
        request.force_stop = data.get('force', False)
        
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Emergency stop response: {response.message}')
        else:
            self.get_logger().error('Emergency stop service call failed')
            
    async def handle_pick_place(self, websocket, data):
        """Handle pick and place command"""
        try:
            request = ExecutePickPlace.Request()
            request.pickup_location = self.dict_to_pose_stamped(data.get('pickup_location', {}))
            request.place_location = self.dict_to_pose_stamped(data.get('place_location', {}))
            request.object_type = data.get('object_type', 'unknown')
            request.object_height = data.get('object_height', 0.05)
            request.gripper_force = data.get('gripper_force', 10.0)
            request.timeout_seconds = data.get('timeout_seconds', 60.0)
            
            future = self.pick_place_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=request.timeout_seconds + 10)
            
            if future.result() is not None:
                response = future.result()
                await websocket.send(json.dumps({
                    'type': 'pick_place_result',
                    'success': response.success,
                    'message': response.message,
                    'execution_time': response.execution_time,
                    'task_id': response.task_id
                }))
            else:
                await websocket.send(json.dumps({
                    'type': 'pick_place_result',
                    'success': False,
                    'message': 'Service call failed'
                }))
                
        except Exception as e:
            await websocket.send(json.dumps({
                'type': 'pick_place_result',
                'success': False,
                'message': str(e)
            }))
            
    async def handle_patrol(self, websocket, data):
        """Handle patrol command"""
        try:
            request = ExecutePatrol.Request()
            
            waypoints = []
            for wp in data.get('waypoints', []):
                waypoints.append(self.dict_to_pose_stamped(wp))
            request.waypoints = waypoints
            
            request.patrol_speed = data.get('patrol_speed', 0.5)
            request.patrol_cycles = data.get('patrol_cycles', 1)
            request.return_to_start = data.get('return_to_start', True)
            request.timeout_seconds = data.get('timeout_seconds', 300.0)
            
            future = self.patrol_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=request.timeout_seconds + 10)
            
            if future.result() is not None:
                response = future.result()
                await websocket.send(json.dumps({
                    'type': 'patrol_result',
                    'success': response.success,
                    'message': response.message,
                    'execution_time': response.execution_time,
                    'completed_cycles': response.completed_cycles,
                    'task_id': response.task_id
                }))
            else:
                await websocket.send(json.dumps({
                    'type': 'patrol_result',
                    'success': False,
                    'message': 'Service call failed'
                }))
                
        except Exception as e:
            await websocket.send(json.dumps({
                'type': 'patrol_result',
                'success': False,
                'message': str(e)
            }))
            
    async def handle_obstacle_avoidance(self, websocket, data):
        """Handle obstacle avoidance command"""
        try:
            request = ExecuteObstacleAvoidance.Request()
            request.target_location = self.dict_to_pose_stamped(data.get('target_location', {}))
            request.avoidance_distance = data.get('avoidance_distance', 0.5)
            request.max_speed = data.get('max_speed', 0.5)
            request.enable_dynamic_avoidance = data.get('enable_dynamic_avoidance', True)
            request.timeout_seconds = data.get('timeout_seconds', 120.0)
            
            future = self.obstacle_avoidance_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=request.timeout_seconds + 10)
            
            if future.result() is not None:
                response = future.result()
                await websocket.send(json.dumps({
                    'type': 'obstacle_avoidance_result',
                    'success': response.success,
                    'message': response.message,
                    'execution_time': response.execution_time,
                    'obstacles_avoided': response.obstacles_avoided,
                    'task_id': response.task_id
                }))
            else:
                await websocket.send(json.dumps({
                    'type': 'obstacle_avoidance_result',
                    'success': False,
                    'message': 'Service call failed'
                }))
                
        except Exception as e:
            await websocket.send(json.dumps({
                'type': 'obstacle_avoidance_result',
                'success': False,
                'message': str(e)
            }))
            
    async def handle_set_mode(self, websocket, data):
        """Handle set robot mode command"""
        try:
            request = SetRobotMode.Request()
            request.mode = data.get('mode', 'MANUAL')
            request.reason = data.get('reason', 'WebSocket command')
            request.force_mode_change = data.get('force', False)
            
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                await websocket.send(json.dumps({
                    'type': 'set_mode_result',
                    'success': response.success,
                    'message': response.message,
                    'previous_mode': response.previous_mode,
                    'current_mode': response.current_mode
                }))
            else:
                await websocket.send(json.dumps({
                    'type': 'set_mode_result',
                    'success': False,
                    'message': 'Service call failed'
                }))
                
        except Exception as e:
            await websocket.send(json.dumps({
                'type': 'set_mode_result',
                'success': False,
                'message': str(e)
            }))
            
    async def send_robot_status(self, websocket):
        """Send current robot status to client"""
        if self.robot_status:
            await websocket.send(json.dumps({
                'type': 'robot_status',
                'data': self.robot_status_to_dict(self.robot_status)
            }))
            
    def robot_status_callback(self, msg):
        """Update robot status"""
        self.robot_status = msg
        
    def safety_status_callback(self, msg):
        """Update safety status"""
        self.safety_status = msg
        
    def laser_scan_callback(self, msg):
        """Update laser scan data"""
        self.latest_laser_scan = msg
        
    def odometry_callback(self, msg):
        """Update odometry data"""
        self.latest_odometry = msg
        
    def joint_states_callback(self, msg):
        """Update joint states"""
        self.latest_joint_states = msg
        
    def broadcast_updates(self):
        """Broadcast updates to all connected clients"""
        if not self.clients:
            return
            
        # Broadcast robot status
        if self.robot_status:
            message = json.dumps({
                'type': 'robot_status',
                'timestamp': datetime.now().isoformat(),
                'data': self.robot_status_to_dict(self.robot_status)
            })
            self.broadcast_message(message)
            
        # Broadcast safety status
        if self.safety_status:
            message = json.dumps({
                'type': 'safety_status',
                'timestamp': datetime.now().isoformat(),
                'data': self.safety_status_to_dict(self.safety_status)
            })
            self.broadcast_message(message)
            
        # Broadcast laser scan (throttled)
        if self.latest_laser_scan and hasattr(self, '_last_laser_time'):
            if time.time() - self._last_laser_time > 0.1:  # 10 Hz
                message = json.dumps({
                    'type': 'laser_scan',
                    'timestamp': datetime.now().isoformat(),
                    'data': self.laser_scan_to_dict(self.latest_laser_scan)
                })
                self.broadcast_message(message)
                self._last_laser_time = time.time()
        elif self.latest_laser_scan:
            self._last_laser_time = time.time()
            
        # Broadcast odometry
        if self.latest_odometry:
            message = json.dumps({
                'type': 'odometry',
                'timestamp': datetime.now().isoformat(),
                'data': self.odometry_to_dict(self.latest_odometry)
            })
            self.broadcast_message(message)
            
    def broadcast_message(self, message):
        """Broadcast message to all connected clients"""
        with self.client_lock:
            disconnected_clients = set()
            for client in self.clients:
                try:
                    # Use asyncio to send message
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(client.send(message))
                    loop.close()
                except:
                    disconnected_clients.add(client)
                    
            # Remove disconnected clients
            self.clients -= disconnected_clients
            
    def dict_to_pose_stamped(self, pose_dict):
        """Convert dictionary to PoseStamped message"""
        from geometry_msgs.msg import PoseStamped, Point, Quaternion
        
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
        
    def laser_scan_to_dict(self, laser_scan):
        """Convert LaserScan message to dictionary"""
        return {
            'angle_min': laser_scan.angle_min,
            'angle_max': laser_scan.angle_max,
            'angle_increment': laser_scan.angle_increment,
            'range_min': laser_scan.range_min,
            'range_max': laser_scan.range_max,
            'ranges': list(laser_scan.ranges)
        }
        
    def odometry_to_dict(self, odometry):
        """Convert Odometry message to dictionary"""
        return {
            'pose': self.pose_stamped_to_dict(odometry.pose.pose),
            'twist': {
                'linear': {
                    'x': odometry.twist.twist.linear.x,
                    'y': odometry.twist.twist.linear.y,
                    'z': odometry.twist.twist.linear.z
                },
                'angular': {
                    'x': odometry.twist.twist.angular.x,
                    'y': odometry.twist.twist.angular.y,
                    'z': odometry.twist.twist.angular.z
                }
            }
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        websocket_server = WebSocketServer()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(websocket_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            websocket_server.destroy_node()
            
    except Exception as e:
        print(f"Error starting WebSocket server: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
