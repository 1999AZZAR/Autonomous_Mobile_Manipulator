#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
import time
import json
import uuid
from datetime import datetime

# ROS2 imports
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

# Custom imports
from my_robot_automation.msg import RobotStatus, AutomationTask, SafetyStatus
from my_robot_automation.srv import (
    ExecutePickPlace, ExecutePatrol, ExecuteObstacleAvoidance,
    GetRobotStatus, EmergencyStop, SetRobotMode
)

class RobotAutomationServer(Node):
    """Main automation server that coordinates all robot operations"""
    
    def __init__(self):
        super().__init__('robot_automation_server')
        
        # Robot state
        self.robot_mode = "MANUAL"
        self.robot_state = "IDLE"
        self.emergency_stop_active = False
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()
        self.battery_level = 100.0
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.active_tasks = []
        self.safety_status = SafetyStatus()
        
        # Task management
        self.task_lock = threading.Lock()
        self.active_task_id = None
        self.task_results = {}
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers
        self.robot_status_pub = self.create_publisher(
            RobotStatus, 'robot_status', qos_profile
        )
        self.safety_status_pub = self.create_publisher(
            SafetyStatus, 'safety_status', qos_profile
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'diff_drive_controller/cmd_vel_unstamped', 10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10
        )
        
        # Services
        self.pick_place_srv = self.create_service(
            ExecutePickPlace, 'execute_pick_place', self.execute_pick_place_callback
        )
        self.patrol_srv = self.create_service(
            ExecutePatrol, 'execute_patrol', self.execute_patrol_callback
        )
        self.obstacle_avoidance_srv = self.create_service(
            ExecuteObstacleAvoidance, 'execute_obstacle_avoidance', 
            self.execute_obstacle_avoidance_callback
        )
        self.robot_status_srv = self.create_service(
            GetRobotStatus, 'get_robot_status', self.get_robot_status_callback
        )
        self.emergency_stop_srv = self.create_service(
            EmergencyStop, 'emergency_stop', self.emergency_stop_callback
        )
        self.set_mode_srv = self.create_service(
            SetRobotMode, 'set_robot_mode', self.set_robot_mode_callback
        )
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)
        self.safety_timer = self.create_timer(0.5, self.update_safety_status)
        self.diagnostics_timer = self.create_timer(5.0, self.update_diagnostics)
        
        self.get_logger().info('Robot Automation Server started successfully')
        
    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
    def laser_callback(self, msg):
        """Update safety status from laser scan"""
        if msg.ranges:
            min_distance = min([r for r in msg.ranges if r > 0.01])
            self.safety_status.min_distance_to_obstacle = min_distance
            
            if min_distance < 0.3:
                self.safety_status.obstacle_detected = True
                self.safety_status.active_safety_warnings.append("Obstacle too close")
            else:
                self.safety_status.obstacle_detected = False
                
    def joint_states_callback(self, msg):
        """Update joint states"""
        pass  # Can be used for arm/gripper status
        
    def execute_pick_place_callback(self, request, response):
        """Execute pick and place operation"""
        if self.emergency_stop_active:
            response.success = False
            response.message = "Emergency stop active"
            return response
            
        task_id = str(uuid.uuid4())
        self.get_logger().info(f'Starting pick and place task: {task_id}')
        
        try:
            # Simulate pick and place operation
            start_time = time.time()
            
            # Navigate to pickup location
            if not self.navigate_to_pose(request.pickup_location):
                response.success = False
                response.message = "Failed to navigate to pickup location"
                return response
                
            # Pick up object
            if not self.pick_object(request.object_type, request.gripper_force):
                response.success = False
                response.message = "Failed to pick object"
                return response
                
            # Navigate to place location
            if not self.navigate_to_pose(request.place_location):
                response.success = False
                response.message = "Failed to navigate to place location"
                return response
                
            # Place object
            if not self.place_object():
                response.success = False
                response.message = "Failed to place object"
                return response
                
            response.success = True
            response.message = "Pick and place completed successfully"
            response.execution_time = time.time() - start_time
            response.final_pose = self.current_pose
            response.task_id = task_id
            
        except Exception as e:
            response.success = False
            response.message = f"Pick and place failed: {str(e)}"
            
        return response
        
    def execute_patrol_callback(self, request, response):
        """Execute patrol operation"""
        if self.emergency_stop_active:
            response.success = False
            response.message = "Emergency stop active"
            return response
            
        task_id = str(uuid.uuid4())
        self.get_logger().info(f'Starting patrol task: {task_id}')
        
        try:
            start_time = time.time()
            completed_cycles = 0
            
            for cycle in range(request.patrol_cycles):
                if self.emergency_stop_active:
                    break
                    
                for waypoint in request.waypoints:
                    if self.emergency_stop_active:
                        break
                        
                    # Navigate to waypoint
                    if not self.navigate_to_pose(waypoint):
                        response.success = False
                        response.message = f"Failed to reach waypoint {waypoint}"
                        return response
                        
                    time.sleep(1.0)  # Brief pause at waypoint
                    
                completed_cycles += 1
                
            if request.return_to_start and request.waypoints:
                self.navigate_to_pose(request.waypoints[0])
                
            response.success = True
            response.message = "Patrol completed successfully"
            response.execution_time = time.time() - start_time
            response.completed_cycles = completed_cycles
            response.final_pose = self.current_pose
            response.task_id = task_id
            
        except Exception as e:
            response.success = False
            response.message = f"Patrol failed: {str(e)}"
            
        return response
        
    def execute_obstacle_avoidance_callback(self, request, response):
        """Execute obstacle avoidance navigation"""
        if self.emergency_stop_active:
            response.success = False
            response.message = "Emergency stop active"
            return response
            
        task_id = str(uuid.uuid4())
        self.get_logger().info(f'Starting obstacle avoidance task: {task_id}')
        
        try:
            start_time = time.time()
            obstacles_avoided = 0
            
            # Navigate with obstacle avoidance
            if not self.navigate_with_obstacle_avoidance(
                request.target_location, 
                request.avoidance_distance,
                request.max_speed
            ):
                response.success = False
                response.message = "Failed to reach target with obstacle avoidance"
                return response
                
            response.success = True
            response.message = "Obstacle avoidance navigation completed"
            response.execution_time = time.time() - start_time
            response.final_pose = self.current_pose
            response.obstacles_avoided = obstacles_avoided
            response.task_id = task_id
            
        except Exception as e:
            response.success = False
            response.message = f"Obstacle avoidance failed: {str(e)}"
            
        return response
        
    def get_robot_status_callback(self, request, response):
        """Get current robot status"""
        response.robot_status = self.create_robot_status_msg()
        response.safety_status = self.safety_status
        response.success = True
        response.message = "Robot status retrieved successfully"
        return response
        
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop"""
        if request.activate:
            self.emergency_stop_active = True
            self.robot_mode = "EMERGENCY"
            self.robot_state = "EMERGENCY_STOP"
            
            # Stop robot movement
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            response.success = True
            response.message = f"Emergency stop activated: {request.reason}"
            response.emergency_stop_active = True
            response.stop_reason = request.reason
            response.stop_location = self.current_pose
            
            self.get_logger().warn(f'Emergency stop activated: {request.reason}')
        else:
            self.emergency_stop_active = False
            response.success = True
            response.message = "Emergency stop deactivated"
            response.emergency_stop_active = False
            
        return response
        
    def set_robot_mode_callback(self, request, response):
        """Set robot operating mode"""
        if self.emergency_stop_active and request.mode != "EMERGENCY":
            response.success = False
            response.message = "Cannot change mode while emergency stop is active"
            return response
            
        previous_mode = self.robot_mode
        self.robot_mode = request.mode
        
        if request.mode == "AUTONOMOUS":
            self.robot_state = "IDLE"
        elif request.mode == "MANUAL":
            self.robot_state = "IDLE"
        elif request.mode == "EMERGENCY":
            self.emergency_stop_active = True
            self.robot_state = "EMERGENCY_STOP"
            
        response.success = True
        response.message = f"Robot mode changed from {previous_mode} to {request.mode}"
        response.previous_mode = previous_mode
        response.current_mode = self.robot_mode
        
        self.get_logger().info(f'Robot mode changed: {previous_mode} -> {request.mode}')
        return response
        
    def navigate_to_pose(self, target_pose):
        """Navigate robot to target pose"""
        # Simplified navigation - in real implementation would use Nav2
        self.robot_state = "NAVIGATING"
        self.get_logger().info(f'Navigating to: {target_pose.pose.position}')
        
        # Simulate navigation time
        time.sleep(2.0)
        
        # Update current pose (simplified)
        self.current_pose = target_pose
        self.robot_state = "IDLE"
        return True
        
    def pick_object(self, object_type, gripper_force):
        """Pick up object with gripper"""
        self.robot_state = "MANIPULATING"
        self.get_logger().info(f'Picking object: {object_type}')
        
        # Simulate pick operation
        time.sleep(1.0)
        
        self.robot_state = "IDLE"
        return True
        
    def place_object(self):
        """Place object with gripper"""
        self.robot_state = "MANIPULATING"
        self.get_logger().info('Placing object')
        
        # Simulate place operation
        time.sleep(1.0)
        
        self.robot_state = "IDLE"
        return True
        
    def navigate_with_obstacle_avoidance(self, target_pose, avoidance_distance, max_speed):
        """Navigate with obstacle avoidance"""
        self.robot_state = "NAVIGATING"
        self.get_logger().info('Navigating with obstacle avoidance')
        
        # Simulate obstacle avoidance navigation
        time.sleep(3.0)
        
        self.robot_state = "IDLE"
        return True
        
    def create_robot_status_msg(self):
        """Create robot status message"""
        status = RobotStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "map"
        status.robot_name = "autonomous_mobile_manipulator"
        status.mode = self.robot_mode
        status.state = self.robot_state
        status.current_pose = self.current_pose
        status.current_velocity = self.current_velocity
        status.emergency_stop_active = self.emergency_stop_active
        status.safety_systems_ok = not self.emergency_stop_active
        status.battery_level = self.battery_level
        status.cpu_usage = self.cpu_usage
        status.memory_usage = self.memory_usage
        status.active_tasks = self.active_tasks
        
        # System operational status
        # Note: diagnostics field removed from RobotStatus message
        
        return status
        
    def update_safety_status(self):
        """Update safety status"""
        self.safety_status.header.stamp = self.get_clock().now().to_msg()
        self.safety_status.header.frame_id = "base_link"
        self.safety_status.emergency_stop_active = self.emergency_stop_active
        self.safety_status.last_safe_pose = self.current_pose
        
        # Clear old warnings
        if len(self.safety_status.active_safety_warnings) > 10:
            self.safety_status.active_safety_warnings = self.safety_status.active_safety_warnings[-5:]
            
        self.safety_status_pub.publish(self.safety_status)
        
    def publish_robot_status(self):
        """Publish robot status"""
        status = self.create_robot_status_msg()
        self.robot_status_pub.publish(status)
        
    def update_diagnostics(self):
        """Update system diagnostics"""
        # Simulate system diagnostics
        self.cpu_usage = 45.0 + (time.time() % 20)
        self.memory_usage = 60.0 + (time.time() % 15)
        self.battery_level = max(0, 100.0 - (time.time() % 100) * 0.1)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        automation_server = RobotAutomationServer()
        
        # Use multi-threaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(automation_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            automation_server.destroy_node()
            
    except Exception as e:
        print(f"Error starting automation server: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
