#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, BatteryState
import math
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_control', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery', self.battery_callback, 10)
        
        # State variables
        self.current_scan = None
        self.battery_level = 100.0
        self.is_obstacle_detected = False
        
        # Demo control
        self.create_timer(2.0, self.demo_behavior)
        
        self.get_logger().info('Robot Controller started')
        self.get_logger().info('Publishing: /cmd_vel, /gripper_control, /goal_pose')
        self.get_logger().info('Subscribing: /scan, /battery')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        self.current_scan = msg
        
        # Simple obstacle detection
        if msg.ranges:
            min_range = min(msg.ranges)
            if min_range < 1.0:  # Obstacle within 1 meter
                self.is_obstacle_detected = True
                self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m!')
            else:
                self.is_obstacle_detected = False
    
    def battery_callback(self, msg):
        """Process battery data"""
        self.battery_level = msg.percentage
        if msg.percentage < 20:
            self.get_logger().warn(f'Low battery: {msg.percentage:.1f}%')
    
    def move_forward(self, speed=0.5):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Moving forward at {speed} m/s')
    
    def turn_left(self, angular_speed=0.5):
        """Turn robot left"""
        cmd = Twist()
        cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Turning left at {angular_speed} rad/s')
    
    def turn_right(self, angular_speed=0.5):
        """Turn robot right"""
        cmd = Twist()
        cmd.angular.z = -angular_speed
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Turning right at {angular_speed} rad/s')
    
    def stop(self):
        """Stop robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped')
    
    def open_gripper(self):
        """Open gripper"""
        cmd = Bool()
        cmd.data = True
        self.gripper_pub.publish(cmd)
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """Close gripper"""
        cmd = Bool()
        cmd.data = False
        self.gripper_pub.publish(cmd)
        self.get_logger().info('Gripper closed')
    
    def set_goal(self, x, y, z=0.0):
        """Set navigation goal"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Goal set: ({x}, {y}, {z})')
    
    def demo_behavior(self):
        """Demo autonomous behavior"""
        if self.battery_level < 10:
            self.get_logger().error('Critical battery level! Returning to base.')
            self.stop()
            return
        
        if self.is_obstacle_detected:
            self.get_logger().info('Avoiding obstacle...')
            self.stop()
            time.sleep(0.5)
            self.turn_right(0.3)
            time.sleep(1.0)
            self.stop()
        else:
            # Random demo behavior
            behavior = int(time.time()) % 6
            if behavior == 0:
                self.move_forward(0.3)
            elif behavior == 1:
                self.turn_left(0.2)
            elif behavior == 2:
                self.turn_right(0.2)
            elif behavior == 3:
                self.open_gripper()
            elif behavior == 4:
                self.close_gripper()
            elif behavior == 5:
                self.stop()

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
