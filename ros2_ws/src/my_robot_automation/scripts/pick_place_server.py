#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
import time

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        
        # Publishers for picker system control based on notes.txt
        self.gripper_pub = self.create_publisher(Bool, '/picker/gripper', 10)
        self.gripper_tilt_pub = self.create_publisher(Float32, '/picker/gripper_tilt', 10)
        self.gripper_neck_pub = self.create_publisher(Float32, '/picker/gripper_neck', 10)
        self.gripper_base_pub = self.create_publisher(Float32, '/picker/gripper_base', 10)
        
        # Publishers for container system
        self.container_left_front_pub = self.create_publisher(String, '/containers/left_front', 10)
        self.container_left_back_pub = self.create_publisher(String, '/containers/left_back', 10)
        self.container_right_front_pub = self.create_publisher(String, '/containers/right_front', 10)
        self.container_right_back_pub = self.create_publisher(String, '/containers/right_back', 10)
        
        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.gripper_open = False
        self.gripper_tilt_angle = 0.0
        self.gripper_neck_position = 0.0
        self.gripper_base_height = 0.0
        
        self.get_logger().info('Pick Place Server started - Updated for notes.txt configuration')
        self.get_logger().info('Picker System: Gripper, Gripper Tilt, Gripper Neck, Gripper Base')
        self.get_logger().info('Container System: 4 containers (left/right, front/back)')
    
    def pick_object(self, object_location):
        """Execute pick operation using picker system"""
        self.get_logger().info(f'Starting pick operation at {object_location}')
        
        # 1. Navigate to object location
        self.navigate_to_location(object_location)
        
        # 2. Lower gripper base
        self.lower_gripper_base()
        
        # 3. Open gripper
        self.open_gripper()
        
        # 4. Position gripper over object
        self.position_gripper_over_object()
        
        # 5. Close gripper
        self.close_gripper()
        
        # 6. Lift gripper base
        self.lift_gripper_base()
        
        self.get_logger().info('Pick operation completed')
    
    def place_object(self, target_location):
        """Execute place operation using picker system"""
        self.get_logger().info(f'Starting place operation at {target_location}')
        
        # 1. Navigate to target location
        self.navigate_to_location(target_location)
        
        # 2. Lower gripper base
        self.lower_gripper_base()
        
        # 3. Open gripper to release object
        self.open_gripper()
        
        # 4. Lift gripper base
        self.lift_gripper_base()
        
        self.get_logger().info('Place operation completed')
    
    def navigate_to_location(self, location):
        """Navigate robot to specified location"""
        cmd = Twist()
        # Simple navigation - would be replaced with proper path planning
        cmd.linear.x = 0.3
        self.cmd_vel_pub.publish(cmd)
        time.sleep(2.0)  # Simulate navigation time
        
        # Stop robot
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Navigated to {location}')
    
    def open_gripper(self):
        """Open gripper (servo)"""
        cmd = Bool()
        cmd.data = True
        self.gripper_pub.publish(cmd)
        self.gripper_open = True
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """Close gripper (servo)"""
        cmd = Bool()
        cmd.data = False
        self.gripper_pub.publish(cmd)
        self.gripper_open = False
        self.get_logger().info('Gripper closed')
    
    def set_gripper_tilt(self, angle):
        """Set gripper tilt angle (servo)"""
        cmd = Float32()
        cmd.data = angle
        self.gripper_tilt_pub.publish(cmd)
        self.gripper_tilt_angle = angle
        self.get_logger().info(f'Gripper tilt set to {angle} degrees')
    
    def set_gripper_neck_position(self, position):
        """Set gripper neck position (servo continuous)"""
        cmd = Float32()
        cmd.data = position
        self.gripper_neck_pub.publish(cmd)
        self.gripper_neck_position = position
        self.get_logger().info(f'Gripper neck position set to {position}')
    
    def set_gripper_base_height(self, height):
        """Set gripper base height (motor)"""
        cmd = Float32()
        cmd.data = height
        self.gripper_base_pub.publish(cmd)
        self.gripper_base_height = height
        self.get_logger().info(f'Gripper base height set to {height}')
    
    def lower_gripper_base(self):
        """Lower gripper base to pick position"""
        self.set_gripper_base_height(0.0)
        time.sleep(1.0)  # Wait for movement
    
    def lift_gripper_base(self):
        """Lift gripper base to transport position"""
        self.set_gripper_base_height(0.5)
        time.sleep(1.0)  # Wait for movement
    
    def position_gripper_over_object(self):
        """Position gripper over object"""
        # Adjust gripper tilt for better grip
        self.set_gripper_tilt(15.0)
        time.sleep(0.5)
        
        # Adjust gripper neck position
        self.set_gripper_neck_position(0.0)
        time.sleep(0.5)
    
    def load_container(self, container_id, load=True):
        """Load or unload container"""
        action = "load" if load else "unload"
        
        if container_id == "left_front":
            self.container_left_front_pub.publish(String(data=action))
        elif container_id == "left_back":
            self.container_left_back_pub.publish(String(data=action))
        elif container_id == "right_front":
            self.container_right_front_pub.publish(String(data=action))
        elif container_id == "right_back":
            self.container_right_back_pub.publish(String(data=action))
        
        self.get_logger().info(f'Container {container_id}: {action}')

def main(args=None):
    rclpy.init(args=args)
    server = PickPlaceServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
