#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, BatteryState, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import math
import time

class DummySensorActuator(Node):
    def __init__(self):
        super().__init__('dummy_sensor_actuator')
        
        # Publishers for sensor data
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers for actuator commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.gripper_sub = self.create_subscription(
            Bool, '/gripper_control', self.gripper_callback, 10)
        
        # Timers for publishing sensor data
        self.create_timer(0.1, self.publish_laser_scan)
        self.create_timer(0.05, self.publish_imu)
        self.create_timer(1.0, self.publish_battery)
        self.create_timer(0.1, self.publish_joint_states)
        
        # State variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0
        self.gripper_open = True
        
        self.get_logger().info('Dummy Sensor/Actuator Node started')
        self.get_logger().info('Publishing: /scan, /imu, /battery, /joint_states')
        self.get_logger().info('Subscribing: /cmd_vel, /gripper_control')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        # Simple kinematics simulation
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Update robot position (simple integration)
        dt = 0.1
        self.robot_x += linear_vel * math.cos(self.robot_theta) * dt
        self.robot_y += linear_vel * math.sin(self.robot_theta) * dt
        self.robot_theta += angular_vel * dt
        
        self.get_logger().info(f'Robot moved to: x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}')
    
    def gripper_callback(self, msg):
        """Handle gripper commands"""
        self.gripper_open = msg.data
        status = "opened" if self.gripper_open else "closed"
        self.get_logger().info(f'Gripper {status}')
    
    def publish_laser_scan(self):
        """Publish dummy laser scan data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        # Laser parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Generate dummy scan data with some obstacles
        ranges = []
        for i in range(361):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Create some dummy obstacles
            if abs(angle) < 0.5:  # Front obstacle
                range_val = 2.0 + 0.5 * math.sin(time.time())
            elif abs(angle - math.pi/2) < 0.3:  # Left obstacle
                range_val = 1.5
            elif abs(angle + math.pi/2) < 0.3:  # Right obstacle
                range_val = 1.8
            else:
                range_val = 8.0 + 2.0 * math.sin(time.time() + angle)
            
            ranges.append(range_val)
        
        scan.ranges = ranges
        self.laser_pub.publish(scan)
    
    def publish_imu(self):
        """Publish dummy IMU data"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu'
        
        # Dummy IMU data with some noise
        imu.orientation.x = 0.1 * math.sin(time.time())
        imu.orientation.y = 0.05 * math.cos(time.time())
        imu.orientation.z = 0.02 * math.sin(time.time() * 2)
        imu.orientation.w = 1.0
        
        imu.angular_velocity.x = 0.01 * math.sin(time.time())
        imu.angular_velocity.y = 0.02 * math.cos(time.time())
        imu.angular_velocity.z = 0.01 * math.sin(time.time() * 1.5)
        
        imu.linear_acceleration.x = 0.1 * math.sin(time.time() * 0.5)
        imu.linear_acceleration.y = 0.05 * math.cos(time.time() * 0.7)
        imu.linear_acceleration.z = 9.81  # Gravity
        
        self.imu_pub.publish(imu)
    
    def publish_battery(self):
        """Publish battery status"""
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = 'battery'
        
        # Slowly decrease battery level
        self.battery_level -= 0.01
        if self.battery_level < 0:
            self.battery_level = 100.0
        
        battery.percentage = self.battery_level
        battery.voltage = 12.6 * (self.battery_level / 100.0)
        battery.current = -2.5  # Charging current
        
        if self.battery_level < 20:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_LOW
        elif self.battery_level < 10:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CRITICAL
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_GOOD
        
        self.battery_pub.publish(battery)
    
    def publish_joint_states(self):
        """Publish joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # Arm joints
        joint_state.name = [
            'base_joint', 'shoulder_joint', 'elbow_joint', 
            'wrist_joint', 'gripper_joint'
        ]
        
        # Dummy joint positions with some movement
        joint_state.position = [
            0.1 * math.sin(time.time() * 0.5),      # base
            0.5 + 0.2 * math.sin(time.time() * 0.3), # shoulder
            -0.3 + 0.1 * math.cos(time.time() * 0.4), # elbow
            0.2 * math.sin(time.time() * 0.6),      # wrist
            0.0 if self.gripper_open else 1.0       # gripper
        ]
        
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DummySensorActuator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
