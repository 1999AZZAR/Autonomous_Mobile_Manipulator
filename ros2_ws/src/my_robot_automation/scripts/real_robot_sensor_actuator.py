#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Range, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32
import math
import time

class RealRobotSensorActuator(Node):
    def __init__(self):
        super().__init__('real_robot_sensor_actuator')
        
        # Publishers for REAL sensor data
        # 1. Ultrasonic sensors
        self.ultrasonic_front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.ultrasonic_back_pub = self.create_publisher(Range, '/ultrasonic/back', 10)
        self.ultrasonic_left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        
        # 2. Infrared (IR Sharp) sensors
        self.ir_front_pub = self.create_publisher(Range, '/ir/front', 10)
        self.ir_left_pub = self.create_publisher(Range, '/ir/left', 10)
        self.ir_right_pub = self.create_publisher(Range, '/ir/right', 10)
        
        # 3. Line sensors
        self.line_sensor_pub = self.create_publisher(Int32, '/line_sensor/raw', 10)
        
        # 4. LIDAR
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # 5. USB Camera
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Joint states for all actuators
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers for actuator commands
        # 1. Omni wheels (3x DC Motors with encoders)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 2. Lifter/Forklift (1x DC Motor with encoder)
        self.lifter_sub = self.create_subscription(
            Float32, '/lifter/position', self.lifter_callback, 10)
        
        # 3. Servo motors (5x)
        self.servo_sub = self.create_subscription(
            JointState, '/servo/command', self.servo_callback, 10)
        
        # Timers for publishing sensor data
        self.create_timer(0.1, self.publish_ultrasonic_sensors)  # 10Hz
        self.create_timer(0.05, self.publish_ir_sensors)         # 20Hz
        self.create_timer(0.2, self.publish_line_sensor)         # 5Hz
        self.create_timer(0.1, self.publish_lidar)              # 10Hz
        self.create_timer(0.033, self.publish_camera)           # 30Hz
        self.create_timer(0.1, self.publish_joint_states)       # 10Hz
        
        # State variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0
        self.lifter_position = 0.0
        self.servo_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # 5 servos
        
        self.get_logger().info('Real Robot Sensor/Actuator Node started')
        self.get_logger().info('SENSORS: Ultrasonic, IR Sharp, Line, LIDAR, USB Camera')
        self.get_logger().info('ACTUATORS: 3x Omni DC Motors, 1x Lifter DC Motor, 5x Servos')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for 3x Omni wheels"""
        # Omni wheel kinematics simulation
        linear_x = msg.linear.x
        linear_y = msg.linear.y  # Omni wheels support lateral movement
        angular_z = msg.angular.z
        
        # Update robot position (omni wheel integration)
        dt = 0.1
        self.robot_x += linear_x * dt
        self.robot_y += linear_y * dt
        self.robot_theta += angular_z * dt
        
        self.get_logger().info(f'Omni wheels: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}')
        self.get_logger().info(f'Robot position: x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}')
    
    def lifter_callback(self, msg):
        """Handle lifter/forklift position commands"""
        target_position = msg.data
        self.lifter_position = target_position
        self.get_logger().info(f'Lifter moved to position: {self.lifter_position:.2f}')
    
    def servo_callback(self, msg):
        """Handle servo motor commands"""
        if len(msg.position) >= 5:
            self.servo_positions = msg.position[:5]
            servo_names = ['servo_1', 'servo_2', 'servo_3', 'servo_4', 'servo_5']
            for i, pos in enumerate(self.servo_positions):
                self.get_logger().info(f'{servo_names[i]} position: {pos:.2f}')
    
    def publish_ultrasonic_sensors(self):
        """Publish ultrasonic sensor data"""
        # Front ultrasonic
        front_range = Range()
        front_range.header.stamp = self.get_clock().now().to_msg()
        front_range.header.frame_id = 'ultrasonic_front'
        front_range.radiation_type = Range.ULTRASOUND
        front_range.field_of_view = 0.1  # ~5.7 degrees
        front_range.min_range = 0.02
        front_range.max_range = 4.0
        front_range.range = 2.0 + 0.5 * math.sin(time.time())  # Simulated obstacle
        self.ultrasonic_front_pub.publish(front_range)
        
        # Back ultrasonic
        back_range = Range()
        back_range.header.stamp = self.get_clock().now().to_msg()
        back_range.header.frame_id = 'ultrasonic_back'
        back_range.radiation_type = Range.ULTRASOUND
        back_range.field_of_view = 0.1
        back_range.min_range = 0.02
        back_range.max_range = 4.0
        back_range.range = 3.5  # No obstacle behind
        self.ultrasonic_back_pub.publish(back_range)
        
        # Left ultrasonic
        left_range = Range()
        left_range.header.stamp = self.get_clock().now().to_msg()
        left_range.header.frame_id = 'ultrasonic_left'
        left_range.radiation_type = Range.ULTRASOUND
        left_range.field_of_view = 0.1
        left_range.min_range = 0.02
        left_range.max_range = 4.0
        left_range.range = 1.5  # Wall on left
        self.ultrasonic_left_pub.publish(left_range)
        
        # Right ultrasonic
        right_range = Range()
        right_range.header.stamp = self.get_clock().now().to_msg()
        right_range.header.frame_id = 'ultrasonic_right'
        right_range.radiation_type = Range.ULTRASOUND
        right_range.field_of_view = 0.1
        right_range.min_range = 0.02
        right_range.max_range = 4.0
        right_range.range = 1.8  # Wall on right
        self.ultrasonic_right_pub.publish(right_range)
    
    def publish_ir_sensors(self):
        """Publish IR Sharp sensor data"""
        # Front IR
        front_ir = Range()
        front_ir.header.stamp = self.get_clock().now().to_msg()
        front_ir.header.frame_id = 'ir_front'
        front_ir.radiation_type = Range.INFRARED
        front_ir.field_of_view = 0.05  # ~2.9 degrees
        front_ir.min_range = 0.04
        front_ir.max_range = 0.8
        front_ir.range = 0.3 + 0.1 * math.sin(time.time())  # Close obstacle
        self.ir_front_pub.publish(front_ir)
        
        # Left IR
        left_ir = Range()
        left_ir.header.stamp = self.get_clock().now().to_msg()
        left_ir.header.frame_id = 'ir_left'
        left_ir.radiation_type = Range.INFRARED
        left_ir.field_of_view = 0.05
        left_ir.min_range = 0.04
        left_ir.max_range = 0.8
        left_ir.range = 0.6  # No close obstacle
        self.ir_left_pub.publish(left_ir)
        
        # Right IR
        right_ir = Range()
        right_ir.header.stamp = self.get_clock().now().to_msg()
        right_ir.header.frame_id = 'ir_right'
        right_ir.radiation_type = Range.INFRARED
        right_ir.field_of_view = 0.05
        right_ir.min_range = 0.04
        right_ir.max_range = 0.8
        right_ir.range = 0.4  # Some obstacle
        self.ir_right_pub.publish(right_ir)
    
    def publish_line_sensor(self):
        """Publish line sensor data"""
        line_data = Int32()
        # Simulate line sensor reading (bit pattern for multiple sensors)
        # 0b11111111 = all sensors see line, 0b00000000 = no line detected
        line_data.data = int(0b10101010 + 0b00000010 * math.sin(time.time()))
        self.line_sensor_pub.publish(line_data)
    
    def publish_lidar(self):
        """Publish LIDAR data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar'
        
        # LIDAR parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.range_min = 0.1
        scan.range_max = 30.0  # Longer range than ultrasonic
        
        # Generate LIDAR scan data
        ranges = []
        for i in range(361):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Create realistic environment
            if abs(angle) < 0.3:  # Front wall
                range_val = 2.5 + 0.2 * math.sin(time.time())
            elif abs(angle - math.pi/2) < 0.2:  # Left wall
                range_val = 1.8
            elif abs(angle + math.pi/2) < 0.2:  # Right wall
                range_val = 2.2
            elif abs(angle - math.pi) < 0.1:  # Back wall
                range_val = 4.0
            else:
                range_val = 15.0 + 5.0 * math.sin(time.time() * 0.1 + angle)
            
            ranges.append(range_val)
        
        scan.ranges = ranges
        self.lidar_pub.publish(scan)
    
    def publish_camera(self):
        """Publish USB camera data"""
        # Create dummy image data
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'camera'
        image.height = 480
        image.width = 640
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = 640 * 3  # width * channels
        
        # Generate dummy image data (simplified)
        dummy_data = [128] * (640 * 480 * 3)  # Gray image
        image.data = dummy_data
        
        self.camera_pub.publish(image)
    
    def publish_joint_states(self):
        """Publish joint states for all actuators"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # All actuator joints
        joint_state.name = [
            # 3x Omni wheel motors with encoders
            'omni_wheel_right', 'omni_wheel_left', 'omni_wheel_back',
            # 1x Lifter/Forklift motor with encoder
            'lifter_motor',
            # 5x Servo motors
            'servo_1', 'servo_2', 'servo_3', 'servo_4', 'servo_5'
        ]
        
        # Joint positions
        joint_state.position = [
            # Omni wheel positions (simulated encoder readings)
            self.robot_x + 0.1 * math.sin(time.time() * 0.5),  # right wheel
            self.robot_x - 0.1 * math.sin(time.time() * 0.5),  # left wheel  
            self.robot_y + 0.1 * math.cos(time.time() * 0.3),  # back wheel
            # Lifter position
            self.lifter_position,
            # Servo positions
            self.servo_positions[0], self.servo_positions[1], self.servo_positions[2],
            self.servo_positions[3], self.servo_positions[4]
        ]
        
        # Joint velocities (simulated)
        joint_state.velocity = [
            # Omni wheel velocities
            0.1 * math.cos(time.time() * 0.5),   # right wheel
            -0.1 * math.cos(time.time() * 0.5),  # left wheel
            0.1 * math.sin(time.time() * 0.3),   # back wheel
            # Lifter velocity
            0.0,  # lifter velocity
            # Servo velocities
            0.0, 0.0, 0.0, 0.0, 0.0  # servo velocities
        ]
        
        # Joint efforts (simulated)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotSensorActuator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
