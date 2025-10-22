#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Range, JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String
import math
import time

class RealRobotSensorActuator(Node):
    def __init__(self):
        super().__init__('real_robot_sensor_actuator')
        
        # Publishers for REAL sensor data based on notes.txt hardware configuration
        
        # 1. Distance sensors (laser base) - Front, Back Left, Back Right
        self.distance_front_pub = self.create_publisher(Range, '/distance/front', 10)
        self.distance_back_left_pub = self.create_publisher(Range, '/distance/back_left', 10)
        self.distance_back_right_pub = self.create_publisher(Range, '/distance/back_right', 10)
        
        # 2. Other sensors from notes.txt
        # - 380 degree lidar sensor (RPLIDAR A1)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # - Microsoft camera (USB) for object recognition
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # - Line sensor for line-based navigation
        self.line_sensor_pub = self.create_publisher(Int32, '/line_sensor/raw', 10)
        
        # - IMU sensor (MPU6050/BNO055)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Joint states for all actuators
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers for actuator commands based on notes.txt configuration
        
        # 1. Omni wheels: Back, Front Left, Front Right
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 2. Picker system components
        # - Gripper (servo)
        self.gripper_sub = self.create_subscription(
            Bool, '/picker/gripper', self.gripper_callback, 10)
        
        # - Gripper tilt (servo)
        self.gripper_tilt_sub = self.create_subscription(
            Float32, '/picker/gripper_tilt', self.gripper_tilt_callback, 10)
        
        # - Gripper neck (forward/backward, servo continuous)
        self.gripper_neck_sub = self.create_subscription(
            Float32, '/picker/gripper_neck', self.gripper_neck_callback, 10)
        
        # - Gripper base (up/down, motor)
        self.gripper_base_sub = self.create_subscription(
            Float32, '/picker/gripper_base', self.gripper_base_callback, 10)
        
        # 3. Container load system (4 containers)
        # - Left front, left back, right front, right back
        self.container_left_front_sub = self.create_subscription(
            String, '/containers/left_front', self.container_left_front_callback, 10)
        self.container_left_back_sub = self.create_subscription(
            String, '/containers/left_back', self.container_left_back_callback, 10)
        self.container_right_front_sub = self.create_subscription(
            String, '/containers/right_front', self.container_right_front_callback, 10)
        self.container_right_back_sub = self.create_subscription(
            String, '/containers/right_back', self.container_right_back_callback, 10)
        
        # 4. Hardware controls
        # - Emergency, start/stop, mode (train/run)
        self.emergency_sub = self.create_subscription(
            Bool, '/hardware/emergency', self.emergency_callback, 10)
        self.start_stop_sub = self.create_subscription(
            Bool, '/hardware/start_stop', self.start_stop_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/hardware/mode', self.mode_callback, 10)
        
        # Timers for publishing sensor data
        self.create_timer(0.1, self.publish_distance_sensors)   # 10Hz
        self.create_timer(0.1, self.publish_lidar)             # 10Hz
        self.create_timer(0.033, self.publish_camera)          # 30Hz
        self.create_timer(0.2, self.publish_line_sensor)        # 5Hz
        self.create_timer(0.1, self.publish_imu)                # 10Hz
        self.create_timer(0.1, self.publish_joint_states)       # 10Hz
        
        # State variables based on notes.txt configuration
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0
        
        # Picker system state
        self.gripper_open = False
        self.gripper_tilt_angle = 0.0
        self.gripper_neck_position = 0.0
        self.gripper_base_height = 0.0
        
        # Container system state
        self.container_left_front_load = False
        self.container_left_back_load = False
        self.container_right_front_load = False
        self.container_right_back_load = False
        
        # Hardware control state
        self.emergency_stop = False
        self.robot_running = False
        self.robot_mode = "run"  # "train" or "run"
        
        self.get_logger().info('Real Robot Sensor/Actuator Node started - Updated for notes.txt configuration')
        self.get_logger().info('SENSORS: Distance (3x), RPLIDAR A1, Microsoft Camera, Line Sensor, IMU')
        self.get_logger().info('ACTUATORS: 3x Omni Wheels, Picker System (4 components), 4x Containers')
        self.get_logger().info('CONTROLS: Emergency, Start/Stop, Mode (Train/Run)')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for 3x Omni wheels: Back, Front Left, Front Right"""
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop active - ignoring velocity commands')
            return
            
        # Omni wheel kinematics simulation for hexagonal robot
        linear_x = msg.linear.x
        linear_y = msg.linear.y  # Omni wheels support lateral movement
        angular_z = msg.angular.z
        
        # Update robot position (omni wheel integration)
        dt = 0.1
        self.robot_x += linear_x * dt
        self.robot_y += linear_y * dt
        self.robot_theta += angular_z * dt
        
        self.get_logger().info(f'Omni wheels (Back, Front Left, Front Right): linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}')
        self.get_logger().info(f'Robot position: x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}')
    
    def gripper_callback(self, msg):
        """Handle gripper commands (servo)"""
        self.gripper_open = msg.data
        action = "opened" if self.gripper_open else "closed"
        self.get_logger().info(f'Gripper {action}')
    
    def gripper_tilt_callback(self, msg):
        """Handle gripper tilt commands (servo)"""
        self.gripper_tilt_angle = msg.data
        self.get_logger().info(f'Gripper tilt angle: {self.gripper_tilt_angle:.2f} degrees')
    
    def gripper_neck_callback(self, msg):
        """Handle gripper neck commands (servo continuous)"""
        self.gripper_neck_position = msg.data
        self.get_logger().info(f'Gripper neck position: {self.gripper_neck_position:.2f}')
    
    def gripper_base_callback(self, msg):
        """Handle gripper base commands (motor)"""
        self.gripper_base_height = msg.data
        self.get_logger().info(f'Gripper base height: {self.gripper_base_height:.2f}')
    
    def container_left_front_callback(self, msg):
        """Handle left front container commands"""
        action = msg.data
        self.container_left_front_load = (action == "load")
        self.get_logger().info(f'Left front container: {action}')
    
    def container_left_back_callback(self, msg):
        """Handle left back container commands"""
        action = msg.data
        self.container_left_back_load = (action == "load")
        self.get_logger().info(f'Left back container: {action}')
    
    def container_right_front_callback(self, msg):
        """Handle right front container commands"""
        action = msg.data
        self.container_right_front_load = (action == "load")
        self.get_logger().info(f'Right front container: {action}')
    
    def container_right_back_callback(self, msg):
        """Handle right back container commands"""
        action = msg.data
        self.container_right_back_load = (action == "load")
        self.get_logger().info(f'Right back container: {action}')
    
    def emergency_callback(self, msg):
        """Handle emergency stop commands"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().error('EMERGENCY STOP ACTIVATED!')
        else:
            self.get_logger().info('Emergency stop released')
    
    def start_stop_callback(self, msg):
        """Handle start/stop commands"""
        self.robot_running = msg.data
        action = "started" if self.robot_running else "stopped"
        self.get_logger().info(f'Robot {action}')
    
    def mode_callback(self, msg):
        """Handle mode commands (train/run)"""
        self.robot_mode = msg.data
        self.get_logger().info(f'Robot mode set to: {self.robot_mode}')
    
    def publish_distance_sensors(self):
        """Publish distance sensor data (laser base) - Front, Back Left, Back Right"""
        # Front distance sensor
        front_range = Range()
        front_range.header.stamp = self.get_clock().now().to_msg()
        front_range.header.frame_id = 'distance_front'
        front_range.radiation_type = Range.LASER
        front_range.field_of_view = 0.1  # ~5.7 degrees
        front_range.min_range = 0.02
        front_range.max_range = 4.0
        front_range.range = 2.0 + 0.5 * math.sin(time.time())  # Simulated obstacle
        self.distance_front_pub.publish(front_range)
        
        # Back left distance sensor
        back_left_range = Range()
        back_left_range.header.stamp = self.get_clock().now().to_msg()
        back_left_range.header.frame_id = 'distance_back_left'
        back_left_range.radiation_type = Range.LASER
        back_left_range.field_of_view = 0.1
        back_left_range.min_range = 0.02
        back_left_range.max_range = 4.0
        back_left_range.range = 1.5  # Wall on back left
        self.distance_back_left_pub.publish(back_left_range)
        
        # Back right distance sensor
        back_right_range = Range()
        back_right_range.header.stamp = self.get_clock().now().to_msg()
        back_right_range.header.frame_id = 'distance_back_right'
        back_right_range.radiation_type = Range.LASER
        back_right_range.field_of_view = 0.1
        back_right_range.min_range = 0.02
        back_right_range.max_range = 4.0
        back_right_range.range = 1.8  # Wall on back right
        self.distance_back_right_pub.publish(back_right_range)
    
    def publish_lidar(self):
        """Publish RPLIDAR A1 380-degree LIDAR data"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'rplidar_a1'
        
        # RPLIDAR A1 parameters (380 degrees)
        scan.angle_min = -math.pi * 190/180  # -190 degrees
        scan.angle_max = math.pi * 190/180   # +190 degrees
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.range_min = 0.1
        scan.range_max = 12.0  # RPLIDAR A1 range
        
        # Generate LIDAR scan data for hexagonal robot environment
        ranges = []
        for i in range(381):  # 380 degrees + 1
            angle = scan.angle_min + i * scan.angle_increment
            
            # Create realistic environment for hexagonal robot
            if abs(angle) < 0.3:  # Front wall
                range_val = 2.5 + 0.2 * math.sin(time.time())
            elif abs(angle - math.pi/2) < 0.2:  # Left wall
                range_val = 1.8
            elif abs(angle + math.pi/2) < 0.2:  # Right wall
                range_val = 2.2
            elif abs(angle - math.pi) < 0.1:  # Back wall
                range_val = 4.0
            else:
                range_val = 8.0 + 3.0 * math.sin(time.time() * 0.1 + angle)
            
            ranges.append(range_val)
        
        scan.ranges = ranges
        self.lidar_pub.publish(scan)
    
    def publish_camera(self):
        """Publish Microsoft USB camera data for object recognition"""
        # Create dummy image data for object recognition
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'microsoft_camera'
        image.height = 480
        image.width = 640
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = 640 * 3  # width * channels
        
        # Generate dummy image data (simplified for object recognition)
        dummy_data = [128] * (640 * 480 * 3)  # Gray image
        image.data = dummy_data
        
        self.camera_pub.publish(image)
    
    def publish_line_sensor(self):
        """Publish line sensor data for line-based navigation"""
        line_data = Int32()
        # Simulate line sensor reading (bit pattern for multiple sensors)
        # 0b11111111 = all sensors see line, 0b00000000 = no line detected
        line_data.data = int(0b10101010 + 0b00000010 * math.sin(time.time()))
        self.line_sensor_pub.publish(line_data)
    
    def publish_imu(self):
        """Publish IMU sensor data (MPU6050/BNO055)"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        
        # Simulate IMU data
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(time.time() * 0.1)
        imu.orientation.w = math.cos(time.time() * 0.1)
        
        # Angular velocity
        imu.angular_velocity.x = 0.1 * math.sin(time.time())
        imu.angular_velocity.y = 0.1 * math.cos(time.time())
        imu.angular_velocity.z = 0.05 * math.sin(time.time() * 0.5)
        
        # Linear acceleration
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81  # Gravity
        
        self.imu_pub.publish(imu)
    
    def publish_joint_states(self):
        """Publish joint states for all actuators based on notes.txt configuration"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # All actuator joints based on notes.txt
        joint_state.name = [
            # 3x Omni wheel motors: Back, Front Left, Front Right
            'omni_wheel_back', 'omni_wheel_front_left', 'omni_wheel_front_right',
            # Picker system components
            'gripper_servo', 'gripper_tilt_servo', 'gripper_neck_servo', 'gripper_base_motor',
            # Container system (4 containers)
            'container_left_front', 'container_left_back', 'container_right_front', 'container_right_back'
        ]
        
        # Joint positions
        joint_state.position = [
            # Omni wheel positions (simulated encoder readings)
            self.robot_x + 0.1 * math.sin(time.time() * 0.5),  # back wheel
            self.robot_x - 0.1 * math.sin(time.time() * 0.5),  # front left wheel  
            self.robot_y + 0.1 * math.cos(time.time() * 0.3),  # front right wheel
            # Picker system positions
            float(self.gripper_open),  # gripper servo
            self.gripper_tilt_angle,   # gripper tilt servo
            self.gripper_neck_position, # gripper neck servo
            self.gripper_base_height,  # gripper base motor
            # Container positions
            float(self.container_left_front_load),   # left front container
            float(self.container_left_back_load),    # left back container
            float(self.container_right_front_load),  # right front container
            float(self.container_right_back_load)    # right back container
        ]
        
        # Joint velocities (simulated)
        joint_state.velocity = [
            # Omni wheel velocities
            0.1 * math.cos(time.time() * 0.5),   # back wheel
            -0.1 * math.cos(time.time() * 0.5),  # front left wheel
            0.1 * math.sin(time.time() * 0.3),   # front right wheel
            # Picker system velocities
            0.0, 0.0, 0.0, 0.0  # picker system velocities
        ] + [0.0] * 4  # container velocities
        
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
