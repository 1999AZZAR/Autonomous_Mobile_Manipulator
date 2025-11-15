#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Range, JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String
import math
import time
import serial
import threading

class RealRobotSensorActuator(Node):
    def __init__(self):
        super().__init__('real_robot_sensor_actuator')

        # Publishers for REAL sensor data based on notes.txt hardware configuration

        # REMOVED: IR Distance sensors (6x Sharp GP2Y0A02YK0F) - now handled by Arduino Mega
        # REMOVED: HC-SR04 Ultrasonic sensors (2x front) - now handled by Arduino Mega

        # 3. Line sensors (3x individual, assembled side by side)
        self.line_sensor_left_pub = self.create_publisher(Int32, '/line_sensor/left', 10)
        self.line_sensor_center_pub = self.create_publisher(Int32, '/line_sensor/center', 10)
        self.line_sensor_right_pub = self.create_publisher(Int32, '/line_sensor/right', 10)

        # 4. TF-Luna single-point lidar sensor
        self.tf_luna_pub = self.create_publisher(Range, '/tf_luna/range', 10)

        # 5. MPU6050 IMU sensor
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # 6. Gripper camera (USB based)
        self.gripper_camera_pub = self.create_publisher(Image, '/gripper/camera/image_raw', 10)

        # Joint states for all actuators
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Serial connection to Arduino Mega
        self.mega_serial = None
        self.mega_connected = False
        self.connect_to_mega()
        
        # Subscribers for actuator commands based on notes.txt configuration

        # 1. Omni wheels: Command sent to Arduino Mega via serial
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
        # REMOVED: IR distance sensors - now handled by Arduino Mega
        # REMOVED: HC-SR04 ultrasonic sensors - now handled by Arduino Mega
        self.create_timer(0.02, self.publish_line_sensors)              # 50Hz
        self.create_timer(0.1, self.publish_tf_luna)                    # 10Hz
        self.create_timer(0.1, self.publish_imu)                        # 10Hz
        self.create_timer(0.033, self.publish_gripper_camera)           # 30Hz
        self.create_timer(0.1, self.publish_joint_states)               # 10Hz
        
        # State variables based on notes.txt configuration
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0

        # TF-Luna sensor state
        self.tf_luna_distance = float('inf')
        self.tf_luna_strength = 0
        self.tf_luna_temperature = 0.0
        
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
        
        self.get_logger().info('Real Robot Sensor/Actuator Node started - Updated for distributed architecture')
        self.get_logger().info('SENSORS: Line Sensors (3x), TF-Luna LiDAR, MPU6050 IMU, Gripper Camera')
        self.get_logger().info('ACTUATORS: Commands sent to Arduino Mega via serial')
        self.get_logger().info('CONTROLS: Emergency, Start/Stop, Mode (Train/Run)')

    def connect_to_mega(self):
        """Connect to Arduino Mega via serial"""
        try:
            self.mega_serial = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # Wait for connection
            self.mega_connected = True
            self.get_logger().info('Connected to Arduino Mega on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Mega: {e}')
            self.mega_connected = False

    def send_command_to_mega(self, command):
        """Send command to Arduino Mega"""
        if not self.mega_connected:
            self.get_logger().warn('Not connected to Mega, attempting reconnect...')
            self.connect_to_mega()
            return

        try:
            self.mega_serial.write(command.encode())
            self.mega_serial.write(b'\n')
            self.mega_serial.flush()
            self.get_logger().debug(f'Sent to Mega: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command to Mega: {e}')
            self.mega_connected = False
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands and send to Arduino Mega"""
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop active - ignoring velocity commands')
            return

        # Extract desired velocities
        linear_x = msg.linear.x    # Forward/backward
        linear_y = msg.linear.y    # Left/right (lateral movement)
        angular_z = msg.angular.z  # Rotation

        # Determine primary movement and send appropriate command to Mega
        command = None

        if abs(linear_x) > 0.1:
            # Forward/backward movement
            command = 'f' if linear_x > 0 else 'b'
        elif abs(linear_y) > 0.1:
            # Strafe movement
            command = 'l' if linear_y > 0 else 'r'
        elif abs(angular_z) > 0.1:
            # Rotation
            command = 'w' if angular_z > 0 else 'c'
        else:
            # Stop
            command = 's'

        if command:
            self.send_command_to_mega(command)
            self.get_logger().debug(f'Sent velocity command: {command} (x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f})')
    
    def gripper_callback(self, msg):
        """Handle gripper commands and send to Mega"""
        self.gripper_open = msg.data
        command = 'no' if self.gripper_open else 'nc'  # no=open, nc=close
        self.send_command_to_mega(command)
        action = "opened" if self.gripper_open else "closed"
        self.get_logger().info(f'Gripper {action} - sent command: {command}')

    def gripper_tilt_callback(self, msg):
        """Handle gripper tilt commands and send to Mega"""
        self.gripper_tilt_angle = msg.data
        command = 'mu' if self.gripper_tilt_angle > 0 else 'md'  # mu=tilt up, md=tilt down
        self.send_command_to_mega(command)
        self.get_logger().info(f'Gripper tilt: {self.gripper_tilt_angle:.2f} degrees - sent command: {command}')
    
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
        """Handle emergency stop commands and send to Mega"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.send_command_to_mega('v')  # Emergency stop command
            self.get_logger().error('EMERGENCY STOP ACTIVATED - sent to Mega!')
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
    
    # REMOVED: publish_laser_distance_sensors() - IR sensors now handled by Arduino Mega
    # REMOVED: publish_ultrasonic_sensors() - HC-SR04 sensors now handled by Arduino Mega

    def publish_tf_luna(self):
        """Publish TF-Luna single-point LIDAR data"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'tf_luna'

        # TF-Luna specifications
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.087  # ~5 degrees in radians
        range_msg.min_range = 0.1
        range_msg.max_range = 8.0

        # Simulate TF-Luna sensor reading (obstacle detection)
        # In real implementation, this would read from actual TF-Luna sensor
        base_distance = 3.0 + 1.5 * math.sin(time.time() * 0.5)
        noise = 0.1 * (2 * math.sin(time.time() * 2.0) - 1)  # Add some noise
        self.tf_luna_distance = max(range_msg.min_range, min(range_msg.max_range, base_distance + noise))

        # Simulate signal strength and temperature
        self.tf_luna_strength = int(30000 + 5000 * math.sin(time.time() * 0.3))
        self.tf_luna_temperature = 25.0 + 5.0 * math.sin(time.time() * 0.1)

        if self.tf_luna_distance >= range_msg.min_range and self.tf_luna_distance <= range_msg.max_range:
            range_msg.range = self.tf_luna_distance
        else:
            range_msg.range = float('inf')

        self.tf_luna_pub.publish(range_msg)

    def publish_gripper_camera(self):
        """Publish gripper USB camera data for object recognition"""
        # Create dummy image data for object recognition
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'gripper_camera_link'
        image.height = 480
        image.width = 640
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = 640 * 3  # width * channels

        # Generate dummy image data (simplified for object recognition)
        dummy_data = [128] * (640 * 480 * 3)  # Gray image
        image.data = dummy_data

        self.gripper_camera_pub.publish(image)
    
    def publish_line_sensors(self):
        """Publish line sensor data for line-based navigation (3x sensors)"""
        # Left line sensor
        left_line_data = Int32()
        left_line_data.data = 1 if math.sin(time.time() * 2.0) > 0.3 else 0  # Detect line periodically
        self.line_sensor_left_pub.publish(left_line_data)

        # Center line sensor
        center_line_data = Int32()
        center_line_data.data = 1 if math.sin(time.time() * 2.0 + 0.5) > 0.2 else 0  # Slightly offset
        self.line_sensor_center_pub.publish(center_line_data)

        # Right line sensor
        right_line_data = Int32()
        right_line_data.data = 1 if math.sin(time.time() * 2.0 + 1.0) > 0.3 else 0  # More offset
        self.line_sensor_right_pub.publish(right_line_data)
    
    def publish_imu(self):
        """Publish MPU6050 IMU sensor data"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'mpu6050_link'
        
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
            # 3x Omni wheel motors: Left Front, Right Front, Back
            'wheel_lf_joint', 'wheel_rf_joint', 'wheel_b_joint',
            # Gripper system components
            'gripper_base_joint', 'gripper_tilt_joint', 'gripper_claw_joint', 'gripper_extension_joint',
            # Container system (4 containers)
            'container_left_front_joint', 'container_left_back_joint', 'container_right_front_joint', 'container_right_back_joint'
        ]
        
        # Joint positions
        joint_state.position = [
            # Omni wheel positions (simulated encoder readings)
            self.robot_x + 0.1 * math.sin(time.time() * 0.5),  # left front wheel
            self.robot_y + 0.1 * math.cos(time.time() * 0.3),  # right front wheel
            self.robot_x - 0.1 * math.sin(time.time() * 0.5),  # back wheel
            # Gripper system positions
            self.gripper_base_height,    # gripper base motor (up/down)
            self.gripper_tilt_angle,     # gripper tilt servo
            float(self.gripper_open),    # gripper claw servo (open/close)
            self.gripper_neck_position,  # gripper extension servo (forward/backward)
            # Container positions (simulated as binary: loaded/not loaded)
            float(self.container_left_front_load),   # left front container
            float(self.container_left_back_load),    # left back container
            float(self.container_right_front_load),  # right front container
            float(self.container_right_back_load)    # right back container
        ]

        # Joint velocities (simulated)
        joint_state.velocity = [
            # Omni wheel velocities
            0.1 * math.cos(time.time() * 0.5),   # left front wheel
            0.1 * math.sin(time.time() * 0.3),   # right front wheel
            -0.1 * math.cos(time.time() * 0.5),  # back wheel
            # Gripper system velocities
            0.0, 0.0, 0.0, 0.0,  # gripper system velocities (continuous joints)
            # Container velocities
            0.0, 0.0, 0.0, 0.0   # container velocities
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
