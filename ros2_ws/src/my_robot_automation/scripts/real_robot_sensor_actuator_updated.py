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
        
        # 1. IR Distance sensors (6x Sharp GP2Y0A02YK0F - wall alignment) - 2 per side
        self.distance_left_front_pub = self.create_publisher(Range, '/distance/left_front', 10)
        self.distance_left_back_pub = self.create_publisher(Range, '/distance/left_back', 10)
        self.distance_right_front_pub = self.create_publisher(Range, '/distance/right_front', 10)
        self.distance_right_back_pub = self.create_publisher(Range, '/distance/right_back', 10)
        self.distance_back_left_pub = self.create_publisher(Range, '/distance/back_left', 10)
        self.distance_back_right_pub = self.create_publisher(Range, '/distance/back_right', 10)

        # 2. HC-SR04 Ultrasonic sensors (2x front)
        self.ultrasonic_front_left_pub = self.create_publisher(Range, '/ultrasonic/front_left', 10)
        self.ultrasonic_front_right_pub = self.create_publisher(Range, '/ultrasonic/front_right', 10)

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
        self.create_timer(0.066, self.publish_laser_distance_sensors)  # 15Hz
        self.create_timer(0.1, self.publish_ultrasonic_sensors)         # 10Hz
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
        
        self.get_logger().info('Real Robot Sensor/Actuator Node started - Updated for notes.txt configuration')
        self.get_logger().info('SENSORS: IR Distance Sharp GP2Y0A02YK0F (6x), HC-SR04 Ultrasonic (2x), Line Sensors (3x), TF-Luna, MPU6050 IMU, Gripper Camera')
        self.get_logger().info('ACTUATORS: 3x Omni Wheels (lf,rf,b), Gripper System (motor+servos), 4x Containers')
        self.get_logger().info('CONTROLS: Emergency, Start/Stop, Mode (Train/Run)')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for 3x Omni wheels: Left Front, Right Front, Back"""
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop active - ignoring velocity commands')
            return

        # Extract desired velocities
        linear_x = msg.linear.x    # Forward/backward
        linear_y = msg.linear.y    # Left/right (lateral movement)
        angular_z = msg.angular.z  # Rotation

        # Omni wheel kinematics for 3-wheeled robot (lf, rf, b configuration)
        # Robot coordinate system: x=forward, y=left, z=up
        # Wheel positions: lf at (d, w/2), rf at (d, -w/2), b at (-d, 0)
        # where d = wheel distance from center, w = wheel separation

        # Wheel velocities calculation for omni wheels
        # V_wheel = V_robot + ω × r_wheel_to_center
        # For 3 omni wheels in triangular configuration

        wheel_base = 0.3  # Distance from center to wheels
        wheel_separation = 0.25  # Distance between front wheels

        # Calculate individual wheel velocities
        # Left Front wheel
        v_lf = linear_x - linear_y + angular_z * wheel_base

        # Right Front wheel
        v_rf = linear_x + linear_y - angular_z * wheel_base

        # Back wheel (center rear)
        v_b = linear_x - angular_z * wheel_base

        # Update robot position using odometry
        dt = 0.1
        self.robot_x += linear_x * dt * math.cos(self.robot_theta) - linear_y * dt * math.sin(self.robot_theta)
        self.robot_y += linear_x * dt * math.sin(self.robot_theta) + linear_y * dt * math.cos(self.robot_theta)
        self.robot_theta += angular_z * dt

        # Normalize theta to [-pi, pi]
        self.robot_theta = math.atan2(math.sin(self.robot_theta), math.cos(self.robot_theta))

        self.get_logger().info(f'Omni wheels (LF, RF, B): linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}')
        self.get_logger().info(f'Wheel velocities: LF={v_lf:.2f}, RF={v_rf:.2f}, B={v_b:.2f}')
        self.get_logger().info(f'Robot pose: x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}')
    
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
    
    def publish_laser_distance_sensors(self):
        """Publish IR distance sensor data (6x Sharp GP2Y0A02YK0F - wall alignment)"""
        # Left front IR sensor (Sharp GP2Y0A02YK0F)
        left_front_range = Range()
        left_front_range.header.stamp = self.get_clock().now().to_msg()
        left_front_range.header.frame_id = 'ir_sensor_left_front_link'
        left_front_range.radiation_type = Range.INFRARED
        left_front_range.field_of_view = 0.087  # ~5 degrees
        left_front_range.min_range = 0.02
        left_front_range.max_range = 4.0
        left_front_range.range = 1.8 + 0.3 * math.sin(time.time() * 0.8)
        self.distance_left_front_pub.publish(left_front_range)

        # Left back IR sensor (Sharp GP2Y0A02YK0F)
        left_back_range = Range()
        left_back_range.header.stamp = self.get_clock().now().to_msg()
        left_back_range.header.frame_id = 'ir_sensor_left_back_link'
        left_back_range.radiation_type = Range.INFRARED
        left_back_range.field_of_view = 0.087
        left_back_range.min_range = 0.02
        left_back_range.max_range = 4.0
        left_back_range.range = 1.5  # Closer to wall
        self.distance_left_back_pub.publish(left_back_range)

        # Right front IR sensor (Sharp GP2Y0A02YK0F)
        right_front_range = Range()
        right_front_range.header.stamp = self.get_clock().now().to_msg()
        right_front_range.header.frame_id = 'ir_sensor_right_front_link'
        right_front_range.radiation_type = Range.INFRARED
        right_front_range.field_of_view = 0.087
        right_front_range.min_range = 0.02
        right_front_range.max_range = 4.0
        right_front_range.range = 2.2 + 0.4 * math.sin(time.time() * 0.6)
        self.distance_right_front_pub.publish(right_front_range)

        # Right back IR sensor (Sharp GP2Y0A02YK0F)
        right_back_range = Range()
        right_back_range.header.stamp = self.get_clock().now().to_msg()
        right_back_range.header.frame_id = 'ir_sensor_right_back_link'
        right_back_range.radiation_type = Range.INFRARED
        right_back_range.field_of_view = 0.087
        right_back_range.min_range = 0.02
        right_back_range.max_range = 4.0
        right_back_range.range = 1.9  # Closer to wall
        self.distance_right_back_pub.publish(right_back_range)

        # Back left IR sensor (Sharp GP2Y0A02YK0F)
        back_left_range = Range()
        back_left_range.header.stamp = self.get_clock().now().to_msg()
        back_left_range.header.frame_id = 'ir_sensor_back_left_link'
        back_left_range.radiation_type = Range.INFRARED
        back_left_range.field_of_view = 0.087
        back_left_range.min_range = 0.02
        back_left_range.max_range = 4.0
        back_left_range.range = 3.5  # Back wall
        self.distance_back_left_pub.publish(back_left_range)

        # Back right IR sensor (Sharp GP2Y0A02YK0F)
        back_right_range = Range()
        back_right_range.header.stamp = self.get_clock().now().to_msg()
        back_right_range.header.frame_id = 'ir_sensor_back_right_link'
        back_right_range.radiation_type = Range.INFRARED
        back_right_range.field_of_view = 0.087
        back_right_range.min_range = 0.02
        back_right_range.max_range = 4.0
        back_right_range.range = 3.2  # Back wall
        self.distance_back_right_pub.publish(back_right_range)
    
    def publish_ultrasonic_sensors(self):
        """Publish HC-SR04 ultrasonic sensor data (2x front)"""
        # Front left ultrasonic sensor
        front_left_range = Range()
        front_left_range.header.stamp = self.get_clock().now().to_msg()
        front_left_range.header.frame_id = 'hcsr04_front_left_link'
        front_left_range.radiation_type = Range.ULTRASOUND
        front_left_range.field_of_view = 0.2618  # ~15 degrees
        front_left_range.min_range = 0.02
        front_left_range.max_range = 4.0
        front_left_range.range = 2.8 + 0.6 * math.sin(time.time() * 0.4)
        self.ultrasonic_front_left_pub.publish(front_left_range)

        # Front right ultrasonic sensor
        front_right_range = Range()
        front_right_range.header.stamp = self.get_clock().now().to_msg()
        front_right_range.header.frame_id = 'hcsr04_front_right_link'
        front_right_range.radiation_type = Range.ULTRASOUND
        front_right_range.field_of_view = 0.2618  # ~15 degrees
        front_right_range.min_range = 0.02
        front_right_range.max_range = 4.0
        front_right_range.range = 3.1 + 0.5 * math.sin(time.time() * 0.5 + 0.5)
        self.ultrasonic_front_right_pub.publish(front_right_range)

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
