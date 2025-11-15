#!/usr/bin/env python3
"""
Serial Interface for Arduino Mega Communication
Handles motor control and sensor data communication with Arduino Mega
"""

import serial
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Range

class MegaSerialInterface(Node):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        super().__init__('mega_serial_interface')

        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connected = False
        self.last_command_time = time.time()

        # Command mapping for Mega
        self.cmd_mapping = {
            # Movement commands
            'forward': 'f',
            'backward': 'b',
            'strafe_left': 'l',
            'strafe_right': 'r',
            'forward_left': 'q',
            'forward_right': 'e',
            'backward_left': 'z',
            'backward_right': 'x',
            'rotate_clockwise': 'c',
            'rotate_counter_clockwise': 'w',
            'turn_left': 't',
            'turn_right': 'y',
            'diagonal_left': 'a',
            'diagonal_right': 'j',

            # Control commands
            'stop': 's',
            'emergency_stop': 'v',

            # Lifter commands
            'lift_up': 'u',
            'lift_down': 'd',

            # Gripper commands
            'gripper_open': 'no',
            'gripper_close': 'nc',
            'gripper_tilt_up': 'mu',
            'gripper_tilt_down': 'md',

            # Sensor commands
            'sensor_readings': 'sr'
        }

        # Speed commands (5-0 for 50%-100%)
        self.speed_mapping = {
            0.5: '5',
            0.6: '6',
            0.7: '7',
            0.8: '8',
            0.9: '9',
            1.0: '0'
        }

        # Publishers for sensor data from Mega
        self.distance_left_front_pub = self.create_publisher(Range, '/distance/left_front', 10)
        self.distance_left_back_pub = self.create_publisher(Range, '/distance/left_back', 10)
        self.distance_right_front_pub = self.create_publisher(Range, '/distance/right_front', 10)
        self.distance_right_back_pub = self.create_publisher(Range, '/distance/right_back', 10)
        self.distance_back_left_pub = self.create_publisher(Range, '/distance/back_left', 10)
        self.distance_right_back_pub = self.create_publisher(Range, '/distance/back_right', 10)

        self.ultrasonic_front_left_pub = self.create_publisher(Range, '/ultrasonic/front_left', 10)
        self.ultrasonic_front_right_pub = self.create_publisher(Range, '/ultrasonic/front_right', 10)

        # Subscribers for actuator commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.gripper_sub = self.create_subscription(
            Bool, '/picker/gripper', self.gripper_callback, 10)

        self.gripper_tilt_sub = self.create_subscription(
            Float32, '/picker/gripper_tilt', self.gripper_tilt_callback, 10)

        self.emergency_sub = self.create_subscription(
            Bool, '/hardware/emergency', self.emergency_callback, 10)

        # Timer for reading sensor data
        self.create_timer(0.1, self.read_sensor_data)  # 10Hz

        # Connect to Mega
        self.connect_to_mega()

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()

        self.get_logger().info(f'Mega Serial Interface initialized on {port}')

    def connect_to_mega(self):
        """Connect to Arduino Mega via serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # Wait for connection
            self.connected = True
            self.get_logger().info(f'Connected to Arduino Mega on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Mega: {e}')
            self.connected = False

    def send_command(self, command):
        """Send command to Mega"""
        if not self.connected:
            self.get_logger().warn('Not connected to Mega, attempting reconnect...')
            self.connect_to_mega()
            return

        try:
            if isinstance(command, str):
                self.serial_conn.write(command.encode())
            else:
                self.serial_conn.write(command)

            self.serial_conn.write(b'\n')  # Send newline
            self.serial_conn.flush()
            self.last_command_time = time.time()

            self.get_logger().debug(f'Sent command: {command}')

        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            self.connected = False

    def serial_reader(self):
        """Background thread to read serial data from Mega"""
        while rclpy.ok():
            if self.serial_conn and self.connected:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode().strip()
                        self.process_serial_data(line)
                except Exception as e:
                    self.get_logger().error(f'Serial read error: {e}')
                    self.connected = False
                    time.sleep(1)  # Wait before retry
            else:
                time.sleep(0.1)

    def process_serial_data(self, line):
        """Process incoming data from Mega"""
        # This will handle sensor data responses from Mega
        # Format: "IR_LF:245.5,IR_LB:238.2,US_FL:85.4,US_FR:92.1"
        if line.startswith('SENSORS:'):
            self.parse_sensor_data(line[8:])  # Remove 'SENSORS:' prefix

    def parse_sensor_data(self, data):
        """Parse sensor data from Mega"""
        try:
            sensors = data.split(',')
            for sensor in sensors:
                if ':' in sensor:
                    sensor_type, value = sensor.split(':', 1)
                    self.publish_sensor_data(sensor_type, float(value))
        except Exception as e:
            self.get_logger().error(f'Failed to parse sensor data: {e}')

    def publish_sensor_data(self, sensor_type, value):
        """Publish sensor data to ROS topics"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = f"{sensor_type.lower()}_frame"
        range_msg.radiation_type = Range.INFRARED if sensor_type.startswith('IR') else Range.ULTRASOUND
        range_msg.field_of_view = 0.1  # 10 degrees
        range_msg.min_range = 0.02 if sensor_type.startswith('IR') else 0.02  # 2cm min
        range_msg.max_range = 1.5 if sensor_type.startswith('IR') else 4.0   # 1.5m for IR, 4m for US

        if sensor_type == 'IR_LF':
            range_msg.range = value / 100.0  # Convert mm to meters
            self.distance_left_front_pub.publish(range_msg)
        elif sensor_type == 'IR_LB':
            range_msg.range = value / 100.0
            self.distance_left_back_pub.publish(range_msg)
        elif sensor_type == 'IR_RF':
            range_msg.range = value / 100.0
            self.distance_right_front_pub.publish(range_msg)
        elif sensor_type == 'IR_RB':
            range_msg.range = value / 100.0
            self.distance_right_back_pub.publish(range_msg)
        elif sensor_type == 'IR_BL':
            range_msg.range = value / 100.0
            self.distance_back_left_pub.publish(range_msg)
        elif sensor_type == 'IR_BR':
            range_msg.range = value / 100.0
            self.distance_right_back_pub.publish(range_msg)
        elif sensor_type == 'US_FL':
            range_msg.range = value / 100.0  # Convert cm to meters
            self.ultrasonic_front_left_pub.publish(range_msg)
        elif sensor_type == 'US_FR':
            range_msg.range = value / 100.0
            self.ultrasonic_front_right_pub.publish(range_msg)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands and translate to Mega commands"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Determine primary movement direction
        if abs(linear_x) > abs(linear_y) and abs(linear_x) > abs(angular_z):
            # Forward/backward movement
            if linear_x > 0.1:
                self.send_command(self.cmd_mapping['forward'])
            elif linear_x < -0.1:
                self.send_command(self.cmd_mapping['backward'])
        elif abs(linear_y) > abs(angular_z):
            # Strafe movement
            if linear_y > 0.1:
                self.send_command(self.cmd_mapping['strafe_right'])
            elif linear_y < -0.1:
                self.send_command(self.cmd_mapping['strafe_left'])
        elif abs(angular_z) > 0.1:
            # Rotation
            if angular_z > 0:
                self.send_command(self.cmd_mapping['rotate_counter_clockwise'])
            else:
                self.send_command(self.cmd_mapping['rotate_clockwise'])
        else:
            # Stop if no significant movement
            self.send_command(self.cmd_mapping['stop'])

    def gripper_callback(self, msg):
        """Handle gripper open/close commands"""
        if msg.data:
            self.send_command(self.cmd_mapping['gripper_open'])
        else:
            self.send_command(self.cmd_mapping['gripper_close'])

    def gripper_tilt_callback(self, msg):
        """Handle gripper tilt commands"""
        angle = msg.data
        if angle > 0:
            self.send_command(self.cmd_mapping['gripper_tilt_up'])
        else:
            self.send_command(self.cmd_mapping['gripper_tilt_down'])

    def emergency_callback(self, msg):
        """Handle emergency stop"""
        if msg.data:
            self.send_command(self.cmd_mapping['emergency_stop'])

    def read_sensor_data(self):
        """Request sensor data from Mega"""
        if self.connected:
            self.send_command(self.cmd_mapping['sensor_readings'])

    def set_speed(self, speed_multiplier):
        """Set speed multiplier (0.5 to 1.0)"""
        if speed_multiplier in self.speed_mapping:
            self.send_command(self.speed_mapping[speed_multiplier])

def main(args=None):
    rclpy.init(args=args)
    node = MegaSerialInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
