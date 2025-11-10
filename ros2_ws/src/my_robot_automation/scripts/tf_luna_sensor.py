#!/usr/bin/env python3

"""
TF-Luna LiDAR Sensor ROS2 Node
Integrates TF-Luna single-point LIDAR sensor with ROS2
Publishes sensor_msgs/Range messages for obstacle detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time
import threading
import math
import random

class TFLunaSensor(Node):
    def __init__(self):
        super().__init__('tf_luna_sensor')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'tf_luna')
        self.declare_parameter('field_of_view', 0.087)  # ~5 degrees in radians
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()

        # Create publisher
        self.range_publisher = self.create_publisher(Range, 'tf_luna/range', 10)

        # Initialize sensor data
        self.last_distance = float('inf')
        self.last_strength = 0
        self.last_temperature = 0.0
        self.data_lock = threading.Lock()

        # Create timer for publishing (includes data reading)
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_range)

        self.get_logger().info(f'TF-Luna sensor initialized on {self.serial_port} at {self.baud_rate} baud')
        self.get_logger().info(f'Publishing to topic: tf_luna/range at {self.publish_rate} Hz')

    def connect_serial(self):
        """Establish serial connection to TF-Luna"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Successfully connected to TF-Luna on {self.serial_port}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Failed to connect to TF-Luna: {e}. Using simulation mode.')
            self.serial_conn = None

    def read_tfluna_data(self):
        """Read data from TF-Luna sensor"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            # Simulation mode: generate realistic sensor data
            # Simulate distance with some variation (2-6 meters range)
            base_distance = 3.0 + 1.5 * math.sin(time.time() * 0.5)
            noise = 0.1 * (2 * math.sin(time.time() * 2.0) - 1)
            distance_m = max(0.1, min(8.0, base_distance + noise))

            # Simulate signal strength (20000-35000 range)
            strength = int(25000 + 5000 * math.sin(time.time() * 0.3))

            # Simulate temperature (20-35°C range)
            temperature_c = 25.0 + 5.0 * math.sin(time.time() * 0.1)

            return distance_m, strength, temperature_c

        try:
            counter = self.serial_conn.in_waiting
            if counter > 8:
                bytes_serial = self.serial_conn.read(9)
                self.serial_conn.reset_input_buffer()

                # Check header bytes (0x59 0x59)
                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                    # Parse distance (bytes 2-3, little endian)
                    distance = bytes_serial[2] + bytes_serial[3] * 256
                    distance_m = distance / 100.0  # Convert to meters

                    # Parse signal strength (bytes 4-5, little endian)
                    strength = bytes_serial[4] + bytes_serial[5] * 256

                    # Parse temperature (bytes 6-7, little endian)
                    temperature_raw = bytes_serial[6] + bytes_serial[7] * 256
                    temperature_c = (temperature_raw / 8.0) - 256.0

                    return distance_m, strength, temperature_c

        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            # Try to reconnect
            self.connect_serial()

        return None

    def publish_range(self):
        """Read sensor data and publish Range message"""
        # Read current sensor data
        data = self.read_tfluna_data()
        if data is not None:
            with self.data_lock:
                self.last_distance, self.last_strength, self.last_temperature = data

        with self.data_lock:
            distance = self.last_distance
            strength = self.last_strength
            temperature = self.last_temperature

        # Create Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.frame_id

        # Set range properties
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = self.field_of_view
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range

        # Set measured range
        if distance >= self.min_range and distance <= self.max_range:
            range_msg.range = distance
        else:
            # Out of range or invalid reading
            range_msg.range = float('inf')

        # Publish message
        self.range_publisher.publish(range_msg)

        # Log occasional status (every 10 seconds)
        if int(time.time()) % 10 == 0:
            if range_msg.range != float('inf'):
                self.get_logger().info('.2f')
            else:
                self.get_logger().warn('TF-Luna: No valid range data')

    def destroy_node(self):
        """Clean shutdown"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFLunaSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
