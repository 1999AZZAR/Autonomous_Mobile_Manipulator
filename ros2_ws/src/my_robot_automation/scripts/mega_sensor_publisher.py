#!/usr/bin/env python3
"""
Mega Sensor Publisher Node

This ROS2 node reads sensor data from Arduino Mega via serial
and publishes it to ROS2 topics for use by the web interface.

Publishes to:
- /distance/left_front (Float32)
- /distance/left_back (Float32)
- /distance/right_front (Float32)
- /distance/right_back (Float32)
- /distance/back_left (Float32)
- /distance/back_right (Float32)
- /ultrasonic/front_left (Float32)
- /ultrasonic/front_right (Float32)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading
import time
import re

class MegaSensorPublisher(Node):
    def __init__(self):
        super().__init__('mega_sensor_publisher')

        # Publishers for distance sensors
        self.dist_left_front_pub = self.create_publisher(Float32, '/distance/left_front', 10)
        self.dist_left_back_pub = self.create_publisher(Float32, '/distance/left_back', 10)
        self.dist_right_front_pub = self.create_publisher(Float32, '/distance/right_front', 10)
        self.dist_right_back_pub = self.create_publisher(Float32, '/distance/right_back', 10)
        self.dist_back_left_pub = self.create_publisher(Float32, '/distance/back_left', 10)
        self.dist_back_right_pub = self.create_publisher(Float32, '/distance/back_right', 10)

        # Publishers for ultrasonic sensors
        self.us_front_left_pub = self.create_publisher(Float32, '/ultrasonic/front_left', 10)
        self.us_front_right_pub = self.create_publisher(Float32, '/ultrasonic/front_right', 10)

        # Serial connection to Mega
        self.serial_port = None
        self.serial_thread = None
        self.running = True

        # Initialize serial connection
        self.initialize_serial()

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_reader)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info('Mega Sensor Publisher initialized')

    def initialize_serial(self):
        """Initialize serial connection to Arduino Mega"""
        possible_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']

        for port in possible_ports:
            try:
                self.get_logger().info(f'Attempting to connect to Mega on {port}')
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=1,
                    write_timeout=1
                )
                time.sleep(2)  # Wait for connection

                # Test connection by sending a simple command
                self.serial_port.write(b'spe\n')  # Enable sensor publishing
                time.sleep(0.5)

                self.get_logger().info(f'Successfully connected to Arduino Mega on {port}')
                return

            except Exception as e:
                self.get_logger().warn(f'Failed to connect on {port}: {e}')
                if self.serial_port:
                    self.serial_port.close()
                    self.serial_port = None

        if not self.serial_port:
            self.get_logger().error('Failed to connect to Arduino Mega on any available port')

    def serial_reader(self):
        """Thread function to read serial data from Mega"""
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode().strip()

                    if line:
                        self.parse_sensor_data(line)

            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(1)  # Wait before retrying

    def parse_sensor_data(self, line):
        """Parse sensor data from Mega serial output"""
        try:
            # Parse IR distance data: DIST:left1,left2,right1,right2,back1,back2
            if line.startswith('DIST:'):
                parts = line[5:].split(',')
                if len(parts) == 6:
                    distances = [float(p) for p in parts]

                    # Publish distance data
                    self.dist_left_front_pub.publish(Float32(data=distances[0]))
                    self.dist_left_back_pub.publish(Float32(data=distances[1]))
                    self.dist_right_front_pub.publish(Float32(data=distances[2]))
                    self.dist_right_back_pub.publish(Float32(data=distances[3]))
                    self.dist_back_left_pub.publish(Float32(data=distances[4]))
                    self.dist_back_right_pub.publish(Float32(data=distances[5]))

            # Parse ultrasonic data: US:front_left,front_right
            elif line.startswith('US:'):
                parts = line[3:].split(',')
                if len(parts) == 2:
                    distances = [float(p) for p in parts]

                    # Publish ultrasonic data
                    self.us_front_left_pub.publish(Float32(data=distances[0]))
                    self.us_front_right_pub.publish(Float32(data=distances[1]))

            # IMU data comes from Pi's MPU6050, not Mega serial
            # This section removed - IMU handled by separate ROS2 node on Pi

        except Exception as e:
            self.get_logger().warn(f'Error parsing sensor data: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False

        if self.serial_thread:
            self.serial_thread.join(timeout=2)

        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
