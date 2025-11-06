#!/usr/bin/env python3
"""
Sensor Simulator for Autonomous Mobile Manipulator Development
Publishes dummy sensor data for testing n8n workflows and ROS2 APIs without Gazebo/hardware
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float32, Int32
import math
import random
import time

class SensorSimulator(Node):
    """ROS2 node that simulates sensor data for development and testing"""

    def __init__(self):
        super().__init__('sensor_simulator')

        # Publishers for simulated sensor data
        self.distance_front_pub = self.create_publisher(Float32, '/distance/front', 10)
        self.distance_back_left_pub = self.create_publisher(Float32, '/distance/back_left', 10)
        self.distance_back_right_pub = self.create_publisher(Float32, '/distance/back_right', 10)
        self.line_sensor_pub = self.create_publisher(Int32, '/line_sensor/raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Simulation parameters
        self.distance_range = (0.2, 5.0)  # Min/max distance in meters
        self.line_sensor_states = [0, 1, 3, 7, 15, 31, 63, 127, 255]  # Different line patterns

        # Timers for publishing sensor data
        self.create_timer(0.1, self.publish_distance_sensors)   # 10Hz
        self.create_timer(0.2, self.publish_line_sensor)        # 5Hz
        self.create_timer(0.1, self.publish_imu_data)          # 10Hz
        self.create_timer(0.1, self.publish_lidar_scan)        # 10Hz

        # Initialize sensor state
        self.current_distances = {
            'front': 2.0,
            'back_left': 3.0,
            'back_right': 2.5
        }

        self.line_sensor_value = 0
        self.imu_orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        self.get_logger().info('Sensor Simulator started - Publishing dummy sensor data')

    def publish_distance_sensors(self):
        """Publish simulated ultrasonic distance sensor data"""
        # Simulate realistic distance variations
        for sensor_name, current_dist in self.current_distances.items():
            # Add small random variations
            variation = random.uniform(-0.05, 0.05)
            new_distance = max(self.distance_range[0],
                             min(self.distance_range[1],
                                 current_dist + variation))

            # Occasionally simulate closer objects (like obstacles)
            if random.random() < 0.02:  # 2% chance
                new_distance = random.uniform(0.3, 1.5)

            self.current_distances[sensor_name] = new_distance

            # Publish the distance
            msg = Float32()
            msg.data = new_distance

            if sensor_name == 'front':
                self.distance_front_pub.publish(msg)
            elif sensor_name == 'back_left':
                self.distance_back_left_pub.publish(msg)
            elif sensor_name == 'back_right':
                self.distance_back_right_pub.publish(msg)

    def publish_line_sensor(self):
        """Publish simulated line sensor data"""
        # Simulate different line following scenarios
        scenarios = [
            0,      # No line detected
            1,      # Line on far right
            3,      # Line on right
            7,      # Line center-right
            15,     # Line center
            31,     # Line center-left
            60,     # Line on left
            120,    # Line on far left
            255     # All sensors on line
        ]

        # Change line sensor state occasionally
        if random.random() < 0.1:  # 10% chance to change state
            self.line_sensor_value = random.choice(scenarios)

        # Occasionally simulate perfect line following
        if random.random() < 0.05:  # 5% chance
            self.line_sensor_value = 15  # Center line

        msg = Int32()
        msg.data = self.line_sensor_value
        self.line_sensor_pub.publish(msg)

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        # Simulate small orientation changes (like robot movement)
        orientation_noise = 0.01

        # Add small random variations to orientation
        self.imu_orientation['x'] += random.uniform(-orientation_noise, orientation_noise)
        self.imu_orientation['y'] += random.uniform(-orientation_noise, orientation_noise)
        self.imu_orientation['z'] += random.uniform(-orientation_noise, orientation_noise)

        # Normalize quaternion (simplified)
        norm = math.sqrt(sum(x*x for x in self.imu_orientation.values()))
        for key in self.imu_orientation:
            self.imu_orientation[key] /= norm

        # Create IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientation
        msg.orientation.x = self.imu_orientation['x']
        msg.orientation.y = self.imu_orientation['y']
        msg.orientation.z = self.imu_orientation['z']
        msg.orientation.w = self.imu_orientation['w']

        # Angular velocity (simulate small movements)
        msg.angular_velocity.x = random.uniform(-0.1, 0.1)
        msg.angular_velocity.y = random.uniform(-0.1, 0.1)
        msg.angular_velocity.z = random.uniform(-0.1, 0.1)

        # Linear acceleration (gravity + small variations)
        msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)  # Gravity

        self.imu_pub.publish(msg)

    def publish_lidar_scan(self):
        """Publish simulated LIDAR scan data"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # RPLIDAR A1 specifications
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180  # 1 degree increments (360 points)
        msg.time_increment = 0.0001  # Time between measurements
        msg.scan_time = 0.1  # 10Hz
        msg.range_min = 0.15
        msg.range_max = 6.0

        # Generate 360 range readings
        ranges = []
        intensities = []

        for i in range(360):
            angle = i * math.pi / 180  # Convert to radians

            # Base distance (simulate open space)
            base_distance = 4.0 + random.uniform(-1.0, 1.0)

            # Simulate obstacles at certain angles
            obstacle_distance = None

            # Front obstacle (simulating a wall/object ahead)
            if -0.3 < angle < 0.3:  # ±17 degrees
                if random.random() < 0.7:  # 70% chance of obstacle
                    obstacle_distance = random.uniform(1.0, 3.0)

            # Side obstacles (simulating walls)
            elif abs(angle) > 2.5:  # Sides
                if random.random() < 0.5:  # 50% chance of obstacle
                    obstacle_distance = random.uniform(1.5, 4.0)

            # Use obstacle distance if present, otherwise base distance
            final_distance = obstacle_distance if obstacle_distance else base_distance

            # Add noise
            noise = random.uniform(-0.1, 0.1)
            final_distance = max(msg.range_min, min(msg.range_max, final_distance + noise))

            ranges.append(final_distance)
            intensities.append(100.0)  # Fixed intensity

        msg.ranges = ranges
        msg.intensities = intensities

        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        simulator = SensorSimulator()
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
