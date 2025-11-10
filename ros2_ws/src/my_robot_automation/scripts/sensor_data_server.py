#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from datetime import datetime

# ROS2 imports
from sensor_msgs.msg import Range, Imu, BatteryState
from std_msgs.msg import Int32, Float32

# Custom imports
from my_robot_automation.srv import GetSensorData
from my_robot_automation.msg import SensorData


class SensorDataServer(Node):
    """ROS2 service server for retrieving sensor data"""

    def __init__(self):
        super().__init__('sensor_data_server')

        # Service for getting sensor data
        self.sensor_data_service = self.create_service(
            GetSensorData, 'get_sensor_data', self.get_sensor_data_callback
        )

        # Subscribers for sensor topics
        self.distance_front_sub = self.create_subscription(
            Float32, '/distance/front', self.distance_front_callback, 10
        )
        self.distance_back_left_sub = self.create_subscription(
            Float32, '/distance/back_left', self.distance_back_left_callback, 10
        )
        self.distance_back_right_sub = self.create_subscription(
            Float32, '/distance/back_right', self.distance_back_right_callback, 10
        )
        self.line_sensor_sub = self.create_subscription(
            Int32, '/line_sensor/raw', self.line_sensor_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery/status', self.battery_callback, 10
        )
        self.tf_luna_sub = self.create_subscription(
            Range, '/tf_luna/range', self.tf_luna_callback, 10
        )

        # Current sensor data storage
        self.sensor_data = SensorData()
        self.last_distance_front = 10.0
        self.last_distance_back_left = 10.0
        self.last_distance_back_right = 10.0
        self.last_line_sensor = 0
        self.last_imu = Imu()
        self.last_battery = BatteryState()
        self.last_tf_luna = Range()

        # Initialize sensor data
        self.reset_sensor_data()

        self.get_logger().info('Sensor Data Server started')

    def reset_sensor_data(self):
        """Reset sensor data to default values"""
        now = self.get_clock().now().to_msg()
        self.sensor_data.header.stamp = now
        self.sensor_data.header.frame_id = 'base_link'

        # Distance sensors (default to max range)
        self.sensor_data.ultrasonic_front = 10.0
        self.sensor_data.ultrasonic_back_left = 10.0
        self.sensor_data.ultrasonic_back_right = 10.0

        # IR sensors (default to max range)
        self.sensor_data.ir_front = 5.0
        self.sensor_data.ir_left = 5.0
        self.sensor_data.ir_right = 5.0

        # TF-Luna LIDAR (default to max range)
        self.sensor_data.tf_luna_distance = 8.0
        self.sensor_data.tf_luna_strength = 0
        self.sensor_data.tf_luna_temperature = 25.0

        # Line sensor (default to no line detected)
        self.sensor_data.line_sensor_raw = 0
        self.sensor_data.line_detected = False
        self.sensor_data.line_position = 0.0

        # IMU data (default to zero)
        self.sensor_data.linear_acceleration.x = 0.0
        self.sensor_data.linear_acceleration.y = 0.0
        self.sensor_data.linear_acceleration.z = 9.81  # Gravity

        self.sensor_data.angular_velocity.x = 0.0
        self.sensor_data.angular_velocity.y = 0.0
        self.sensor_data.angular_velocity.z = 0.0

        self.sensor_data.orientation.x = 0.0
        self.sensor_data.orientation.y = 0.0
        self.sensor_data.orientation.z = 0.0
        self.sensor_data.orientation.w = 1.0

        # Battery status
        self.sensor_data.battery_voltage = 12.0
        self.sensor_data.battery_percentage = 100.0
        self.sensor_data.battery_low_warning = False

        # Sensor health (assume healthy by default)
        self.sensor_data.ultrasonic_healthy = True
        self.sensor_data.ir_healthy = True
        self.sensor_data.line_sensor_healthy = True
        self.sensor_data.imu_healthy = True
        self.sensor_data.battery_healthy = True
        self.sensor_data.tf_luna_healthy = True

    def distance_front_callback(self, msg):
        """Callback for front distance sensor"""
        self.last_distance_front = msg.data
        self.sensor_data.ultrasonic_front = msg.data

    def distance_back_left_callback(self, msg):
        """Callback for back left distance sensor"""
        self.last_distance_back_left = msg.data
        self.sensor_data.ultrasonic_back_left = msg.data

    def distance_back_right_callback(self, msg):
        """Callback for back right distance sensor"""
        self.last_distance_back_right = msg.data
        self.sensor_data.ultrasonic_back_right = msg.data

    def line_sensor_callback(self, msg):
        """Callback for line sensor"""
        self.last_line_sensor = msg.data
        self.sensor_data.line_sensor_raw = msg.data

        # Process line sensor data
        # Assume 8-bit sensor array, calculate position
        sensor_bits = msg.data & 0xFF  # Get lower 8 bits
        if sensor_bits == 0:
            self.sensor_data.line_detected = False
            self.sensor_data.line_position = 0.0
        else:
            self.sensor_data.line_detected = True
            # Calculate weighted average position
            total_weight = 0
            weighted_sum = 0
            for i in range(8):
                if sensor_bits & (1 << i):
                    weight = i - 3.5  # Center at 0, range -3.5 to 3.5
                    total_weight += 1
                    weighted_sum += weight
            if total_weight > 0:
                self.sensor_data.line_position = weighted_sum / total_weight / 3.5  # Normalize to -1.0 to 1.0

    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.last_imu = msg
        self.sensor_data.linear_acceleration = msg.linear_acceleration
        self.sensor_data.angular_velocity = msg.angular_velocity
        self.sensor_data.orientation = msg.orientation

    def battery_callback(self, msg):
        """Callback for battery status"""
        self.last_battery = msg
        self.sensor_data.battery_voltage = msg.voltage
        self.sensor_data.battery_percentage = msg.percentage * 100.0  # Convert to percentage
        self.sensor_data.battery_low_warning = msg.percentage < 0.2  # Low if < 20%

    def tf_luna_callback(self, msg):
        """Callback for TF-Luna LIDAR sensor"""
        self.last_tf_luna = msg
        self.sensor_data.tf_luna_distance = msg.range
        # Note: TF-Luna doesn't provide strength/temperature in Range message
        # These would need to be obtained from the dedicated TF-Luna node
        # For now, use defaults
        self.sensor_data.tf_luna_strength = 30000  # Default strength
        self.sensor_data.tf_luna_temperature = 25.0  # Default temperature

    def get_sensor_data_callback(self, request, response):
        """Service callback for getting sensor data"""
        try:
            self.get_logger().debug('Sensor data request received')

            # Update header timestamp
            self.sensor_data.header.stamp = self.get_clock().now().to_msg()

            # For IR sensors, we'll use ultrasonic data as proxy since we don't have separate IR topics
            # In a real implementation, you'd have separate IR sensor topics
            self.sensor_data.ir_front = min(self.sensor_data.ultrasonic_front, 5.0)
            self.sensor_data.ir_left = min(self.sensor_data.ultrasonic_back_left, 5.0)
            self.sensor_data.ir_right = min(self.sensor_data.ultrasonic_back_right, 5.0)

            response.sensor_data = self.sensor_data
            response.success = True
            response.message = 'Sensor data retrieved successfully'

            self.get_logger().debug('Sensor data response sent')

        except Exception as e:
            self.get_logger().error(f'Error getting sensor data: {str(e)}')
            response.success = False
            response.message = f'Error retrieving sensor data: {str(e)}'
            # Return default sensor data on error
            response.sensor_data = self.sensor_data

        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        sensor_server = SensorDataServer()

        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(sensor_server)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            sensor_server.destroy_node()

    except Exception as e:
        print(f"Error starting sensor data server: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
