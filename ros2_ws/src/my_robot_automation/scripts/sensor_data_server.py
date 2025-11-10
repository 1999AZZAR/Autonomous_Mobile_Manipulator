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

        # Subscribers for sensor topics (6x laser distance sensors)
        self.distance_left_front_sub = self.create_subscription(
            Range, '/distance/left_front', self.distance_left_front_callback, 10
        )
        self.distance_left_back_sub = self.create_subscription(
            Range, '/distance/left_back', self.distance_left_back_callback, 10
        )
        self.distance_right_front_sub = self.create_subscription(
            Range, '/distance/right_front', self.distance_right_front_callback, 10
        )
        self.distance_right_back_sub = self.create_subscription(
            Range, '/distance/right_back', self.distance_right_back_callback, 10
        )
        self.distance_back_left_sub = self.create_subscription(
            Range, '/distance/back_left', self.distance_back_left_callback, 10
        )
        self.distance_back_right_sub = self.create_subscription(
            Range, '/distance/back_right', self.distance_back_right_callback, 10
        )

        # HC-SR04 Ultrasonic sensors (2x front)
        self.ultrasonic_front_left_sub = self.create_subscription(
            Range, '/ultrasonic/front_left', self.ultrasonic_front_left_callback, 10
        )
        self.ultrasonic_front_right_sub = self.create_subscription(
            Range, '/ultrasonic/front_right', self.ultrasonic_front_right_callback, 10
        )

        # Line sensors (3x individual)
        self.line_sensor_left_sub = self.create_subscription(
            Int32, '/line_sensor/left', self.line_sensor_left_callback, 10
        )
        self.line_sensor_center_sub = self.create_subscription(
            Int32, '/line_sensor/center', self.line_sensor_center_callback, 10
        )
        self.line_sensor_right_sub = self.create_subscription(
            Int32, '/line_sensor/right', self.line_sensor_right_callback, 10
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
        self.last_distance_left_front = 10.0
        self.last_distance_left_back = 10.0
        self.last_distance_right_front = 10.0
        self.last_distance_right_back = 10.0
        self.last_distance_back_left = 10.0
        self.last_distance_back_right = 10.0
        self.last_ultrasonic_front_left = 4.0
        self.last_ultrasonic_front_right = 4.0
        self.last_line_sensor_left = 0
        self.last_line_sensor_center = 0
        self.last_line_sensor_right = 0
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

        # Laser distance sensors (default to max range)
        self.sensor_data.distance_left_front = 10.0
        self.sensor_data.distance_left_back = 10.0
        self.sensor_data.distance_right_front = 10.0
        self.sensor_data.distance_right_back = 10.0
        self.sensor_data.distance_back_left = 10.0
        self.sensor_data.distance_back_right = 10.0

        # HC-SR04 ultrasonic sensors (default to max range)
        self.sensor_data.ultrasonic_front_left = 4.0
        self.sensor_data.ultrasonic_front_right = 4.0

        # Line sensors (default to no line detected)
        self.sensor_data.line_sensor_left = 0
        self.sensor_data.line_sensor_center = 0
        self.sensor_data.line_sensor_right = 0

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

    def distance_left_front_callback(self, msg):
        """Callback for left front laser distance sensor"""
        self.last_distance_left_front = msg.range
        self.sensor_data.distance_left_front = msg.range

    def distance_left_back_callback(self, msg):
        """Callback for left back laser distance sensor"""
        self.last_distance_left_back = msg.range
        self.sensor_data.distance_left_back = msg.range

    def distance_right_front_callback(self, msg):
        """Callback for right front laser distance sensor"""
        self.last_distance_right_front = msg.range
        self.sensor_data.distance_right_front = msg.range

    def distance_right_back_callback(self, msg):
        """Callback for right back laser distance sensor"""
        self.last_distance_right_back = msg.range
        self.sensor_data.distance_right_back = msg.range

    def distance_back_left_callback(self, msg):
        """Callback for back left laser distance sensor"""
        self.last_distance_back_left = msg.range
        self.sensor_data.distance_back_left = msg.range

    def distance_back_right_callback(self, msg):
        """Callback for back right laser distance sensor"""
        self.last_distance_back_right = msg.range
        self.sensor_data.distance_back_right = msg.range

    def ultrasonic_front_left_callback(self, msg):
        """Callback for front left HC-SR04 ultrasonic sensor"""
        self.last_ultrasonic_front_left = msg.range
        self.sensor_data.ultrasonic_front_left = msg.range

    def ultrasonic_front_right_callback(self, msg):
        """Callback for front right HC-SR04 ultrasonic sensor"""
        self.last_ultrasonic_front_right = msg.range
        self.sensor_data.ultrasonic_front_right = msg.range

    def line_sensor_left_callback(self, msg):
        """Callback for left line sensor"""
        self.last_line_sensor_left = msg.data
        self.sensor_data.line_sensor_left = msg.data
        self.update_line_sensor_position()

    def line_sensor_center_callback(self, msg):
        """Callback for center line sensor"""
        self.last_line_sensor_center = msg.data
        self.sensor_data.line_sensor_center = msg.data
        self.update_line_sensor_position()

    def line_sensor_right_callback(self, msg):
        """Callback for right line sensor"""
        self.last_line_sensor_right = msg.data
        self.sensor_data.line_sensor_right = msg.data
        self.update_line_sensor_position()

    def update_line_sensor_position(self):
        """Update line sensor position calculation based on 3 individual sensors"""
        left = self.sensor_data.line_sensor_left
        center = self.sensor_data.line_sensor_center
        right = self.sensor_data.line_sensor_right

        # Calculate line position (-1 to 1, where 0 is center)
        line_detected = left or center or right
        self.sensor_data.line_detected = line_detected

        if not line_detected:
            self.sensor_data.line_position = 0.0
        elif left and center and right:
            self.sensor_data.line_position = 0.0  # All sensors on line - centered
        elif left and center:
            self.sensor_data.line_position = -0.3  # Slightly left
        elif center and right:
            self.sensor_data.line_position = 0.3   # Slightly right
        elif left and right:
            self.sensor_data.line_position = 0.0   # Line between sensors
        elif left:
            self.sensor_data.line_position = -0.8  # Far left
        elif right:
            self.sensor_data.line_position = 0.8   # Far right
        elif center:
            self.sensor_data.line_position = 0.0   # Perfect center
        else:
            self.sensor_data.line_position = 0.0   # No line detected

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
