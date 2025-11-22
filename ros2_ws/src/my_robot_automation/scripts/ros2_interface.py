"""
ROS2 Interface
Handles ROS2 node, services, and topic subscriptions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import logging
from config import ROS2_NODE_NAME, ROS2_ACTUATOR_SERVICE_TIMEOUT

logger = logging.getLogger(__name__)

class ROS2Interface(Node):
    """ROS2 interface for actuator control and sensor data"""

    def __init__(self, sensor_manager=None):
        super().__init__(ROS2_NODE_NAME)
        self.sensor_manager = sensor_manager

        # Initialize services and clients
        self.actuator_services_created = False
        self.actuator_clients_created = False

        # Sensor subscribers
        self._setup_sensor_subscribers()

        # Actuator services (will be created later)
        self.control_gripper_service = None
        self.set_gripper_tilt_service = None
        self.move_robot_service = None
        self.control_container_service = None

        logger.info("ROS2 interface initialized")

    def _setup_sensor_subscribers(self):
        """Setup ROS2 subscribers for sensor data from Arduino Mega"""
        # IR Distance sensors (from Mega) - Float32 from mega_sensor_publisher
        self.distance_left_front_sub = self.create_subscription(
            Float32, '/distance/left_front', self.distance_left_front_callback, 10)
        self.distance_left_back_sub = self.create_subscription(
            Float32, '/distance/left_back', self.distance_left_back_callback, 10)
        self.distance_right_front_sub = self.create_subscription(
            Float32, '/distance/right_front', self.distance_right_front_callback, 10)
        self.distance_right_back_sub = self.create_subscription(
            Float32, '/distance/right_back', self.distance_right_back_callback, 10)
        self.distance_back_left_sub = self.create_subscription(
            Float32, '/distance/back_left', self.distance_back_left_callback, 10)
        self.distance_back_right_sub = self.create_subscription(
            Float32, '/distance/back_right', self.distance_back_right_callback, 10)

        # Ultrasonic sensors
        self.ultrasonic_front_left_sub = self.create_subscription(
            Float32, '/ultrasonic/front_left', self.ultrasonic_front_left_callback, 10)
        self.ultrasonic_front_right_sub = self.create_subscription(
            Float32, '/ultrasonic/front_right', self.ultrasonic_front_right_callback, 10)

        # IMU data (from Pi IMU)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        logger.info('ROS2 sensor subscribers initialized')

    def distance_left_front_callback(self, msg):
        self.sensor_data['laser_sensors']['left_front'] = msg.data

    def distance_left_back_callback(self, msg):
        self.sensor_data['laser_sensors']['left_back'] = msg.data

    def distance_right_front_callback(self, msg):
        self.sensor_data['laser_sensors']['right_front'] = msg.data

    def distance_right_back_callback(self, msg):
        self.sensor_data['laser_sensors']['right_back'] = msg.data

    def distance_back_left_callback(self, msg):
        self.sensor_data['laser_sensors']['back_left'] = msg.data

    def distance_back_right_callback(self, msg):
        self.sensor_data['laser_sensors']['back_right'] = msg.data

    def ultrasonic_front_left_callback(self, msg):
        self.sensor_data['ultrasonic_sensors']['front_left'] = msg.data

    def ultrasonic_front_right_callback(self, msg):
        self.sensor_data['ultrasonic_sensors']['front_right'] = msg.data

    def imu_callback(self, msg):
        """Handle IMU data from ROS2 topic"""
        imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'temperature': 0.0  # IMU temperature if available
        }

        if self.sensor_manager:
            self.sensor_manager.update_imu_data(imu_data)

    def initialize_actuator_services(self):
        """Initialize ROS2 services for actuator control"""
        if self.actuator_services_created:
            return

        try:
            from my_robot_automation.srv import (
                ControlGripper, SetGripperTilt, MoveRobot, ControlContainer
            )

            # Create services
            self.control_gripper_service = self.create_service(
                ControlGripper, 'actuator/control_gripper', self.ros2_control_gripper_callback)
            self.set_gripper_tilt_service = self.create_service(
                SetGripperTilt, 'actuator/set_gripper_tilt', self.ros2_set_gripper_tilt_callback)
            self.move_robot_service = self.create_service(
                MoveRobot, 'actuator/move_robot', self.ros2_move_robot_callback)
            self.control_container_service = self.create_service(
                ControlContainer, 'actuator/control_container', self.ros2_control_container_callback)

            self.actuator_services_created = True
            self.get_logger().info('ROS2 actuator services created')

        except Exception as e:
            self.get_logger().error(f'Failed to create actuator services: {str(e)}')

    def initialize_actuator_clients(self):
        """Initialize ROS2 clients for actuator control"""
        if self.actuator_clients_created:
            return

        try:
            from my_robot_automation.srv import (
                ControlGripper, SetGripperTilt, MoveRobot, ControlContainer
            )

            # Create clients
            self.control_gripper_client = self.create_client(ControlGripper, 'actuator/control_gripper')
            self.set_gripper_tilt_client = self.create_client(SetGripperTilt, 'actuator/set_gripper_tilt')
            self.move_robot_client = self.create_client(MoveRobot, 'actuator/move_robot')
            self.control_container_client = self.create_client(ControlContainer, 'actuator/control_container')

            # Wait for services to be available
            self.get_logger().info('Waiting for ROS2 actuator services...')
            self.control_gripper_client.wait_for_service(timeout_sec=ROS2_ACTUATOR_SERVICE_TIMEOUT)
            self.set_gripper_tilt_client.wait_for_service(timeout_sec=ROS2_ACTUATOR_SERVICE_TIMEOUT)
            self.move_robot_client.wait_for_service(timeout_sec=ROS2_ACTUATOR_SERVICE_TIMEOUT)
            self.control_container_client.wait_for_service(timeout_sec=ROS2_ACTUATOR_SERVICE_TIMEOUT)

            self.actuator_clients_created = True
            self.get_logger().info('ROS2 actuator service clients initialized')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize actuator clients: {str(e)}')

    def ros2_control_gripper_callback(self, request, response):
        """ROS2 service callback for gripper control"""
        # This would interface with Mega or GPIO controller
        response.success = True
        response.message = f"Gripper {request.command} executed"
        return response

    def ros2_set_gripper_tilt_callback(self, request, response):
        """ROS2 service callback for gripper tilt control"""
        response.success = True
        response.message = f"Gripper tilt set to {request.angle}°"
        response.current_angle = request.angle
        return response

    def ros2_move_robot_callback(self, request, response):
        """ROS2 service callback for robot movement"""
        response.success = True
        response.message = f"Robot moved {request.direction} at speed {request.speed}"
        response.status = "completed"
        return response

    def ros2_control_container_callback(self, request, response):
        """ROS2 service callback for container control"""
        response.success = True
        response.message = f"Container {request.container_id} {request.action} executed"
        return response

    def call_control_gripper(self, command):
        """Call ROS2 control gripper service"""
        if not self.actuator_clients_created:
            return False

        try:
            from my_robot_automation.srv import ControlGripper
            request = ControlGripper.Request()
            request.command = command

            future = self.control_gripper_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                return response.success
            return False
        except Exception as e:
            self.get_logger().error(f'ROS2 gripper control error: {str(e)}')
            return False

    def call_set_gripper_tilt(self, angle):
        """Call ROS2 set gripper tilt service"""
        if not self.actuator_clients_created:
            return False

        try:
            from my_robot_automation.srv import SetGripperTilt
            request = SetGripperTilt.Request()
            request.angle = float(angle)

            future = self.set_gripper_tilt_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                return response.success
            return False
        except Exception as e:
            self.get_logger().error(f'ROS2 gripper tilt error: {str(e)}')
            return False

    def call_move_robot(self, direction, speed=0.5, duration=0.0):
        """Call ROS2 move robot service"""
        if not self.actuator_clients_created:
            return False

        try:
            from my_robot_automation.srv import MoveRobot
            request = MoveRobot.Request()
            request.direction = direction
            request.speed = float(speed)
            request.duration = float(duration)

            future = self.move_robot_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                return response.success
            return False
        except Exception as e:
            self.get_logger().error(f'ROS2 move robot error: {str(e)}')
            return False

    def call_control_container(self, container_id, action):
        """Call ROS2 control container service"""
        if not self.actuator_clients_created:
            return False

        try:
            from my_robot_automation.srv import ControlContainer
            request = ControlContainer.Request()
            request.container_id = container_id
            request.action = action

            future = self.control_container_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                return response.success
            return False
        except Exception as e:
            self.get_logger().error(f'ROS2 container control error: {str(e)}')
            return False
