#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio

# ROS2 service imports
from my_robot_automation.srv import (
    ControlGripper, SetGripperTilt, MoveRobot,
    ControlContainer
)

class SimpleActuatorServer(Node):
    """Simple ROS2 server for actuator control"""

    def __init__(self):
        super().__init__('simple_actuator_server')

        # GPIO setup
        self.gpio_handle = None
        self.gpio_initialized = False

        # GPIO pin definitions
        self.PINS = {
            'GRIPPER_OPEN_CLOSE': 19,
            'GRIPPER_TILT': 18,
            'MOTOR_FL_DIR': 17,
            'MOTOR_FR_DIR': 22,
            'MOTOR_BACK_DIR': 24,
            'CONTAINER_LF': 26,
            'CONTAINER_RF': 6,
        }

        # Initialize GPIO
        self.initialize_gpio()

        # Create ROS2 services
        self.create_services()

        self.get_logger().info('Simple Actuator Server initialized')

    def initialize_gpio(self):
        """Initialize GPIO hardware"""
        try:
            self.get_logger().info('Initializing GPIO with lgpio...')

            # Open GPIO chip
            self.gpio_handle = lgpio.gpiochip_open(0)
            self.get_logger().info('GPIO chip opened successfully')

            # Claim pins as outputs
            for pin in self.PINS.values():
                lgpio.gpio_claim_output(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as output')

            self.gpio_initialized = True
            self.get_logger().info('✓ GPIO initialized successfully')

        except Exception as e:
            self.get_logger().error(f'✗ GPIO initialization failed: {str(e)}')
            self.gpio_initialized = False

    def create_services(self):
        """Create ROS2 services"""
        self.control_gripper_srv = self.create_service(
            ControlGripper, 'actuator/control_gripper', self.control_gripper_callback)
        self.set_gripper_tilt_srv = self.create_service(
            SetGripperTilt, 'actuator/set_gripper_tilt', self.set_gripper_tilt_callback)
        self.move_robot_srv = self.create_service(
            MoveRobot, 'actuator/move_robot', self.move_robot_callback)
        self.control_container_srv = self.create_service(
            ControlContainer, 'actuator/control_container', self.control_container_callback)

        self.get_logger().info('ROS2 actuator services created')

    def control_gripper_callback(self, request, response):
        """Handle gripper control"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.status = "ERROR"
            return response

        try:
            pin = self.PINS['GRIPPER_OPEN_CLOSE']
            if request.command == 'open':
                lgpio.gpio_write(self.gpio_handle, pin, 1)
                response.success = True
                response.message = f"Gripper opened (GPIO{pin} = 1)"
                response.status = "OPEN"
            elif request.command == 'close':
                lgpio.gpio_write(self.gpio_handle, pin, 0)
                response.success = True
                response.message = f"Gripper closed (GPIO{pin} = 0)"
                response.status = "CLOSED"
            else:
                response.success = False
                response.message = f"Unknown command: {request.command}"
                response.status = "ERROR"
        except Exception as e:
            response.success = False
            response.message = f"Gripper error: {str(e)}"
            response.status = "ERROR"

        self.get_logger().info(response.message)
        return response

    def set_gripper_tilt_callback(self, request, response):
        """Handle gripper tilt"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.current_angle = 0.0
            return response

        try:
            angle = max(0, min(180, request.angle))
            pin = self.PINS['GRIPPER_TILT']
            value = 1 if angle > 90 else 0
            lgpio.gpio_write(self.gpio_handle, pin, value)

            response.success = True
            response.message = f"Gripper tilt set to {angle}° (GPIO{pin} = {value})"
            response.current_angle = angle
        except Exception as e:
            response.success = False
            response.message = f"Gripper tilt error: {str(e)}"
            response.current_angle = 0.0

        self.get_logger().info(response.message)
        return response

    def move_robot_callback(self, request, response):
        """Handle robot movement"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.status = "ERROR"
            return response

        try:
            direction = request.direction
            if direction == 'forward':
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 1)
                response.success = True
                response.message = "Moving forward"
                response.status = "MOVING"
            else:
                response.success = False
                response.message = f"Direction {direction} not implemented"
                response.status = "ERROR"
        except Exception as e:
            response.success = False
            response.message = f"Move error: {str(e)}"
            response.status = "ERROR"

        self.get_logger().info(response.message)
        return response

    def control_container_callback(self, request, response):
        """Handle container control"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.status = "ERROR"
            return response

        try:
            container_id = request.container_id
            action = request.action

            container_pins = {
                'left_front': self.PINS['CONTAINER_LF'],
                'right_front': self.PINS['CONTAINER_RF']
            }

            if container_id not in container_pins:
                response.success = False
                response.message = f"Unknown container: {container_id}"
                response.status = "ERROR"
                return response

            pin = container_pins[container_id]
            if action == 'load':
                lgpio.gpio_write(self.gpio_handle, pin, 0)
                response.success = True
                response.message = f"Container {container_id} loaded"
                response.status = "LOADED"
            elif action == 'unload':
                lgpio.gpio_write(self.gpio_handle, pin, 1)
                response.success = True
                response.message = f"Container {container_id} unloaded"
                response.status = "UNLOADED"
            else:
                response.success = False
                response.message = f"Unknown action: {action}"
                response.status = "ERROR"
        except Exception as e:
            response.success = False
            response.message = f"Container error: {str(e)}"
            response.status = "ERROR"

        self.get_logger().info(response.message)
        return response

    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.gpio_handle is not None and self.gpio_initialized:
            try:
                lgpio.gpiochip_close(self.gpio_handle)
                self.get_logger().info("GPIO resources cleaned up")
            except Exception as e:
                self.get_logger().error(f"GPIO cleanup error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleActuatorServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
