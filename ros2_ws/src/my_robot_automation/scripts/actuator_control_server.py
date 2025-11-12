#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
import time

# ROS2 service imports
from my_robot_automation.srv import (
    ControlGripper, SetGripperTilt, MoveRobot,
    ControlContainer
)

class ActuatorControlServer(Node):
    """ROS2 server for actuator control with GPIO hardware"""

    def __init__(self):
        super().__init__('actuator_control_server')

        # GPIO setup
        self.gpio_handle = None
        self.gpio_initialized = False

        # GPIO pin definitions
        # PG23 Motor Pinout: M+, M-, GND, VIN, DATA(A), DATA(B)
        # Note: PG23 motors have built-in drivers - no external L298N needed
        # DATA pins are used for serial communication (UART/SPI) to control motor
        self.PINS = {
            # Servos (Hardware PWM)
            'GRIPPER_TILT': 18,
            'GRIPPER_OPEN_CLOSE': 19,
            'GRIPPER_NECK': 21,
            'GRIPPER_BASE': 12,

            # Omni Wheel Motors (PG23 with built-in driver - serial control via DATA pins)
            # Front Left Motor - Serial Communication
            'MOTOR_FL_TX': 17,       # GPIO17 - Front Left Motor Serial TX (DATA control)
            'MOTOR_FL_RX': 27,       # GPIO27 - Front Left Motor Serial RX (DATA feedback)
            'MOTOR_FL_ENCODER_A': 5, # GPIO5 - Front Left Encoder DATA(A) - built-in encoder
            'MOTOR_FL_ENCODER_B': 6, # GPIO6 - Front Left Encoder DATA(B) - built-in encoder
            
            # Front Right Motor - Serial Communication
            'MOTOR_FR_TX': 22,       # GPIO22 - Front Right Motor Serial TX (DATA control)
            'MOTOR_FR_RX': 23,       # GPIO23 - Front Right Motor Serial RX (DATA feedback)
            'MOTOR_FR_ENCODER_A': 20,# GPIO20 - Front Right Encoder DATA(A) - built-in encoder
            'MOTOR_FR_ENCODER_B': 21,# GPIO21 - Front Right Encoder DATA(B) - built-in encoder
            
            # Back Motor - Serial Communication
            'MOTOR_BACK_TX': 24,     # GPIO24 - Back Motor Serial TX (DATA control)
            'MOTOR_BACK_RX': 25,     # GPIO25 - Back Motor Serial RX (DATA feedback)
            'MOTOR_BACK_ENCODER_A': 22, # GPIO22 - Back Encoder DATA(A) - built-in encoder
            'MOTOR_BACK_ENCODER_B': 23, # GPIO23 - Back Encoder DATA(B) - built-in encoder

            # Gripper Lifter Motor (PG23 with built-in driver)
            'LIFTER_TX': 13,         # GPIO13 - Lifter Motor Serial TX (DATA control)
            'LIFTER_RX': 12,         # GPIO12 - Lifter Motor Serial RX (DATA feedback)
            'LIFTER_ENCODER_A': 19,  # GPIO19 - Lifter Encoder DATA(A) - built-in encoder
            'LIFTER_ENCODER_B': 16,  # GPIO16 - Lifter Encoder DATA(B) - built-in encoder

            # Container Servos
            'CONTAINER_LF': 26,
            'CONTAINER_LB': 5,
            'CONTAINER_RF': 6,
            'CONTAINER_RB': 7,
        }

        # Initialize GPIO
        self.initialize_gpio()

        # Create ROS2 services
        self.create_services()

        self.get_logger().info('Actuator Control Server initialized')

    def initialize_gpio(self):
        """Initialize GPIO hardware"""
        try:
            self.get_logger().info('Initializing GPIO with lgpio...')

            # Open GPIO chip
            self.gpio_handle = lgpio.gpiochip_open(0)
            self.get_logger().info('GPIO chip opened successfully')

            # Initialize all GPIO pins as outputs
            servo_pins = [
                self.PINS['GRIPPER_TILT'],
                self.PINS['GRIPPER_OPEN_CLOSE'],
                self.PINS['GRIPPER_NECK'],
                self.PINS['GRIPPER_BASE']
            ]

            # Motor control pins (serial TX/RX for PG23 built-in driver)
            motor_tx_pins = [
                self.PINS['MOTOR_FL_TX'], self.PINS['MOTOR_FR_TX'],
                self.PINS['MOTOR_BACK_TX'], self.PINS['LIFTER_TX']
            ]
            motor_rx_pins = [
                self.PINS['MOTOR_FL_RX'], self.PINS['MOTOR_FR_RX'],
                self.PINS['MOTOR_BACK_RX'], self.PINS['LIFTER_RX']
            ]
            
            # Encoder pins (inputs for PG23 built-in encoder DATA channels)
            encoder_pins = [
                self.PINS['MOTOR_FL_ENCODER_A'], self.PINS['MOTOR_FL_ENCODER_B'],
                self.PINS['MOTOR_FR_ENCODER_A'], self.PINS['MOTOR_FR_ENCODER_B'],
                self.PINS['MOTOR_BACK_ENCODER_A'], self.PINS['MOTOR_BACK_ENCODER_B'],
                self.PINS['LIFTER_ENCODER_A'], self.PINS['LIFTER_ENCODER_B']
            ]

            container_pins = [
                self.PINS['CONTAINER_LF'], self.PINS['CONTAINER_LB'],
                self.PINS['CONTAINER_RF'], self.PINS['CONTAINER_RB']
            ]

            # Claim servo pins as outputs
            all_output_pins = servo_pins + container_pins
            for pin in all_output_pins:
                lgpio.gpio_claim_output(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as output')
            
            # Motor control pins: TX as outputs, RX as inputs (for serial communication)
            for pin in motor_tx_pins:
                lgpio.gpio_claim_output(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as output (motor TX)')
            
            for pin in motor_rx_pins:
                lgpio.gpio_claim_input(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as input (motor RX)')
            
            # Claim encoder pins as inputs (for PG23 built-in encoder DATA channels)
            for pin in encoder_pins:
                lgpio.gpio_claim_input(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as input (encoder)')
            
            all_pins = all_output_pins + motor_tx_pins + motor_rx_pins + encoder_pins

            self.gpio_initialized = True
            self.get_logger().info('✓ GPIO Controller initialized successfully with lgpio!')
            self.get_logger().info(f'  - Total pins initialized: {len(all_pins)}')

        except Exception as e:
            self.get_logger().error(f'✗ GPIO initialization failed: {str(e)}')
            self.gpio_initialized = False

    def create_services(self):
        """Create ROS2 services"""
        self.control_gripper_srv = self.create_service(
            ControlGripper,
            'actuator/control_gripper',
            self.control_gripper_callback
        )

        self.set_gripper_tilt_srv = self.create_service(
            SetGripperTilt,
            'actuator/set_gripper_tilt',
            self.set_gripper_tilt_callback
        )

        self.move_robot_srv = self.create_service(
            MoveRobot,
            'actuator/move_robot',
            self.move_robot_callback
        )

        self.control_container_srv = self.create_service(
            ControlContainer,
            'actuator/control_container',
            self.control_container_callback
        )

        self.get_logger().info('ROS2 actuator control services created')

    def control_gripper_callback(self, request, response):
        """Handle gripper control service calls"""
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
                self.get_logger().info(response.message)
            elif request.command == 'close':
                lgpio.gpio_write(self.gpio_handle, pin, 0)
                response.success = True
                response.message = f"Gripper closed (GPIO{pin} = 0)"
                response.status = "CLOSED"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"Unknown command: {request.command}"
                response.status = "ERROR"
        except Exception as e:
            response.success = False
            response.message = f"Gripper control error: {str(e)}"
            response.status = "ERROR"
            self.get_logger().error(response.message)

        return response

    def set_gripper_tilt_callback(self, request, response):
        """Handle gripper tilt service calls"""
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
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Gripper tilt error: {str(e)}"
            response.current_angle = 0.0
            self.get_logger().error(response.message)

        return response

    def move_robot_callback(self, request, response):
        """Handle robot movement service calls"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.status = "ERROR"
            return response

        try:
            direction = request.direction
            speed = max(0, min(1, request.speed))
            duration = max(0, request.duration)

            # Execute movement
            self.execute_movement(direction, speed)

            # If duration specified, stop after delay
            if duration > 0:
                time.sleep(duration)
                self.stop_robot()

            response.success = True
            response.message = f"Robot moved {direction} at speed {speed}"
            response.status = "COMPLETED"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Robot movement error: {str(e)}"
            response.status = "ERROR"
            self.get_logger().error(response.message)

        return response

    def execute_movement(self, direction, speed):
        """Execute robot movement"""
        if direction == 'stop':
            # Stop all motors
            self.stop_robot()
        elif direction == 'forward':
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 1)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 1)
        elif direction == 'backward':
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 0)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
        elif direction == 'strafe_left':
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 0)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
        elif direction == 'strafe_right':
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 1)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)

    def stop_robot(self):
        """Stop all robot motors"""
        lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_DIR'], 0)
        lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)
        lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
        self.get_logger().info("Robot stopped")

    def control_container_callback(self, request, response):
        """Handle container control service calls"""
        if not self.gpio_initialized:
            response.success = False
            response.message = "GPIO not initialized"
            response.status = "ERROR"
            return response

        try:
            container_id = request.container_id
            action = request.action

            # Map container_id to GPIO pin
            container_pins = {
                'left_front': self.PINS['CONTAINER_LF'],
                'left_back': self.PINS['CONTAINER_LB'],
                'right_front': self.PINS['CONTAINER_RF'],
                'right_back': self.PINS['CONTAINER_RB']
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
                response.message = f"Container {container_id} loaded (GPIO{pin} = 0)"
                response.status = "LOADED"
            elif action == 'unload':
                lgpio.gpio_write(self.gpio_handle, pin, 1)
                response.success = True
                response.message = f"Container {container_id} unloaded (GPIO{pin} = 1)"
                response.status = "UNLOADED"
            else:
                response.success = False
                response.message = f"Unknown action: {action}"
                response.status = "ERROR"

            if response.success:
                self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Container control error: {str(e)}"
            response.status = "ERROR"
            self.get_logger().error(response.message)

        return response

    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.gpio_handle is not None and self.gpio_initialized:
            try:
                lgpio.gpiochip_close(self.gpio_handle)
                self.gpio_handle = None
                self.gpio_initialized = False
                self.get_logger().info("GPIO resources cleaned up")
            except Exception as e:
                self.get_logger().error(f"GPIO cleanup error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControlServer()

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
