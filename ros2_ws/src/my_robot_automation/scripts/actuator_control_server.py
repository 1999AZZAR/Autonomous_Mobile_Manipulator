#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
import time
import sys
import os

# Add script directory to path to import pg23_motor_controller
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# Import PG23 motor controller
try:
    from pg23_motor_controller import PG23MotorManager
    MOTOR_CONTROLLER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Could not import PG23MotorController: {e}")
    MOTOR_CONTROLLER_AVAILABLE = False

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
        
        # Motor controller manager
        self.motor_manager = None

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
            # Note: DATA(A) and DATA(B) pins serve dual purpose: serial control AND encoder feedback
            # Front Left Motor
            'MOTOR_FL_DATA_A': 17,   # GPIO17 - DATA(A) pin (Serial TX / Encoder A - same pin)
            'MOTOR_FL_DATA_B': 27,   # GPIO27 - DATA(B) pin (Serial RX / Encoder B - same pin)
            
            # Front Right Motor
            'MOTOR_FR_DATA_A': 22,   # GPIO22 - DATA(A) pin (Serial TX / Encoder A - same pin)
            'MOTOR_FR_DATA_B': 23,   # GPIO23 - DATA(B) pin (Serial RX / Encoder B - same pin)
            
            # Back Motor
            'MOTOR_BACK_DATA_A': 24, # GPIO24 - DATA(A) pin (Serial TX / Encoder A - same pin)
            'MOTOR_BACK_DATA_B': 25, # GPIO25 - DATA(B) pin (Serial RX / Encoder B - same pin)

            # Gripper Lifter Motor
            'LIFTER_DATA_A': 13,     # GPIO13 - DATA(A) pin (Serial TX / Encoder A - same pin)
            'LIFTER_DATA_B': 12,     # GPIO12 - DATA(B) pin (Serial RX / Encoder B - same pin)

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

            # Motor DATA pins (PG23 built-in driver - same pins for serial control AND encoder)
            motor_data_a_pins = [
                self.PINS['MOTOR_FL_DATA_A'], self.PINS['MOTOR_FR_DATA_A'],
                self.PINS['MOTOR_BACK_DATA_A'], self.PINS['LIFTER_DATA_A']
            ]
            motor_data_b_pins = [
                self.PINS['MOTOR_FL_DATA_B'], self.PINS['MOTOR_FR_DATA_B'],
                self.PINS['MOTOR_BACK_DATA_B'], self.PINS['LIFTER_DATA_B']
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
            
            # Motor DATA pins: Configure for serial communication
            # DATA(A) pins: Output for serial TX, also read encoder A
            for pin in motor_data_a_pins:
                lgpio.gpio_claim_output(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as output (DATA A - Serial TX / Encoder A)')
            
            # DATA(B) pins: Input for serial RX, also read encoder B
            for pin in motor_data_b_pins:
                lgpio.gpio_claim_input(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as input (DATA B - Serial RX / Encoder B)')
            
            all_pins = all_output_pins + motor_data_a_pins + motor_data_b_pins

            self.gpio_initialized = True
            self.get_logger().info('✓ GPIO Controller initialized successfully with lgpio!')
            self.get_logger().info(f'  - Total pins initialized: {len(all_pins)}')
            
            # Initialize motor controllers
            if MOTOR_CONTROLLER_AVAILABLE:
                self.motor_manager = PG23MotorManager(self.gpio_handle)
                
                # Add all motors
                self.motor_manager.add_motor(
                    'front_left',
                    self.PINS['MOTOR_FL_DATA_A'],
                    self.PINS['MOTOR_FL_DATA_B']
                )
                self.motor_manager.add_motor(
                    'front_right',
                    self.PINS['MOTOR_FR_DATA_A'],
                    self.PINS['MOTOR_FR_DATA_B']
                )
                self.motor_manager.add_motor(
                    'back',
                    self.PINS['MOTOR_BACK_DATA_A'],
                    self.PINS['MOTOR_BACK_DATA_B']
                )
                self.motor_manager.add_motor(
                    'lifter',
                    self.PINS['LIFTER_DATA_A'],
                    self.PINS['LIFTER_DATA_B']
                )
                
                # Ensure all motors are stopped on initialization
                self.motor_manager.stop_all()
                self.get_logger().info('✓ PG23 Motor controllers initialized - all motors stopped')
            else:
                self.get_logger().warn('⚠ Motor controller not available - motors will not be controlled')

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
        """Execute robot movement using PG23 motor controllers"""
        if not self.motor_manager:
            self.get_logger().error("Motor manager not initialized")
            return
            
        # Clamp speed
        speed = max(0.0, min(1.0, speed))
        
        try:
            if direction == 'stop':
                self.stop_robot()
            elif direction == 'forward':
                # All motors forward
                self.motor_manager['front_left'].forward(speed)
                self.motor_manager['front_right'].forward(speed)
                self.motor_manager['back'].forward(speed)
                self.get_logger().info(f"Moving forward at speed {speed:.2f}")
            elif direction == 'backward':
                # All motors reverse
                self.motor_manager['front_left'].reverse(speed)
                self.motor_manager['front_right'].reverse(speed)
                self.motor_manager['back'].reverse(speed)
                self.get_logger().info(f"Moving backward at speed {speed:.2f}")
            elif direction == 'strafe_left':
                # Front left reverse, front right forward, back stopped
                self.motor_manager['front_left'].reverse(speed)
                self.motor_manager['front_right'].forward(speed)
                self.motor_manager['back'].stop()
                self.get_logger().info(f"Strafing left at speed {speed:.2f}")
            elif direction == 'strafe_right':
                # Front left forward, front right reverse, back stopped
                self.motor_manager['front_left'].forward(speed)
                self.motor_manager['front_right'].reverse(speed)
                self.motor_manager['back'].stop()
                self.get_logger().info(f"Strafing right at speed {speed:.2f}")
            elif direction == 'turn_left':
                # Front left reverse, front right forward, back forward
                self.motor_manager['front_left'].reverse(speed)
                self.motor_manager['front_right'].forward(speed)
                self.motor_manager['back'].forward(speed)
                self.get_logger().info(f"Turning left at speed {speed:.2f}")
            elif direction == 'turn_right':
                # Front left forward, front right reverse, back reverse
                self.motor_manager['front_left'].forward(speed)
                self.motor_manager['front_right'].reverse(speed)
                self.motor_manager['back'].reverse(speed)
                self.get_logger().info(f"Turning right at speed {speed:.2f}")
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")

    def stop_robot(self):
        """Stop all robot motors"""
        if self.motor_manager:
            self.motor_manager.stop_all()
            self.get_logger().info("Robot stopped - all motors sent STOP command")
        else:
            self.get_logger().warn("Motor manager not available - cannot stop motors")

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
        """Cleanup GPIO resources and stop all motors"""
        try:
            # Stop all motors before cleanup
            if self.motor_manager:
                self.get_logger().info('Stopping all motors before cleanup...')
                self.motor_manager.stop_all()
                time.sleep(0.1)  # Give motors time to stop
            
            if self.gpio_handle is not None and self.gpio_initialized:
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
