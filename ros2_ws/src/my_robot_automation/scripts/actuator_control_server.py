#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
import time
import sys
import os

# Note: PG23 motors have built-in encoders only - controlled via L298N drivers
# No serial motor controller needed - using direct GPIO control for L298N DIR/PWM pins

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
        # Note: PG23 motors have built-in encoders only - controlled via L298N drivers
        # Motor Control: L298N DIR and PWM pins
        # Encoder Reading: DATA(A) and DATA(B) pins (encoder feedback only)
        self.PINS = {
            # Servos (Hardware PWM)
            'GRIPPER_TILT': 18,
            'GRIPPER_OPEN_CLOSE': 19,
            'GRIPPER_NECK': 21,
            # NOTE: GRIPPER_BASE removed - GPIO12 is needed for LIFTER_PWM motor control

            # Omni Wheel Motors (PG23 with built-in encoder - controlled via L298N drivers)
            # Front Left Motor (using IN1, IN2, ENA configuration)
            'MOTOR_FL_IN1': 17,      # GPIO17 (Pin 11) - L298N IN1 pin (direction control)
            'MOTOR_FL_IN2': 10,      # GPIO10 (Pin 19) - L298N IN2 pin (direction control)
            'MOTOR_FL_ENA': 11,      # GPIO11 (Pin 23) - L298N ENA pin (PWM speed control) - NOTE: GPIO27 doesn't work
            'MOTOR_FL_ENCODER_A': 22, # GPIO22 - Encoder DATA(A) pin (read only)
            'MOTOR_FL_ENCODER_B': 23, # GPIO23 - Encoder DATA(B) pin (read only)
            
            # Front Right Motor
            'MOTOR_FR_DIR': 24,      # GPIO24 - L298N Direction pin
            'MOTOR_FR_PWM': 25,      # GPIO25 - L298N PWM pin
            'MOTOR_FR_ENCODER_A': 16, # GPIO16 - Encoder DATA(A) pin (read only)
            'MOTOR_FR_ENCODER_B': 26, # GPIO26 - Encoder DATA(B) pin (read only)
            
            # Back Motor
            'MOTOR_BACK_DIR': 5,     # GPIO5 - L298N Direction pin
            'MOTOR_BACK_PWM': 6,     # GPIO6 - L298N PWM pin
            'MOTOR_BACK_ENCODER_A': 7, # GPIO7 - Encoder DATA(A) pin (read only)
            'MOTOR_BACK_ENCODER_B': 9, # GPIO9 - Encoder DATA(B) pin (read only)

            # Gripper Lifter Motor
            'LIFTER_DIR': 13,        # GPIO13 - L298N Direction pin
            'LIFTER_PWM': 12,       # GPIO12 - L298N PWM pin
            'LIFTER_ENCODER_A': 20,  # GPIO20 - Encoder DATA(A) pin (read only)
            'LIFTER_ENCODER_B': 21,  # GPIO21 - Encoder DATA(B) pin (read only)

            # Container Servos (reassigned to avoid conflicts with motor pins)
            'CONTAINER_LF': 14,      # GPIO14 - Left Front Container
            'CONTAINER_LB': 15,      # GPIO15 - Left Back Container
            'CONTAINER_RF': 4,       # GPIO4 - Right Front Container
            'CONTAINER_RB': 8,       # GPIO8 - Right Back Container
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

            # Initialize all GPIO pins
            servo_pins = [
                self.PINS['GRIPPER_TILT'],
                self.PINS['GRIPPER_OPEN_CLOSE'],
                self.PINS['GRIPPER_NECK']
                # NOTE: GRIPPER_BASE removed - GPIO12 is needed for LIFTER_PWM motor control
            ]

            # L298N Motor Control pins
            # Front Left uses IN1/IN2/ENA (verified working configuration)
            motor_fl_in1_pin = self.PINS['MOTOR_FL_IN1']
            motor_fl_in2_pin = self.PINS['MOTOR_FL_IN2']
            motor_fl_ena_pin = self.PINS['MOTOR_FL_ENA']
            
            # Other motors still use DIR/PWM (will update later)
            motor_dir_pins = [
                self.PINS['MOTOR_FR_DIR'],
                self.PINS['MOTOR_BACK_DIR'],
                self.PINS['LIFTER_DIR']
            ]
            motor_pwm_pins = [
                self.PINS['MOTOR_FR_PWM'],
                self.PINS['MOTOR_BACK_PWM'],
                self.PINS['LIFTER_PWM']
            ]
            
            # Encoder pins (DATA A and B - read only)
            encoder_a_pins = [
                self.PINS['MOTOR_FL_ENCODER_A'],
                self.PINS['MOTOR_FR_ENCODER_A'],
                self.PINS['MOTOR_BACK_ENCODER_A'],
                self.PINS['LIFTER_ENCODER_A']
            ]
            encoder_b_pins = [
                self.PINS['MOTOR_FL_ENCODER_B'],
                self.PINS['MOTOR_FR_ENCODER_B'],
                self.PINS['MOTOR_BACK_ENCODER_B'],
                self.PINS['LIFTER_ENCODER_B']
            ]

            container_pins = [
                self.PINS['CONTAINER_LF'], self.PINS['CONTAINER_LB'],
                self.PINS['CONTAINER_RF'], self.PINS['CONTAINER_RB']
            ]

            # Claim servo pins as outputs
            # Front Left motor uses IN1/IN2/ENA
            fl_motor_pins = [motor_fl_in1_pin, motor_fl_in2_pin, motor_fl_ena_pin]
            all_output_pins = servo_pins + fl_motor_pins + motor_dir_pins + motor_pwm_pins + container_pins
            for pin in all_output_pins:
                lgpio.gpio_claim_output(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as output')
            
            # Encoder pins: Input only (read encoder feedback)
            for pin in encoder_a_pins + encoder_b_pins:
                lgpio.gpio_claim_input(self.gpio_handle, pin)
                self.get_logger().info(f'✓ GPIO{pin} claimed as input (Encoder)')
            
            # Initialize all motors to STOP
            # Front Left: IN1=0, IN2=0, ENA=0
            lgpio.gpio_write(self.gpio_handle, motor_fl_in1_pin, 0)
            lgpio.gpio_write(self.gpio_handle, motor_fl_in2_pin, 0)
            lgpio.gpio_write(self.gpio_handle, motor_fl_ena_pin, 0)
            # Other motors: DIR=0, PWM=0
            for dir_pin in motor_dir_pins:
                lgpio.gpio_write(self.gpio_handle, dir_pin, 0)
            for pwm_pin in motor_pwm_pins:
                lgpio.gpio_write(self.gpio_handle, pwm_pin, 0)
            
            all_pins = all_output_pins + encoder_a_pins + encoder_b_pins

            self.gpio_initialized = True
            self.get_logger().info('✓ GPIO Controller initialized successfully with lgpio!')
            self.get_logger().info(f'  - Total pins initialized: {len(all_pins)}')
            self.get_logger().info('  - Servos: 3 pins')
            self.get_logger().info('  - Front Left Motor: IN1/IN2/ENA (GPIO17/10/11)')
            self.get_logger().info('  - Other Motors: DIR+PWM (6 pins)')
            self.get_logger().info('  - Encoders: 8 pins (read only), Containers: 4 pins')
            self.get_logger().info('  - Note: PG23 motors have built-in encoders - controlled via L298N drivers')
            self.get_logger().info('  - All motors initialized to STOP state')

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
        """Execute robot movement using L298N motor drivers"""
        if not self.gpio_initialized:
            self.get_logger().error("GPIO not initialized")
            return
            
        # Clamp speed
        speed = max(0.0, min(1.0, speed))
        pwm_value = 1 if speed > 0.5 else 0  # Simple on/off for now (can use PWM later)
        
        try:
            if direction == 'stop':
                self.stop_robot()
            elif direction == 'forward':
                # All motors forward
                # Front Left: IN1=1, IN2=0, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors: DIR=1, PWM=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], pwm_value)
                self.get_logger().info(f"Moving forward at speed {speed:.2f}")
            elif direction == 'backward':
                # All motors reverse
                # Front Left: IN1=0, IN2=1, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors: DIR=0, PWM=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], pwm_value)
                self.get_logger().info(f"Moving backward at speed {speed:.2f}")
            elif direction == 'strafe_left':
                # Front left reverse, front right forward, back stopped
                # Front Left: IN1=0, IN2=1, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], 0)
                self.get_logger().info(f"Strafing left at speed {speed:.2f}")
            elif direction == 'strafe_right':
                # Front left forward, front right reverse, back stopped
                # Front Left: IN1=1, IN2=0, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], 0)
                self.get_logger().info(f"Strafing right at speed {speed:.2f}")
            elif direction == 'turn_left':
                # FL backward, FR forward, Back forward
                # Front Left: IN1=0, IN2=1, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], pwm_value)
                self.get_logger().info(f"Turning left at speed {speed:.2f}")
            elif direction == 'turn_right':
                # FL forward, FR backward, Back reverse
                # Front Left: IN1=1, IN2=0, ENA=1
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN1'], 1)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_IN2'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], pwm_value)
                # Other motors
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], pwm_value)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_DIR'], 0)
                lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], pwm_value)
                self.get_logger().info(f"Turning right at speed {speed:.2f}")
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")

    def stop_robot(self):
        """Stop all robot motors using L298N motor drivers"""
        if not self.gpio_initialized:
            self.get_logger().warn("GPIO not initialized - cannot stop motors")
            return
        
        try:
            # Stop Front Left motor: ENA=0
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FL_ENA'], 0)
            # Stop other motors: PWM=0
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_FR_PWM'], 0)
            lgpio.gpio_write(self.gpio_handle, self.PINS['MOTOR_BACK_PWM'], 0)
            lgpio.gpio_write(self.gpio_handle, self.PINS['LIFTER_PWM'], 0)
            self.get_logger().info("Robot stopped - all motors stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping robot: {e}")

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
            # Stop all motors before cleanup (set PWM pins to 0)
            if self.gpio_initialized:
                self.get_logger().info('Stopping all motors before cleanup...')
                self.stop_robot()
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
