#!/usr/bin/env python3
"""
Standalone GPIO Test - Tests GPIO functionality without ROS2 dependencies
"""

import sys
import time

# Try to import GPIO libraries
try:
    from gpiozero import Servo, Motor, AngularServo, OutputDevice
    from gpiozero.pins.pigpio import PiGPIOFactory
    import pigpio
    GPIOZERO_AVAILABLE = True
except ImportError as e:
    GPIOZERO_AVAILABLE = False
    print(f"WARNING: gpiozero not available: {e}")

try:
    import RPi.GPIO as GPIO
    RPI_GPIO_AVAILABLE = True
except ImportError as e:
    RPI_GPIO_AVAILABLE = False
    print(f"WARNING: RPi.GPIO not available: {e}")

class GPIOControllerTest:
    """Test version of GPIOController without ROS2 dependencies"""

    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        self.gpio_initialized = False

        # GPIO pin definitions
        self.PINS = {
            'GRIPPER_TILT': 18,
            'GRIPPER_OPEN_CLOSE': 19,
            'GRIPPER_NECK': 21,
            'GRIPPER_BASE': 12,
            'MOTOR_FL_DIR': 17,
            'MOTOR_FL_PWM': 27,
            'MOTOR_FR_DIR': 22,
            'MOTOR_FR_PWM': 23,
            'MOTOR_BACK_DIR': 24,
            'MOTOR_BACK_PWM': 25,
        }

        # Hardware objects
        self.servos = {}
        self.motors = {}

        # Initialize GPIO if not in simulation mode
        if not self.simulation_mode and GPIOZERO_AVAILABLE:
            try:
                print("Attempting to initialize GPIO...")

                # Check system
                try:
                    with open('/proc/cpuinfo', 'r') as f:
                        cpuinfo = f.read()
                        if 'Raspberry Pi' not in cpuinfo:
                            print("WARNING: Not running on Raspberry Pi")
                except:
                    print("WARNING: Cannot check CPU info")

                # Use pigpio for better PWM control
                factory = PiGPIOFactory()

                # Test basic servo
                print("Testing servo creation...")
                self.servos['test_servo'] = AngularServo(
                    self.PINS['GRIPPER_TILT'],
                    min_angle=0,
                    max_angle=180,
                    pin_factory=factory
                )

                # Test basic motor
                print("Testing motor creation...")
                self.motors['test_motor'] = Motor(
                    forward=self.PINS['MOTOR_FL_DIR'],
                    backward=self.PINS['MOTOR_FL_PWM'],
                    pin_factory=factory
                )

                self.gpio_initialized = True
                print("✓ GPIO initialized successfully!")

            except Exception as e:
                print(f"✗ GPIO initialization failed: {e}")
                import traceback
                traceback.print_exc()
                self.simulation_mode = True
        else:
            print("Running in SIMULATION mode")

def main():
    print("=" * 50)
    print("Standalone GPIO Test")
    print("=" * 50)

    print(f"gpiozero available: {GPIOZERO_AVAILABLE}")
    print(f"RPi.GPIO available: {RPI_GPIO_AVAILABLE}")

    # Test simulation mode
    print("\n--- Testing Simulation Mode ---")
    gpio_sim = GPIOControllerTest(simulation_mode=True)
    print(f"Simulation mode: {gpio_sim.simulation_mode}")
    print(f"GPIO initialized: {gpio_sim.gpio_initialized}")

    # Test hardware mode (will likely fail on non-Pi systems)
    print("\n--- Testing Hardware Mode ---")
    gpio_hw = GPIOControllerTest(simulation_mode=False)
    print(f"Simulation mode: {gpio_hw.simulation_mode}")
    print(f"GPIO initialized: {gpio_hw.gpio_initialized}")

    if gpio_hw.gpio_initialized:
        print("\n✓ GPIO is working! ROS2 should be able to control hardware.")
    else:
        print("\n⚠ GPIO not working - check setup for Raspberry Pi deployment.")
        print("This is expected on development machines.")

    print("\n" + "=" * 50)
    print("Test Complete")
    print("=" * 50)

if __name__ == '__main__':
    main()
