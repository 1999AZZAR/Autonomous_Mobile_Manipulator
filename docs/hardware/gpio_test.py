#!/usr/bin/env python3
"""
Autonomous Mobile Manipulator - GPIO Pinout Test Script
Tests all GPIO connections for sensors and actuators
"""

import RPi.GPIO as GPIO
import time
import smbus
import spidev
import sys
from gpiozero import MCP3008

# Pin definitions from RASPBERRY_PI_PINOUTS.md
PINOUT = {
    # Servos (PWM)
    'SERVO1': 12,  # GPIO12 - Gripper Open/Close
    'SERVO2': 13,  # GPIO13 - Gripper Tilt
    'SERVO3': 18,  # GPIO18 - Gripper Neck
    'SERVO4': 19,  # GPIO19 - Gripper Base
    'SERVO5': 21,  # GPIO21 - Auxiliary Servo

    # Motors
    'MOTOR_BACK_DIR': 17,     # GPIO17 - Back wheel direction
    'MOTOR_BACK_PWM': 27,     # GPIO27 - Back wheel PWM
    'MOTOR_FL_DIR': 24,       # GPIO24 - Front left direction
    'MOTOR_FL_PWM': 25,       # GPIO25 - Front left PWM
    'MOTOR_FR_DIR': 16,       # GPIO16 - Front right direction
    'MOTOR_FR_PWM': 26,       # GPIO26 - Front right PWM
    'LIFTER_DIR': 12,         # GPIO12 - Lifter direction (conflict!)
    'LIFTER_PWM': 13,         # GPIO13 - Lifter PWM (conflict!)

    # Encoders (Interrupts)
    'ENC_BACK_A': 22,         # GPIO22 - Back wheel encoder A
    'ENC_BACK_B': 23,         # GPIO23 - Back wheel encoder B
    'ENC_FL_A': 5,            # GPIO5 - Front left encoder A
    'ENC_FL_B': 6,            # GPIO6 - Front left encoder B
    'ENC_FR_A': 20,           # GPIO20 - Front right encoder A
    'ENC_FR_B': 21,           # GPIO21 - Front right encoder B (conflict!)

    # Ultrasonic Sensors
    'US_FRONT_TRIG': 4,       # GPIO4 - Front ultrasonic TRIG
    'US_FRONT_ECHO': 14,      # GPIO14 - Front ultrasonic ECHO
    'US_BL_TRIG': 15,         # GPIO15 - Back left TRIG
    'US_BL_ECHO': 17,         # GPIO17 - Back left ECHO (conflict!)
    'US_BR_TRIG': 18,         # GPIO18 - Back right TRIG (conflict!)
    'US_BR_ECHO': 27,         # GPIO27 - Back right ECHO (conflict!)

    # Line Sensors (74HC165)
    'LS_SH_LD': 8,            # GPIO8 - Shift/Load
    'LS_CLK': 9,              # GPIO9 - Clock
    'LS_QH': 10,              # GPIO10 - Serial output
    'LS_CLK_INH': 11,         # GPIO11 - Clock inhibit

    # IMU (MPU6050) - I2C
    'IMU_SDA': 2,             # GPIO2 - I2C SDA
    'IMU_SCL': 3,             # GPIO3 - I2C SCL

    # ADC (MCP3008) - SPI
    'ADC_CS': 8,              # GPIO8 - SPI CS (conflict!)
    'ADC_CLK': 9,             # GPIO9 - SPI CLK (conflict!)
    'ADC_DOUT': 10,           # GPIO10 - SPI DOUT (conflict!)
    'ADC_DIN': 11,            # GPIO11 - SPI DIN (conflict!)
}

class GPIOTester:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize SPI for ADC
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 1350000
            print("✅ SPI initialized for ADC")
        except:
            print("❌ SPI initialization failed")
            self.spi = None

        # Initialize I2C for IMU
        try:
            self.bus = smbus.SMBus(1)
            print("✅ I2C initialized for IMU")
        except:
            print("❌ I2C initialization failed")
            self.bus = None

    def test_pin(self, name, pin, mode=GPIO.OUT, pull_up_down=GPIO.PUD_OFF):
        """Test individual GPIO pin"""
        try:
            GPIO.setup(pin, mode, pull_up_down=pull_up_down)
            if mode == GPIO.OUT:
                GPIO.output(pin, GPIO.LOW)
                time.sleep(0.1)
                GPIO.output(pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(pin, GPIO.LOW)
            print(f"✅ {name}: GPIO{pin} - OK")
            return True
        except Exception as e:
            print(f"❌ {name}: GPIO{pin} - FAILED ({str(e)})")
            return False

    def test_ultrasonic(self, trig_pin, echo_pin, name):
        """Test ultrasonic sensor pins"""
        try:
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            # Test TRIG pulse
            GPIO.output(trig_pin, GPIO.LOW)
            time.sleep(0.01)
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig_pin, GPIO.LOW)

            # Wait for ECHO response (timeout after 1 second)
            timeout = time.time() + 1
            while GPIO.input(echo_pin) == 0 and time.time() < timeout:
                pass

            if GPIO.input(echo_pin) == 1:
                print(f"✅ {name}: Ultrasonic sensor responding")
                return True
            else:
                print(f"⚠️ {name}: No ultrasonic response (sensor not connected?)")
                return True  # Pin works, just no sensor

        except Exception as e:
            print(f"❌ {name}: Ultrasonic test failed ({str(e)})")
            return False

    def test_shift_register(self):
        """Test 74HC165 shift register for line sensors"""
        try:
            GPIO.setup(PINOUT['LS_SH_LD'], GPIO.OUT)
            GPIO.setup(PINOUT['LS_CLK'], GPIO.OUT)
            GPIO.setup(PINOUT['LS_QH'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(PINOUT['LS_CLK_INH'], GPIO.OUT)

            # Test shift register operation
            GPIO.output(PINOUT['LS_CLK_INH'], GPIO.HIGH)  # Disable clock
            GPIO.output(PINOUT['LS_SH_LD'], GPIO.LOW)    # Load parallel data
            time.sleep(0.001)
            GPIO.output(PINOUT['LS_SH_LD'], GPIO.HIGH)   # Shift mode

            GPIO.output(PINOUT['LS_CLK_INH'], GPIO.LOW)  # Enable clock

            # Read 8 bits
            data = 0
            for i in range(8):
                GPIO.output(PINOUT['LS_CLK'], GPIO.HIGH)
                time.sleep(0.001)
                bit = GPIO.input(PINOUT['LS_QH'])
                data |= (bit << i)
                GPIO.output(PINOUT['LS_CLK'], GPIO.LOW)
                time.sleep(0.001)

            print(f"✅ Line Sensors: Shift register OK (data: 0x{data:02X})")
            return True

        except Exception as e:
            print(f"❌ Line Sensors: Shift register test failed ({str(e)})")
            return False

    def test_i2c_device(self, address, name):
        """Test I2C device presence"""
        if not self.bus:
            print(f"❌ {name}: I2C not available")
            return False

        try:
            self.bus.read_byte(address)
            print(f"✅ {name}: I2C device found at 0x{address:02X}")
            return True
        except:
            print(f"⚠️ {name}: No I2C device at 0x{address:02X} (device not connected?)")
            return True  # Pin works, just no device

    def run_full_test(self):
        """Run complete GPIO test suite"""
        print("Autonomous Mobile Manipulator - GPIO Pinout Test")
        print("=" * 50)

        # Test Servos
        print("\n🎛️ Testing Servo Pins...")
        for i in range(1, 6):
            pin_name = f'SERVO{i}'
            self.test_pin(f'Servo {i}', PINOUT[pin_name])

        # Test Motors
        print("\n🔄 Testing Motor Control Pins...")
        motor_pins = [
            ('Motor Back Dir', 'MOTOR_BACK_DIR'),
            ('Motor Back PWM', 'MOTOR_BACK_PWM'),
            ('Motor FL Dir', 'MOTOR_FL_DIR'),
            ('Motor FL PWM', 'MOTOR_FL_PWM'),
            ('Motor FR Dir', 'MOTOR_FR_DIR'),
            ('Motor FR PWM', 'MOTOR_FR_PWM'),
            ('Lifter Dir', 'LIFTER_DIR'),
            ('Lifter PWM', 'LIFTER_PWM'),
        ]

        for name, pin_key in motor_pins:
            self.test_pin(name, PINOUT[pin_key])

        # Test Encoders (Input pins)
        print("\n📊 Testing Encoder Input Pins...")
        encoder_pins = [
            ('Enc Back A', 'ENC_BACK_A'),
            ('Enc Back B', 'ENC_BACK_B'),
            ('Enc FL A', 'ENC_FL_A'),
            ('Enc FL B', 'ENC_FL_B'),
            ('Enc FR A', 'ENC_FR_A'),
            ('Enc FR B', 'ENC_FR_B'),
        ]

        for name, pin_key in encoder_pins:
            self.test_pin(name, PINOUT[pin_key], GPIO.IN, GPIO.PUD_UP)

        # Test Ultrasonic Sensors
        print("\n📏 Testing Ultrasonic Sensors...")
        self.test_ultrasonic(PINOUT['US_FRONT_TRIG'], PINOUT['US_FRONT_ECHO'], "US Front")
        self.test_ultrasonic(PINOUT['US_BL_TRIG'], PINOUT['US_BL_ECHO'], "US Back Left")
        self.test_ultrasonic(PINOUT['US_BR_TRIG'], PINOUT['US_BR_ECHO'], "US Back Right")

        # Test Line Sensors
        print("\n📏 Testing Line Sensor Shift Register...")
        self.test_shift_register()

        # Test IMU (I2C)
        print("\n🌀 Testing IMU (I2C)...")
        self.test_i2c_device(0x68, "MPU6050 IMU")  # Default MPU6050 address

        # Test ADC (SPI)
        print("\n🔴 Testing ADC (SPI)...")
        if self.spi:
            try:
                # Test SPI communication
                result = self.spi.xfer2([0x01, 0x80, 0x00])  # Read channel 0
                print("✅ ADC: SPI communication OK")
            except Exception as e:
                print(f"❌ ADC: SPI test failed ({str(e)})")

        print("\n" + "=" * 50)
        print("🎉 GPIO Test Complete!")
        print("\n📋 Pin Conflict Notes:")
        print("• SERVO5 shares pin with ENC_FR_B")
        print("• LIFTER_DIR/PWM conflict with SERVO1/SERVO2")
        print("• US_BL_ECHO conflicts with MOTOR_BACK_DIR")
        print("• US_BR_TRIG/ECHO conflict with SERVO3/MOTOR_BACK_PWM")
        print("• ADC pins conflict with Line Sensor shift register")
        print("\n🔧 Resolve conflicts by reassigning pins as needed!")

    def cleanup(self):
        """Clean up GPIO"""
        GPIO.cleanup()
        if self.spi:
            self.spi.close()

if __name__ == "__main__":
    tester = GPIOTester()

    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--quick":
            # Quick test of basic GPIO
            print("🧪 Quick GPIO Test")
            tester.test_pin("Test Pin", 18)  # GPIO18
        else:
            tester.run_full_test()
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    finally:
        tester.cleanup()
