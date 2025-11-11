#!/usr/bin/env python3
"""
Test GPIO functionality from within ROS2 Docker container
This script checks if GPIO libraries are available and working
"""

import sys
import subprocess
import time

def check_system():
    """Check system information"""
    print("=" * 50)
    print("GPIO Container Test")
    print("=" * 50)

    # Check if we're in Docker
    try:
        with open('/proc/1/cgroup', 'r') as f:
            if 'docker' in f.read():
                print("✓ Running inside Docker container")
            else:
                print("⚠ Not running inside Docker container")
    except:
        print("⚠ Cannot determine if running in Docker")

    # Check CPU architecture
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()
            if 'Raspberry Pi' in cpuinfo:
                print("✓ Running on Raspberry Pi")
            else:
                print("⚠ Not running on Raspberry Pi - GPIO may not work")
                print("  CPU Info:", cpuinfo.split('\n')[4] if len(cpuinfo.split('\n')) > 4 else "Unknown")
    except:
        print("⚠ Cannot read CPU info")

def check_devices():
    """Check if GPIO devices are available"""
    print("\nChecking GPIO devices...")

    devices_to_check = [
        '/dev/gpiomem',
        '/dev/mem',
        '/dev/i2c-1',
        '/dev/spidev0.0'
    ]

    for device in devices_to_check:
        try:
            with open(device, 'r'):
                print(f"✓ {device} is accessible")
        except:
            print(f"✗ {device} is not accessible")

def check_processes():
    """Check if pigpiod is running"""
    print("\nChecking processes...")

    try:
        result = subprocess.run(['pgrep', 'pigpiod'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ pigpiod daemon is running")
        else:
            print("✗ pigpiod daemon is not running")
            print("  Try starting it with: sudo pigpiod")
    except:
        print("⚠ Cannot check pigpiod status")

def test_imports():
    """Test Python GPIO library imports"""
    print("\nTesting Python imports...")

    libraries = [
        'gpiozero',
        'pigpio',
        'RPi.GPIO',
        'smbus2',
        'spidev'
    ]

    for lib in libraries:
        try:
            __import__(lib)
            print(f"✓ {lib} imported successfully")
        except ImportError as e:
            print(f"✗ {lib} import failed: {e}")
        except Exception as e:
            print(f"⚠ {lib} import error: {e}")

def test_gpio_basic():
    """Test basic GPIO functionality"""
    print("\nTesting basic GPIO functionality...")

    try:
        from gpiozero import LED
        from gpiozero.pins.pigpio import PiGPIOFactory

        # Try to create a PiGPIOFactory
        factory = PiGPIOFactory()
        print("✓ PiGPIOFactory created successfully")

        # Try to create a test LED (won't actually work without hardware)
        # This tests if the factory can connect to pigpiod
        try:
            led = LED(17, pin_factory=factory)
            print("✓ GPIO LED object created successfully")
            led.close()
        except Exception as e:
            print(f"⚠ GPIO LED test failed (expected on non-Pi): {e}")

    except Exception as e:
        print(f"✗ GPIO basic test failed: {e}")

def test_pigpio_connection():
    """Test connection to pigpiod"""
    print("\nTesting pigpiod connection...")

    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            print("✓ Connected to pigpiod successfully")
            pi.stop()
        else:
            print("✗ Cannot connect to pigpiod")
    except Exception as e:
        print(f"✗ pigpio connection test failed: {e}")

def main():
    """Run all tests"""
    check_system()
    check_devices()
    check_processes()
    test_imports()
    test_gpio_basic()
    test_pigpio_connection()

    print("\n" + "=" * 50)
    print("Test Summary")
    print("=" * 50)
    print("If you're seeing mostly ✓ marks, GPIO should work!")
    print("If you're seeing ✗ marks, check:")
    print("  1. Are you running on a Raspberry Pi?")
    print("  2. Did you start the container with GPIO device access?")
    print("  3. Is pigpiod running?")
    print("  4. Are GPIO libraries installed?")
    print("\nFor Docker, use:")
    print("  docker run --device /dev/gpiomem --privileged ...")

if __name__ == '__main__':
    main()
