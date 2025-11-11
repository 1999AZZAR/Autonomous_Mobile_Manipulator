#!/usr/bin/env python3
"""
Simple GPIO Test Script
Tests basic servo and motor control functionality
"""

import sys
import time

try:
    from gpiozero import Servo, AngularServo, Motor
    from gpiozero.pins.pigpio import PiGPIOFactory
    print("✓ gpiozero imported successfully")
except ImportError as e:
    print(f"✗ Failed to import gpiozero: {e}")
    print("\nPlease run: sudo apt install python3-gpiozero python3-pigpio")
    sys.exit(1)

def test_servo():
    """Test servo motor on GPIO18"""
    print("\n=== Testing Servo (GPIO18) ===")
    try:
        factory = PiGPIOFactory()
        servo = AngularServo(18, min_angle=0, max_angle=180, pin_factory=factory)
        
        print("Moving servo to 0°...")
        servo.angle = 0
        time.sleep(1)
        
        print("Moving servo to 90°...")
        servo.angle = 90
        time.sleep(1)
        
        print("Moving servo to 180°...")
        servo.angle = 180
        time.sleep(1)
        
        print("Returning to 90°...")
        servo.angle = 90
        time.sleep(1)
        
        servo.close()
        print("✓ Servo test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Servo test failed: {e}")
        return False

def test_motor():
    """Test motor control on GPIO17/GPIO27"""
    print("\n=== Testing Motor (GPIO17/GPIO27) ===")
    try:
        factory = PiGPIOFactory()
        motor = Motor(forward=17, backward=27, pin_factory=factory)
        
        print("Motor forward at 50%...")
        motor.forward(0.5)
        time.sleep(2)
        
        print("Motor stop...")
        motor.stop()
        time.sleep(1)
        
        print("Motor backward at 50%...")
        motor.backward(0.5)
        time.sleep(2)
        
        print("Motor stop...")
        motor.stop()
        
        motor.close()
        print("✓ Motor test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Motor test failed: {e}")
        return False

def main():
    print("========================================")
    print("GPIO Hardware Test")
    print("========================================")
    print("\nThis script will test basic GPIO functionality.")
    print("Make sure you have connected:")
    print("  - Servo on GPIO18")
    print("  - Motor driver on GPIO17/GPIO27")
    print("")
    
    response = input("Continue with tests? (y/n): ")
    if response.lower() != 'y':
        print("Test cancelled.")
        return
    
    # Run tests
    results = []
    results.append(("Servo Test", test_servo()))
    results.append(("Motor Test", test_motor()))
    
    # Print summary
    print("\n========================================")
    print("Test Summary")
    print("========================================")
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name}: {status}")
    
    all_passed = all(result[1] for result in results)
    if all_passed:
        print("\n✓ All tests passed!")
        print("\nYou can now run the web interface:")
        print("  python3 web_robot_interface.py")
    else:
        print("\n✗ Some tests failed.")
        print("\nTroubleshooting:")
        print("  1. Check that pigpiod is running: sudo systemctl status pigpiod")
        print("  2. Verify wiring connections")
        print("  3. Check GPIO permissions: groups $USER | grep gpio")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()

