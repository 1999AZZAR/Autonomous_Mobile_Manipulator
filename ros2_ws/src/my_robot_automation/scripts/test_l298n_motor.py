#!/usr/bin/env python3
"""
Test script for L298N motor control
Tests different control methods to verify L298N wiring
"""

import lgpio
import time
import sys

def test_l298n_control():
    """Test L298N motor control with different pin configurations"""
    
    print("=" * 60)
    print("L298N Motor Control Diagnostic Test")
    print("=" * 60)
    
    try:
        # Open GPIO chip
        h = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")
    except Exception as e:
        print(f"✗ Failed to open GPIO chip: {e}")
        return
    
    # Front Left Motor pins (from current configuration)
    DIR_PIN = 17  # GPIO17
    PWM_PIN = 27  # GPIO27
    
    print(f"\nTesting Front Left Motor:")
    print(f"  DIR pin: GPIO{DIR_PIN}")
    print(f"  PWM pin: GPIO{PWM_PIN}")
    
    try:
        # Configure pins as outputs
        lgpio.gpio_claim_output(h, DIR_PIN)
        lgpio.gpio_claim_output(h, PWM_PIN)
        print("✓ Pins configured as outputs")
    except Exception as e:
        print(f"✗ Failed to configure pins: {e}")
        lgpio.gpiochip_close(h)
        return
    
    print("\n" + "=" * 60)
    print("TEST 1: Stop Motor (PWM=0)")
    print("=" * 60)
    print("Setting DIR=0, PWM=0...")
    lgpio.gpio_write(h, DIR_PIN, 0)
    lgpio.gpio_write(h, PWM_PIN, 0)
    print("Waiting 5 seconds - CHECK IF MOTOR STOPS")
    print("(Motor should be STOPPED)")
    time.sleep(5)
    
    print("\n" + "=" * 60)
    print("TEST 2: Forward (DIR=1, PWM=1)")
    print("=" * 60)
    print("Setting DIR=1, PWM=1...")
    lgpio.gpio_write(h, DIR_PIN, 1)
    lgpio.gpio_write(h, PWM_PIN, 1)
    print("Waiting 3 seconds - CHECK MOTOR DIRECTION")
    print("(Motor should run FORWARD)")
    time.sleep(3)
    
    print("\n" + "=" * 60)
    print("TEST 3: Stop Again (PWM=0)")
    print("=" * 60)
    print("Setting PWM=0...")
    lgpio.gpio_write(h, PWM_PIN, 0)
    print("Waiting 3 seconds - CHECK IF MOTOR STOPS")
    print("(Motor should STOP)")
    time.sleep(3)
    
    print("\n" + "=" * 60)
    print("TEST 4: Reverse (DIR=0, PWM=1)")
    print("=" * 60)
    print("Setting DIR=0, PWM=1...")
    lgpio.gpio_write(h, DIR_PIN, 0)
    lgpio.gpio_write(h, PWM_PIN, 1)
    print("Waiting 3 seconds - CHECK MOTOR DIRECTION")
    print("(Motor should run REVERSE)")
    time.sleep(3)
    
    print("\n" + "=" * 60)
    print("TEST 5: Final Stop (PWM=0)")
    print("=" * 60)
    print("Setting PWM=0...")
    lgpio.gpio_write(h, PWM_PIN, 0)
    print("Waiting 3 seconds - CHECK IF MOTOR STOPS")
    print("(Motor should STOP)")
    time.sleep(3)
    
    print("\n" + "=" * 60)
    print("Diagnostic Complete")
    print("=" * 60)
    print("\nResults:")
    print("  - If motor stopped in TEST 1 and TEST 3: PWM control works ✓")
    print("  - If motor ran forward in TEST 2: Forward direction works ✓")
    print("  - If motor ran reverse in TEST 4: Reverse direction works ✓")
    print("\n  - If motor did NOT stop: Check L298N ENA pin connection")
    print("  - If motor did NOT change direction: Check L298N IN1/IN2 pin connection")
    print("  - If motor spins freely: L298N may not be connected or powered")
    
    # Cleanup
    lgpio.gpio_write(h, PWM_PIN, 0)
    lgpio.gpiochip_close(h)
    print("\n✓ GPIO cleaned up")

if __name__ == '__main__':
    try:
        test_l298n_control()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

