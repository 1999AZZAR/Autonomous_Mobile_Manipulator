#!/usr/bin/env python3
"""
Dedicated Motor Control Test Script
Tests Front Left Motor with exact wiring configuration
"""

import lgpio
import time
import sys

# Pin definitions based on your wiring
IN1_PIN = 17   # GPIO17 (Pin 11) - Direction control
IN2_PIN = 10   # GPIO10 (Pin 19) - Direction control (optional but recommended)
ENA_PIN = 11   # GPIO11 (Pin 23) - Speed control
# NOTE: GPIO27 doesn't work (stays HIGH), using GPIO11 instead

def print_status(in1, in2, ena, description):
    """Print current pin status"""
    print(f"\n{'='*70}")
    print(f"{description}")
    print(f"{'='*70}")
    print(f"IN1 (GPIO{IN1_PIN}): {in1} ({'HIGH' if in1 else 'LOW'})")
    print(f"IN2 (GPIO{IN2_PIN}): {in2} ({'HIGH' if in2 else 'LOW'})")
    print(f"ENA (GPIO{ENA_PIN}): {ena} ({'HIGH' if ena else 'LOW'})")
    print(f"\nExpected Motor Behavior:")
    if ena == 0:
        print(f"  → Motor should be STOPPED")
    elif in1 == 1 and in2 == 0:
        print(f"  → Motor should move FORWARD")
    elif in1 == 0 and in2 == 1:
        print(f"  → Motor should move REVERSE")
    elif in1 == 1 and in2 == 1:
        print(f"  → Motor should BRAKE (stop quickly)")
    else:
        print(f"  → Motor should be STOPPED (coast)")

def set_motor(gpio_handle, in1_val, in2_val, ena_val, description, duration=5):
    """Set motor pins and wait"""
    print_status(in1_val, in2_val, ena_val, description)
    
    try:
        lgpio.gpio_write(gpio_handle, IN1_PIN, in1_val)
        lgpio.gpio_write(gpio_handle, IN2_PIN, in2_val)
        lgpio.gpio_write(gpio_handle, ENA_PIN, ena_val)
        
        # Verify pins
        in1_read = lgpio.gpio_read(gpio_handle, IN1_PIN)
        in2_read = lgpio.gpio_read(gpio_handle, IN2_PIN)
        ena_read = lgpio.gpio_read(gpio_handle, ENA_PIN)
        
        print(f"\nPin Verification:")
        print(f"  IN1 read: {in1_read} {'✓' if in1_read == in1_val else '✗ MISMATCH!'}")
        print(f"  IN2 read: {in2_read} {'✓' if in2_read == in2_val else '✗ MISMATCH!'}")
        print(f"  ENA read: {ena_read} {'✓' if ena_read == ena_val else '✗ MISMATCH!'}")
        
        if in1_read == in1_val and in2_read == in2_val and ena_read == ena_val:
            print(f"\n✓ Pins set correctly")
            print(f"  → Waiting {duration} seconds - OBSERVE MOTOR")
            print(f"  → Measure GPIO pin voltages with multimeter if needed")
            time.sleep(duration)
        else:
            print(f"\n✗ Pin mismatch detected!")
            time.sleep(2)
            
    except Exception as e:
        print(f"✗ Error setting pins: {e}")

def main():
    print("=" * 70)
    print("DEDICATED MOTOR CONTROL TEST")
    print("=" * 70)
    print("\nWiring Configuration:")
    print(f"  IN1 → GPIO{IN1_PIN} (Pin 11)")
    print(f"  IN2 → GPIO{IN2_PIN} (Pin 19)")
    print(f"  ENA → GPIO{ENA_PIN} (Pin 23) - NOTE: GPIO27 doesn't work, using GPIO11")
    print("\nL298N Power:")
    print("  VS → 12V Power Supply")
    print("  VCC → 5V Power Supply")
    print("  GND → Common Ground")
    print("\nMotor Connections:")
    print("  M+ → L298N OUT1")
    print("  M- → L298N OUT2")
    print("=" * 70)
    
    gpio_handle = None
    try:
        # Open GPIO
        gpio_handle = lgpio.gpiochip_open(0)
        print("\n✓ GPIO chip opened")
        
        # Configure pins as outputs
        print("\nConfiguring GPIO pins as outputs...")
        try:
            lgpio.gpio_claim_output(gpio_handle, IN1_PIN)
            print(f"  ✓ GPIO{IN1_PIN} (IN1) configured")
        except Exception as e:
            print(f"  ✗ GPIO{IN1_PIN} (IN1) failed: {e}")
            return
        
        try:
            lgpio.gpio_claim_output(gpio_handle, IN2_PIN)
            print(f"  ✓ GPIO{IN2_PIN} (IN2) configured")
        except Exception as e:
            print(f"  ✗ GPIO{IN2_PIN} (IN2) failed: {e}")
            print(f"  → Continuing without IN2 (motor may not work correctly)")
        
        try:
            lgpio.gpio_claim_output(gpio_handle, ENA_PIN)
            print(f"  ✓ GPIO{ENA_PIN} (ENA) configured")
        except Exception as e:
            print(f"  ✗ GPIO{ENA_PIN} (ENA) failed: {e}")
            print(f"  → ENA pin is critical - motor won't work without it!")
            return
        
        # Initialize all pins to LOW (motor stopped)
        lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        print("\n✓ All pins initialized to LOW (motor stopped)")
        print("\nStarting motor tests in 2 seconds...")
        time.sleep(2)
        
        # Test 1: Stop (ENA=0)
        set_motor(gpio_handle, 0, 0, 0, "TEST 1: STOP (ENA=0)", duration=3)
        
        # Test 2: Forward with IN1 only (if IN2 not working)
        set_motor(gpio_handle, 1, 0, 1, "TEST 2: FORWARD (IN1=1, IN2=0, ENA=1)", duration=5)
        
        # Test 3: Stop
        set_motor(gpio_handle, 0, 0, 0, "TEST 3: STOP (ENA=0)", duration=2)
        
        # Test 4: Reverse with IN2 only
        set_motor(gpio_handle, 0, 1, 1, "TEST 4: REVERSE (IN1=0, IN2=1, ENA=1)", duration=5)
        
        # Test 5: Stop
        set_motor(gpio_handle, 0, 0, 0, "TEST 5: STOP (ENA=0)", duration=2)
        
        # Test 6: Forward with both IN1 and IN2 (should be same as IN1 only)
        set_motor(gpio_handle, 1, 0, 1, "TEST 6: FORWARD again (IN1=1, IN2=0, ENA=1)", duration=5)
        
        # Test 7: Brake (both IN1 and IN2 HIGH)
        set_motor(gpio_handle, 1, 1, 1, "TEST 7: BRAKE (IN1=1, IN2=1, ENA=1)", duration=3)
        
        # Final stop
        set_motor(gpio_handle, 0, 0, 0, "FINAL: STOP (ENA=0)", duration=2)
        
        print("\n" + "=" * 70)
        print("TEST COMPLETE")
        print("=" * 70)
        print("\nResults:")
        print("  - If motor moved in TEST 2 or TEST 4:")
        print("    ✓ Motor control is WORKING")
        print("    ✓ L298N is responding to GPIO signals")
        print("\n  - If motor did NOT move:")
        print("    ✗ Check L298N VCC is connected to 5V (measure with multimeter)")
        print("    ✗ Check L298N VS is connected to 12V (measure with multimeter)")
        print("    ✗ Check motor connections (M+ to OUT1, M- to OUT2)")
        print("    ✗ Check GPIO pin connections")
        print("    ✗ Measure GPIO pin voltages with multimeter")
        print("\n  - If ENA pin (GPIO27) didn't work:")
        print("    → Try GPIO11 instead (update ENA_PIN in script)")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        if gpio_handle:
            try:
                lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
                print("✓ Motor stopped (ENA=0)")
            except:
                pass
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if gpio_handle:
            try:
                # Ensure motor is stopped
                lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
                lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
                lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
                lgpio.gpiochip_close(gpio_handle)
                print("\n✓ GPIO cleaned up - motor stopped")
            except:
                pass

if __name__ == '__main__':
    main()

