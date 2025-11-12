#!/usr/bin/env python3
"""
Comprehensive L298N diagnostic - test all pins and verify outputs
"""

import lgpio
import time
import sys

def test_pin_output(gpio_handle, pin, pin_name):
    """Test if a pin can be set HIGH and LOW"""
    try:
        print(f"\nTesting {pin_name} (GPIO{pin}):")
        lgpio.gpio_claim_output(gpio_handle, pin)
        
        # Set HIGH
        lgpio.gpio_write(gpio_handle, pin, 1)
        time.sleep(0.5)
        level = lgpio.gpio_read(gpio_handle, pin)
        print(f"  HIGH: GPIO level = {level} {'✓' if level == 1 else '✗ FAILED'}")
        
        # Set LOW
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(0.5)
        level = lgpio.gpio_read(gpio_handle, pin)
        print(f"  LOW: GPIO level = {level} {'✓' if level == 0 else '✗ FAILED'}")
        
        return True
    except Exception as e:
        print(f"  ✗ ERROR: {e}")
        return False

def main():
    gpio_handle = None
    try:
        print("=" * 70)
        print("L298N Comprehensive Diagnostic")
        print("=" * 70)
        
        gpio_handle = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")
        
        # Pin definitions
        IN1_PIN = 17
        IN2_PIN = 10
        ENA_PIN = 27
        
        print("\n" + "=" * 70)
        print("STEP 1: Test Individual Pin Outputs")
        print("=" * 70)
        
        # Test each pin individually
        test_pin_output(gpio_handle, IN1_PIN, "IN1")
        test_pin_output(gpio_handle, IN2_PIN, "IN2")
        test_pin_output(gpio_handle, ENA_PIN, "ENA")
        
        print("\n" + "=" * 70)
        print("STEP 2: Test All Motor Control Combinations")
        print("=" * 70)
        
        # Claim all pins as outputs
        lgpio.gpio_claim_output(gpio_handle, IN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, IN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, ENA_PIN)
        
        # Test combinations
        test_combinations = [
            ("STOP (ENA=0)", 0, 0, 0, 3),
            ("FORWARD (IN1=1, IN2=0, ENA=1)", 1, 0, 1, 5),
            ("STOP (ENA=0)", 0, 0, 0, 2),
            ("REVERSE (IN1=0, IN2=1, ENA=1)", 0, 1, 1, 5),
            ("STOP (ENA=0)", 0, 0, 0, 2),
            ("BRAKE (IN1=1, IN2=1, ENA=1)", 1, 1, 1, 3),
            ("STOP (ENA=0)", 0, 0, 0, 2),
        ]
        
        for test_name, in1_val, in2_val, ena_val, duration in test_combinations:
            print(f"\n{test_name}:")
            print(f"  Setting IN1={in1_val}, IN2={in2_val}, ENA={ena_val}")
            lgpio.gpio_write(gpio_handle, IN1_PIN, in1_val)
            lgpio.gpio_write(gpio_handle, IN2_PIN, in2_val)
            lgpio.gpio_write(gpio_handle, ENA_PIN, ena_val)
            
            # Verify pins
            in1_read = lgpio.gpio_read(gpio_handle, IN1_PIN)
            in2_read = lgpio.gpio_read(gpio_handle, IN2_PIN)
            ena_read = lgpio.gpio_read(gpio_handle, ENA_PIN)
            print(f"  Verified: IN1={in1_read}, IN2={in2_read}, ENA={ena_read}")
            
            if in1_read == in1_val and in2_read == in2_val and ena_read == ena_val:
                print(f"  ✓ Pins set correctly - waiting {duration}s - CHECK MOTOR")
            else:
                print(f"  ✗ Pin mismatch!")
            
            time.sleep(duration)
        
        print("\n" + "=" * 70)
        print("STEP 3: Hardware Checklist")
        print("=" * 70)
        print("\nPlease verify:")
        print("  1. L298N VS pin → 12V power supply (measure with multimeter)")
        print("  2. L298N VCC pin → 5V power supply (measure with multimeter)")
        print("  3. L298N GND → Common ground (shared with Raspberry Pi)")
        print("  4. Motor M+ → L298N OUT1")
        print("  5. Motor M- → L298N OUT2")
        print("  6. L298N IN1 → GPIO17 (Pin 11)")
        print("  7. L298N IN2 → GPIO10 (Pin 19)")
        print("  8. L298N ENA → GPIO27 (Pin 13)")
        print("\n  If all connections are correct but motor doesn't move:")
        print("    - Check if L298N module has jumpers on ENA/ENB (remove if present)")
        print("    - Try swapping OUT1 and OUT2 connections")
        print("    - Test motor directly with 12V (bypass L298N)")
        print("    - Check if L298N module is damaged")
        
        print("\n" + "=" * 70)
        print("STEP 4: Test ENA Pin Continuously HIGH")
        print("=" * 70)
        print("Setting ENA=HIGH continuously, then toggling IN1/IN2...")
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        
        for i in range(5):
            print(f"\nCycle {i+1}/5:")
            print("  IN1=1, IN2=0 (should be forward)")
            lgpio.gpio_write(gpio_handle, IN1_PIN, 1)
            lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
            time.sleep(2)
            
            print("  IN1=0, IN2=1 (should be reverse)")
            lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
            lgpio.gpio_write(gpio_handle, IN2_PIN, 1)
            time.sleep(2)
        
        # Final stop
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        print("\n✓ Test complete - motor should be stopped")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if gpio_handle is not None:
            try:
                lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
                lgpio.gpiochip_close(gpio_handle)
            except:
                pass

if __name__ == '__main__':
    main()

