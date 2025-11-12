#!/usr/bin/env python3
"""
Test GPIO output voltage levels - verify Pi can control L298N
"""

import lgpio
import time

def main():
    print("=" * 70)
    print("GPIO Output Voltage Test")
    print("=" * 70)
    print("\nThis test verifies GPIO pins can output HIGH/LOW correctly.")
    print("If GPIO works but L298N doesn't respond, check:")
    print("  1. L298N VCC pin → 5V (required for logic level recognition)")
    print("  2. L298N might need 5V logic (Pi outputs 3.3V)")
    print("  3. Use multimeter to measure GPIO pin voltage")
    print("=" * 70)
    
    gpio_handle = lgpio.gpiochip_open(0)
    
    test_pins = [
        (17, "IN1"),
        (10, "IN2"),
        (11, "ENA")
    ]
    
    print("\nTesting GPIO pins:")
    for pin, name in test_pins:
        print(f"\n{name} (GPIO{pin}):")
        lgpio.gpio_claim_output(gpio_handle, pin)
        
        # Set HIGH
        lgpio.gpio_write(gpio_handle, pin, 1)
        time.sleep(0.2)
        level_high = lgpio.gpio_read(gpio_handle, pin)
        print(f"  HIGH: GPIO level = {level_high} (should be 1)")
        print(f"  → Measure pin voltage with multimeter: should be ~3.3V")
        time.sleep(1)
        
        # Set LOW
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(0.2)
        level_low = lgpio.gpio_read(gpio_handle, pin)
        print(f"  LOW: GPIO level = {level_low} (should be 0)")
        print(f"  → Measure pin voltage with multimeter: should be ~0V")
        time.sleep(1)
        
        if level_high == 1 and level_low == 0:
            print(f"  ✓ GPIO{pin} working correctly")
        else:
            print(f"  ✗ GPIO{pin} NOT working correctly!")
    
    print("\n" + "=" * 70)
    print("L298N Logic Level Check")
    print("=" * 70)
    print("\nMost L298N modules require:")
    print("  - VCC pin → 5V (powers the logic circuitry)")
    print("  - IN1/IN2/ENA can accept 3.3V signals IF VCC=5V")
    print("\nIf your L298N VCC is NOT connected to 5V:")
    print("  → Connect L298N VCC pin to 5V power supply")
    print("  → This enables the L298N to recognize 3.3V signals from Pi")
    print("\nIf VCC=5V but still not working:")
    print("  → L298N module might need 5V logic (use level shifter)")
    print("  → Or try different L298N module")
    
    # Final test: rapid toggle
    print("\n" + "=" * 70)
    print("Rapid Toggle Test (should see voltage change on multimeter)")
    print("=" * 70)
    pin = 17
    print(f"Toggling GPIO{pin} rapidly...")
    for i in range(10):
        lgpio.gpio_write(gpio_handle, pin, 1)
        time.sleep(0.1)
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(0.1)
    print("✓ Toggle complete - check multimeter for voltage changes")
    
    lgpio.gpiochip_close(gpio_handle)

if __name__ == '__main__':
    main()

