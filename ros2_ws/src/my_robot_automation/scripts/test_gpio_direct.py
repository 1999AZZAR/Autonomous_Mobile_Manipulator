#!/usr/bin/env python3
"""
Direct GPIO Test - Verify pins actually output voltage
Tests with different methods to ensure GPIO is working
"""

import lgpio
import time

def test_pin_direct(gpio_handle, pin, name):
    """Test a pin directly with extended HIGH period"""
    print(f"\n{'='*70}")
    print(f"Testing {name} (GPIO{pin})")
    print(f"{'='*70}")
    
    try:
        # Claim as output
        lgpio.gpio_claim_output(gpio_handle, pin, lFlags=0)
        print(f"✓ Pin claimed as output")
        
        # Set LOW first
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(1)
        level_low = lgpio.gpio_read(gpio_handle, pin)
        print(f"LOW: GPIO level = {level_low}")
        print(f"  → Measure voltage NOW: Should be ~0V")
        print(f"  → Touch multimeter probe to GPIO{pin} pin on Pi")
        print(f"  → Touch other probe to Pi GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39)")
        time.sleep(5)
        
        # Set HIGH
        print(f"\nSetting HIGH...")
        lgpio.gpio_write(gpio_handle, pin, 1)
        time.sleep(0.5)
        level_high = lgpio.gpio_read(gpio_handle, pin)
        print(f"HIGH: GPIO level = {level_high}")
        print(f"  → Measure voltage NOW: Should be ~3.3V")
        print(f"  → Keep multimeter connected for 15 seconds...")
        print(f"  → If you see 0V, GPIO pin is NOT outputting voltage!")
        time.sleep(15)
        
        # Set LOW again
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(1)
        print(f"LOW again: GPIO level = {lgpio.gpio_read(gpio_handle, pin)}")
        print(f"  → Measure voltage NOW: Should be ~0V")
        time.sleep(3)
        
        if level_high == 1 and level_low == 0:
            print(f"\n✓ {name} GPIO control working (software)")
            print(f"  → If multimeter showed 0V when HIGH: Hardware issue!")
        else:
            print(f"\n✗ {name} GPIO control NOT working (software)")
            
    except Exception as e:
        print(f"✗ Error testing {name}: {e}")

def main():
    print("=" * 70)
    print("Direct GPIO Voltage Test")
    print("=" * 70)
    print("\nThis test will keep each pin HIGH for 15 seconds")
    print("so you can measure voltage with a multimeter.")
    print("\nIMPORTANT:")
    print("  - Connect multimeter red probe to GPIO pin")
    print("  - Connect multimeter black probe to Pi GND")
    print("  - Set multimeter to DC voltage mode")
    print("=" * 70)
    
    gpio_handle = lgpio.gpiochip_open(0)
    print("\n✓ GPIO chip opened")
    
    # Test pins
    test_pins = [
        (17, "IN1"),
        (10, "IN2"),
        (11, "ENA")
    ]
    
    # Also test a known working pin (like GPIO18 for PWM)
    # to verify multimeter setup is correct
    print("\n" + "=" * 70)
    print("FIRST: Test GPIO18 (known PWM pin) to verify multimeter setup")
    print("=" * 70)
    test_pin_direct(gpio_handle, 18, "GPIO18 (Test Pin)")
    
    # Now test motor control pins
    for pin, name in test_pins:
        test_pin_direct(gpio_handle, pin, name)
    
    print("\n" + "=" * 70)
    print("Test Complete")
    print("=" * 70)
    print("\nIf GPIO18 showed ~3.3V but motor pins showed 0V:")
    print("  → Motor control pins might be damaged or misconfigured")
    print("  → Try different GPIO pins")
    print("\nIf ALL pins showed 0V:")
    print("  → Check multimeter connections")
    print("  → Verify multimeter is set to DC voltage mode")
    print("  → Check if Pi GPIO is enabled in config")
    
    lgpio.gpiochip_close(gpio_handle)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

