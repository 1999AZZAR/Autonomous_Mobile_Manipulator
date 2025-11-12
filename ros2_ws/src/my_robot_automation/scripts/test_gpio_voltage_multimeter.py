#!/usr/bin/env python3
"""
GPIO Voltage Test for Multimeter - Keeps pins HIGH for extended periods
"""

import lgpio
import time

def main():
    print("=" * 70)
    print("GPIO Voltage Test for Multimeter")
    print("=" * 70)
    print("\nThis test keeps GPIO pins HIGH for 10 seconds each")
    print("so you can measure voltage with a multimeter.")
    print("=" * 70)
    
    gpio_handle = lgpio.gpiochip_open(0)
    
    test_pins = [
        (17, "IN1"),
        (10, "IN2"),
        (11, "ENA")
    ]
    
    print("\nConfiguring pins as outputs...")
    for pin, name in test_pins:
        lgpio.gpio_claim_output(gpio_handle, pin)
        lgpio.gpio_write(gpio_handle, pin, 0)  # Start LOW
        print(f"  ✓ {name} (GPIO{pin}) configured")
    
    print("\n" + "=" * 70)
    print("TEST: Each pin will be HIGH for 10 seconds")
    print("Measure voltage on each pin with multimeter")
    print("Expected: ~3.3V when HIGH, ~0V when LOW")
    print("=" * 70)
    
    for pin, name in test_pins:
        print(f"\n{name} (GPIO{pin}):")
        print(f"  Setting LOW (0V) - wait 3 seconds...")
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(3)
        
        print(f"  Setting HIGH (3.3V) - measure NOW with multimeter!")
        print(f"  Pin should read ~3.3V for 10 seconds...")
        lgpio.gpio_write(gpio_handle, pin, 1)
        
        # Verify pin is HIGH
        level = lgpio.gpio_read(gpio_handle, pin)
        print(f"  GPIO level read: {level} {'✓' if level == 1 else '✗ ERROR'}")
        
        time.sleep(10)  # Keep HIGH for 10 seconds
        
        print(f"  Setting LOW (0V) - measure NOW with multimeter!")
        lgpio.gpio_write(gpio_handle, pin, 0)
        time.sleep(3)
        
        level = lgpio.gpio_read(gpio_handle, pin)
        print(f"  GPIO level read: {level} {'✓' if level == 0 else '✗ ERROR'}")
        print(f"  ✓ {name} test complete")
    
    print("\n" + "=" * 70)
    print("All pins LOW - test complete")
    print("=" * 70)
    print("\nResults:")
    print("  - If you measured ~3.3V when pins were HIGH:")
    print("    ✓ GPIO pins are outputting voltage correctly")
    print("    → Problem is L298N not recognizing signals (check VCC=5V)")
    print("\n  - If you measured 0V even when pins were HIGH:")
    print("    ✗ GPIO pins are NOT outputting voltage")
    print("    → Check GPIO pin connections")
    print("    → Check if pins are configured correctly")
    print("    → Try different GPIO pins")
    
    # Keep all LOW
    for pin, name in test_pins:
        lgpio.gpio_write(gpio_handle, pin, 0)
    
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

