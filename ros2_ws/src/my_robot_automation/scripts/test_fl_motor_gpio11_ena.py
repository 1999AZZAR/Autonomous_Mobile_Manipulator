#!/usr/bin/env python3
"""
Test Front Left Motor with GPIO11 as ENA (GPIO27 not working)
"""

import lgpio
import time
import sys

def main():
    gpio_handle = None
    try:
        print("=" * 60)
        print("Front Left Motor Test - Using GPIO11 for ENA")
        print("=" * 60)
        print("\nWiring:")
        print("  IN1 → GPIO17")
        print("  IN2 → GPIO10")
        print("  ENA → GPIO11 (GPIO27 not working)")
        print("=" * 60)

        gpio_handle = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")

        IN1_PIN = 17
        IN2_PIN = 10
        ENA_PIN = 11  # Changed from GPIO27

        lgpio.gpio_claim_output(gpio_handle, IN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, IN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, ENA_PIN)
        print("✓ Pins configured")

        # Test ENA pin
        print("\nTesting ENA pin (GPIO11):")
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        time.sleep(0.5)
        ena_high = lgpio.gpio_read(gpio_handle, ENA_PIN)
        print(f"  HIGH: {ena_high} {'✓' if ena_high == 1 else '✗'}")
        
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        time.sleep(0.5)
        ena_low = lgpio.gpio_read(gpio_handle, ENA_PIN)
        print(f"  LOW: {ena_low} {'✓' if ena_low == 0 else '✗'}")

        # Stop initially
        lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        print("\n✓ Motor stopped initially")

        print("\n" + "=" * 60)
        print("TEST: Forward (IN1=1, IN2=0, ENA=1)")
        print("=" * 60)
        lgpio.gpio_write(gpio_handle, IN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        
        # Verify
        in1 = lgpio.gpio_read(gpio_handle, IN1_PIN)
        in2 = lgpio.gpio_read(gpio_handle, IN2_PIN)
        ena = lgpio.gpio_read(gpio_handle, ENA_PIN)
        print(f"Pins: IN1={in1}, IN2={in2}, ENA={ena}")
        print("Waiting 5 seconds - CHECK IF MOTOR MOVES FORWARD")
        time.sleep(5)

        print("\n" + "=" * 60)
        print("STOP (ENA=0)")
        print("=" * 60)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        time.sleep(2)

        print("\n" + "=" * 60)
        print("TEST: Reverse (IN1=0, IN2=1, ENA=1)")
        print("=" * 60)
        lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 1)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        
        # Verify
        in1 = lgpio.gpio_read(gpio_handle, IN1_PIN)
        in2 = lgpio.gpio_read(gpio_handle, IN2_PIN)
        ena = lgpio.gpio_read(gpio_handle, ENA_PIN)
        print(f"Pins: IN1={in1}, IN2={in2}, ENA={ena}")
        print("Waiting 5 seconds - CHECK IF MOTOR MOVES REVERSE")
        time.sleep(5)

        # Final stop
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        print("\n✓ Test complete")

    except Exception as e:
        print(f"✗ Error: {e}")
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

