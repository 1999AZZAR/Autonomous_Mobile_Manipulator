#!/usr/bin/env python3
"""
Test Front Left Motor with IN1, IN2, ENA configuration
"""

import lgpio
import time
import sys

def main():
    gpio_handle = None
    try:
        print("=" * 60)
        print("Front Left Motor Test - IN1, IN2, ENA Configuration")
        print("=" * 60)
        print("\nWiring:")
        print("  IN1 → GPIO17")
        print("  IN2 → GPIO10 (SPI MOSI - temporarily reassigned)")
        print("  ENA → GPIO27")
        print("\nL298N Control:")
        print("  Forward: IN1=HIGH, IN2=LOW, ENA=HIGH")
        print("  Reverse: IN1=LOW, IN2=HIGH, ENA=HIGH")
        print("  Stop: ENA=LOW")
        print("=" * 60)

        gpio_handle = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")

        IN1_PIN = 17  # GPIO17
        IN2_PIN = 10  # GPIO10 (SPI MOSI - temporarily used)
        ENA_PIN = 27  # GPIO27

        print(f"\nConfiguring pins:")
        print(f"  IN1: GPIO{IN1_PIN}")
        print(f"  IN2: GPIO{IN2_PIN}")
        print(f"  ENA: GPIO{ENA_PIN}")

        lgpio.gpio_claim_output(gpio_handle, IN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, IN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, ENA_PIN)
        print("✓ Pins configured as outputs")

        # Ensure motor is stopped initially
        lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)

        print("\n" + "=" * 60)
        print("TEST 1: Stop Motor (ENA=0)")
        print("=" * 60)
        print("Setting IN1=0, IN2=0, ENA=0...")
        time.sleep(3)
        print("(Motor should be STOPPED)")

        print("\n" + "=" * 60)
        print("TEST 2: Forward (IN1=1, IN2=0, ENA=1)")
        print("=" * 60)
        print("Setting IN1=1, IN2=0, ENA=1...")
        lgpio.gpio_write(gpio_handle, IN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        print("Waiting 5 seconds - CHECK IF MOTOR RUNS FORWARD")
        time.sleep(5)

        print("\n" + "=" * 60)
        print("TEST 3: Stop Again (ENA=0)")
        print("=" * 60)
        print("Setting ENA=0...")
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        time.sleep(3)
        print("(Motor should STOP)")

        print("\n" + "=" * 60)
        print("TEST 4: Reverse (IN1=0, IN2=1, ENA=1)")
        print("=" * 60)
        print("Setting IN1=0, IN2=1, ENA=1...")
        lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, IN2_PIN, 1)
        lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
        print("Waiting 5 seconds - CHECK IF MOTOR RUNS REVERSE")
        time.sleep(5)

        print("\n" + "=" * 60)
        print("TEST 5: Final Stop (ENA=0)")
        print("=" * 60)
        print("Setting ENA=0...")
        lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
        time.sleep(3)
        print("(Motor should STOP)")

        print("\n" + "=" * 60)
        print("Diagnostic Complete")
        print("=" * 60)
        print("\nResults:")
        print("  - If motor stopped in TEST 1 and TEST 3: ENA control works ✓")
        print("  - If motor ran forward in TEST 2: Forward direction works ✓")
        print("  - If motor ran reverse in TEST 4: Reverse direction works ✓")
        print("\n  - If motor did NOT move: Check L298N power (VS=12V, VCC=5V)")
        print("  - If motor did NOT change direction: Check IN1/IN2 connections")
        print("  - If motor spins freely: Check ENA pin connection")

    except Exception as e:
        print(f"✗ An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if gpio_handle is not None:
            try:
                lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
                lgpio.gpio_free(gpio_handle, IN1_PIN)
                lgpio.gpio_free(gpio_handle, IN2_PIN)
                lgpio.gpio_free(gpio_handle, ENA_PIN)
                lgpio.gpiochip_close(gpio_handle)
                print("\n✓ GPIO cleaned up")
            except:
                pass

if __name__ == '__main__':
    main()

