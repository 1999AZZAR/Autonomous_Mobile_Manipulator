#!/usr/bin/env python3
"""
Final Motor Test - After connecting L298N VCC to 5V
"""

import lgpio
import time

def main():
    print("=" * 70)
    print("Final Motor Control Test")
    print("=" * 70)
    print("\nPREREQUISITES:")
    print("  ✓ L298N VCC connected to 5V")
    print("  ✓ L298N VS connected to 12V")
    print("  ✓ L298N GND connected to common ground")
    print("  ✓ Motor M+ → L298N OUT1")
    print("  ✓ Motor M- → L298N OUT2")
    print("  ✓ IN1 → GPIO17, IN2 → GPIO10, ENA → GPIO11")
    print("=" * 70)
    
    gpio_handle = lgpio.gpiochip_open(0)
    
    IN1_PIN = 17
    IN2_PIN = 10
    ENA_PIN = 11
    
    lgpio.gpio_claim_output(gpio_handle, IN1_PIN)
    lgpio.gpio_claim_output(gpio_handle, IN2_PIN)
    lgpio.gpio_claim_output(gpio_handle, ENA_PIN)
    
    # Stop initially
    lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
    lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
    lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
    print("\n✓ Motor stopped initially")
    
    print("\n" + "=" * 70)
    print("TEST 1: Forward (IN1=1, IN2=0, ENA=1)")
    print("=" * 70)
    lgpio.gpio_write(gpio_handle, IN1_PIN, 1)
    lgpio.gpio_write(gpio_handle, IN2_PIN, 0)
    lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
    print("Motor should move FORWARD")
    print("Waiting 5 seconds...")
    time.sleep(5)
    
    print("\n" + "=" * 70)
    print("STOP (ENA=0)")
    print("=" * 70)
    lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
    print("Motor should STOP")
    time.sleep(2)
    
    print("\n" + "=" * 70)
    print("TEST 2: Reverse (IN1=0, IN2=1, ENA=1)")
    print("=" * 70)
    lgpio.gpio_write(gpio_handle, IN1_PIN, 0)
    lgpio.gpio_write(gpio_handle, IN2_PIN, 1)
    lgpio.gpio_write(gpio_handle, ENA_PIN, 1)
    print("Motor should move REVERSE")
    print("Waiting 5 seconds...")
    time.sleep(5)
    
    print("\n" + "=" * 70)
    print("FINAL STOP (ENA=0)")
    print("=" * 70)
    lgpio.gpio_write(gpio_handle, ENA_PIN, 0)
    print("Motor should STOP")
    
    print("\n" + "=" * 70)
    print("Test Complete")
    print("=" * 70)
    print("\nIf motor moved:")
    print("  ✓ L298N VCC connection is correct")
    print("  ✓ Motor control is working")
    print("  ✓ You can now use web_robot_interface.py")
    print("\nIf motor did NOT move:")
    print("  ✗ Check L298N VCC is connected to 5V")
    print("  ✗ Measure VCC pin voltage with multimeter (should be ~5V)")
    print("  ✗ Verify all wiring connections")
    
    lgpio.gpiochip_close(gpio_handle)

if __name__ == '__main__':
    main()

