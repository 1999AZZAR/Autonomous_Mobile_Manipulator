#!/usr/bin/env python3
"""
Test script to find how to disable PG23 motor
Tests different DATA pin states to see which stops the motor
"""

import lgpio
import time

def test_pin_states():
    """Test different DATA pin states to disable motor"""
    h = lgpio.gpiochip_open(0)
    tx_pin = 17  # Front left motor DATA(A)
    rx_pin = 27  # Front left motor DATA(B)
    
    print("=" * 60)
    print("PG23 Motor Disable Test")
    print("=" * 60)
    print(f"DATA(A) pin: GPIO{tx_pin}")
    print(f"DATA(B) pin: GPIO{rx_pin}")
    print()
    
    # Configure pins
    lgpio.gpio_claim_output(h, tx_pin)
    lgpio.gpio_claim_output(h, rx_pin)  # Try as output too
    
    test_cases = [
        ("Both LOW", 0, 0),
        ("DATA(A) LOW, DATA(B) HIGH", 0, 1),
        ("DATA(A) HIGH, DATA(B) LOW", 1, 0),
        ("Both HIGH", 1, 1),
    ]
    
    for name, tx_state, rx_state in test_cases:
        print(f"\n{'='*60}")
        print(f"TEST: {name}")
        print(f"  DATA(A) = {tx_state} (LOW=0, HIGH=1)")
        print(f"  DATA(B) = {rx_state} (LOW=0, HIGH=1)")
        print(f"{'='*60}")
        
        lgpio.gpio_write(h, tx_pin, tx_state)
        lgpio.gpio_write(h, rx_pin, rx_state)
        
        print("Pin states set. Check if motor stopped.")
        print("Waiting 5 seconds...")
        time.sleep(5)
        
        response = input("Did the motor stop? (y/n): ").strip().lower()
        if response == 'y':
            print(f"\n✓ SUCCESS! Motor stops when: {name}")
            print(f"  DATA(A) = {tx_state}, DATA(B) = {rx_state}")
            break
    
    print("\n" + "="*60)
    print("Test complete")
    print("="*60)
    
    # Keep pins in last tested state
    print("\nKeeping pins in last tested state...")
    time.sleep(2)
    
    lgpio.gpiochip_close(h)

if __name__ == "__main__":
    try:
        test_pin_states()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    except Exception as e:
        print(f"\nERROR: {e}")

