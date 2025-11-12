#!/usr/bin/env python3
"""
Test L298N with standard IN1, IN2, ENA configuration
"""

import lgpio
import time
import sys

def test_l298n_standard():
    """Test L298N with IN1, IN2, ENA pins"""
    
    print("=" * 60)
    print("L298N Standard Configuration Test (IN1, IN2, ENA)")
    print("=" * 60)
    
    try:
        h = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")
    except Exception as e:
        print(f"✗ Failed to open GPIO chip: {e}")
        return
    
    # Try different pin combinations
    # Option 1: Use GPIO17 as IN1, GPIO27 as IN2, GPIO22 as ENA
    # Option 2: Use GPIO17 as IN1, GPIO27 as ENA (IN2 tied to GND)
    
    print("\nTesting Front Left Motor with different configurations...")
    
    # Configuration 1: IN1=GPIO17, IN2=GPIO27, ENA=GPIO22
    print("\n" + "=" * 60)
    print("CONFIGURATION 1: IN1=GPIO17, IN2=GPIO27, ENA=GPIO22")
    print("=" * 60)
    
    IN1_PIN = 17
    IN2_PIN = 27
    ENA_PIN = 22
    
    try:
        lgpio.gpio_claim_output(h, IN1_PIN)
        lgpio.gpio_claim_output(h, IN2_PIN)
        lgpio.gpio_claim_output(h, ENA_PIN)
        print("✓ Pins configured")
        
        # Test 1: Stop (IN1=0, IN2=0, ENA=0)
        print("\nTEST 1: Stop (IN1=0, IN2=0, ENA=0)")
        lgpio.gpio_write(h, IN1_PIN, 0)
        lgpio.gpio_write(h, IN2_PIN, 0)
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("Waiting 3 seconds...")
        time.sleep(3)
        
        # Test 2: Forward (IN1=1, IN2=0, ENA=1)
        print("\nTEST 2: Forward (IN1=1, IN2=0, ENA=1)")
        lgpio.gpio_write(h, IN1_PIN, 1)
        lgpio.gpio_write(h, IN2_PIN, 0)
        lgpio.gpio_write(h, ENA_PIN, 1)
        print("Waiting 3 seconds - CHECK IF MOTOR RUNS FORWARD")
        time.sleep(3)
        
        # Test 3: Stop
        print("\nTEST 3: Stop (ENA=0)")
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("Waiting 2 seconds...")
        time.sleep(2)
        
        # Test 4: Reverse (IN1=0, IN2=1, ENA=1)
        print("\nTEST 4: Reverse (IN1=0, IN2=1, ENA=1)")
        lgpio.gpio_write(h, IN1_PIN, 0)
        lgpio.gpio_write(h, IN2_PIN, 1)
        lgpio.gpio_write(h, ENA_PIN, 1)
        print("Waiting 3 seconds - CHECK IF MOTOR RUNS REVERSE")
        time.sleep(3)
        
        # Final stop
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("\n✓ Configuration 1 test complete")
        
    except Exception as e:
        print(f"✗ Configuration 1 failed: {e}")
    
    # Configuration 2: IN1=GPIO17, ENA=GPIO27 (IN2 tied to GND on L298N board)
    print("\n" + "=" * 60)
    print("CONFIGURATION 2: IN1=GPIO17, ENA=GPIO27 (IN2 tied to GND)")
    print("=" * 60)
    
    IN1_PIN = 17
    ENA_PIN = 27
    
    try:
        lgpio.gpio_claim_output(h, IN1_PIN)
        lgpio.gpio_claim_output(h, ENA_PIN)
        print("✓ Pins configured")
        
        # Test 1: Stop (ENA=0)
        print("\nTEST 1: Stop (ENA=0)")
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("Waiting 3 seconds...")
        time.sleep(3)
        
        # Test 2: Forward (IN1=1, ENA=1)
        print("\nTEST 2: Forward (IN1=1, ENA=1)")
        lgpio.gpio_write(h, IN1_PIN, 1)
        lgpio.gpio_write(h, ENA_PIN, 1)
        print("Waiting 3 seconds - CHECK IF MOTOR RUNS FORWARD")
        time.sleep(3)
        
        # Test 3: Stop
        print("\nTEST 3: Stop (ENA=0)")
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("Waiting 2 seconds...")
        time.sleep(2)
        
        # Test 4: Reverse (IN1=0, ENA=1)
        print("\nTEST 4: Reverse (IN1=0, ENA=1)")
        lgpio.gpio_write(h, IN1_PIN, 0)
        lgpio.gpio_write(h, ENA_PIN, 1)
        print("Waiting 3 seconds - CHECK IF MOTOR RUNS REVERSE")
        time.sleep(3)
        
        # Final stop
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("\n✓ Configuration 2 test complete")
        
    except Exception as e:
        print(f"✗ Configuration 2 failed: {e}")
    
    print("\n" + "=" * 60)
    print("Diagnostic Complete")
    print("=" * 60)
    print("\nPlease report which configuration (if any) made the motor move.")
    print("If neither worked, check:")
    print("  1. L298N power: VS=12V, VCC=5V, GND=common ground")
    print("  2. Motor connections: M+ to OUT1, M- to OUT2")
    print("  3. GPIO pin connections to L298N IN1, IN2, ENA")
    
    lgpio.gpiochip_close(h)

if __name__ == '__main__':
    try:
        test_l298n_standard()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

