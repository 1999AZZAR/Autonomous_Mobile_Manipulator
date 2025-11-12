#!/usr/bin/env python3
"""
Test L298N with IN1, IN2, ENA - checking if IN2 needs to be connected
"""

import lgpio
import time
import sys

def test_l298n_with_in2():
    """Test L298N with IN1, IN2, ENA pins"""
    
    print("=" * 60)
    print("L298N Test with IN1, IN2, ENA")
    print("=" * 60)
    print("\nCurrent wiring:")
    print("  IN1 → GPIO17")
    print("  ENA → GPIO27")
    print("  IN2 → ??? (not connected)")
    print("\nTesting if IN2 needs to be connected...")
    
    try:
        h = lgpio.gpiochip_open(0)
        print("✓ GPIO chip opened")
    except Exception as e:
        print(f"✗ Failed to open GPIO chip: {e}")
        return
    
    IN1_PIN = 17  # GPIO17
    IN2_PIN = 23  # Try GPIO23 (currently encoder B, but let's test)
    ENA_PIN = 27  # GPIO27
    
    try:
        lgpio.gpio_claim_output(h, IN1_PIN)
        lgpio.gpio_claim_output(h, ENA_PIN)
        print("✓ IN1 and ENA pins configured")
        
        # First, test with IN2 not connected (current setup)
        print("\n" + "=" * 60)
        print("TEST 1: Current Setup (IN2 not connected)")
        print("=" * 60)
        
        # Stop: ENA=0
        print("\n1. Stop (ENA=0)")
        lgpio.gpio_write(h, IN1_PIN, 0)
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("   Waiting 3 seconds...")
        time.sleep(3)
        
        # Forward attempt: IN1=1, ENA=1 (IN2 floating)
        print("\n2. Forward attempt (IN1=1, ENA=1, IN2=floating)")
        lgpio.gpio_write(h, IN1_PIN, 1)
        lgpio.gpio_write(h, ENA_PIN, 1)
        print("   Waiting 5 seconds - CHECK IF MOTOR MOVES")
        time.sleep(5)
        
        # Stop
        lgpio.gpio_write(h, ENA_PIN, 0)
        print("\n   Stopped")
        time.sleep(2)
        
        # Now test with IN2 connected
        print("\n" + "=" * 60)
        print("TEST 2: With IN2 Connected (IN2=GPIO23)")
        print("=" * 60)
        print("NOTE: GPIO23 is currently used for encoder. This is just a test.")
        print("If this works, we'll need to reassign encoder pins.")
        
        try:
            lgpio.gpio_claim_output(h, IN2_PIN)
            print("✓ IN2 pin configured")
            
            # Stop: IN1=0, IN2=0, ENA=0
            print("\n1. Stop (IN1=0, IN2=0, ENA=0)")
            lgpio.gpio_write(h, IN1_PIN, 0)
            lgpio.gpio_write(h, IN2_PIN, 0)
            lgpio.gpio_write(h, ENA_PIN, 0)
            print("   Waiting 3 seconds...")
            time.sleep(3)
            
            # Forward: IN1=1, IN2=0, ENA=1
            print("\n2. Forward (IN1=1, IN2=0, ENA=1)")
            lgpio.gpio_write(h, IN1_PIN, 1)
            lgpio.gpio_write(h, IN2_PIN, 0)
            lgpio.gpio_write(h, ENA_PIN, 1)
            print("   Waiting 5 seconds - CHECK IF MOTOR MOVES FORWARD")
            time.sleep(5)
            
            # Stop
            lgpio.gpio_write(h, ENA_PIN, 0)
            print("\n   Stopped")
            time.sleep(2)
            
            # Reverse: IN1=0, IN2=1, ENA=1
            print("\n3. Reverse (IN1=0, IN2=1, ENA=1)")
            lgpio.gpio_write(h, IN1_PIN, 0)
            lgpio.gpio_write(h, IN2_PIN, 1)
            lgpio.gpio_write(h, ENA_PIN, 1)
            print("   Waiting 5 seconds - CHECK IF MOTOR MOVES REVERSE")
            time.sleep(5)
            
            # Final stop
            lgpio.gpio_write(h, ENA_PIN, 0)
            print("\n   Stopped")
            
        except Exception as e:
            print(f"✗ Could not configure IN2 pin: {e}")
            print("  This might mean GPIO23 is already in use for encoder")
        
        print("\n" + "=" * 60)
        print("Diagnostic Complete")
        print("=" * 60)
        print("\nRESULTS:")
        print("  - If motor moved in TEST 2 but not TEST 1:")
        print("    → IN2 MUST be connected to a GPIO pin")
        print("  - If motor didn't move in either test:")
        print("    → Check L298N power (VS=12V, VCC=5V)")
        print("    → Check motor connections (M+ to OUT1, M- to OUT2)")
        print("    → Check ENA pin connection")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        try:
            lgpio.gpio_write(h, ENA_PIN, 0)
            lgpio.gpiochip_close(h)
        except:
            pass

if __name__ == '__main__':
    try:
        test_l298n_with_in2()
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

