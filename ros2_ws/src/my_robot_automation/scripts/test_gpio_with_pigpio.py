#!/usr/bin/env python3
"""
Test GPIO pins using pigpio (alternative to lgpio)
pigpio uses a daemon and might work better than lgpio
"""

import time

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False

def test_gpio_pigpio():
    """Test GPIO pins using pigpio"""
    print("=" * 70)
    print("GPIO Test Using pigpio (Alternative to lgpio)")
    print("=" * 70)
    print("\npigpio uses a daemon service and might work better")
    print("than lgpio for GPIO control.")
    print("=" * 70)
    
    if not PIGPIO_AVAILABLE:
        print("\n✗ pigpio not installed")
        print("\nInstall with:")
        print("  sudo apt install pigpio python3-pigpio")
        print("  sudo systemctl enable pigpiod")
        print("  sudo systemctl start pigpiod")
        return False
    
    try:
        pi = pigpio.pi()
        if not pi.connected:
            print("\n✗ Cannot connect to pigpio daemon")
            print("\nStart daemon:")
            print("  sudo systemctl start pigpiod")
            return False
        
        print("\n✓ Connected to pigpio daemon")
        
        test_pins = [
            (17, "IN1"),
            (10, "IN2"),
            (11, "ENA")
        ]
        
        print("\nTesting GPIO pins:")
        for pin, name in test_pins:
            print(f"\n{name} (GPIO{pin}):")
            
            # Set as output
            pi.set_mode(pin, pigpio.OUTPUT)
            print(f"  ✓ Configured as output")
            
            # Set LOW
            pi.write(pin, 0)
            time.sleep(1)
            level_low = pi.read(pin)
            print(f"  LOW: GPIO level = {level_low}")
            print(f"  → Measure voltage NOW: Should be ~0V")
            time.sleep(3)
            
            # Set HIGH
            pi.write(pin, 1)
            time.sleep(0.5)
            level_high = pi.read(pin)
            print(f"  HIGH: GPIO level = {level_high}")
            print(f"  → Measure voltage NOW: Should be ~3.3V")
            print(f"  → Keep multimeter connected for 10 seconds...")
            time.sleep(10)
            
            # Set LOW again
            pi.write(pin, 0)
            time.sleep(1)
            print(f"  LOW again: GPIO level = {pi.read(pin)}")
            print(f"  → Measure voltage NOW: Should be ~0V")
            time.sleep(2)
            
            if level_high == 1 and level_low == 0:
                print(f"  ✓ {name} GPIO control working (software)")
                print(f"  → If multimeter showed voltage: Hardware working!")
            else:
                print(f"  ✗ {name} GPIO control NOT working")
        
        pi.stop()
        
        print("\n" + "=" * 70)
        print("Test Complete")
        print("=" * 70)
        print("\nIf pigpio worked but lgpio didn't:")
        print("  → Use pigpio for GPIO control instead of lgpio")
        print("  → pigpio daemon handles GPIO more reliably")
        
        return True
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    test_gpio_pigpio()

