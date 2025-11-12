#!/usr/bin/env python3
"""
Test GPIO using gpiod (libgpiod) - Modern GPIO library for Linux
Works on Raspberry Pi 5
"""

import time

try:
    import gpiod
    GPIOD_AVAILABLE = True
except ImportError:
    GPIOD_AVAILABLE = False

def test_gpiod():
    """Test GPIO pins using gpiod library"""
    print("=" * 70)
    print("GPIO Test Using gpiod (libgpiod)")
    print("=" * 70)
    print("\ngpiod is the modern Linux GPIO interface")
    print("Works on Raspberry Pi 5 (unlike pigpio)")
    print("=" * 70)
    
    if not GPIOD_AVAILABLE:
        print("\n✗ gpiod not available")
        print("\nInstall with:")
        print("  sudo apt install python3-libgpiod")
        return False
    
    try:
        # Open GPIO chip
        chip = gpiod.Chip('gpiochip0')
        print("\n✓ GPIO chip opened")
        
        test_pins = [
            (17, "IN1"),
            (10, "IN2"),
            (11, "ENA")
        ]
        
        print("\nTesting GPIO pins:")
        for pin_num, name in test_pins:
            print(f"\n{name} (GPIO{pin_num}):")
            
            try:
                # Get line
                line = chip.get_line(pin_num)
                
                # Request as output
                line.request(consumer='motor_test', type=gpiod.LINE_REQ_DIR_OUT)
                print(f"  ✓ Configured as output")
                
                # Set LOW
                line.set_value(0)
                time.sleep(1)
                value_low = line.get_value()
                print(f"  LOW: GPIO value = {value_low}")
                print(f"  → Measure voltage NOW: Should be ~0V")
                time.sleep(3)
                
                # Set HIGH
                line.set_value(1)
                time.sleep(0.5)
                value_high = line.get_value()
                print(f"  HIGH: GPIO value = {value_high}")
                print(f"  → Measure voltage NOW: Should be ~3.3V")
                print(f"  → Keep multimeter connected for 10 seconds...")
                time.sleep(10)
                
                # Set LOW again
                line.set_value(0)
                time.sleep(1)
                print(f"  LOW again: GPIO value = {line.get_value()}")
                print(f"  → Measure voltage NOW: Should be ~0V")
                time.sleep(2)
                
                # Release line
                line.release()
                
                if value_high == 1 and value_low == 0:
                    print(f"  ✓ {name} GPIO control working (software)")
                    print(f"  → If multimeter showed voltage: Hardware working!")
                else:
                    print(f"  ✗ {name} GPIO control NOT working")
                    
            except Exception as e:
                print(f"  ✗ Error testing {name}: {e}")
        
        chip.close()
        
        print("\n" + "=" * 70)
        print("Test Complete")
        print("=" * 70)
        print("\nIf gpiod worked but lgpio didn't:")
        print("  → Use gpiod for GPIO control instead of lgpio")
        print("  → gpiod is more reliable on Raspberry Pi 5")
        
        return True
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    test_gpiod()

