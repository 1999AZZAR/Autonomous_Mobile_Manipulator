#!/usr/bin/env python3
"""
Test Hardware PWM Pins - GPIO12, GPIO13, GPIO18, GPIO19
These pins have dedicated hardware PWM and might work better
"""

import time

# Try pigpio (alternative to lgpio)
try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False
    print("pigpio not available - install with: sudo apt install pigpio python3-pigpio")

def test_hardware_pwm():
    """Test hardware PWM pins"""
    print("=" * 70)
    print("Hardware PWM Test")
    print("=" * 70)
    print("\nRaspberry Pi has dedicated hardware PWM pins:")
    print("  GPIO12, GPIO13, GPIO18, GPIO19")
    print("\nThese pins have hardware PWM support and might work")
    print("even if regular GPIO pins don't output voltage.")
    print("=" * 70)
    
    if not PIGPIO_AVAILABLE:
        print("\n✗ pigpio not available")
        print("\nInstall pigpio:")
        print("  sudo apt install pigpio python3-pigpio")
        print("  sudo systemctl enable pigpiod")
        print("  sudo systemctl start pigpiod")
        return False
    
    try:
        # Connect to pigpio daemon
        pi = pigpio.pi()
        if not pi.connected:
            print("\n✗ Cannot connect to pigpio daemon")
            print("\nStart pigpio daemon:")
            print("  sudo systemctl start pigpiod")
            return False
        
        print("\n✓ Connected to pigpio daemon")
        
        # Test hardware PWM pin (GPIO18)
        PWM_PIN = 18
        print(f"\nTesting GPIO{PWM_PIN} (Hardware PWM pin):")
        
        # Set hardware PWM frequency (50Hz for motors)
        pi.set_PWM_frequency(PWM_PIN, 50)
        print(f"  PWM frequency set to 50Hz")
        
        # Test different PWM values
        test_values = [0, 128, 255]
        for pwm_val in test_values:
            print(f"\n  Setting PWM duty cycle: {pwm_val}/255 ({pwm_val*100//255}%)")
            pi.set_PWM_dutycycle(PWM_PIN, pwm_val)
            print(f"  → Measure voltage on GPIO{PWM_PIN} with multimeter")
            print(f"  → Should see varying voltage (0V to ~3.3V)")
            time.sleep(5)
        
        # Stop PWM
        pi.set_PWM_dutycycle(PWM_PIN, 0)
        pi.stop()
        
        print("\n" + "=" * 70)
        print("Hardware PWM Test Complete")
        print("=" * 70)
        print("\nIf you saw voltage changes:")
        print("  ✓ Hardware PWM pins work!")
        print("  → We can use GPIO12/13/18/19 for motor PWM control")
        print("\nIf you saw 0V:")
        print("  ✗ Hardware PWM also not working")
        print("  → May need different approach or hardware issue")
        
        return True
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    test_hardware_pwm()

