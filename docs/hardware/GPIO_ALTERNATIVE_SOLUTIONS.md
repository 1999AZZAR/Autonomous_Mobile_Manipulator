# GPIO Control Alternative Solutions

## Problem
GPIO pins (GPIO17, GPIO10, GPIO11) are not outputting voltage signals that can control L298N motors, even though software reports they're set HIGH.

## Current Status
- ✗ `lgpio` library: GPIO pins report HIGH but multimeter shows 0V
- ✗ `pigpio` library: Not compatible with Raspberry Pi 5
- ✓ I2C communication: Working (IMU at 0x68 detected)
- ✗ PCA9685 module: Not available

## Solution Options

### Option 1: Use Hardware PWM Pins (Recommended - No Extra Hardware)

Raspberry Pi 5 has dedicated hardware PWM pins that might work better than regular GPIO:

**Hardware PWM Pins:**
- GPIO12 (Pin 32) - PWM0 Channel 0
- GPIO13 (Pin 33) - PWM0 Channel 1  
- GPIO18 (Pin 12) - PWM0 Channel 0 (alternate)
- GPIO19 (Pin 35) - PWM0 Channel 1 (alternate)

**Implementation:**
- Use hardware PWM pins for motor speed control (ENA)
- Keep regular GPIO for direction (IN1/IN2) - simple HIGH/LOW should work
- Hardware PWM is more reliable than software PWM

**Test Script:** `test_hardware_pwm.py`

### Option 2: Use gpiod (libgpiod) Library

`gpiod` is the modern GPIO library for Linux (works on Pi 5):

```bash
sudo apt install gpiod python3-libgpiod
```

**Advantages:**
- Modern Linux GPIO interface
- Works on Raspberry Pi 5
- More reliable than lgpio

**Test:** Create test script using `gpiod` library

### Option 3: Check GPIO Configuration

Verify GPIO pins are actually configured correctly:

1. **Check if pins are claimed by another process:**
```bash
sudo lsof | grep gpio
```

2. **Check GPIO device permissions:**
```bash
ls -l /dev/gpiochip*
sudo chmod 666 /dev/gpiochip0
```

3. **Verify pin configuration:**
```bash
gpioinfo gpiochip0 | grep -A 2 "GPIO17\|GPIO10\|GPIO11"
```

### Option 4: Hardware Issue Diagnosis

If software shows HIGH but multimeter shows 0V:

1. **Check power supply:**
   - Verify Raspberry Pi 5V rail is working
   - Check if other GPIO pins work (test GPIO18 which has hardware PWM)

2. **Check pin connections:**
   - Verify wires are connected correctly
   - Test continuity from GPIO pin to L298N input

3. **Test with LED:**
   - Connect LED + resistor to GPIO pin
   - If LED doesn't light, GPIO pin is not outputting

### Option 5: Use I2C GPIO Expander (If Available)

If you have an I2C GPIO expander (MCP23017, PCF8574):

- Use I2C (proven to work) for GPIO control
- No direct GPIO pin access needed
- Requires I2C GPIO expander module

## Recommended Next Steps

1. **Test Hardware PWM pins** (GPIO12/13/18/19) - No extra hardware needed
2. **Try gpiod library** - Modern alternative to lgpio
3. **Diagnose hardware** - Check if pins are damaged or misconfigured
4. **Consider I2C GPIO expander** - If other solutions don't work

## Test Scripts Available

- `test_hardware_pwm.py` - Test hardware PWM pins
- `test_gpio_with_pigpio.py` - Test pigpio (won't work on Pi 5)
- `test_gpio_output_voltage.py` - Test GPIO voltage output

## Quick Test: Hardware PWM

```bash
# Test GPIO18 (hardware PWM pin)
cd /home/raspi/Autonomous_Mobile_Manipulator/ros2_ws
python3 src/my_robot_automation/scripts/test_hardware_pwm.py
```

If hardware PWM works, we can use GPIO12/13/18/19 for motor control instead of regular GPIO pins.

