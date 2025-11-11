# Quick Start - GPIO Control

## Get Your Robot Moving in 3 Steps!

### Step 1: Install Dependencies (2 minutes)

On your Raspberry Pi, run:

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation/scripts
sudo ./install_gpio_dependencies.sh
```

This installs:
- pigpio (for hardware PWM)
- gpiozero (GPIO library)
- RPi.GPIO (low-level access)

### Step 2: Test Hardware (1 minute)

Test if GPIO works:

```bash
python3 test_gpio.py
```

You'll see:
- Servo test (GPIO18) - servo should move to 0°, 90°, 180°
- Motor test (GPIO17/27) - motor should run forward, stop, backward

If tests pass, you're ready!

### Step 3: Start Web Interface

```bash
python3 web_robot_interface.py
```

Or force hardware mode:
```bash
python3 web_robot_interface.py --hardware
```

### Step 4: Control Your Robot

Open browser: **http://localhost:8000** (or **http://<pi-ip>:8000** from another device)

#### Try These Controls:

1. **Movement Tab**
   - Click "Forward" → wheels should move
   - Click "Turn Left" → robot should rotate
   - Click "Stop" → all motion stops

2. **Manipulation Tab**
   - Click "Open" → gripper opens
   - Move "Gripper Tilt" slider → servo tilts
   - Click "Home All" → servos return to center

3. **Containers Tab**
   - Click "Front Load" → container servo moves
   - Click "Front Unload" → container opens

4. **Sensors Tab**
   - See real-time IR distance sensor readings
   - See IMU orientation and acceleration
   - Click "Calibrate IMU" to zero IMU

## Troubleshooting

### Servos Not Moving?

1. Check pigpiod is running:
```bash
sudo systemctl status pigpiod
```

If stopped, start it:
```bash
sudo systemctl start pigpiod
```

2. Check wiring - servo signal wires to correct GPIO pins

3. Check power - servos need 5-6V with good current supply

### Motors Not Moving?

1. Check motor driver (L298N) is powered (12V)
2. Verify DIR/PWM pins connected correctly
3. Check common ground between Pi and motor driver

### Web Interface in Simulation Mode?

The system automatically falls back to simulation if GPIO can't be initialized.

Force hardware mode to see errors:
```bash
python3 web_robot_interface.py --hardware
```

Check the output for specific errors.

### Still Having Issues?

1. Check full documentation: `GPIO_CONTROL_SETUP.md`
2. Verify pin wiring: `../SENSOR_WIRING.md`
3. Check system logs: `sudo journalctl -xe`

## GPIO Pin Reference

### Servos (connect signal wire to these pins)
```
GPIO18 - Gripper Tilt
GPIO19 - Gripper Open/Close
GPIO21 - Gripper Neck (continuous)
GPIO12 - Gripper Base
```

### Motors (connect to motor driver)
```
Front Left:  GPIO17 (DIR), GPIO27 (PWM)
Front Right: GPIO22 (DIR), GPIO23 (PWM)
Back Wheel:  GPIO24 (DIR), GPIO25 (PWM)
```

### Containers (connect signal wire to these pins)
```
GPIO26 - Left Front
GPIO5  - Left Back
GPIO6  - Right Front
GPIO7  - Right Back
```

## What's Working Now?

✓ **Direct GPIO Control** - No ROS2 server needed
✓ **Servo Control** - All 4 gripper servos
✓ **Motor Control** - 3 omni wheels
✓ **Container Control** - 4 servo actuators
✓ **Sensor Reading** - IR, IMU, ultrasonic
✓ **Web Interface** - Real-time control and monitoring
✓ **Simulation Mode** - Works without hardware for testing

## Need More Help?

- Full setup guide: `GPIO_CONTROL_SETUP.md`
- Changes summary: `CHANGES_SUMMARY.md`
- Sensor wiring: `../SENSOR_WIRING.md`
- Hardware pinouts: `../../../docs/hardware/RASPBERRY_PI_PINOUTS.md`

## Quick Command Reference

```bash
# Install dependencies
sudo ./install_gpio_dependencies.sh

# Test GPIO
python3 test_gpio.py

# Start web interface
python3 web_robot_interface.py

# Start in hardware mode (force)
python3 web_robot_interface.py --hardware

# Start in simulation mode (no GPIO)
python3 web_robot_interface.py --simulation

# Check pigpiod status
sudo systemctl status pigpiod

# Restart pigpiod
sudo systemctl restart pigpiod

# View web interface
# Local: http://localhost:8000
# Remote: http://<pi-ip>:8000
```

Now go test your robot! 🤖

