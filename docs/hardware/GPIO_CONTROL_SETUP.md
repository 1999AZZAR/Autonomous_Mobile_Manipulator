# GPIO Control Setup Guide

## Overview

The web interface now has **direct GPIO control** for all servos, motors, and actuators. No separate ROS2 control server is needed - the web interface controls the hardware directly using `gpiozero` and `pigpio`.

## Changes Made

### 1. Direct GPIO Control Architecture

**Before:**
- Web interface (port 8000) → Forwarded commands to → ROS2 server (port 5000) → GPIO
- Problem: Port 5000 server didn't exist, so controls didn't work

**After:**
- Web interface (port 8000) → Direct GPIO control → Hardware
- Solution: Integrated GPIO controller directly into web interface

### 2. New Components

#### GPIOController Class
Location: `web_robot_interface.py`

Manages all GPIO hardware:
- **Servos**: Gripper tilt, open/close, neck, base
- **Motors**: 3x omni wheels (front-left, front-right, back)
- **Containers**: 4x servo-controlled container actuators

#### GPIO Pin Mapping

```python
# Servos
GPIO18 - Gripper Tilt Servo
GPIO19 - Gripper Open/Close Servo
GPIO21 - Gripper Neck (360° continuous)
GPIO12 - Gripper Base Height

# Omni Wheel Motors
GPIO17/GPIO27 - Front Left Wheel (DIR/PWM)
GPIO22/GPIO23 - Front Right Wheel (DIR/PWM)
GPIO24/GPIO25 - Back Wheel (DIR/PWM)

# Container Servos
GPIO26 - Left Front Container
GPIO5  - Left Back Container
GPIO6  - Right Front Container
GPIO7  - Right Back Container
```

## Installation

### Step 1: Install Dependencies

Run the installation script on your Raspberry Pi:

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation/scripts
sudo ./install_gpio_dependencies.sh
```

This installs:
- **pigpio**: Hardware PWM daemon for precise servo control
- **gpiozero**: High-level GPIO library
- **RPi.GPIO**: Low-level GPIO access

### Step 2: Verify Installation

Test GPIO functionality:

```bash
python3 test_gpio.py
```

This will test:
- Servo control (GPIO18)
- Motor control (GPIO17/GPIO27)

### Step 3: Start the Web Interface

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation/scripts
python3 web_robot_interface.py
```

Or in hardware mode (force real GPIO):
```bash
python3 web_robot_interface.py --hardware
```

Or in simulation mode (no GPIO):
```bash
python3 web_robot_interface.py --simulation
```

### Step 4: Access the Web Interface

Open your browser:
- **Local**: http://localhost:8000
- **Remote**: http://<raspberry-pi-ip>:8000

## API Endpoints

All control endpoints now work directly with GPIO:

### Movement Control
```bash
# Move robot
POST /api/robot/move
{"direction": "forward", "speed": 0.5}

# Turn robot
POST /api/robot/turn
{"direction": "left", "speed": 0.5}

# Stop robot
POST /api/robot/stop
```

### Gripper Control
```bash
# Open/Close gripper
POST /api/robot/picker/gripper
{"command": "open"}  # or "close"

# Set gripper tilt
POST /api/robot/picker/gripper_tilt
{"angle": 90}  # 0-180 degrees

# Set gripper neck position
POST /api/robot/picker/gripper_neck
{"position": 0.5}  # -1 to 1 (continuous servo)

# Set gripper base height
POST /api/robot/picker/gripper_base
{"height": 0.5}  # 0-1 normalized

# Home all servos
POST /api/robot/servos
{"action": "home"}
```

### Container Control
```bash
# Control containers
POST /api/robot/containers/left_front
{"action": "load"}  # or "unload"
```

### Sensor Data (unchanged)
```bash
# Get sensor data (IR, ultrasonic, IMU)
GET /api/robot/sensors

# Get IMU data
GET /api/robot/imu/position

# Calibrate IMU
POST /api/robot/imu/calibrate
```

## Port Configuration

The web interface uses **two different ports** for different purposes:

- **Port 8000** (WEB_API_BASE): Sensor data, IMU, GPIO control, status
- **Port 5000** (ROS2_API_BASE): Advanced navigation, path planning (optional)

JavaScript automatically uses the correct port for each function.

## Troubleshooting

### GPIO Control Not Working

1. **Check pigpiod is running:**
```bash
sudo systemctl status pigpiod
```

If not running:
```bash
sudo systemctl start pigpiod
sudo systemctl enable pigpiod
```

2. **Check GPIO permissions:**
```bash
groups $USER | grep gpio
```

If not in gpio group:
```bash
sudo usermod -a -G gpio $USER
# Log out and log back in
```

3. **Test GPIO directly:**
```bash
python3 test_gpio.py
```

### Servos Not Responding

1. **Check power supply**: Servos need 5-6V with sufficient current
2. **Verify wiring**: Check signal wire is connected to correct GPIO pin
3. **Check pigpio**: Servos require pigpiod for hardware PWM
4. **Test single servo**: Use test_gpio.py to isolate the issue

### Motors Not Moving

1. **Check motor driver**: Verify L298N or similar is properly powered
2. **Verify connections**: DIR and PWM pins must be correct
3. **Check motor power**: Motors need separate 12V power supply
4. **Test single motor**: Use test_gpio.py to test one motor at a time

### Web Interface Shows Simulation Mode

This means GPIO couldn't be initialized. Check:
1. pigpiod is running
2. gpiozero and pigpio libraries are installed
3. Run with `--hardware` flag to force hardware mode and see errors

## Hardware Requirements

### Power Supply
- **5V @ 3A**: Raspberry Pi + sensors
- **6V @ 5A**: Servo motors (4x)
- **12V @ 5A**: DC motors (3x omni wheels)

### Libraries Used
- **gpiozero**: High-level GPIO interface
- **pigpio**: Hardware PWM for smooth servo control
- **spidev**: SPI communication for ADC (sensors)
- **smbus2**: I2C communication for IMU

### GPIO Capabilities
- **Hardware PWM pins**: GPIO12, 13, 18, 19 (best for servos)
- **Software PWM**: Any GPIO pin (used for motors)
- **SPI**: GPIO8-11 (for MCP3008 ADC)
- **I2C**: GPIO2-3 (for MPU6050 IMU)

## Testing Commands

### Test Individual Components

```bash
# Test servo on GPIO18
python3 -c "from gpiozero import AngularServo; from gpiozero.pins.pigpio import PiGPIOFactory; s = AngularServo(18, pin_factory=PiGPIOFactory()); s.angle = 90; import time; time.sleep(2)"

# Test motor on GPIO17/27
python3 -c "from gpiozero import Motor; from gpiozero.pins.pigpio import PiGPIOFactory; m = Motor(17, 27, pin_factory=PiGPIOFactory()); m.forward(0.5); import time; time.sleep(2); m.stop()"
```

### Monitor GPIO Status

```bash
# Check GPIO state
gpio readall

# Monitor pigpiod
sudo journalctl -u pigpiod -f
```

## Safety Notes

1. **Always connect common ground** between Raspberry Pi and all motors/servos
2. **Use appropriate power supplies** - don't power motors from Pi's 5V rail
3. **Add voltage dividers** for 5V sensors on 3.3V GPIO pins
4. **Emergency stop button** should be easily accessible
5. **Test in simulation mode first** before connecting real hardware

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Run the test script: `python3 test_gpio.py`
3. Check system logs: `sudo journalctl -xe`
4. Verify wiring against `SENSOR_WIRING.md`

## Next Steps

1. Install dependencies: `sudo ./install_gpio_dependencies.sh`
2. Test GPIO: `python3 test_gpio.py`
3. Start web interface: `python3 web_robot_interface.py`
4. Access UI: http://localhost:8000
5. Test controls in the web interface

Now your robot's servos and motors should respond to commands from the web UI!

