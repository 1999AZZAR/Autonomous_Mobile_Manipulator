# Raspberry Pi GPIO Setup for ROS2

## Overview

This guide explains how to configure ROS2 to access GPIO pins on a Raspberry Pi for robot control. The system uses Docker containers with GPIO device passthrough.

## System Architecture

```
Raspberry Pi Host
├── pigpiod daemon (running on host)
├── GPIO devices (/dev/gpiomem, /dev/mem, etc.)
└── Docker Container (ROS2)
    ├── gpiozero library
    ├── pigpio client
    └── web_robot_interface.py
```

## Prerequisites

### 1. Raspberry Pi Hardware
- Raspberry Pi 4 or 5 (recommended)
- GPIO header properly connected
- External power supplies for motors/servos (do not power from Pi's 5V rail)

### 2. Host System Setup

Install GPIO dependencies on the Raspberry Pi host:

```bash
cd /home/azzar/project/robotic/lks_robot_project
sudo ./install_gpio_dependencies.sh
```

This installs:
- `pigpio` - Hardware PWM daemon
- `gpiozero` - High-level GPIO library
- `python3-rpi.gpio` - Low-level GPIO access

### 3. Start pigpiod Daemon

```bash
# Check status
sudo systemctl status pigpiod

# Start if not running
sudo systemctl start pigpiod

# Enable on boot
sudo systemctl enable pigpiod
```

### 4. GPIO Permissions

Ensure your user is in the gpio group:

```bash
# Check current groups
groups $USER

# Add to gpio group if needed
sudo usermod -a -G gpio $USER

# Log out and back in, or run:
newgrp gpio
```

## Docker Configuration

### GPIO Device Access

Use the Raspberry Pi specific docker-compose file:

```bash
# For Raspberry Pi deployment
docker-compose -f docker-compose.rpi.yml up

# For development/testing
docker-compose -f docker-compose.dev.yml up
```

The RPi compose file includes:
- Device passthrough: `/dev/gpiomem`, `/dev/mem`, `/dev/i2c-1`, `/dev/spidev0.0`
- Privileged mode for GPIO access
- GPIO libraries installed in container

### Dockerfile.rpi

The Raspberry Pi Dockerfile includes:
- ROS2 Jazzy base image
- GPIO libraries: `pigpio`, `gpiozero`, `RPi.GPIO`
- Python dependencies for web interface
- Sensor libraries: `smbus2`, `spidev`

## Testing GPIO Functionality

### 1. Test Host GPIO

Test GPIO on the Raspberry Pi host:

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation/scripts
python3 test_gpio.py
```

### 2. Test Container GPIO

Test GPIO from within the Docker container:

```bash
cd /home/azzar/project/robotic/lks_robot_project
python3 test_gpio_container.py
```

### 3. Test ROS2 GPIO

Start the ROS2 system and check logs:

```bash
# Start ROS2 with GPIO
docker-compose -f docker-compose.rpi.yml up ros2-hardware

# Check logs for GPIO initialization
docker logs ros2_hardware_container
```

## GPIO Pin Mapping

### Servos (Hardware PWM - GPIO 12,13,18,19)
```
GPIO18 - Gripper Tilt Servo
GPIO19 - Gripper Open/Close Servo
GPIO21 - Gripper Neck (360° continuous)
GPIO12 - Gripper Base Height
```

### Motors (Software PWM - any GPIO)
```
GPIO17/GPIO27 - Front Left Wheel (DIR/PWM)
GPIO22/GPIO23 - Front Right Wheel (DIR/PWM)
GPIO24/GPIO25 - Back Wheel (DIR/PWM)
```

### Container Servos
```
GPIO26 - Left Front Container
GPIO5  - Left Back Container
GPIO6  - Right Front Container
GPIO7  - Right Back Container
```

### Sensors
```
I2C-1 (GPIO2/GPIO3) - MPU6050 IMU
SPI0 (GPIO8-11)     - MCP3008 ADC for IR sensors
```

## Web Interface Control

### Start the Web Interface

```bash
# The web interface starts automatically with ROS2
# Access at: http://raspberry-pi-ip:8000
```

### API Endpoints

Control GPIO hardware via REST API:

```bash
# Move robot
curl -X POST http://localhost:8000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Control gripper
curl -X POST http://localhost:8000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command": "open"}'

# Home servos
curl -X POST http://localhost:8000/api/robot/servos \
  -H "Content-Type: application/json" \
  -d '{"action": "home"}'
```

## Troubleshooting

### GPIO Not Working

1. **Check pigpiod status:**
   ```bash
   sudo systemctl status pigpiod
   sudo journalctl -u pigpiod -f
   ```

2. **Test GPIO access:**
   ```bash
   # On host
   python3 test_gpio.py

   # In container
   docker exec ros2_hardware_container python3 test_gpio_container.py
   ```

3. **Check device permissions:**
   ```bash
   ls -la /dev/gpiomem
   groups $USER | grep gpio
   ```

4. **Container logs:**
   ```bash
   docker logs ros2_hardware_container 2>&1 | grep -i gpio
   ```

### Common Issues

#### "pigpiod daemon not running"
```bash
sudo systemctl start pigpiod
sudo systemctl enable pigpiod
```

#### "Permission denied" on GPIO devices
```bash
sudo usermod -a -G gpio $USER
# Log out and back in
```

#### "Cannot connect to pigpiod"
- Ensure pigpiod is running on host
- Check Docker device passthrough
- Try running container with `--privileged`

#### "GPIO libraries not available"
- Check if Dockerfile.rpi was used
- Verify packages installed in container
- Rebuild container: `docker-compose -f docker-compose.rpi.yml build`

### Servo/Motor Not Responding

1. **Check power supply:** Servos need 5-6V, motors need 12V
2. **Verify wiring:** Signal wire to correct GPIO pin
3. **Test pigpiod:** `sudo pigpiod -g` (should show "started")
4. **Check PWM pins:** Use hardware PWM pins (12,13,18,19) for servos

### Web Interface Shows Simulation Mode

This means GPIO initialization failed. Check:
1. pigpiod is running
2. GPIO devices are accessible in container
3. gpiozero library installed
4. Run with `--hardware` flag to see detailed errors

## Performance Notes

### Hardware PWM vs Software PWM
- **Hardware PWM pins** (12,13,18,19): Best for servos, precise timing
- **Software PWM** (other pins): Good for motors, less precise but sufficient

### pigpiod Benefits
- Hardware PWM support for smooth servo control
- Remote GPIO access (useful for Docker)
- Better performance than RPi.GPIO for PWM

### Resource Usage
- pigpiod: ~2MB RAM, low CPU usage
- gpiozero: Lightweight, efficient GPIO control
- Container overhead: ~50MB RAM for full ROS2 stack

## Development vs Production

### Development (x86 PC)
- Use `docker-compose.dev.yml`
- GPIO simulated (no real hardware control)
- Good for testing logic and UI

### Production (Raspberry Pi)
- Use `docker-compose.rpi.yml`
- Full GPIO hardware control
- Requires Raspberry Pi with GPIO header

## Next Steps

1. **Hardware testing:** Connect servos and motors, test basic movement
2. **Integration testing:** Test full robot control via web interface
3. **Sensor integration:** Add IMU and IR sensor data
4. **Navigation:** Implement autonomous movement algorithms
5. **Safety systems:** Add emergency stop and collision avoidance

## Support

If GPIO issues persist:
1. Run diagnostic scripts: `test_gpio.py`, `test_gpio_container.py`
2. Check system logs: `sudo journalctl -xe`
3. Verify wiring against pinout diagrams
4. Test individual components before full integration

The GPIO system should now allow ROS2 to control your robot's servos, motors, and sensors directly through the web interface!
