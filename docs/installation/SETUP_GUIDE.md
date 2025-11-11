# Robot System Setup Guide

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Software Installation](#software-installation)
3. [Hardware Setup](#hardware-setup)
4. [Configuration](#configuration)
5. [Running the System](#running-the-system)
6. [Testing](#testing)
7. [Troubleshooting](#troubleshooting)

## Hardware Requirements

### Main Components

- Raspberry Pi 5 with Ubuntu Server
- MCP3008 10-bit ADC (SPI interface)
- 6x Sharp GP2Y0A02YK0F IR Distance Sensors
- 2x HC-SR04 Ultrasonic Sensors
- 3x IR Line Sensors
- TF-Luna LIDAR (USB/Serial)
- MPU6050 IMU (I2C)
- 3x Omni-wheel Motors with drivers
- Gripper system (servos + motor)
- Power supply (12V, 5A minimum)

### Connections Summary

- SPI0 for MCP3008 ADC
- I2C for IMU
- GPIO for motors, servos, and digital sensors
- USB for LIDAR and camera

## Software Installation

### System Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-dev git

# Install ROS2 (if not already installed)
# Follow official ROS2 installation guide for Ubuntu
```

### Enable Hardware Interfaces

```bash
# Enable SPI interface
sudo raspi-config
# Navigate to: Interface Options → SPI → Enable

# Enable I2C interface
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Reboot to apply changes
sudo reboot
```

### Python Dependencies

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation

# Install Python packages
pip3 install -r requirements.txt

# Install additional ROS2 dependencies
pip3 install flask rclpy
```

### Verify Hardware Access

```bash
# Check SPI devices
ls -l /dev/spidev*
# Should show: /dev/spidev0.0 /dev/spidev0.1

# Check I2C devices
ls -l /dev/i2c*
# Should show: /dev/i2c-1

# Test SPI communication
sudo apt install -y python3-spidev
python3 -c "import spidev; print('SPI OK')"
```

## Hardware Setup

### 1. MCP3008 ADC Wiring

Connect MCP3008 to Raspberry Pi SPI0:

```
MCP3008 Pin  →  RPi Pin  →  Function
Pin 16 (VDD)  →  Pin 1   →  3.3V Power
Pin 15 (VREF) →  Pin 1   →  3.3V Reference
Pin 14 (AGND) →  Pin 6   →  Ground
Pin 13 (CLK)  →  Pin 23  →  SPI0 SCLK (GPIO11)
Pin 12 (DOUT) →  Pin 21  →  SPI0 MISO (GPIO9)
Pin 11 (DIN)  →  Pin 19  →  SPI0 MOSI (GPIO10)
Pin 10 (CS)   →  Pin 24  →  SPI0 CE0 (GPIO8)
Pin 9 (DGND)  →  Pin 6   →  Ground
```

### 2. Sharp GP2Y0A02YK0F Sensor Connections

Each sensor has 3 wires (Red=5V, Black=GND, Yellow=Signal):

```
Sensor Location    →  MCP3008 Channel  →  Wire Color
Left Front         →  CH0 (Pin 1)      →  Yellow
Left Back          →  CH1 (Pin 2)      →  Yellow
Right Front        →  CH2 (Pin 3)      →  Yellow
Right Back         →  CH3 (Pin 4)      →  Yellow
Back Left          →  CH4 (Pin 5)      →  Yellow
Back Right         →  CH5 (Pin 6)      →  Yellow
```

Power all sensors:
- Red wires → 5V power rail
- Black wires → Common ground
- Yellow wires → MCP3008 channels (listed above)

### 3. Additional Sensor Connections

Refer to `SENSOR_WIRING.md` for detailed wiring of:
- HC-SR04 ultrasonic sensors
- Line sensors
- MPU6050 IMU
- TF-Luna LIDAR
- Container load sensors

## Configuration

### 1. Sensor Calibration

Run the calibration utility:

```bash
cd ros2_ws/src/my_robot_automation/scripts
python3 test_sharp_sensors.py
```

Select option 3 (Calibration mode) and follow prompts to calibrate each sensor at known distances.

### 2. ROS2 Workspace Setup

```bash
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws

# Build workspace
colcon build --packages-select my_robot_automation

# Source workspace
source install/setup.bash
```

### 3. Environment Variables

Add to `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /home/azzar/project/robotic/lks_robot_project/ros2_ws/install/setup.bash
```

Reload:
```bash
source ~/.bashrc
```

## Running the System

### Normal Operation (with Hardware)

```bash
# Start the web interface
cd ~/project/robotic/lks_robot_project/ros2_ws
ros2 run my_robot_automation web_robot_interface.py
```

### Simulation Mode (without Hardware)

For testing without physical hardware:

```bash
# System automatically detects missing hardware and enables simulation mode
ros2 run my_robot_automation web_robot_interface.py
```

You can also force simulation mode by setting environment variable:

```bash
export ROBOT_SIMULATION_MODE=1
ros2 run my_robot_automation web_robot_interface.py
```

### Access Web Interface

Open browser and navigate to:
- Local: `http://localhost:8000`
- Remote: `http://<robot-ip>:8000`

Default port is 8000 (configurable in code).

## Testing

### 1. Test Sharp Sensors

```bash
# Interactive testing utility
python3 scripts/test_sharp_sensors.py

# Select:
# 1 = Test single sensor
# 2 = Monitor all sensors continuously
# 3 = Calibration mode
```

### 2. Test API Endpoints

```bash
# Check health
curl http://localhost:8000/health

# Get sensor data
curl http://localhost:8000/api/robot/sensors

# Get sensor diagnostics
curl http://localhost:8000/api/robot/sensors/diagnostics

# Get system status
curl http://localhost:8000/api/robot/status
```

### 3. Verify Sensor Readings

In web interface:
1. Navigate to "Sensors" tab
2. Verify all 6 IR sensors show readings
3. Check values update every second
4. Valid range: 200-1500 mm

## Troubleshooting

### SPI Not Working

```bash
# Check SPI is enabled
lsmod | grep spi
# Should show: spi_bcm2835

# Check permissions
ls -l /dev/spidev0.0
# Add user to spi group if needed:
sudo usermod -a -G spi $USER
# Logout and login again
```

### Sensor Readings Always Zero

1. Check 5V power supply to sensors
2. Verify sensor output wires connected to correct MCP3008 channels
3. Measure voltage on sensor output (should be 0.4V-2.7V)
4. Check MCP3008 reference voltage (Pin 15) is 3.3V

### Inconsistent Readings

1. Add 10µF capacitors between Vcc and GND near each sensor
2. Check power supply can provide sufficient current (200mA for 6 sensors)
3. Ensure common ground between all components
4. Try reducing SPI speed in code (currently 1.35MHz)

### Web Interface Not Loading

```bash
# Check Flask server is running
ps aux | grep web_robot_interface

# Check port 8000 is not in use
sudo netstat -tlnp | grep :8000

# Check firewall
sudo ufw status
sudo ufw allow 8000/tcp
```

### Import Errors

```bash
# Install missing dependencies
pip3 install spidev flask rclpy

# Verify installations
python3 -c "import spidev; import flask; import rclpy; print('All imports OK')"
```

### ROS2 Node Not Starting

```bash
# Source workspace
source ~/project/robotic/lks_robot_project/ros2_ws/install/setup.bash

# Check ROS2 environment
printenv | grep ROS

# Rebuild workspace
cd ~/project/robotic/lks_robot_project/ros2_ws
colcon build --packages-select my_robot_automation --symlink-install
```

## Advanced Configuration

### Custom Sensor Calibration

Edit sensor calibration constants in `web_robot_interface.py`:

```python
# Line ~2633: Adjust formula constants
distance_cm = 60 * (voltage ** -1.1) - 1
# Modify 60 and -1.1 based on your calibration data
```

### Change Web Interface Port

Edit `web_robot_interface.py` line ~2753:

```python
self.app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)
# Change port=8000 to desired port
```

### Adjust Sensor Sampling

Edit `web_robot_interface.py` line ~2657:

```python
for _ in range(5):  # Number of samples
    # ... sampling code ...
    time.sleep(0.001)  # Delay between samples
```

## System Startup Service

Create systemd service for auto-start:

```bash
sudo nano /etc/systemd/system/robot-interface.service
```

Add:

```ini
[Unit]
Description=Robot Web Interface
After=network.target

[Service]
Type=simple
User=azzar
WorkingDirectory=/home/azzar/project/robotic/lks_robot_project/ros2_ws
ExecStart=/usr/bin/python3 /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-interface.service
sudo systemctl start robot-interface.service

# Check status
sudo systemctl status robot-interface.service
```

## Safety Notes

1. Always test emergency stop before operation
2. Keep clear of moving parts during testing
3. Use appropriate fuses for motor circuits
4. Ensure stable power supply (ripple < 50mV)
5. Verify all ground connections before powering on
6. Use voltage dividers for 5V sensor outputs to 3.3V inputs
7. Test in simulation mode before connecting actual hardware

## Support

For issues or questions:
1. Check log files in `/var/log/robot/`
2. Review sensor diagnostics at `/api/robot/sensors/diagnostics`
3. Run test utilities to isolate problems
4. Refer to component datasheets in documentation folder

