# Raspberry Pi Setup Guide (ARM64)

This comprehensive guide provides step-by-step instructions for configuring and deploying the LKS Robot Project on a Raspberry Pi 5 running Ubuntu Server 22.04 LTS (64-bit ARM).

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Requirements](#software-requirements)
- [Step 1: Raspberry Pi OS Installation](#step-1-raspberry-pi-os-installation)
  - [1.1 Download Ubuntu Server for Raspberry Pi](#11-download-ubuntu-server-for-raspberry-pi)
  - [1.2 Flash Ubuntu Server to microSD Card](#12-flash-ubuntu-server-to-microsd-card)
  - [1.3 Initial Boot and Configuration](#13-initial-boot-and-configuration)
- [Step 2: System Configuration](#step-2-system-configuration)
  - [2.1 Enable Required Interfaces](#21-enable-required-interfaces)
  - [2.2 Install System Dependencies](#22-install-system-dependencies)
  - [2.3 Configure User Permissions](#23-configure-user-permissions)
- [Step 3: Docker Installation and Configuration](#step-3-docker-installation-and-configuration)
  - [3.1 Install Docker Engine](#31-install-docker-engine)
  - [3.2 Configure Docker for Production](#32-configure-docker-for-production)
- [Step 4: GPIO and Hardware Interface Setup](#step-4-gpio-and-hardware-interface-setup)
  - [4.1 GPIO Pin Configuration](#41-gpio-pin-configuration)
  - [4.2 I2C Configuration](#42-i2c-configuration)
  - [4.3 SPI Configuration (if needed)](#43-spi-configuration-if-needed)
  - [4.4 UART/Serial Configuration](#44-uartserial-configuration)
- [Step 5: Motor Control Setup](#step-5-motor-control-setup)
  - [5.1 Install Motor Control Dependencies](#51-install-motor-control-dependencies)
  - [5.2 Create Motor Control Service](#52-create-motor-control-service)
- [Step 6: Sensor Integration](#step-6-sensor-integration)
  - [6.1 RPLIDAR Setup](#61-rplidar-setup)
  - [6.2 IMU (MPU6050/BNO055) Setup](#62-imu-mpu6050bno055-setup)
  - [6.3 Camera Setup](#63-camera-setup)
- [Step 7: Power Management](#step-7-power-management)
  - [7.1 Battery Monitoring Setup](#71-battery-monitoring-setup)
  - [7.2 Power Optimization](#72-power-optimization)
- [Step 8: Network Configuration](#step-8-network-configuration)
  - [8.1 Static IP Configuration](#81-static-ip-configuration)
  - [8.2 SSH Configuration](#82-ssh-configuration)
  - [8.3 Firewall Configuration](#83-firewall-configuration)
- [Step 9: Project Deployment](#step-9-project-deployment)
  - [9.1 Clone and Configure Project](#91-clone-and-configure-project)
  - [9.2 Build and Deploy](#92-build-and-deploy)
- [Step 10: Performance Optimization](#step-10-performance-optimization)
  - [10.1 Memory Optimization](#101-memory-optimization)
  - [10.2 CPU Optimization](#102-cpu-optimization)
  - [10.3 Storage Optimization](#103-storage-optimization)
- [Step 11: Startup Automation](#step-11-startup-automation)
  - [11.1 Systemd Service Configuration](#111-systemd-service-configuration)
  - [11.2 Auto-restart Configuration](#112-auto-restart-configuration)
- [Step 12: Testing and Verification](#step-12-testing-and-verification)
  - [12.1 Hardware Testing](#121-hardware-testing)
  - [12.2 API Testing](#122-api-testing)
  - [12.3 ROS 2 Testing](#123-ros-2-testing)
- [Step 13: Monitoring and Maintenance](#step-13-monitoring-and-maintenance)
  - [13.1 System Monitoring Setup](#131-system-monitoring-setup)
  - [13.2 Log Management](#132-log-management)
- [Step 14: Backup and Recovery](#step-14-backup-and-recovery)
  - [14.1 Backup Configuration](#141-backup-configuration)
  - [14.2 Recovery Procedures](#142-recovery-procedures)
- [Troubleshooting Common Issues](#troubleshooting-common-issues)
  - [Hardware Issues](#hardware-issues)
  - [Software Issues](#software-issues)

## Overview

The Raspberry Pi serves as the main computing platform for the robot, handling:
- ROS 2 robot control and navigation
- Hardware interface management
- Sensor data processing
- Network communication
- Workflow automation

## Prerequisites

### Hardware Requirements
- **Raspberry Pi 5** (8GB RAM recommended, 4GB minimum)
- **Power Supply**: 5V/5A USB-C PD power adapter
- **Storage**: 64GB microSD card (A2 speed rating or higher)
- **Cooling**: Active cooling fan or heat sinks
- **Network**: Ethernet cable (recommended) or WiFi

### Software Requirements
- Ubuntu Server 22.04 LTS (64-bit ARM64)
- Docker Engine with ARM64 support
- Docker Compose v2.x
- Basic Linux command line knowledge

## Step 1: Raspberry Pi OS Installation

### 1.1 Download Ubuntu Server for Raspberry Pi

```bash
# On your development machine
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.3-live-server-arm64.iso

# Or use Raspberry Pi Imager for easier setup
# Download from: https://www.raspberrypi.com/software/
```

### 1.2 Flash Ubuntu Server to microSD Card

#### Using Raspberry Pi Imager (Recommended)
1. Install Raspberry Pi Imager on your development machine
2. Select "Ubuntu Server 22.04 LTS (64-bit)" for Raspberry Pi 5
3. Choose your microSD card
4. Configure WiFi and SSH during setup:
   - Set hostname: `lks-robot`
   - Enable SSH
   - Configure WiFi (if not using Ethernet)
   - Set username: `ubuntu` (default)

#### Using Command Line (Advanced)
```bash
# Find your microSD device (be careful!)
lsblk

# Flash the image (replace /dev/sdX with your device)
sudo dd if=ubuntu-22.04.3-live-server-arm64.iso of=/dev/sdX bs=4M status=progress

# Create user-data file for cloud-init (optional)
# This enables headless setup
```

### 1.3 Initial Boot and Configuration

```bash
# Connect to Raspberry Pi via SSH
ssh ubuntu@raspberrypi.local
# Or use the IP address if known

# Update system packages
sudo apt update && sudo apt upgrade -y

# Configure timezone
sudo timedatectl set-timezone Asia/Jakarta

# Set hostname
sudo hostnamectl set-hostname lks-robot
```

## Step 2: System Configuration

### 2.1 Enable Required Interfaces

```bash
# Enable I2C, SPI, and GPIO interfaces
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1

# Enable camera interface (if using camera)
sudo raspi-config nonint do_camera 0

# Apply changes
sudo reboot
```

### 2.2 Install System Dependencies

```bash
# Update package lists
sudo apt update

# Install essential packages
sudo apt install -y \
    curl \
    wget \
    git \
    htop \
    vim \
    net-tools \
    i2c-tools \
    python3-pip \
    python3-dev \
    build-essential \
    libi2c-dev \
    libgpiod-dev \
    gpiod \
    pigpio \
    python3-pigpio

# Install Docker prerequisites
sudo apt install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    uidmap \
    dbus-user-session

# Clean up
sudo apt autoremove -y
sudo apt clean
```

### 2.3 Configure User Permissions

```bash
# Add user to required groups
sudo usermod -aG dialout,i2c,gpio,video ubuntu

# Enable lingering for systemd user services
sudo loginctl enable-linger ubuntu

# Verify group membership
groups ubuntu
```

## Step 3: Docker Installation and Configuration

### 3.1 Install Docker Engine

```bash
# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up the stable repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Start and enable Docker
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group
sudo usermod -aG docker ubuntu

# Configure Docker for ARM64
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "experimental": true,
  "features": {
    "buildkit": true
  },
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2",
  "default-runtime": "runc"
}
EOF

# Restart Docker
sudo systemctl restart docker

# Test Docker installation
docker run --rm hello-world
docker compose version
```

### 3.2 Configure Docker for Production

```bash
# Create Docker data directory on faster storage if available
# For Raspberry Pi 5, consider using external SSD via USB

# Configure Docker to start on boot
sudo systemctl enable docker

# Set up Docker cleanup (run weekly)
sudo tee /etc/cron.weekly/docker-cleanup > /dev/null <<EOF
#!/bin/bash
docker system prune -f --volumes
EOF
sudo chmod +x /etc/cron.weekly/docker-cleanup
```

## Step 4: GPIO and Hardware Interface Setup

### 4.1 GPIO Pin Configuration

```bash
# Install GPIO tools and libraries
sudo apt install -y python3-rpi.gpio python3-lgpio

# Test GPIO access
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO working')"

# Create GPIO permission script
sudo tee /usr/local/bin/setup-gpio-permissions > /dev/null <<EOF
#!/bin/bash
# Set GPIO permissions for user access
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo chown root:i2c /dev/i2c-1
sudo chmod g+rw /dev/i2c-1
EOF
sudo chmod +x /usr/local/bin/setup-gpio-permissions

# Add to startup
sudo tee /etc/rc.local > /dev/null <<EOF
#!/bin/bash
/usr/local/bin/setup-gpio-permissions
exit 0
EOF
sudo chmod +x /etc/rc.local
```

### 4.2 I2C Configuration

```bash
# Enable I2C interface
sudo raspi-config nonint do_i2c 0

# Install I2C tools
sudo apt install -y i2c-tools python3-smbus

# Test I2C bus
sudo i2cdetect -y 1

# Configure I2C speed (if needed)
# Add to /boot/firmware/config.txt:
# dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

### 4.3 SPI Configuration (if needed)

```bash
# Enable SPI interface
sudo raspi-config nonint do_spi 0

# Test SPI
ls /dev/spidev*
```

### 4.4 UART/Serial Configuration

```bash
# Configure UART for GPS or other serial devices
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1

# Check serial ports
ls /dev/ttyS* /dev/ttyAMA*

# Configure serial console (disable if using UART)
sudo systemctl disable serial-getty@ttyS0.service
```

## Step 5: Motor Control Setup

### 5.1 Install Motor Control Dependencies

```bash
# Install stepper motor control libraries
sudo apt install -y python3-gpiozero python3-pigpio

# Enable pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Test motor control pins (adjust pin numbers for your setup)
python3 -c "
import gpiozero
import time

# Test stepper motor pins (example for TB6600 driver)
step_pin = gpiozero.OutputDevice(18)
dir_pin = gpiozero.OutputDevice(22)
enable_pin = gpiozero.OutputDevice(17)

# Quick test
enable_pin.off()  # Enable driver
dir_pin.on()      # Set direction
for i in range(100):
    step_pin.on()
    time.sleep(0.001)
    step_pin.off()
    time.sleep(0.001)

print('Motor test completed')
"
```

### 5.2 Create Motor Control Service

```bash
# Create systemd service for motor control
sudo tee /etc/systemd/system/motor-control.service > /dev/null <<EOF
[Unit]
Description=Motor Control Service
After=pigpiod.service
Requires=pigpiod.service

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu
ExecStart=/usr/bin/python3 /home/ubuntu/motor_control.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Create basic motor control script
tee ~/motor_control.py > /dev/null <<EOF
#!/usr/bin/env python3
"""
Basic motor control script for Raspberry Pi
"""

import gpiozero
import time
import sys

class MotorController:
    def __init__(self):
        # Configure pins for 3 motors (adjust as needed)
        self.motors = [
            {
                'step': gpiozero.OutputDevice(18),
                'dir': gpiozero.OutputDevice(22),
                'enable': gpiozero.OutputDevice(17)
            },
            {
                'step': gpiozero.OutputDevice(19),
                'dir': gpiozero.OutputDevice(24),
                'enable': gpiozero.OutputDevice(17)
            },
            {
                'step': gpiozero.OutputDevice(21),
                'dir': gpiozero.OutputDevice(26),
                'enable': gpiozero.OutputDevice(17)
            }
        ]

        # Enable all motors
        for motor in self.motors:
            motor['enable'].off()

    def step_motor(self, motor_id, steps, direction=1, delay=0.001):
        if motor_id >= len(self.motors):
            return False

        motor = self.motors[motor_id]
        motor['dir'].value = direction

        for _ in range(steps):
            motor['step'].on()
            time.sleep(delay)
            motor['step'].off()
            time.sleep(delay)

        return True

if __name__ == '__main__':
    controller = MotorController()
    print("Motor control service started")

    # Keep service running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Motor control service stopped")
        sys.exit(0)
EOF

# Make executable and start service
chmod +x ~/motor_control.py
sudo systemctl daemon-reload
sudo systemctl enable motor-control
sudo systemctl start motor-control
```

## Step 6: Sensor Integration

### 6.1 RPLIDAR Setup

```bash
# Install RPLIDAR SDK dependencies
sudo apt install -y cmake pkg-config

# Test RPLIDAR connection
lsusb | grep -i rplidar

# Create RPLIDAR udev rule
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Test RPLIDAR
ls -la /dev/rplidar
```

### 6.2 IMU (MPU6050/BNO055) Setup

```bash
# Install IMU libraries
sudo apt install -y python3-smbus python3-rtimulib

# Test I2C IMU connection
sudo i2cdetect -y 1

# Create IMU calibration script
tee ~/calibrate_imu.py > /dev/null <<EOF
#!/usr/bin/env python3
"""
IMU calibration script for MPU6050/BNO055
"""

import smbus
import time

class IMUCalibrator:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.imu_addr = 0x68  # MPU6050 default address

    def calibrate(self):
        print("Starting IMU calibration...")
        # MPU6050 calibration sequence
        self.bus.write_byte_data(self.imu_addr, 0x6B, 0x00)  # Wake up
        time.sleep(0.1)

        # Read accelerometer and gyroscope data
        accel_x = self.read_word_2c(0x3B)
        accel_y = self.read_word_2c(0x3D)
        accel_z = self.read_word_2c(0x3F)

        gyro_x = self.read_word_2c(0x43)
        gyro_y = self.read_word_2c(0x45)
        gyro_z = self.read_word_2c(0x47)

        print(f"Accel: X={accel_x}, Y={accel_y}, Z={accel_z}")
        print(f"Gyro: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
        print("IMU calibration completed")

    def read_word_2c(self, addr):
        val = self.read_word(addr)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def read_word(self, addr):
        high = self.bus.read_byte_data(self.imu_addr, addr)
        low = self.bus.read_byte_data(self.imu_addr, addr + 1)
        val = (high << 8) + low
        return val

if __name__ == '__main__':
    calibrator = IMUCalibrator()
    calibrator.calibrate()
EOF

chmod +x ~/calibrate_imu.py
python3 ~/calibrate_imu.py
```

### 6.3 Camera Setup

```bash
# Enable camera interface
sudo raspi-config nonint do_camera 0

# Install camera libraries
sudo apt install -y python3-picamera2

# Test camera
python3 -c "
from picamera2 import Picamera2
import time

picam2 = Picamera2()
picam2.start()
time.sleep(2)
picam2.capture_file('test_image.jpg')
picam2.stop()
print('Camera test completed - check test_image.jpg')
"
```

## Step 7: Power Management

### 7.1 Battery Monitoring Setup

```bash
# Install ADC libraries for battery voltage monitoring
sudo apt install -y python3-numpy

# Create battery monitoring script
tee ~/battery_monitor.py > /dev/null <<EOF
#!/usr/bin/env python3
"""
Battery monitoring script for LiPo battery
"""

import gpiozero
import time
import statistics

class BatteryMonitor:
    def __init__(self, adc_pin=27, r1=10000, r2=3300):
        # ADC pin for voltage divider
        self.adc = gpiozero.MCP3008(channel=0)  # SPI ADC
        self.r1 = r1  # 10k resistor
        self.r2 = r2  # 3.3k resistor
        self.v_ref = 3.3  # ADC reference voltage
        self.samples = []

    def read_voltage(self):
        # Read ADC value (0-1)
        adc_value = self.adc.value

        # Convert to actual voltage
        adc_voltage = adc_value * self.v_ref

        # Calculate battery voltage using voltage divider
        battery_voltage = adc_voltage * (self.r1 + self.r2) / self.r2

        return battery_voltage

    def get_battery_percentage(self, voltage):
        # LiPo battery discharge curve approximation
        if voltage >= 4.2:
            return 100.0
        elif voltage >= 4.0:
            return 90.0 + (voltage - 4.0) * 25.0
        elif voltage >= 3.7:
            return 50.0 + (voltage - 3.7) * 40.0
        elif voltage >= 3.4:
            return 10.0 + (voltage - 3.4) * 25.0
        else:
            return max(0.0, (voltage - 3.0) * 33.3)

    def monitor_battery(self):
        print("Starting battery monitoring...")
        print("Press Ctrl+C to stop")

        try:
            while True:
                voltage = self.read_voltage()
                percentage = self.get_battery_percentage(voltage)

                self.samples.append(voltage)
                if len(self.samples) > 10:
                    self.samples.pop(0)

                avg_voltage = statistics.mean(self.samples)

                print(".1f"
                # Warning for low battery
                if percentage < 20:
                    print("⚠️  WARNING: Low battery!")

                time.sleep(5)

        except KeyboardInterrupt:
            print("\nBattery monitoring stopped")

if __name__ == '__main__':
    monitor = BatteryMonitor()
    monitor.monitor_battery()
EOF

chmod +x ~/battery_monitor.py
```

### 7.2 Power Optimization

```bash
# Configure CPU governor for performance
sudo tee /etc/default/cpufrequtils > /dev/null <<EOF
GOVERNOR="performance"
EOF

# Disable HDMI to save power (if not using display)
sudo tee /boot/firmware/config.txt > /dev/null <<EOF
# Disable HDMI
hdmi_blanking=1
EOF

# Configure power management
sudo tee /etc/rc.local > /dev/null <<EOF
#!/bin/bash
# Power management optimizations
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo 1 > /sys/devices/platform/soc/soc:gpu/devfreq/devfreq0/min_freq
exit 0
EOF
```

## Step 8: Network Configuration

### 8.1 Static IP Configuration

```bash
# Configure static IP for reliable access
sudo tee /etc/netplan/50-cloud-init.yaml > /dev/null <<EOF
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: false
            addresses:
                - 192.168.1.100/24
            gateway4: 192.168.1.1
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
EOF

# Apply network configuration
sudo netplan apply
```

### 8.2 SSH Configuration

```bash
# Configure SSH for secure remote access
sudo tee /etc/ssh/sshd_config.d/custom.conf > /dev/null <<EOF
# Custom SSH configuration
PermitRootLogin no
PasswordAuthentication yes
PubkeyAuthentication yes
AuthorizedKeysFile .ssh/authorized_keys
ChallengeResponseAuthentication no
UsePAM yes
PrintMotd no
Subsystem sftp /usr/lib/openssh/sftp-server
EOF

# Restart SSH service
sudo systemctl restart ssh

# Add SSH key for passwordless access (optional)
ssh-keygen -t rsa -b 4096 -C "lks-robot"
# Copy public key to authorized_keys
```

### 8.3 Firewall Configuration

```bash
# Install and configure UFW firewall
sudo apt install -y ufw

# Configure firewall rules
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow ssh
sudo ufw allow 5000  # Robot API
sudo ufw allow 5678  # n8n interface
sudo ufw allow 8765  # WebSocket
sudo ufw allow 11311 # ROS master (if needed)

# Enable firewall
sudo ufw enable
```

## Step 9: Project Deployment

### 9.1 Clone and Configure Project

```bash
# Clone the project repository
git clone https://github.com/1999AZZAR/LKS_Robot_Project.git
cd LKS_Robot_Project

# Create production configuration
cp docker-compose.yml docker-compose.prod.yml

# Edit production configuration for Raspberry Pi
tee docker-compose.prod.yml > /dev/null <<EOF
version: '3.8'
services:
  ros2-sim:
    build:
      context: ./ros2_ws
      dockerfile: Dockerfile
      args:
        BASE_IMAGE: arm64v8/ros:iron
    container_name: lks_robot
    restart: unless-stopped
    network_mode: host
    privileged: true
    volumes:
      - ./ros2_ws/src:/root/ros2_ws/src:ro
      - /dev:/dev:rw
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_LOCALHOST_ONLY=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    command: >
      bash -c "
        source /opt/ros/iron/setup.bash &&
        cd /root/ros2_ws &&
        colcon build --packages-select my_robot_automation --event-handlers console_direct+ &&
        source install/setup.bash &&
        echo 'Starting LKS Robot services...' &&
        ros2 launch my_robot_automation automation_launch.py
      "

  n8n:
    image: arm64v8/n8n:latest
    container_name: lks_n8n
    restart: unless-stopped
    ports:
      - "5678:5678"
    environment:
      - N8N_HOST=0.0.0.0
      - N8N_PORT=5678
      - N8N_PROTOCOL=http
      - N8N_BASIC_AUTH_ACTIVE=false
      - GENERIC_TIMEZONE=Asia/Jakarta
      - N8N_ENCRYPTION_KEY=your-encryption-key-here
    volumes:
      - ./n8n_data:/home/node/.n8n
    depends_on:
      - ros2-sim
EOF
```

### 9.2 Build and Deploy

```bash
# Pull ARM64 compatible images
docker pull arm64v8/ros:iron
docker pull arm64v8/n8n:latest

# Build ROS 2 image for ARM64
docker compose -f docker-compose.prod.yml build --no-cache

# Deploy services
docker compose -f docker-compose.prod.yml up -d

# Check deployment status
docker compose -f docker-compose.prod.yml ps
docker compose -f docker-compose.prod.yml logs -f
```

## Step 10: Performance Optimization

### 10.1 Memory Optimization

```bash
# Configure swap file for better memory management
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make swap permanent
sudo tee -a /etc/fstab > /dev/null <<EOF
/swapfile none swap sw 0 0
EOF

# Configure swappiness
sudo tee /etc/sysctl.d/99-swap.conf > /dev/null <<EOF
vm.swappiness=10
EOF
```

### 10.2 CPU Optimization

```bash
# Enable CPU frequency scaling
sudo apt install -y cpufrequtils

# Set CPU governor to performance
sudo tee /etc/default/cpufrequtils > /dev/null <<EOF
GOVERNOR="performance"
MIN_SPEED="1500000"
MAX_SPEED="2400000"
EOF

# Apply CPU settings
sudo systemctl restart cpufrequtils
```

### 10.3 Storage Optimization

```bash
# Use faster I/O scheduler
sudo tee /etc/udev/rules.d/60-io-scheduler.rules > /dev/null <<EOF
ACTION=="add|change", KERNEL=="sd*[!0-9]", ATTR{queue/scheduler}="bfq"
ACTION=="add|change", KERNEL=="mmcblk*", ATTR{queue/scheduler}="bfq"
EOF

# Optimize Docker storage
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "data-root": "/var/lib/docker",
  "storage-driver": "overlay2",
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
EOF
```

## Step 11: Startup Automation

### 11.1 Systemd Service Configuration

```bash
# Create systemd service for robot deployment
sudo tee /etc/systemd/system/lks-robot.service > /dev/null <<EOF
[Unit]
Description=LKS Robot Project
After=docker.service network.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=ubuntu
WorkingDirectory=/home/ubuntu/LKS_Robot_Project
ExecStart=/usr/bin/docker compose -f docker-compose.prod.yml up -d
ExecStop=/usr/bin/docker compose -f docker-compose.prod.yml down
TimeoutStartSec=300
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable lks-robot
sudo systemctl start lks-robot

# Check service status
sudo systemctl status lks-robot
```

### 11.2 Auto-restart Configuration

```bash
# Configure Docker containers to restart automatically
docker update --restart unless-stopped lks_robot
docker update --restart unless-stopped lks_n8n

# Create health check script
tee ~/health_check.sh > /dev/null <<EOF
#!/bin/bash
# Health check script for LKS Robot

echo "Performing health checks..."

# Check Docker containers
if ! docker ps | grep -q lks_robot; then
    echo "ERROR: Robot container not running"
    exit 1
fi

if ! docker ps | grep -q lks_n8n; then
    echo "ERROR: n8n container not running"
    exit 1
fi

# Check API endpoints
if ! curl -s --max-time 5 http://localhost:5000/health > /dev/null; then
    echo "ERROR: Robot API not responding"
    exit 1
fi

if ! curl -s --max-time 5 http://localhost:5678 > /dev/null; then
    echo "ERROR: n8n interface not responding"
    exit 1
fi

echo "All services healthy"
EOF

chmod +x ~/health_check.sh

# Add to cron for regular health checks
(crontab -l ; echo "*/5 * * * * /home/ubuntu/health_check.sh") | crontab -
```

## Step 12: Testing and Verification

### 12.1 Hardware Testing

```bash
# Test GPIO functionality
python3 -c "
import gpiozero
import time

led = gpiozero.LED(17)
for i in range(5):
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
print('GPIO test completed')
"

# Test motor control
python3 -c "
import gpiozero
import time

step = gpiozero.OutputDevice(18)
dir_pin = gpiozero.OutputDevice(22)
enable = gpiozero.OutputDevice(17)

enable.off()
dir_pin.on()

for i in range(200):
    step.on()
    time.sleep(0.01)
    step.off()
    time.sleep(0.01)

print('Motor test completed')
"
```

### 12.2 API Testing

```bash
# Test robot API endpoints
curl -s http://localhost:5000/health
curl -s http://localhost:5000/api/robot/status
curl -s http://localhost:5000/api/robot/sensors
curl -s http://localhost:5000/api/robot/tasks

# Test n8n interface
curl -s -I http://localhost:5678
```

### 12.3 ROS 2 Testing

```bash
# Test ROS 2 functionality
docker exec lks_robot bash -c "
source /opt/ros/iron/setup.bash
source /root/ros2_ws/install/setup.bash

# List ROS 2 nodes
ros2 node list

# List topics
ros2 topic list

# Test service calls
ros2 service call /get_robot_status my_robot_automation/srv/GetRobotStatus '{}'
"
```

## Step 13: Monitoring and Maintenance

### 13.1 System Monitoring Setup

```bash
# Install monitoring tools
sudo apt install -y htop iotop sysstat

# Create monitoring script
tee ~/system_monitor.sh > /dev/null <<EOF
#!/bin/bash
# System monitoring script

echo "=== System Status ==="
echo "Uptime: $(uptime -p)"
echo "Load: $(uptime | awk -F'load average:' '{print $2}')"
echo "Memory: $(free -h | awk 'NR==2{printf "%.1f%% used", $3*100/$2}')"
echo "Disk: $(df -h / | awk 'NR==2{print $5}') used"

echo ""
echo "=== Docker Status ==="
docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"

echo ""
echo "=== ROS 2 Status ==="
docker exec lks_robot bash -c "
source /opt/ros/iron/setup.bash
source /root/ros2_ws/install/setup.bash
echo 'Nodes:'
ros2 node list | wc -l
echo 'Topics:'
ros2 topic list | wc -l
" 2>/dev/null || echo "ROS 2 not accessible"
EOF

chmod +x ~/system_monitor.sh
```

### 13.2 Log Management

```bash
# Configure log rotation
sudo tee /etc/logrotate.d/lks-robot > /dev/null <<EOF
/home/ubuntu/LKS_Robot_Project/logs/*.log {
    daily
    missingok
    rotate 7
    compress
    delaycompress
    notifempty
    create 644 ubuntu ubuntu
}
EOF

# Create log directory
mkdir -p ~/LKS_Robot_Project/logs

# Configure Docker log rotation
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
EOF
```

## Step 14: Backup and Recovery

### 14.1 Backup Configuration

```bash
# Create backup script
tee ~/backup_robot.sh > /dev/null <<EOF
#!/bin/bash
# Backup script for LKS Robot

BACKUP_DIR="/home/ubuntu/backups"
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_NAME="lks_robot_backup_$DATE"

mkdir -p $BACKUP_DIR

echo "Creating backup: $BACKUP_NAME"

# Stop services temporarily
docker compose -f docker-compose.prod.yml stop

# Backup configuration files
tar -czf $BACKUP_DIR/${BACKUP_NAME}_config.tar.gz \
    ~/LKS_Robot_Project/docker-compose.prod.yml \
    ~/LKS_Robot_Project/n8n_data \
    /etc/systemd/system/lks-robot.service \
    /etc/docker/daemon.json

# Backup Docker volumes
docker run --rm -v lks_robot_n8n_data:/data -v $BACKUP_DIR:/backup \
    alpine tar czf /backup/${BACKUP_NAME}_volumes.tar.gz -C / data

# Restart services
docker compose -f docker-compose.prod.yml start

echo "Backup completed: $BACKUP_DIR/${BACKUP_NAME}_*"
EOF

chmod +x ~/backup_robot.sh

# Schedule weekly backups
(crontab -l ; echo "0 2 * * 1 /home/ubuntu/backup_robot.sh") | crontab -
```

### 14.2 Recovery Procedures

```bash
# Create recovery script
tee ~/recover_robot.sh > /dev/null <<EOF
#!/bin/bash
# Recovery script for LKS Robot

if [ $# -eq 0 ]; then
    echo "Usage: $0 <backup_name>"
    echo "Available backups:"
    ls ~/backups/ | grep "lks_robot_backup"
    exit 1
fi

BACKUP_NAME=$1
BACKUP_DIR="/home/ubuntu/backups"

echo "Starting recovery from: $BACKUP_NAME"

# Stop services
docker compose -f docker-compose.prod.yml down

# Restore configuration
tar -xzf $BACKUP_DIR/${BACKUP_NAME}_config.tar.gz -C /

# Restore Docker volumes
docker run --rm -v lks_robot_n8n_data:/data -v $BACKUP_DIR:/backup \
    alpine sh -c "cd /backup && tar xzf ${BACKUP_NAME}_volumes.tar.gz -C / && mv /data/* /data_backup/ 2>/dev/null; true"

# Restart services
docker compose -f docker-compose.prod.yml up -d

echo "Recovery completed"
EOF

chmod +x ~/recover_robot.sh
```

## Troubleshooting Common Issues

### Hardware Issues

**GPIO Permission Denied**
```bash
# Add user to gpio group
sudo usermod -aG gpio ubuntu
# Reboot or logout/login
```

**I2C Not Working**
```bash
# Check I2C is enabled
sudo raspi-config nonint do_i2c 0
# Test bus
sudo i2cdetect -y 1
```

**Motor Not Moving**
```bash
# Check power supply voltage
# Verify motor driver connections
# Test with simple Python script
python3 -c "import gpiozero; m = gpiozero.OutputDevice(18); m.on(); import time; time.sleep(1); m.off()"
```

### Software Issues

**Docker Build Fails**
```bash
# Clear Docker cache
docker system prune -a
# Rebuild with no cache
docker compose build --no-cache
```

**ROS 2 Services Not Available**
```bash
# Check ROS 2 logs
docker logs lks_robot
# Verify services are running
docker exec lks_robot ros2 service list
```

**Network Issues**
```bash
# Check network configuration
ip addr show
# Test connectivity
ping 8.8.8.8
# Restart network
sudo systemctl restart networking
```

---

This comprehensive Raspberry Pi setup guide ensures successful deployment of the LKS Robot Project on ARM64 architecture. Follow each step carefully and test functionality at each stage before proceeding to the next step.
