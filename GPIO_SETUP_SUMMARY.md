# ROS2 GPIO Setup Summary

## What Was Implemented

### 1. GPIO Dependencies and Installation

- `install_gpio_dependencies.sh` installs pigpio, gpiozero, RPi.GPIO, and sensor libraries on the Raspberry Pi host.
- Configures pigpiod daemon and GPIO permissions for container access.

### 2. Docker GPIO Configuration

- `docker-compose.rpi.yml` added for Raspberry Pi deployment with device passthrough.
- Devices exposed: `/dev/gpiomem`, `/dev/mem`, I2C (`/dev/i2c-1`), SPI (`/dev/spidev0.0`).
- Privileged mode enabled for hardware access.

### 3. GPIO Controller

- `gpio_controller.py` provides the `GPIOController` class.
- Uses `lgpio` as primary library, `gpiozero` as fallback.
- Falls back to simulation mode when no GPIO libraries are available.
- Current implementation covers ADC reading (Sharp IR sensors via SPI) and sensor initialization.
- Motor and servo control routes through the Arduino Mega via serial, not direct GPIO.

### 4. Testing and Diagnostics

- `test_gpio_standalone.py` — basic GPIO testing on the host.
- `test_gpio_container.py` — Docker container GPIO testing.
- Logging and error handling included for troubleshooting.

### 5. Documentation

- `RASPBERRY_PI_GPIO_SETUP.md` — full setup guide.
- `GPIO_CONTROL_SETUP.md` — updated with Docker instructions.

## How to Use on Raspberry Pi

### Step 1: Install Dependencies (on Raspberry Pi host)

```bash
cd ~/Autonomous_Mobile_Manipulator
sudo ./install_gpio_dependencies.sh
```

### Step 2: Start ROS2 with GPIO

```bash
docker compose -f docker-compose.rpi.yml up ros2-hardware
```

### Step 3: Test GPIO

```bash
# From host
python3 ros2_ws/src/my_robot_automation/scripts/test_gpio.py

# From container
docker exec ros2_hardware_container python3 test_gpio_container.py
```

### Step 4: Access Web Interface

Open `http://<raspberry-pi-ip>:8000`. All controls route through the Arduino Mega for hardware actuation.

## GPIO Pin Configuration

### Raspberry Pi Pins (Direct Control)

| Function | Pin | Type |
|----------|-----|------|
| IMU (MPU6050) SDA | GPIO2 | I2C |
| IMU (MPU6050) SCL | GPIO3 | I2C |
| Line Sensor Left | GPIO4 | Digital IN |
| Line Sensor Center | GPIO17 | Digital IN |
| Line Sensor Right | GPIO27 | Digital IN |
| LiDAR (TF-Luna) TX | GPIO14 | UART |
| LiDAR (TF-Luna) RX | GPIO15 | UART |

### Arduino Mega Pins (via YFROBOT Shield)

| Function | Interface |
|----------|-----------|
| 4x PG23 Motors (M1-M4) | I2C to YFROBOT shield |
| IR Distance Sensors (6x) | Analog A0-A5 |
| Ultrasonic Sensors (2x) | Digital (Trigger/Echo) |
| Line Sensors (3x) | Digital pins |
| IMU (MPU6050) | I2C (SDA/SCL) |
| Servo Actuators | PWM via shield |

## Troubleshooting

### If GPIO does not work:

1. **Check pigpiod status:**
   ```bash
   sudo systemctl status pigpiod
   ```

2. **Check GPIO permissions:**
   ```bash
   groups $USER | grep gpio
   ```

3. **Test GPIO access:**
   ```bash
   python3 test_gpio_standalone.py
   ```

4. **Check container logs:**
   ```bash
   docker logs ros2_hardware_container | grep GPIO
   ```

### Common Issues

| Problem | Fix |
|---------|-----|
| pigpiod not running | `sudo systemctl start pigpiod` |
| Permission denied | Add user to gpio group: `sudo usermod -aG gpio $USER` |
| Libraries not found | Use `docker-compose.rpi.yml`, not the dev compose file |
| I2C not detected | Run `i2cdetect -y 1` — should show MPU6050 at 0x68 |

## Current Status

- GPIO controller initialized in `web_robot_interface.py` on startup.
- Web API endpoints (`/api/robot/move`, `/api/robot/picker/gripper`, etc.) route commands to the Arduino Mega via serial.
- Motor and servo actuation handled by Arduino Mega firmware, not direct GPIO.
- System falls back to simulation mode if GPIO or serial is unavailable.
