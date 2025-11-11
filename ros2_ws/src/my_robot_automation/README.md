# Autonomous Mobile Manipulator Robot System

## Overview

Hexagonal-shaped autonomous mobile robot with omnidirectional movement, object manipulation capabilities, and comprehensive sensor suite. The robot uses 6x Sharp GP2Y0A02YK0F analog IR distance sensors for wall alignment and obstacle detection, connected via MCP3008 ADC over SPI interface.

## Key Features

- Omnidirectional movement with 3 omni wheels
- Gripper system for object manipulation
- 6x analog IR distance sensors (Sharp GP2Y0A02YK0F)
- Ultrasonic sensors and LIDAR for navigation
- IMU for orientation tracking
- Line following capability
- Container storage system
- Professional web interface for control and monitoring
- ROS2 integration
- Simulation mode for development without hardware

## Hardware Architecture

### Movement System
- 3x omni wheels (front-left, front-right, back)
- Hexagonal robot chassis
- Omnidirectional motion capability

### Sensor Suite
- **IR Distance Sensors**: 6x Sharp GP2Y0A02YK0F (20-150cm range)
  - Connected via MCP3008 10-bit ADC (SPI interface)
  - Left/right/back positioning for wall alignment
- **Ultrasonic**: 2x HC-SR04 (front sensors)
- **LIDAR**: TF-Luna single-point (USB/Serial)
- **IMU**: MPU6050 (I2C)
- **Line Sensors**: 3x IR sensors for line following
- **Container Sensors**: 4x load detection sensors

### Manipulation System
- Gripper with open/close servo
- Tilt control servo
- Optional rotation servo (360°)
- Vertical lifter motor
- USB camera on gripper assembly

### Control System
- Raspberry Pi 5 with Ubuntu Server
- Docker containerized services
- ROS2 for low-level control
- n8n for workflow automation
- Tailscale for remote access

## Software Architecture

```
┌─────────────────────────────────────┐
│   Web Interface (Flask + HTML)     │
│   Port 8000 - Control Dashboard     │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  WebRobotInterface (ROS2 Node)     │
│  - Sensor reading (SPI/GPIO/I2C)   │
│  - Motor control                    │
│  - API endpoints                    │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│       Hardware Layer                │
│  - MCP3008 ADC (SPI)                │
│  - MPU6050 IMU (I2C)                │
│  - Motor drivers (GPIO/PWM)         │
│  - Servo controllers (GPIO/PWM)     │
└─────────────────────────────────────┘
```

## Quick Start

### Prerequisites

- Raspberry Pi 5 with Ubuntu Server
- SPI and I2C interfaces enabled
- ROS2 installed
- Python 3.8+

### Installation

```bash
# Clone repository
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws/src/my_robot_automation

# Install dependencies
pip3 install -r requirements.txt

# Enable SPI interface
sudo raspi-config
# Navigate to: Interface Options → SPI → Enable

# Build ROS2 workspace
cd /home/azzar/project/robotic/lks_robot_project/ros2_ws
colcon build --packages-select my_robot_automation
source install/setup.bash
```

### Running

```bash
# Start web interface (with hardware)
ros2 run my_robot_automation web_robot_interface.py

# Access web interface
# Open browser: http://localhost:8000
# Or remote: http://<robot-ip>:8000
```

### Testing Without Hardware

```bash
# System automatically detects missing hardware and enables simulation mode
ros2 run my_robot_automation web_robot_interface.py
# Sensor data will be simulated for development/testing
```

## Documentation

- [Setup Guide](SETUP_GUIDE.md) - Complete hardware and software setup instructions
- [Sensor Wiring](SENSOR_WIRING.md) - Detailed wiring diagrams for Sharp sensors
- [API Documentation](API_DOCUMENTATION.md) - REST API reference
- [Sharp Sensor Update Summary](SHARP_SENSOR_UPDATE_SUMMARY.md) - Implementation details

## Web Interface Features

### Tabs

1. **Movement** - Manual control, speed adjustment, directional movement
2. **Manipulation** - Gripper control, tilt/position adjustment
3. **Containers** - Storage container management
4. **Automation** - Patrol routes, pick-and-place, n8n integration
5. **Safety** - Emergency stop controls
6. **Status** - System health, logs, command history
7. **Path Planning** - Waypoint management, route visualization
8. **Sensors** - Real-time sensor readings and diagnostics
9. **Hardware** - System specifications and GPIO pinout

### API Endpoints

- `GET /health` - Health check
- `GET /api/robot/sensors` - Current sensor readings
- `GET /api/robot/sensors/diagnostics` - Sensor health and diagnostics
- `GET /api/robot/status` - System status

See [API Documentation](API_DOCUMENTATION.md) for complete reference.

## Sensor Specifications

### Sharp GP2Y0A02YK0F IR Distance Sensors

- Type: Analog IR Distance Sensor
- Range: 20-150 cm (200-1500 mm)
- Output: Analog voltage 0.4V-2.7V
- Power: 5V DC, 33 mA typical
- Response Time: 38±10 ms
- Interface: MCP3008 ADC via SPI
- Resolution: 10-bit (1024 levels)
- Sample Rate: 5 readings per sensor, median filtered

### MCP3008 ADC Configuration

- Resolution: 10-bit (1024 levels)
- Reference Voltage: 3.3V
- SPI Speed: 1.35 MHz
- Channels Used: 0-5 (6 sensors)
- Interface: SPI0 on Raspberry Pi

## Testing Tools

### Sharp Sensor Test Utility

```bash
python3 scripts/test_sharp_sensors.py
```

Features:
- Test individual sensors
- Continuous monitoring of all 6 sensors
- Calibration mode with voltage measurement
- Visual bar graphs for distance display

### API Testing

```bash
# Get sensor data
curl http://localhost:8000/api/robot/sensors | jq

# Get diagnostics
curl http://localhost:8000/api/robot/sensors/diagnostics | jq

# Monitor continuously
watch -n 1 'curl -s http://localhost:8000/api/robot/sensors | jq ".data.laser_sensors"'
```

## Configuration

### Sensor Calibration

Adjust conversion formula in `web_robot_interface.py`:

```python
# Line ~2633
distance_cm = 60 * (voltage ** -1.1) - 1
```

Calibrate using test utility:
```bash
python3 scripts/test_sharp_sensors.py
# Select option 3: Calibration mode
```

### Web Interface Port

Edit `web_robot_interface.py` line ~2753:

```python
self.app.run(host='0.0.0.0', port=8000)
```

## Troubleshooting

### SPI Not Working

```bash
# Check SPI enabled
lsmod | grep spi

# Check devices
ls -l /dev/spidev*

# Add user to spi group
sudo usermod -a -G spi $USER
```

### Sensor Readings Zero

1. Verify 5V power to sensors
2. Check MCP3008 wiring (especially VREF = 3.3V)
3. Measure sensor output voltage (should be 0.4-2.7V)
4. Test with individual sensor test utility

### Web Interface Not Loading

```bash
# Check Flask server running
ps aux | grep web_robot_interface

# Check port available
sudo netstat -tlnp | grep :8000

# Check firewall
sudo ufw allow 8000/tcp
```

See [Setup Guide](SETUP_GUIDE.md) for comprehensive troubleshooting.

## Development

### Simulation Mode

For development without hardware:

```python
# Automatic detection
# System falls back to simulation if SPI unavailable

# Force simulation mode
interface = WebRobotInterface(simulation_mode=True)
```

Simulation features:
- Simulated sensor readings with sinusoidal patterns
- All API endpoints functional
- No hardware access required
- Useful for UI development and testing

### Adding New Sensors

1. Update `sensor_channels` dictionary in `WebRobotInterface.__init__`
2. Add sensor reading logic in `read_all_sensors()`
3. Update HTML template with display elements
4. Add JavaScript update code for live readings

## Project Structure

```
my_robot_automation/
├── scripts/
│   ├── web_robot_interface.py    # Main interface application
│   └── test_sharp_sensors.py     # Sensor testing utility
├── requirements.txt               # Python dependencies
├── README.md                      # This file
├── SETUP_GUIDE.md                # Complete setup instructions
├── SENSOR_WIRING.md              # Hardware wiring guide
├── API_DOCUMENTATION.md          # API reference
└── SHARP_SENSOR_UPDATE_SUMMARY.md # Implementation notes
```

## Requirements

### Hardware
- Raspberry Pi 5
- MCP3008 ADC
- Sharp GP2Y0A02YK0F sensors (6x)
- Power supply (12V, 5A+)

### Software
- Ubuntu Server 22.04+
- ROS2 (Humble or later)
- Python 3.8+
- Flask 2.3+
- spidev 3.6+

## License

Internal project documentation.

## Contributors

Development team.

## Support

For issues:
1. Check sensor diagnostics: `/api/robot/sensors/diagnostics`
2. Review setup guide troubleshooting section
3. Test individual components with provided utilities
4. Check system logs

## Roadmap

- [ ] Add calibration file storage
- [ ] Implement WebSocket for real-time streaming
- [ ] Add historical data logging
- [ ] Implement obstacle avoidance algorithms
- [ ] Add path planning visualization
- [ ] Integrate object recognition
- [ ] Add voice control interface
- [ ] Implement SLAM capabilities

