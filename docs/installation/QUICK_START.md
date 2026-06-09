# Quick Start Guide

## Getting Started

### On Raspberry Pi

```bash
cd ~/Autonomous_Mobile_Manipulator
./start --hw
```

The system starts and becomes accessible at `http://<your-pi-ip>:8000`.

### On Development PC

```bash
cd ~/Autonomous_Mobile_Manipulator
./start --sim
```

## Common Commands

### Setup (one-time)

```bash
./setup --rpi         # Raspberry Pi setup
./setup --pc          # PC/development setup
./setup --help        # Show all options
```

### Starting the System

```bash
./start --hw          # Hardware mode (real sensors on Raspberry Pi)
./start --sim         # Simulation mode (no hardware required)
./start --test        # Test mode with interactive menu
./start --hw --build  # Rebuild containers and start
```

### System Management

```bash
./start --status      # Show system status
./start --logs        # View all logs
./start --stop        # Stop all containers
./start --restart     # Restart containers
./start --shell       # Enter container shell
./start --clean       # Clean Docker system
```

## Access Points

Once started:

- **Web Interface**: http://localhost:8000 (or http://<raspberry-pi-ip>:8000)
- **REST API**: http://localhost:8000/api/*
- **ROS 2 Container**: `docker exec -it ros2_sim_container bash`

## Sensors Tab

Navigate to the **Sensors** tab in the web interface:

- IMU (MPU6050) -- real-time orientation, acceleration, gyroscope
- IR Distance Sensors (6x Sharp GP2Y0A02YK0F)
- Ultrasonic Sensors (2x HC-SR04)
- Line Sensors (3x IR)
- Container Load Sensors (4x)

## Troubleshooting

### Container will not start

```bash
docker ps                        # Check Docker is running
./start --logs                   # View logs
./start --stop && ./start --hw   # Restart everything
```

### Web interface not accessible

```bash
./start --status
docker exec -it ros2_sim_container bash
curl http://localhost:8000/health
```

### Sensors showing "--"

```bash
# Check if container has I2C/SPI access
docker exec -it ros2_sim_container bash
ls -l /dev/i2c-1
i2cdetect -y 1  # Should show 0x68 for MPU6050
```

### Permission errors

```bash
sudo usermod -aG docker $USER
newgrp docker  # Or logout and login
groups | grep docker
```

## Development Mode vs Production Mode

| Feature | Development (`./start --sim`) | Production (`./start --hw`) |
|---------|-------------------------------|------------------------------|
| Fast startup | Yes | Slightly slower |
| Simulated sensors | Yes | No (real hardware) |
| Hardware required | No | Yes (Raspberry Pi + Arduino Mega) |
| Real sensor data | No | Yes |

## File Locations

```
Autonomous_Mobile_Manipulator/
├── setup                       # One-time setup script
├── start                       # Runtime management script
├── docker-compose.yml          # Container orchestration
├── ros2_ws/
│   └── src/my_robot_automation/
│       └── scripts/
│           ├── web_robot_interface.py    # Web interface
│           ├── rest_api_server.py        # REST API server
│           ├── mega_interface.py         # Arduino Mega serial interface
│           ├── gpio_controller.py        # GPIO controller
│           ├── sensor_manager.py         # Sensor data processing
│           └── path_planning.py          # Navigation algorithms
```

## Emergency Commands

```bash
./start --stop                  # Stop all containers
docker stop $(docker ps -aq)    # Force stop all containers
./start --clean                 # Remove all containers and volumes
```

## Next Steps

1. Access the web interface at http://<your-pi-ip>:8000
2. Explore the Sensors tab for real-time IMU and sensor data
3. Use the Path Planning tab for waypoint navigation
4. Test movement controls in the Movement tab
