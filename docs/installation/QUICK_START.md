# Quick Start Guide - Autonomous Mobile Manipulator

## Getting Started in 30 Seconds

### On Raspberry Pi

```bash
cd ~/Autonomous_Mobile_Manipulator
./run.sh start
```

That's it! The system will start and be accessible at **http://your-pi-ip:8000**

## Common Commands

### Basic Control

```bash
# Start the robot system
./run.sh start

# Start in development mode (simulated sensors)
./run.sh start --dev

# Stop the system
./run.sh stop

# Restart the system
./run.sh restart

# Check status
./run.sh status
```

### Monitoring

```bash
# View all logs
./run.sh logs

# View specific container logs
./run.sh logs ros2_sim_container

# Check system health
./run.sh status

# Enter container shell
./run.sh shell
```

### Maintenance

```bash
# Check prerequisites
./run.sh check

# Update system
./run.sh update

# Clean everything (removes all containers/volumes)
./run.sh clean
```

## Access Points

Once started, access the robot at:

- **Web Interface**: http://localhost:8000 (or http://raspberry-pi-ip:8000)
- **n8n Automation**: http://localhost:5678
- **API Endpoint**: http://localhost:5000/api/robot/status

## Sensors Tab

Navigate to the **Sensors** tab in the web interface to see:
- ✅ IMU (MPU6050) - Real-time orientation, acceleration, gyroscope
- ✅ IR Distance Sensors (6x Sharp GP2Y0A02YK0F)
- ✅ Ultrasonic Sensors (2x HC-SR04)
- ✅ Line Sensors (3x IR)
- ✅ Container Load Sensors (4x)

## Troubleshooting

### Container won't start
```bash
# Check Docker is running
docker ps

# View logs
./run.sh logs

# Restart everything
./run.sh restart
```

### Web interface not accessible
```bash
# Check status
./run.sh status

# Enter container and check manually
./run.sh shell
curl http://localhost:8000/health
```

### Sensors showing "--"
```bash
# Check if container has I2C/SPI access
./run.sh shell
ls -l /dev/i2c-1
i2cdetect -y 1  # Should show 0x68 for MPU6050
```

### Permission errors
```bash
# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker  # Or logout and login

# Verify
groups | grep docker
```

## Development Mode vs Production Mode

### Development Mode (`./run.sh start --dev`)
- ✅ Fast startup
- ✅ Simulated sensors
- ✅ No hardware required
- ✅ Perfect for testing workflows
- ❌ No real sensor data

### Production Mode (`./run.sh start`)
- ✅ Real hardware sensors
- ✅ MPU6050 IMU data
- ✅ Sharp IR sensors
- ✅ Full robot capabilities
- ⏱️ Slightly slower startup

## File Locations

```
~/Autonomous_Mobile_Manipulator/
├── run.sh                    # Main management script
├── docker-compose.yml        # Production compose file
├── docker-compose.dev.yml    # Development compose file
└── ros2_ws/
    └── src/my_robot_automation/
        ├── scripts/
        │   ├── web_robot_interface.py    # Web interface
        │   └── mpu6050_reader.py         # IMU reader
        ├── SETUP_GUIDE.md
        ├── API_DOCUMENTATION.md
        ├── SENSOR_WIRING.md
        └── MPU6050_SETUP.md
```

## Getting Help

```bash
# Show full help
./run.sh help

# Check prerequisites
./run.sh check

# View detailed status
./run.sh status
```

## Next Steps

1. **Access the web interface** - Navigate to http://your-pi-ip:8000
2. **Explore the Sensors tab** - See real-time IMU and sensor data
3. **Try Path Planning** - Use the waypoint manager to create paths
4. **Test Movement** - Use the Movement tab to control the robot
5. **Configure n8n** - Set up automation workflows at port 5678

## Emergency Commands

```bash
# Emergency stop all containers
./run.sh stop

# Force remove everything
docker stop $(docker ps -aq) && docker rm $(docker ps -aq)

# Nuclear option (removes everything including images)
./run.sh clean
```

## Status Indicators

When you run `./run.sh status`, you'll see:

- ✓ **Green** - Service is healthy
- ⚠ **Yellow** - Service has warnings
- ✗ **Red** - Service has errors

## Tips

1. **Always start with** `./run.sh check` **on first run**
2. **Use** `./run.sh logs` **to debug issues**
3. **Development mode is faster** for testing
4. **Production mode** requires hardware to be connected
5. **Hard refresh browser** (Ctrl+Shift+R) after system restarts

---

**Need more help?** Check the comprehensive guides in `ros2_ws/src/my_robot_automation/`

