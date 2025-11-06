# Troubleshooting Guide

This guide provides comprehensive troubleshooting procedures for common issues encountered with the Autonomous Mobile Manipulator robot system.

## Table of Contents

- [Quick Diagnostic Commands](#quick-diagnostic-commands)
  - [System Status Checks](#system-status-checks)
- [Common Issues and Solutions](#common-issues-and-solutions)
- [Installation Issues](#installation-issues)
  - [Docker Installation Problems](#docker-installation-problems)
  - [Repository Clone Issues](#repository-clone-issues)
- [Runtime Issues](#runtime-issues)
  - [Container Startup Problems](#container-startup-problems)
  - [ROS 2 Import Errors](#ros-2-import-errors)
  - [ROS 2 Communication Issues](#ros-2-communication-issues)
- [Hardware Issues](#hardware-issues)
  - [Motor Control Problems](#motor-control-problems)
  - [Sensor Integration Issues](#sensor-integration-issues)
- [Network and Communication Issues](#network-and-communication-issues)
  - [API Connectivity Problems](#api-connectivity-problems)
  - [ROS 2 Network Issues](#ros-2-network-issues)
- [Performance Issues](#performance-issues)
  - [System Performance Problems](#system-performance-problems)
- [LabVIEW Integration Issues](#labview-integration-issues)
  - [TCP Communication Problems](#tcp-communication-problems)
  - [VI Execution Issues](#vi-execution-issues)
- [Emergency Procedures](#emergency-procedures)
  - [Emergency Stop Not Responding](#emergency-stop-not-responding)
  - [System Lockup](#system-lockup)
- [Diagnostic Tools](#diagnostic-tools)
  - [Built-in Diagnostic Commands](#built-in-diagnostic-commands)
  - [Log Analysis](#log-analysis)
  - [Network Diagnostics](#network-diagnostics)
- [Recovery Procedures](#recovery-procedures)
  - [Clean System Reset](#clean-system-reset)
  - [Configuration Reset](#configuration-reset)
- [Prevention Measures](#prevention-measures)
  - [Regular Maintenance](#regular-maintenance)
  - [Monitoring Setup](#monitoring-setup)
- [Getting Help](#getting-help)
  - [Self-Service Resources](#self-service-resources)
  - [Professional Support](#professional-support)
  - [Emergency Contacts](#emergency-contacts)

## Quick Diagnostic Commands

### System Status Checks

```bash
# Check Docker services
docker compose ps

# Check container logs
docker compose logs ros2_sim_container
docker compose logs n8n_container

# Check ROS 2 nodes
docker exec -it ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && ros2 node list"

# Check ROS 2 topics
docker exec -it ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && ros2 topic list"

# Check system resources
docker stats
```

## Common Issues and Solutions

## Installation Issues

### Docker Installation Problems

**Issue**: Docker command not found
```bash
# Check if Docker is installed
which docker

# Reinstall Docker if necessary
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
# Logout and login again for changes to take effect
```

**Issue**: Docker daemon not running
```bash
# Check Docker service status
sudo systemctl status docker

# Start Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Check Docker info
docker info
```

**Issue**: Permission denied for Docker commands
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Alternative: Run Docker with sudo
sudo docker compose up --build -d
```

### Repository Clone Issues

**Issue**: Git clone fails
```bash
# Check internet connectivity
ping github.com

# Try alternative clone method
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git

# Check disk space
df -h
```

**Issue**: Permission denied in cloned directory
```bash
# Fix ownership permissions
sudo chown -R $USER:$USER Autonomous_Mobile_Manipulator/

# Check file permissions
ls -la Autonomous_Mobile_Manipulator/
```

## Runtime Issues

### Container Startup Problems

**Issue**: Containers fail to start
```bash
# Check container logs for errors
docker compose logs

# Clean up Docker system
docker system prune -a

# Rebuild containers
docker compose build --no-cache
docker compose up -d

# Check available disk space
df -h
```

**Issue**: Port conflicts
```bash
# Check if ports are in use
netstat -tuln | grep -E '(5678|8765)'

# Modify port mappings in docker-compose.yml
# Change ports if conflicts exist
```

**Issue**: X11 display issues (GUI applications)
```bash
# Allow Docker X11 access
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Test X11 forwarding
xeyes  # Should show animated eyes

# Alternative: Disable GUI if not needed
# Comment out DISPLAY and X11 volumes in docker-compose.yml
```

### ROS 2 Import Errors

**Issue**: `Import "ament_index_python.packages" could not be resolved` or similar ROS2 import errors

**Symptoms**:
- Launch files show import errors when opened in IDE
- Python files fail to import ROS2 packages on host system
- Error messages about missing `ament_index_python`, `launch`, or `launch_ros` packages

**Cause**: ROS2 packages are only available within Docker containers, not on host systems.

**Solutions**:
```bash
# Verify you're in the correct environment - this should work in container:
docker exec -it ros2_sim_container bash
cd /root/ros2_ws
source /opt/ros/iron/setup.bash
python3 -c "from ament_index_python.packages import get_package_share_directory; print('ROS2 import successful')"

# For development on host system - this is expected behavior:
# Launch files include conditional imports that provide clear error messages
# directing you to use the Docker container environment

# If you need to install ROS2 on host for development:
# Ubuntu/Debian:
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-iron-desktop -y
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Prevention**: Always run ROS2 commands and imports within the Docker container environment. The conditional imports in launch files are designed to guide you to the correct execution environment.

### ROS 2 Communication Issues

**Issue**: ROS 2 nodes not communicating
```bash
# Check ROS domain ID consistency
echo $ROS_DOMAIN_ID  # Should be same across containers

# Set consistent domain ID
export ROS_DOMAIN_ID=0

# Restart containers with consistent domain
docker compose down
docker compose up -d
```

**Issue**: Topic echo shows no data
```bash
# Check if nodes are running
ros2 node list

# Check topic publishers
ros2 topic info /topic_name

# Verify topic types match
ros2 topic type /topic_name
ros2 interface show topic_type
```

**Issue**: Controller manager fails to start
```bash
# Check controller configuration
ros2 control list_controllers

# Validate URDF and controller config
ros2 launch my_robot_bringup robot.launch.py --show-args

# Check for missing dependencies
apt list --installed | grep ros-iron
```

## Hardware Issues

### Motor Control Problems

**Issue**: Motors not responding to commands
```bash
# Test motor connections
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Check motor driver power and connections
# Verify GPIO pin assignments in hardware setup

# Test individual motor control
ros2 run my_robot_bringup motor_test
```

**Issue**: Motors running in wrong direction
```bash
# Reverse direction logic in software
# Edit teleop_joy.yaml or controller configuration

# Swap motor phase connections if hardware allows

# Update coordinate system transformation
```

**Issue**: Motor overheating or stalling
```bash
# Check current limits in motor driver configuration
# Reduce maximum velocity settings
# Verify wheel friction and mechanical binding

# Monitor motor temperature
ros2 topic echo /joint_states  # Check effort values
```

### Sensor Integration Issues

**Issue**: LiDAR not detected
```bash
# Check USB connection and permissions
ls /dev/ttyUSB*

# Verify RPLidar service is running
ros2 node list | grep rplidar

# Test with simple scan
ros2 topic echo /scan --once

# Check power supply to LiDAR
# Verify baud rate settings (115200)
```

**Issue**: IMU data incorrect or noisy
```bash
# Check I2C bus connectivity
i2cdetect -y 1

# Verify sensor mounting orientation
# Check for electromagnetic interference

# Calibrate IMU offsets
ros2 run imu_filter_madgwick imu_filter_madgwick_node _use_mag:=false
```

**Issue**: Camera not streaming
```bash
# Check camera connection
ls /dev/video*

# Verify camera permissions
sudo usermod -aG video $USER

# Test camera with simple viewer
ros2 run image_view image_view image:=/camera/image_raw
```

## Network and Communication Issues

### API Connectivity Problems

**Issue**: Webhook endpoints not responding
```bash
# Check n8n container status
docker compose ps | grep n8n

# Test webhook directly
curl -I http://localhost:5678/webhook/robot-control

# Check n8n logs for errors
docker logs n8n_container

# Verify port accessibility
netstat -tuln | grep 5678
```

**Issue**: WebSocket connection fails
```bash
# Check Foxglove Bridge status
docker compose ps | grep ros2_sim_container

# Test WebSocket connection
websocat ws://localhost:8765 --text '{"type": "ping"}'

# Check firewall settings
sudo ufw status

# Verify WebSocket server logs
docker logs ros2_sim_container | grep foxglove
```

### ROS 2 Network Issues

**Issue**: ROS 2 nodes not discovering each other
```bash
# Check ROS domain ID
echo $ROS_DOMAIN_ID

# Set consistent domain ID across all nodes
export ROS_DOMAIN_ID=42

# Check network interfaces
ip addr show

# Test ROS 2 daemon
ros2 daemon status
```

## Performance Issues

### System Performance Problems

**Issue**: High CPU usage
```bash
# Monitor resource usage
docker stats

# Check for memory leaks
ros2 doctor

# Reduce simulation complexity if using Gazebo
export GAZEBO_GPU=0  # Use software rendering

# Optimize topic publication rates
# Edit sensor configurations to reduce frequency
```

**Issue**: Slow response times
```bash
# Check network latency
ping localhost

# Monitor container performance
docker exec ros2_sim_container top

# Profile ROS 2 node performance
ros2 topic hz /topic_name

# Optimize message queue sizes
# Reduce queue_size in publisher configurations
```

**Issue**: Memory consumption growing
```bash
# Monitor memory usage over time
docker stats --no-stream

# Check for memory leaks in custom nodes
ros2 doctor --report

# Clean up old log files
docker system prune -a

# Restart containers periodically if needed
```

## LabVIEW Integration Issues

### TCP Communication Problems

**Issue**: LabVIEW TCP connection fails
```labview
# Diagnostic VI for connection testing:
TCP Open Connection Test:
├── IP Address: localhost or robot IP
├── Port: 8765 (WebSocket) or 5678 (HTTP)
├── Timeout: 5000ms
└── Error handling for connection failures

# Check firewall and port accessibility
# Verify network configuration
```

**Issue**: Data parsing errors in LabVIEW
```labview
# Add robust JSON parsing:
Try-Catch Structure:
├── JSON Parse with error handling
├── Validate message structure
├── Handle missing fields gracefully
└── Log parsing errors for debugging
```

### VI Execution Issues

**Issue**: LabVIEW VI runs slowly
```labview
# Performance optimization:
SubVI Optimization:
├── Reduce unnecessary operations
├── Use shift registers for data flow
├── Implement asynchronous processing
└── Profile execution time with timing VIs
```

**Issue**: Memory usage in LabVIEW
```labview
# Memory management:
VI Memory Optimization:
├── Clear large data arrays when not needed
├── Use data value references for large datasets
├── Implement proper cleanup in error cases
└── Monitor memory usage with profiling tools
```

## Emergency Procedures

### Emergency Stop Not Responding

**Issue**: Emergency stop command ignored
```bash
# Immediate hardware intervention:
1. Disconnect power supply immediately
2. Check emergency stop button wiring
3. Verify emergency stop topic subscription

# Software debugging:
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"

# Check emergency stop handler
ros2 service call /emergency_stop std_srvs/Trigger
```

### System Lockup

**Issue**: Complete system freeze
```bash
# Force restart procedures:
# 1. Hardware reset: Power cycle the robot
# 2. Software reset: Kill all Docker containers
docker stop $(docker ps -q)

# 3. System reboot if necessary
sudo reboot

# 4. Check system logs after recovery
journalctl -u docker --since "1 hour ago"
```

## Diagnostic Tools

### Built-in Diagnostic Commands

```bash
# ROS 2 system health check
ros2 doctor

# Topic communication analysis
ros2 topic bw /cmd_vel  # Bandwidth usage
ros2 topic hz /scan     # Publication rate

# Node status monitoring
ros2 node info /node_name

# Service availability check
ros2 service list
ros2 service call /service_name std_srvs/Trigger
```

### Log Analysis

```bash
# Check system logs
sudo journalctl -u docker -f

# Monitor container logs
docker compose logs -f ros2_sim_container

# Check n8n workflow logs
docker logs n8n_container | grep ERROR

# Review ROS 2 logs
docker exec ros2_sim_container bash -c "tail -f /root/.ros/log/latest/*.log"
```

### Network Diagnostics

```bash
# Check network connectivity
ping localhost
ping 8.8.8.8

# Check port accessibility
telnet localhost 5678
telnet localhost 8765

# Monitor network traffic
sudo tcpdump -i lo port 5678 or port 8765

# Check firewall rules
sudo ufw status
```

## Recovery Procedures

### Clean System Reset

```bash
# Stop all services
docker compose down

# Clean up Docker system
docker system prune -a --volumes

# Clean up ROS 2 workspace
docker exec ros2_sim_container bash -c "rm -rf /root/ros2_ws/*"

# Rebuild and restart
docker compose build --no-cache
docker compose up -d
```

### Configuration Reset

```bash
# Reset to default configuration
cp docker-compose.yml.backup docker-compose.yml

# Reset ROS 2 configurations
docker exec ros2_sim_container bash -c "cp -r /opt/ros/iron/share/* /root/ros2_ws/"

# Restore n8n workflows
docker compose restart n8n_container
```

## Prevention Measures

### Regular Maintenance

```bash
# Weekly system checks:
# 1. Update system packages
sudo apt update && sudo apt upgrade -y

# 2. Clean Docker system
docker system prune -f

# 3. Check disk space
df -h && du -sh /var/lib/docker/

# 4. Verify backups
ls -la ~/backups/

# 5. Test emergency stop functionality
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

### Monitoring Setup

```bash
# Automated monitoring script
#!/bin/bash
# Check system health daily
docker compose ps
docker stats --no-stream
df -h
```

## Getting Help

### Self-Service Resources
1. **Documentation**: Check relevant sections in this troubleshooting guide
2. **Logs**: Review system and container logs for error messages
3. **Examples**: Test with provided example commands and configurations
4. **Community**: Search GitHub issues for similar problems

### Professional Support
- **GitHub Issues**: [Report bugs and request features](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues)
- **ROS Community**: [ROS Discourse](https://discourse.ros.org/) and [ROS Answers](https://answers.ros.org/)
- **Docker Support**: [Docker Community Forums](https://forums.docker.com/)
- **LabVIEW Support**: [NI Community](https://forums.ni.com/)

### Emergency Contacts
- **Critical System Failure**: Check emergency stop procedures above
- **Hardware Damage**: Document issue and contact manufacturer
- **Safety Concerns**: Immediately disconnect power and assess situation

---

*This troubleshooting guide provides systematic approaches to diagnose and resolve issues with the Autonomous Mobile Manipulator robot system. Follow the diagnostic procedures in order and document your findings for future reference.*
