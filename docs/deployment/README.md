# Deployment Guide

This guide provides comprehensive instructions for deploying the Autonomous Mobile Manipulator robot system to production environments, including Raspberry Pi 5 target hardware.

## Table of Contents

- [Deployment Overview](#deployment-overview)
  - [Deployment Targets](#deployment-targets)
- [Pre-deployment Checklist](#pre-deployment-checklist)
  - [System Requirements Verification](#system-requirements-verification)
  - [Software Dependencies](#software-dependencies)
  - [Hardware Connectivity](#hardware-connectivity)
- [Development Deployment](#development-deployment)
  - [Local Development Setup](#local-development-setup)
  - [Development Workflow](#development-workflow)
- [Production Deployment](#production-deployment)
  - [Raspberry Pi 5 Target Deployment](#raspberry-pi-5-target-deployment)
  - [Production Optimization](#production-optimization)
- [Edge Deployment](#edge-deployment)
  - [Industrial PC Deployment](#industrial-pc-deployment)
  - [Remote Monitoring Setup](#remote-monitoring-setup)
- [Deployment Validation](#deployment-validation)
  - [System Health Checks](#system-health-checks)
  - [Performance Benchmarks](#performance-benchmarks)
  - [Security Validation](#security-validation)
- [Scaling and High Availability](#scaling-and-high-availability)
  - [Multi-Robot Deployment](#multi-robot-deployment)
  - [Backup and Recovery](#backup-and-recovery)
- [Monitoring and Maintenance](#monitoring-and-maintenance)
  - [System Monitoring](#system-monitoring)
  - [Application Monitoring](#application-monitoring)
  - [Maintenance Procedures](#maintenance-procedures)
- [Troubleshooting Deployment Issues](#troubleshooting-deployment-issues)
  - [Common Deployment Problems](#common-deployment-problems)
- [Performance Optimization](#performance-optimization)
  - [Production Performance Tuning](#production-performance-tuning)
- [Support and Updates](#support-and-updates)
  - [Version Management](#version-management)
  - [Rollback Procedures](#rollback-procedures)
  - [Documentation Maintenance](#documentation-maintenance)
- [Resources and References](#resources-and-references)
  - [Official Documentation](#official-documentation)
  - [Security Resources](#security-resources)
  - [Monitoring Tools](#monitoring-tools)
  - [Community Resources](#community-resources)

## Deployment Overview

### Deployment Targets

#### Development Deployment
- **Platform**: Development PC (Ubuntu/Debian)
- **Purpose**: Development, testing, and demonstration
- **Access**: Local network, direct hardware access

#### Production Deployment
- **Platform**: Raspberry Pi 5 (Ubuntu Server)
- **Purpose**: Autonomous operation, field deployment
- **Access**: Remote network, wireless connectivity

#### Edge Deployment
- **Platform**: Industrial PC or embedded system
- **Purpose**: High-reliability applications, 24/7 operation
- **Access**: VPN, remote monitoring systems

## Pre-deployment Checklist

### System Requirements Verification

```bash
# Check system resources
df -h  # Disk space
free -h  # Memory
lscpu   # CPU information

# Verify network configuration
ip addr show
ping 8.8.8.8  # Internet connectivity

# Check USB devices (for sensors)
lsusb

# Verify serial ports
ls /dev/tty*
```

### Software Dependencies

```bash
# Verify Docker installation
docker --version
docker compose version

# Check ROS 2 environment (in container)
docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && ros2 --help"

# Verify n8n web interface
curl -I http://localhost:5678
```

### Hardware Connectivity

```bash
# Test motor connections
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# Verify sensor data
ros2 topic echo /scan --once
ros2 topic echo /imu/data --once

# Check camera functionality
ros2 run image_view image_view image:=/camera/image_raw _image_transport:=compressed
```

## Development Deployment

### Local Development Setup

#### 1. Clone and Configure

```bash
# Clone repository to development machine
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git
cd Autonomous_Mobile_Manipulator

# Configure for development
cp docker-compose.dev.yml docker-compose.override.yml
```

#### 2. Development Configuration

```yaml
# docker-compose.override.yml
version: '3.8'
services:
  ros2-sim:
    environment:
      - ROS_LOG_LEVEL=debug
      - DISPLAY=${DISPLAY}
    volumes:
      - ./ros2_ws/src:/root/ros2_ws/src
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

  n8n:
    ports:
      - "5678:5678"
```

#### 3. Launch Development Environment

```bash
# Build and start development environment
docker compose up --build -d

# Verify all services running
docker compose ps

# Access development interfaces
# n8n: http://localhost:5678
# ROS 2: docker exec -it ros2_sim_container bash
```

### Development Workflow

#### Hot Reloading Setup

```bash
# Enable live code reloading
# Edit files in ros2_ws/src/ on host
# Changes automatically sync to container

# Rebuild specific packages
docker exec -it ros2_sim_container bash
cd /root/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# Test changes immediately
ros2 launch my_robot_bringup robot.launch.py
```

#### Debugging Setup

```bash
# Enable debug logging
export ROS_LOG_LEVEL=debug
export GAZEBO_VERBOSE=1

# Launch with debug information
ros2 launch my_robot_bringup gazebo_world.launch.py --debug

# Monitor system resources
docker stats
htop
```

## Production Deployment

### Raspberry Pi 5 Target Deployment

#### 1. Hardware Preparation

**Required Components**:
- Raspberry Pi 5 (8GB RAM recommended)
- Ubuntu Server 22.04 microSD card (64-bit)
- Power supply (5V/3A USB-C)
- Network connectivity (Ethernet or WiFi)
- USB sensors (LiDAR, camera)

**Installation Steps**:

```bash
# Flash Ubuntu Server to microSD
# Use Raspberry Pi Imager tool
# Select: Ubuntu Server 22.04 LTS (64-bit)

# Boot Raspberry Pi and configure
# Set hostname, enable SSH, configure network
sudo raspi-config

# Update system packages
sudo apt update && sudo apt upgrade -y

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker ubuntu
```

#### 2. Project Transfer

```bash
# On development machine
# Create deployment package
tar -czf autonomous_mobile_manipulator.tar.gz Autonomous_Mobile_Manipulator/

# Transfer to Raspberry Pi
scp autonomous_mobile_manipulator.tar.gz ubuntu@raspberrypi5:~/

# On Raspberry Pi
# Extract project files
tar -xzf autonomous_mobile_manipulator.tar.gz
cd Autonomous_Mobile_Manipulator

# Set proper ownership
sudo chown -R ubuntu:ubuntu Autonomous_Mobile_Manipulator/
```

#### 3. Production Configuration

```yaml
# docker-compose.prod.yml
version: '3.8'
services:
  ros2-sim:
    build:
      context: ./ros2_ws
      dockerfile: Dockerfile
    container_name: ros2_robot
    restart: unless-stopped
    network_mode: "host"
    privileged: true
    volumes:
      - ./ros2_ws/src:/root/ros2_ws/src:ro
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=${DISPLAY}
    command: >
      bash -c "
        source /opt/ros/iron/setup.bash &&
        ros2 launch my_robot_bringup robot.launch.py
      "

  n8n:
    image: docker.io/n8nio/n8n:latest
    container_name: n8n_robot
    restart: unless-stopped
    ports:
      - "5678:5678"
    environment:
      - GENERIC_TIMEZONE=Asia/Jakarta
      - N8N_HOST=0.0.0.0
      - N8N_PORT=5678
      - N8N_PROTOCOL=http
    volumes:
      - ./n8n_data:/home/node/.n8n
```

#### 4. Production Deployment

```bash
# On Raspberry Pi
cd Autonomous_Mobile_Manipulator

# Pull n8n image (smaller than building ROS 2)
docker compose -f docker-compose.prod.yml pull n8n

# Build ROS 2 image for ARM64
docker compose -f docker-compose.prod.yml build --no-cache ros2-sim

# Deploy in production mode
docker compose -f docker-compose.prod.yml up -d

# Verify deployment
docker compose -f docker-compose.prod.yml ps

# Test robot functionality
curl http://localhost:5678  # Should return n8n interface
```

### Production Optimization

#### Resource Management

```bash
# Monitor resource usage
docker stats

# Set resource limits (if needed)
# Edit docker-compose.prod.yml:
# deploy:
#   resources:
#     limits:
#       memory: 2G
#     reservations:
#       memory: 1G
```

#### Startup Optimization

```bash
# Enable auto-start on boot
sudo systemctl enable docker

# Create systemd service for robot
sudo tee /etc/systemd/system/robot.service > /dev/null <<EOF
[Unit]
Description=Autonomous Mobile Manipulator Robot
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/ubuntu/Autonomous_Mobile_Manipulator
ExecStart=/usr/bin/docker compose -f docker-compose.prod.yml up -d
ExecStop=/usr/bin/docker compose -f docker-compose.prod.yml down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl start robot
```

## Edge Deployment

### Industrial PC Deployment

#### Hardware Requirements

| Component | Minimum Specification | Recommended |
|-----------|---------------------|-------------|
| **CPU** | Intel i5 (4 cores) | Intel i7 (8 cores) |
| **RAM** | 16GB DDR4 | 32GB DDR4 |
| **Storage** | 256GB SSD | 512GB NVMe SSD |
| **Network** | Gigabit Ethernet | 10Gb Ethernet + WiFi 6 |
| **Power** | UPS-protected | Redundant power supplies |

#### Deployment Steps

```bash
# Install Ubuntu Server 22.04 LTS
# Configure static IP address
# Set up firewall rules

# Install Docker and dependencies
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Configure Docker for production
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2"
}
EOF

# Restart Docker daemon
sudo systemctl restart docker

# Deploy robot software
# Transfer project files
# Configure for production
# Start services
```

### Remote Monitoring Setup

#### Monitoring Dashboard

```bash
# Install monitoring tools
sudo apt install prometheus grafana -y

# Configure Prometheus for robot metrics
# Configure Grafana dashboards for visualization

# Set up alerting for critical events
# Emergency stop notifications
# Battery level warnings
# System health alerts
```

#### Remote Access

```bash
# Set up secure remote access
# Configure SSH keys for key-based authentication
# Set up VPN for secure remote connections
# Configure firewall rules for remote monitoring

# Enable remote API access
# Configure CORS for web interfaces
# Set up authentication for remote control
```

## Deployment Validation

### System Health Checks

#### Automated Health Monitoring

```bash
#!/bin/bash
# deployment-health-check.sh

# Check Docker services
if ! docker compose ps | grep -q "Up"; then
    echo "ERROR: Docker services not running"
    exit 1
fi

# Check ROS 2 nodes
if ! docker exec ros2_sim_container ros2 node list | grep -q "my_robot"; then
    echo "ERROR: ROS 2 nodes not running"
    exit 1
fi

# Check API endpoints
if ! curl -s http://localhost:5678 > /dev/null; then
    echo "ERROR: n8n API not accessible"
    exit 1
fi

# Check sensor data
if ! docker exec ros2_sim_container ros2 topic echo /scan --once > /dev/null 2>&1; then
    echo "ERROR: Sensor data not available"
    exit 1
fi

echo "SUCCESS: All systems operational"
```

#### Performance Benchmarks

```bash
# Measure startup time
time docker compose up -d

# Measure API response time
curl -w "@curl-format.txt" -s -o /dev/null http://localhost:5678

# Monitor resource usage
docker stats --no-stream ros2_sim_container n8n_container

# Test robot responsiveness
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
ros2 topic echo /odom --once
```

### Security Validation

#### Network Security

```bash
# Check open ports
netstat -tuln

# Verify firewall rules
sudo ufw status

# Test API security
curl -I http://localhost:5678/webhook/robot-control

# Check SSL/TLS configuration (if enabled)
curl -I https://secure-robot-api.example.com
```

#### Access Control

```bash
# Verify user permissions
id $USER

# Check Docker group membership
groups $USER | grep docker

# Validate file permissions
ls -la Autonomous_Mobile_Manipulator/
find Autonomous_Mobile_Manipulator/ -type f -perm 0777
```

## Scaling and High Availability

### Multi-Robot Deployment

#### Robot Fleet Management

```yaml
# docker-compose.fleet.yml
version: '3.8'
services:
  robot_1:
    build: ./ros2_ws
    container_name: robot_1
    environment:
      - ROBOT_ID=1
      - ROS_DOMAIN_ID=1

  robot_2:
    build: ./ros2_ws
    container_name: robot_2
    environment:
      - ROBOT_ID=2
      - ROS_DOMAIN_ID=2

  fleet_manager:
    build: ./fleet_management
    ports:
      - "8080:8080"
    depends_on:
      - robot_1
      - robot_2
```

#### Load Balancing

```bash
# Distribute tasks across robot fleet
# Implement task allocation algorithms
# Coordinate multi-robot operations
# Manage communication between robots

# Monitor fleet performance
# Health status of all robots
# Task completion tracking
# Resource utilization optimization
```

### Backup and Recovery

#### Automated Backup Strategy

```bash
#!/bin/bash
# backup-strategy.sh

# Daily configuration backup
cp -r Autonomous_Mobile_Manipulator/ ~/backups/daily/$(date +%Y%m%d)/

# Weekly full system backup
docker commit ros2_sim_container robot_backup:$(date +%Y%m%d)
docker save robot_backup:$(date +%Y%m%d) > ~/backups/weekly/robot_$(date +%Y%m%d).tar

# Monthly archive
tar -czf ~/backups/monthly/robot_$(date +%Y%m%d).tar.gz Autonomous_Mobile_Manipulator/

# Cleanup old backups (keep last 30 days)
find ~/backups/daily/ -type d -mtime +30 -exec rm -rf {} +
find ~/backups/weekly/ -name "*.tar" -mtime +60 -delete
```

#### Disaster Recovery

```bash
# Emergency recovery procedure:
# 1. Restore latest backup
# 2. Verify system integrity
# 3. Test critical functions
# 4. Resume normal operations

# Recovery steps:
cp -r ~/backups/daily/$(ls -t ~/backups/daily/ | head -1)/* Autonomous_Mobile_Manipulator/
docker compose up --build -d
# Verify system functionality
```

## Monitoring and Maintenance

### System Monitoring

#### Resource Monitoring

```bash
# Real-time resource monitoring
docker stats --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}"

# Historical performance tracking
# Set up Prometheus/Grafana for long-term monitoring
# Configure alerting for resource thresholds

# Network monitoring
iftop -i eth0  # Monitor network traffic
ping -c 4 8.8.8.8  # Test connectivity
```

#### Application Monitoring

```bash
# ROS 2 system monitoring
ros2 doctor --report

# n8n workflow monitoring
docker logs n8n_container --tail 100

# Robot state monitoring
ros2 topic echo /robot_status
ros2 topic echo /diagnostics
```

### Maintenance Procedures

#### Daily Maintenance

```bash
# System health checks
./deployment-health-check.sh

# Log rotation
docker exec ros2_sim_container bash -c "find /root/.ros/log -name '*.log' -size +100M -delete"

# Temporary file cleanup
docker system prune -f
```

#### Weekly Maintenance

```bash
# Security updates
sudo apt update && sudo apt upgrade -y

# Docker image updates
docker compose pull n8n
docker compose build --no-cache ros2-sim

# Full system backup
./backup-strategy.sh

# Performance benchmarking
./performance-benchmark.sh
```

#### Monthly Maintenance

```bash
# Hardware inspection
# Check motor temperatures
# Verify sensor calibration
# Test battery health

# Software audit
# Review and merge pending updates
# Update documentation
# Security assessment

# Long-term testing
# 24-hour stability test
# Battery discharge cycle
# Extreme condition testing
```

## Troubleshooting Deployment Issues

### Common Deployment Problems

#### Container Startup Issues

**Issue**: Containers fail to start
```bash
# Check system resources
df -h  # Disk space
free -h  # Memory

# Check Docker daemon
sudo systemctl status docker

# Clean Docker system
docker system prune -a

# Rebuild containers
docker compose build --no-cache
```

#### Network Configuration Issues

**Issue**: Network connectivity problems
```bash
# Check network interfaces
ip addr show

# Test network connectivity
ping 8.8.8.8

# Verify firewall rules
sudo ufw status

# Check port availability
netstat -tuln | grep -E '(5678|8765)'
```

#### Hardware Interface Issues

**Issue**: Sensors not detected
```bash
# Check USB devices
lsusb

# Verify device permissions
ls -la /dev/ttyUSB*

# Test sensor connections
ros2 topic echo /scan --once
```

## Performance Optimization

### Production Performance Tuning

#### Memory Optimization

```yaml
# docker-compose.prod.yml optimization
services:
  ros2-sim:
    deploy:
      resources:
        limits:
          memory: 2G
          cpus: '2.0'
        reservations:
          memory: 1G
          cpus: '1.0'
```

#### Network Optimization

```bash
# Optimize network settings
# Increase socket buffer sizes
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.wmem_max=134217728

# Configure for low-latency networking
sudo ethtool -s eth0 speed 1000 duplex full
```

#### Storage Optimization

```bash
# Use faster storage for containers
# Mount SSD for Docker data
sudo mkdir -p /ssd/docker
sudo mount /dev/sdb1 /ssd/docker

# Configure Docker to use SSD
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "data-root": "/ssd/docker",
  "storage-driver": "overlay2"
}
EOF

sudo systemctl restart docker
```

## Support and Updates

### Version Management

#### Update Strategy

```bash
# Check for updates
git fetch origin
git status

# Review changes
git log --oneline origin/main..HEAD

# Update to latest version
git pull origin main

# Test updates in staging environment
docker compose -f docker-compose.staging.yml up --build -d

# Deploy to production after testing
docker compose -f docker-compose.prod.yml up --build -d
```

### Rollback Procedures

#### Emergency Rollback

```bash
# Quick rollback to previous version
git reset --hard HEAD~1
docker compose down
docker compose up --build -d

# Verify rollback success
docker compose ps
curl -I http://localhost:5678
```

#### Gradual Rollback

```bash
# Rollback specific components
# Stop new services
docker compose stop ros2-sim

# Start previous version
docker compose start ros2-sim-backup

# Verify functionality
# Gradually migrate traffic back
```

## Documentation Maintenance

### Deployment Documentation Updates

When updating deployment procedures:
1. **Test Changes**: Validate all procedures in test environment
2. **Update Documentation**: Reflect any configuration changes
3. **Version Control**: Commit documentation updates with code changes
4. **Peer Review**: Have team members review deployment documentation

### Deployment Checklist

#### Pre-deployment Checklist
- [ ] System requirements verified
- [ ] All dependencies installed
- [ ] Network configuration tested
- [ ] Hardware interfaces validated
- [ ] Backup strategy implemented
- [ ] Emergency procedures documented

#### Post-deployment Checklist
- [ ] All services running correctly
- [ ] API endpoints responding
- [ ] Robot functionality verified
- [ ] Monitoring systems operational
- [ ] Security measures in place
- [ ] Documentation updated

## Resources and References

### Official Documentation
- [Docker Production Deployment](https://docs.docker.com/engine/reference/commandline/dockerd/)
- [ROS 2 Production Deployment](https://docs.ros.org/en/iron/How-To-Guides/Deployment.html)
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)

### Security Resources
- [Docker Security Best Practices](https://docs.docker.com/engine/security/)
- [ROS 2 Security Guide](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Security.html)
- [Ubuntu Server Security](https://ubuntu.com/server/docs/security)

### Monitoring Tools
- [Prometheus](https://prometheus.io/) - System monitoring
- [Grafana](https://grafana.com/) - Visualization dashboard
- [ELK Stack](https://www.elastic.co/elastic-stack/) - Log analysis

### Community Resources
- [ROS Production Users](https://discourse.ros.org/c/production-users/)
- [Docker Community](https://forums.docker.com/c/production/)
- [Raspberry Pi Forums](https://forums.raspberrypi.com/)

---

*This deployment guide ensures successful production deployment of the Autonomous Mobile Manipulator robot system across various target platforms, from development environments to production Raspberry Pi deployments.*
