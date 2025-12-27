# System Architecture - Control & Monitoring

**Last Updated:** 2025-11-20
**Status:** Production Ready with Pi-Mega Integration

## Overview

The system provides a unified web-based control interface for complete robot control and monitoring:

1. **Web UI** - Primary control and monitoring interface (Required)
2. **REST API** - Programmatic access for automation and integration

## Hardware Architecture

### Pi-Mega Control System

The robot uses a distributed control architecture with clear role separation:

#### Raspberry Pi 5 (High-Level Control)
- **Operating System**: Ubuntu Server 22.04 LTS
- **Framework**: ROS2 Iron with Docker containerization
- **Responsibilities**:
  - High-level task coordination and planning
  - Web interface and API server hosting
  - Sensor data processing and fusion
  - Path planning and navigation logic

#### Arduino Mega 2560 (Low-Level Control)
- **Motor Shield**: YFROBOT v2 with I2C communication
- **Motors**: 4x PG28 DC motors with built-in encoders
- **Responsibilities**:
  - Real-time PID motor control
  - Hexagonal 3-wheel omni kinematics
  - Sensor data acquisition
  - Emergency stop and safety systems

#### Communication Protocol
- **Interface**: Serial (UART) at 115200 baud
- **Direction**: Bidirectional command/data exchange
- **Commands**: Single-character Arduino Mega commands
- **Data Flow**: Real-time sensor data from Mega to Pi

## Architecture Diagram

```
┌─────────────┐
│   Web UI    │
│ (Port 8000) │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  REST API   │
│ (Port 5000) │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    ROS2     │
│  Services   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Serial UART │
│ 115200 baud │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│Arduino Mega │
│             │
│ - PID Ctrl  │
│ - Motors    │
│ - Sensors   │
└─────────────┘
```

## Web UI - Primary Interface

### Core Features
- **Movement Control**: Directional movement with speed control
- **Real-time Monitoring**: Live sensor data and system status
- **Path Planning**: Grid-based navigation and waypoint management
- **System Diagnostics**: Connection status and error reporting
- **Emergency Controls**: Stop buttons and safety overrides

### Technology Stack
- **Frontend**: HTML5, CSS3, JavaScript
- **Backend**: Flask (Python)
- **Real-time**: WebSocket for live updates
- **API**: RESTful HTTP endpoints

## Web UI Usage Guide

### Best For:
- **Manual Control:** Direct robot operation and testing
- **Real-time Monitoring:** Live sensor data and system status
- **Quick Navigation:** Simple movement and path planning
- **Feature Testing:** Validating new functionality
- **Emergency Response:** Immediate intervention and safety controls
- **Operator Training:** Learning and familiarization
- **System Diagnostics:** Troubleshooting and calibration

### Use Cases:
- **Development:** Testing and validation of robot capabilities
- **Manual Operation:** Direct control for specific tasks
- **Monitoring:** Live oversight and status checking
- **Training:** Operator skill development
- **Maintenance:** System diagnostics and calibration

---

## System Summary

The system provides a complete robotics platform with:

- **Web UI** for comprehensive control and monitoring
- **REST API** for programmatic access and integration
- **ROS2** for advanced robotics capabilities
- **Arduino Mega** for real-time motor control and sensing
- **Serial communication** for reliable hardware integration
- **Docker deployment** for consistent environments

### Access Points
- **Web UI:** http://localhost:8000
- **REST API:** http://localhost:5000
- **Arduino Serial:** /dev/ttyACM0 (115200 baud)

### Key Features
- Real-time motor control with PID
- Comprehensive sensor integration
- Path planning and navigation
- Emergency stop and safety systems
- Manual and programmatic control