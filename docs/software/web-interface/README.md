# Web Interface Documentation

This directory contains comprehensive documentation for the Web-based Control Interface.

## Overview

The Web Interface (http://localhost:8000) is the **primary control and monitoring interface** for the Autonomous Mobile Manipulator. It provides complete robot control without requiring N8N workflows.

## Documentation Files

### Core Features

- **[WEB_INTERFACE_CONTROLS.md](./WEB_INTERFACE_CONTROLS.md)**
  - Complete list of all available controls
  - Movement, gripper, container operations
  - System management functions

- **[WEB_INTERFACE_COMPLETE_UPDATE.md](./WEB_INTERFACE_COMPLETE_UPDATE.md)**
  - Comprehensive feature overview
  - All capabilities documented
  - 100% feature coverage details

### Feature-Specific Documentation

- **[PATH_PLANNING.md](./WEB_INTERFACE_PATH_PLANNING.md)**
  - Waypoint management system
  - Visual path planning with canvas
  - Pattern templates (square, triangle, hexagon)
  - Path save/load functionality
  - Go-to-point and return home features

- **[IMU_CALIBRATION.md](./IMU_CALIBRATION_FEATURE.md)**
  - IMU sensor calibration interface
  - Zero reference setting
  - Calibration procedures and best practices
  - API endpoint documentation

- **[ACTIVITY_STREAM.md](./ACTIVITY_STREAM_FEATURE.md)**
  - Real-time activity logging on all tabs
  - Command feedback and verification
  - Backend implementation visibility

### Update History

- **[NETWORK_ACCESS_FIX.md](./NETWORK_ACCESS_FIX.md)**
  - Fixed remote access from other devices
  - Dynamic API URL using hostname
  - Enables control from laptop/tablet/phone

- **[WEB_INTERFACE_UPDATE.md](./WEB_INTERFACE_UPDATE.md)**
  - Initial web interface implementation
  - Hardware tab and GPIO pinout display
  - API endpoint corrections

## Quick Links

### Getting Started

1. Start the system: `docker compose up -d`
2. Access Web Interface:
   - **Local (on robot)**: http://localhost:8000
   - **Remote (from laptop)**: http://<robot-ip>:8000
   - Find robot IP: `hostname -I` (run on robot)
3. Navigate through tabs:
   - **Movement** - Robot movement controls
   - **Gripper** - Manipulation operations
   - **Containers** - Load management
   - **Status** - System monitoring
   - **Path Planning** - Autonomous navigation
   - **Sensors** - Real-time sensor data
   - **Hardware** - GPIO and pinout reference

### Key Features

✅ **Direct Robot Control**
- All movement operations
- Gripper and container management
- Emergency stop

✅ **Path Planning & Navigation**
- Visual waypoint manager
- Pre-built patterns
- Save/load paths
- Real-time path visualization

✅ **Real-time Monitoring**
- All sensor data (6 laser, 2 ultrasonic, TF-Luna LIDAR, IMU, line sensors)
- System status and logs
- Last 3 commands history
- Container load status

✅ **System Management**
- IMU calibration
- Mode control
- Hardware reference
- Activity stream on all tabs

## Architecture

The Web Interface is built with:
- **Backend**: Flask (Python) - ROS2 integration
- **Frontend**: HTML5, CSS3, JavaScript
- **Communication**: REST API (http://127.0.0.1:5000)
- **Visualization**: HTML5 Canvas for path planning

## Related Documentation

- [System Architecture](../../SYSTEM_ARCHITECTURE.md) - Overall system design
- [ROS2 Performance](../ros2/ROS2_PERFORMANCE_OPTIMIZATION.md) - Backend optimization
- [API Documentation](../../api/API_VERIFICATION_REPORT.md) - REST API details

## Development

**File Location**: `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py`

**Port**: 8000 (exposed from Docker container)

**Technology Stack**:
- Flask web framework
- ROS2 integration via rclpy
- Real-time updates with JavaScript
- Responsive CSS design

## Support

For issues or questions:
1. Check the [Troubleshooting](../../troubleshooting/) section
2. Review Activity Stream in the Web UI for errors
3. Check logs: `docker logs ros2_sim_container`

---

**The Web Interface provides complete, independent robot control and monitoring capabilities.**

