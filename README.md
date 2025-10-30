# LKS Robot Project

A complete hexagonal-shaped autonomous mobile manipulator system built with ROS 2 Iron, featuring real-time automation workflows, advanced manipulation capabilities, and comprehensive robot control for industrial automation tasks.

## System Overview

This project provides a complete production-ready robotics platform featuring:
- **Hexagonal robot design** with 3-wheel omnidirectional movement system
- **Advanced manipulation** with 4-component picker system for precise object handling
- **Container load management** with 4-container system for material transport
- **Comprehensive sensor suite** including LIDAR, camera, IMU, and distance sensors
- **ROS 2 Iron** for robust robot control and navigation
- **n8n workflow automation** for high-level task orchestration
- **Docker containerization** for reliable deployment
- **Hardware control systems** for safety and operational management

## Table of Contents

- [Hardware Specifications](#hardware-specifications)
- [System Architecture](#system-architecture)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Workflow Automation](#workflow-automation)
- [API Documentation](#api-documentation)
- [Development Guide](#development-guide)
- [Deployment](#deployment)
- [Documentation](#documentation)

## Hardware Specifications

### Robot Configuration
- **Shape**: Hexagonal shaped autonomous mobile robot
- **Drive System**: 3-wheel omnidirectional movement (Back, Front Left, Front Right)

### Sensors
- **Distance sensors** (laser base): 3 units for obstacle detection
  - Front distance sensor
  - Back left distance sensor
  - Back right distance sensor
- **380° LIDAR sensor** (RPLIDAR A1): For mapping and navigation
- **Microsoft USB Camera**: For object recognition and computer vision
- **Line sensor**: For line-based navigation capabilities
- **IMU sensor** (MPU6050/BNO055): For orientation and motion sensing

### Actuators & Manipulation
- **Picker System** (4 components):
  - Gripper (servo): Open/close control for object grasping
  - Gripper tilt (servo): Angle adjustment for precise positioning
  - Gripper neck (servo continuous): Forward/backward movement
  - Gripper base (motor): Up/down height control for lifting/lowering
- **Container Load System** (4 containers):
  - 2 on left side (left front, left back)
  - 2 on right side (right front, right back)

### Control Systems
- **Hardware Controls**:
  - Emergency stop system
  - Start/stop control
  - Mode selection (train/run)
- **Advanced Control Mechanisms**:
  - Path planning with map data training
  - Obstacle avoidance algorithms
  - Line follower with PID control
  - Object pick and place automation
  - Object recognition and analysis

## System Architecture

The system consists of two main components:

### ROS 2 Container
- **Robot control**: Hardware interface and low-level control
- **Navigation**: Path planning and obstacle avoidance
- **Manipulation**: Arm control and manipulation tasks
- **API server**: REST API for external control
- **WebSocket server**: Real-time communication

### n8n Workflow Engine
- **Workflow automation**: High-level task orchestration
- **HTTP API integration**: Control robot via REST endpoints
- **Visual workflow designer**: Drag-and-drop automation
- **Scheduled tasks**: Automated patrol and monitoring

## Quick Start

### Prerequisites
- Docker and Docker Compose
- Git

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd lks_robot_project

# Build and start services
docker compose up --build -d

# Verify services are running
docker compose ps
```

### Access Points
- **n8n Web Interface**: http://localhost:5678
- **Robot Web Interface**: http://localhost:5000
- **ROS 2 Development**: `docker exec -it ros2_sim_container bash`

## Project Structure

```
lks_robot_project/
├── docker-compose.yml              # Service orchestration
├── README.md                       # This file
├── docs/                          # Comprehensive documentation
│   ├── api/                       # API documentation
│   ├── deployment/                # Deployment guides
│   ├── development/               # Development guides
│   ├── hardware/                  # Hardware specifications
│   ├── installation/              # Installation guides
│   └── troubleshooting/           # Troubleshooting guides
├── n8n_data/                     # n8n workflow data
│   └── workflows/                # Workflow definitions
│       ├── robot_basic_control.json
│       ├── robot_patrol.json
│       ├── robot_emergency_stop.json
│       ├── robot_path_planning.json
│       ├── robot_line_follower.json
│       ├── robot_object_recognition.json
│       ├── individual_control_omni_wheels.json
│       ├── individual_control_picker_system.json
│       ├── individual_control_container_system.json
│       ├── individual_control_hardware_controls.json
│       └── individual_control_*.json
└── ros2_ws/                      # ROS 2 workspace
    └── src/                      # ROS 2 packages
        ├── my_robot_automation/  # Automation services
        ├── my_robot_bringup/     # System bringup
        ├── my_robot_description/ # Robot model
        ├── my_robot_manipulation/# Manipulation control
        └── my_robot_navigation/  # Navigation control
```

## Workflow Automation

The system includes comprehensive n8n workflows matching the actual robot hardware configuration:

### Combination Workflows
- **Robot Basic Control**: Manual movement commands for omni wheels
- **Autonomous Patrol**: Automated patrol with obstacle avoidance
- **Emergency Stop Monitor**: Safety monitoring and emergency response
- **Obstacle Avoidance**: Reactive obstacle avoidance behavior
- **Pick and Place**: Complete manipulation tasks using picker system
- **Path Planning**: Advanced path planning with map data training
- **Line Follower**: Line-based navigation with PID control
- **Object Recognition**: Computer vision for object detection

### Individual Control Workflows
- **Control Omni Wheels**: Direct control of 3 omni wheels (Back, Front Left, Front Right)
- **Control Picker System**: Complete control of 4-component picker system
  - Gripper control (servo)
  - Gripper tilt adjustment (servo)
  - Gripper neck positioning (servo continuous)
  - Gripper base height (motor)
- **Control Container System**: Management of 4-container load system
- **Control Hardware Controls**: Emergency, start/stop, and mode controls
- **Control Servo**: Individual servo positioning (legacy)
- **Get Robot Status**: Comprehensive status monitoring and reporting

All workflows use HTTP API calls to the robot's REST interface at `http://10.0.3.1:5000`.

## API Documentation

The robot provides a REST API for external control:

### Movement Control
```bash
# Move robot forward/backward
curl -X POST http://localhost:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn robot left/right
curl -X POST http://localhost:5000/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Stop robot
curl -X POST http://localhost:5000/api/robot/stop
```

### Picker System Control
```bash
# Control gripper (open/close)
curl -X POST http://localhost:5000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command": "open"}'

# Control gripper tilt
curl -X POST http://localhost:5000/api/robot/picker/gripper_tilt \
  -H "Content-Type: application/json" \
  -d '{"angle": 15}'

# Control gripper neck position
curl -X POST http://localhost:5000/api/robot/picker/gripper_neck \
  -H "Content-Type: application/json" \
  -d '{"position": 0.5}'

# Control gripper base height
curl -X POST http://localhost:5000/api/robot/picker/gripper_base \
  -H "Content-Type: application/json" \
  -d '{"height": 0.3}'
```

### Container System Control
```bash
# Control left front container
curl -X POST http://localhost:5000/api/robot/containers/left_front \
  -H "Content-Type: application/json" \
  -d '{"action": "load"}'

# Control left back container
curl -X POST http://localhost:5000/api/robot/containers/left_back \
  -H "Content-Type: application/json" \
  -d '{"action": "unload"}'

# Control right front container
curl -X POST http://localhost:5000/api/robot/containers/right_front \
  -H "Content-Type: application/json" \
  -d '{"action": "load"}'

# Control right back container
curl -X POST http://localhost:5000/api/robot/containers/right_back \
  -H "Content-Type: application/json" \
  -d '{"action": "unload"}'
```

### Hardware Control
```bash
# Emergency stop
curl -X POST http://localhost:5000/api/robot/hardware/emergency \
  -H "Content-Type: application/json" \
  -d '{"action": "stop"}'

# Start/stop robot
curl -X POST http://localhost:5000/api/robot/hardware/start_stop \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'

# Set robot mode
curl -X POST http://localhost:5000/api/robot/hardware/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "run"}'
```

### Legacy Actuator Control (for compatibility)
```bash
# Control lifter (legacy)
curl -X POST http://localhost:5000/api/robot/lifter \
  -H "Content-Type: application/json" \
  -d '{"action": "up", "speed": 0.5}'

# Control servos (legacy)
curl -X POST http://localhost:5000/api/robot/servo \
  -H "Content-Type: application/json" \
  -d '{"servo": 1, "angle": 90}'
```

### Status Monitoring
```bash
# Get robot status
curl http://localhost:5000/api/robot/status
```

## Development Guide

### ROS 2 Development
```bash
# Access ROS 2 container
docker exec -it ros2_sim_container bash

# Build workspace
cd /root/ros2_ws
colcon build

# Source workspace
source install/setup.bash

# Run robot bringup
ros2 launch my_robot_bringup robot.launch.py
```

#### Development Environment Notes
- **Host System Development**: Launch files include conditional imports to handle ROS2 package availability. If ROS2 is not installed on your host system, imports will fail gracefully with clear error messages directing you to use the Docker container environment.
- **Container Development**: All ROS2 packages are available within the Docker containers. Edit files on your host (they're volume-mounted) and execute within the container.
- **IDE Integration**: Use VS Code with Docker container integration for seamless development experience.

### Workflow Development
1. Access n8n at http://localhost:5678
2. Import workflows from `n8n_data/workflows/`
3. Modify workflows using the visual editor
4. Test workflows with manual triggers
5. Export updated workflows back to files

### API Development
The robot API is implemented in `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py` and can be extended for additional endpoints.

## Deployment

### Production Deployment
```bash
# Stop development containers
docker compose down

# Build production images
docker compose build --no-cache

# Start production services
docker compose up -d

# Verify deployment
docker compose ps
curl http://localhost:5678  # n8n interface
curl http://localhost:5000  # robot API
```

### Raspberry Pi Deployment
1. Install Ubuntu Server 22.04 on Raspberry Pi 5
2. Install Docker and Docker Compose
3. Transfer project files to Raspberry Pi
4. Build ARM64 compatible images
5. Start services with production configuration

## Documentation

Comprehensive documentation is available in the `docs/` directory:

- **[API Documentation](docs/api/)** - Complete API reference and examples
- **[Hardware Guide](docs/hardware/)** - Hardware specifications and connections
- **[Installation Guide](docs/installation/)** - Detailed installation instructions
- **[Development Guide](docs/development/)** - Development workflow and best practices
- **[Deployment Guide](docs/deployment/)** - Production deployment procedures
- **[Troubleshooting](docs/troubleshooting/)** - Common issues and solutions

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

