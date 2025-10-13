# LKS Robot Project

A complete autonomous mobile manipulator system built with ROS 2 Iron, featuring real-time automation workflows and comprehensive robot control capabilities.

## System Overview

This project provides a containerized robotics development environment with:
- **ROS 2 Iron** for robot control and automation
- **n8n workflow automation** for high-level task management
- **Docker containerization** for reproducible development
- **Real hardware integration** with comprehensive sensor and actuator support

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

### Sensors
- **Ultrasonic sensors**: 4 units for obstacle detection
- **Infrared sensors**: 3 IR Sharp sensors for proximity detection
- **Line sensor**: For line following capabilities
- **LIDAR**: For mapping and navigation
- **USB Camera**: For computer vision tasks

### Actuators
- **Omni wheels**: 3 DC motors with encoders for omnidirectional movement
- **Lifter**: 1 DC motor with encoder for vertical movement
- **Servo motors**: 5 units for precise positioning and manipulation

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

The system includes pre-configured n8n workflows for robot control:

### Combination Workflows
- **Robot Basic Control**: Manual movement commands
- **Autonomous Patrol**: Automated square patrol pattern
- **Emergency Stop Monitor**: Safety monitoring and emergency stop
- **Obstacle Avoidance**: Reactive obstacle avoidance behavior
- **Pick and Place**: Complete manipulation tasks

### Individual Control Workflows
- **Control Omni Wheels**: Direct wheel control
- **Control Lifter**: Vertical movement control
- **Control Servo**: Individual servo positioning
- **Get Robot Status**: Status monitoring and reporting

All workflows are configured to use HTTP API calls to the robot's REST interface.

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

### Actuator Control
```bash
# Control lifter
curl -X POST http://localhost:5000/api/robot/lifter \
  -H "Content-Type: application/json" \
  -d '{"action": "up", "speed": 0.5}'

# Control servos
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

