# Autonomous Mobile Manipulator

A complete hexagonal-shaped autonomous mobile manipulator system built with ROS 2 Iron, featuring real-time automation workflows, advanced manipulation capabilities, and comprehensive robot control for industrial automation tasks.

## System Overview

The Autonomous Mobile Manipulator is a comprehensive and containerized ROS2-based platform for a versatile mobile manipulator, designed for simulation in Gazebo and deployment on real hardware, with a focus on modularity, flexibility, and ease of use.

### Project Description

This document provides a high-level overview of the Autonomous Mobile Manipulator, a comprehensive and containerized ROS2-based platform for a versatile mobile manipulator. The project is designed for simulation in Gazebo and deployment on real hardware, with a focus on modularity, flexibility, and ease of use.

It integrates advanced robotics capabilities, including navigation, manipulation, and automation, all orchestrated through a variety of modern interfaces.

### Key Features

- **Mobile Manipulation:** Combines a mobile base with a servo-based manipulator system for pick-and-place tasks in a dynamic environment.
- **Autonomous Navigation:** Utilizes the ROS2 Navigation Stack (Nav2) for path planning, obstacle avoidance, and autonomous patrol missions.
- **Advanced Manipulation:** Servo-based manipulation with 4-component picker system (gripper, tilt, neck, base) for precise object handling.
- **Simulation & Reality:** Supports both realistic Gazebo simulation and real-world hardware deployment with a unified launch system.
- **Flexible Control Interfaces:** Offers multiple points of control and integration:
    - **n8n Workflow Automation:** A powerful, low-code platform for designing complex automation sequences (e.g., "patrol until an object is found, then pick and place it").
    - **REST API:** A simple HTTP interface for scripting and high-level control.
    - **WebSockets:** For real-time data streaming and interactive control.
    - **MQTT Bridge:** For integration with IoT ecosystems and devices.
- **Containerized Deployment:** Uses Docker and Docker Compose for consistent, reproducible, and isolated development and production environments.

### Technology Stack

- **Robotics Framework:** ROS 2 (Iron)
- **Simulation:** Gazebo
- **Containerization:** Docker, Docker Compose
- **Navigation:** ROS 2 Navigation Stack (Nav2)
- **Manipulation:** Servo-based control system
- **Workflow Automation:** n8n.io
- **Primary Language:** Python 3
- **Robot Modeling:** URDF, xacro

### System Capabilities

This project provides a complete production-ready robotics platform featuring:

- **Hexagonal robot design** with 3-wheel omnidirectional movement system
- **Advanced manipulation** with 4-component picker system for precise object handling
- **Container load management** with 4-container system for material transport
- **Comprehensive sensor suite** including LIDAR, camera, IMU, and distance sensors
- **ROS 2 Iron** for robust robot control and navigation
- **n8n workflow automation** for high-level task orchestration
- **Docker containerization** for reliable deployment
- **Hardware control systems** for safety and operational management

### Software Architecture

The architecture is centered around a ROS 2 workspace (`ros2_ws`) containing several modular packages.

- **`my_robot_description`:** Defines the robot's physical structure (URDF) and configuration for controllers.
- **`my_robot_bringup`:** Contains launch files to start the robot in various modes (simulation, real hardware, visualization).
- **`my_robot_navigation`:** Manages the Nav2 stack, including configuration for localization, planning, and obstacle avoidance.
- **`my_robot_manipulation`:** Servo-based manipulation control (currently minimal implementation).
- **`my_robot_automation`:** The core automation package. It acts as a bridge between ROS 2 and external systems, hosting the REST, WebSocket, and MQTT servers, and implementing servo-based manipulation, navigation, and automation services.

External control systems interact with the `my_robot_automation` package, which translates high-level commands into specific ROS 2 messages, service calls, and action goals.

## Table of Contents

- [System Overview](#system-overview)
  - [Project Description](#project-description)
  - [Key Features](#key-features)
  - [Technology Stack](#technology-stack)
  - [System Capabilities](#system-capabilities)
  - [Software Architecture](#software-architecture)
- [Hardware Specifications](#hardware-specifications)
- [System Architecture](#system-architecture)
- [API Capabilities Overview](#api-capabilities-overview)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Workflow Automation](#workflow-automation)
- [API Documentation](#api-documentation)
- [Development Guide](#development-guide)
- [Deployment](#deployment)
- [Documentation](#documentation)
- [License](#license)

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
- **Manipulation**: Advanced 4-component picker system control
- **Container management**: 4-container load/unload operations
- **REST API server**: Comprehensive HTTP API (5000) with 20+ endpoints
- **WebSocket server**: Real-time communication (8765)
- **ROS 2 services**: Modular automation services (patrol, pick-place, obstacle avoidance)

### Professional Web Interface

- **User-friendly control center**: Modern web-based robot control interface
- **Real-time status monitoring**: Live system status and diagnostics
- **Tabbed control panels**: Organized access to all robot functions
- **Professional UI/UX**: Responsive design with intuitive controls
- **API integration**: Frontend for all ROS2 REST API endpoints

### n8n Workflow Engine

- **Workflow automation**: High-level task orchestration
- **HTTP API integration**: Direct control via REST endpoints
- **Visual workflow designer**: Drag-and-drop automation
- **Webhook integration**: Real-time workflow triggers
- **Scheduled automation**: Patrol, monitoring, and maintenance tasks
- **Pre-configured workflows**: 33+ robot control workflows

## API Capabilities Overview

The Autonomous Mobile Manipulator provides a comprehensive API ecosystem supporting full robotic automation:

### **Robot Control APIs (6 endpoints)**

- **Basic Movement**: Forward/backward, turning, strafing, emergency stop
- **Mode Management**: AUTONOMOUS, MANUAL, EMERGENCY, MAINTENANCE modes
- **Status Monitoring**: Real-time robot state, safety systems, diagnostics

### **Advanced Manipulation APIs (4 endpoints)**

- **Picker System**: 4-component gripper control (open/close, tilt, neck, base height)
- **Precision Control**: Angle and position control with validation
- **Force Management**: Configurable gripper force for different objects

### **Container Management APIs (4 endpoints)**

- **Load/Unload Operations**: Individual container control
- **Multi-container Support**: Left/right, front/back positioning
- **State Management**: Load status tracking and validation

### **Automation APIs (3 endpoints)**

- **Pick & Place**: Complete manipulation sequences with object handling
- **Autonomous Patrol**: Multi-waypoint navigation with cycle control
- **Obstacle Avoidance**: Intelligent navigation with dynamic path planning

### **Safety & Emergency APIs (1 endpoint)**

- **Emergency Stop**: Immediate halt with reason logging
- **Safety Monitoring**: Collision detection and system health
- **Force Stop**: Override capabilities for critical situations

### **Sensor Data APIs (1 endpoint)**

- **Comprehensive Sensing**: Ultrasonic, IR, line sensor, IMU, battery monitoring
- **Real-time Data**: Live sensor readings with health status
- **Multi-sensor Fusion**: Integrated sensor data processing

### **Task Management APIs (2 endpoints)**

- **Task Monitoring**: Active task tracking with progress and status
- **Task Control**: Cancel running tasks with reason logging
- **Lifecycle Management**: Complete task state management

### **Navigation APIs (1 endpoint)**

- **Localization Status**: Real-time position and confidence tracking
- **Path Planning**: Current path and navigation state monitoring
- **Map Integration**: Map availability and navigation performance metrics

### **Integration APIs (3 webhooks)**

- **n8n Workflow Integration**: Direct workflow triggers
- **Webhook Support**: HTTP callbacks for external systems
- **Event-driven Automation**: Real-time response capabilities

**Total: 25 API endpoints** providing complete robotic control, sensing, and automation capabilities.

## Quick Start

### Prerequisites

- Docker and Docker Compose
- Git

### Installation Options

#### Option 1: Development Setup (Ubuntu/Debian)

```bash
# Clone the repository
git clone <repository-url>
cd lks_robot_project

# Development mode with sensor simulation (recommended for testing)
./run.sh --dev

# Or use development Docker Compose
docker compose -f docker-compose.dev.yml up --build -d

# Traditional setup (requires Gazebo)
./run.sh

# Verify services are running
docker compose ps
```

#### Option 2: Raspberry Pi Production Setup

For Raspberry Pi deployment, follow the comprehensive setup guide:

```bash
# On Raspberry Pi 5 with Ubuntu Server 22.04
# Run the automated setup script
./setup_raspberry_pi.sh

# Or follow the detailed guide
# See: docs/deployment/raspberry_pi_setup.md
```

### Development vs Production Mode

#### Development Mode (Sensor Simulation)

- **Purpose**: Testing n8n workflows and ROS2 APIs without Gazebo
- **Sensors**: Simulated ultrasonic, line sensor, IMU, and LIDAR data
- **Performance**: Lightweight, fast startup, ideal for development
- **Access**: Same interfaces as production, but with dummy sensor data

#### Production Mode (Full Simulation)

- **Purpose**: Complete robot simulation with Gazebo physics
- **Sensors**: Real sensor simulation through Gazebo plugins
- **Performance**: Full physics simulation, more resource intensive
- **Access**: Includes Gazebo GUI and complete sensor integration

### Access Points

- **Professional Web Interface**: http://localhost:8000 *(Recommended for users)*
- **n8n Workflow Automation**: http://localhost:5678 *(Advanced workflow design)*
- **Robot REST API**: http://localhost:5000 *(Direct API access)*
- **ROS 2 Development Environment**: `docker exec -it ros2_dev_container bash`

### Startup Scripts

The project includes convenient startup scripts for easy deployment:

#### Quick Start (`./run.sh`)

One-command startup for the entire system:

```bash
./run.sh  # Builds, starts, and shows access info
```

#### Advanced Startup (`./start_robot.sh`)

Full-featured startup script with options:

```bash
./start_robot.sh --help  # Show all options

# Common usage
./start_robot.sh              # Start services
./start_robot.sh -b           # Force rebuild and start
./start_robot.sh -s           # Show service status
./start_robot.sh -l           # Show service logs
./start_robot.sh -x           # Stop all services
./start_robot.sh -r           # Restart all services
```

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
│       ├── robot_simple_test.json
│       ├── robot_pick_place.json
│       ├── robot_mobile_pick_place.json
│       ├── robot_emergency_stop.json
│       ├── robot_inspection_patrol.json
│       ├── robot_material_transport.json
│       ├── robot_system_calibration.json
│       ├── individual_sensor_ultrasonic_monitoring.json
│       ├── individual_control_picker_system.json
│       ├── individual_control_omni_wheels.json
│       ├── individual_control_container_system.json
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

- **Robot Simple Test**: Comprehensive system test of all robot capabilities
- **Robot Emergency Stop**: Emergency safety system and monitoring
- **Robot Pick and Place**: Basic servo-based pick and place automation
- **Mobile Pick and Place**: Complete mobile manipulation with navigation
- **Inspection Patrol**: Autonomous security and inspection patrols
- **Material Transport**: Container-to-container material transport
- **Search and Retrieve**: Sensor-based object detection and retrieval
- **Emergency Response**: Comprehensive safety and emergency response
- **System Calibration**: Complete system testing and calibration
- **Production Line**: Manufacturing automation with multiple stations
- **Object Recognition**: Computer vision for object detection
- **Obstacle Avoidance**: Reactive obstacle avoidance behavior
- **Path Planning**: Advanced path planning with sensor integration

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

All workflows use HTTP API calls to the robot's comprehensive REST interface at `http://10.0.3.1:5000`, providing full integration between n8n automation and ROS2 robotic control.

## API Documentation

The robot provides a comprehensive REST API on port 5000 for external control, featuring both direct control and advanced automation capabilities.

### System Health & Status

```bash
# Health check
curl http://localhost:5000/health

# Get comprehensive robot status
curl http://localhost:5000/api/robot/status
```

### Robot Mode Management

```bash
# Set robot operating mode
curl -X POST http://localhost:5000/api/robot/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "AUTONOMOUS", "reason": "Starting automated operation"}'

# Available modes: AUTONOMOUS, MANUAL, EMERGENCY, MAINTENANCE
```

### Basic Movement Control

```bash
# Move robot in cardinal directions
curl -X POST http://localhost:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn robot left/right
curl -X POST http://localhost:5000/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Strafe movement (omni-directional)
curl -X POST http://localhost:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "strafe_left", "speed": 0.4}'

# Stop all movement
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
# Legacy servo control (for compatibility)
curl -X POST http://localhost:5000/api/robot/servo \
  -H "Content-Type: application/json" \
  -d '{"servo": 1, "angle": 90}'

# Note: Lifter control is integrated into picker system
# Use /api/robot/picker/gripper_base for height control
```

### Advanced Automation Operations

```bash
# Execute pick and place operation
curl -X POST http://localhost:5000/api/robot/pick-place \
  -H "Content-Type: application/json" \
  -d '{
    "pickup_location": {
      "position": {"x": 1.0, "y": 0.0, "z": 0.0}
    },
    "place_location": {
      "position": {"x": -1.0, "y": 0.0, "z": 0.0}
    },
    "object_type": "box",
    "gripper_force": 15.0
  }'

# Execute autonomous patrol
curl -X POST http://localhost:5000/api/robot/patrol \
  -H "Content-Type: application/json" \
  -d '{
    "waypoints": [
      {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
      {"position": {"x": 2.0, "y": 0.0, "z": 0.0}},
      {"position": {"x": 2.0, "y": 2.0, "z": 0.0}}
    ],
    "patrol_speed": 0.5,
    "patrol_cycles": 2
  }'

# Execute obstacle avoidance navigation
curl -X POST http://localhost:5000/api/robot/obstacle-avoidance \
  -H "Content-Type: application/json" \
  -d '{
    "target_location": {
      "position": {"x": 3.0, "y": 1.0, "z": 0.0}
    },
    "avoidance_distance": 0.5,
    "max_speed": 0.6
  }'
```

### Emergency & Safety Systems

```bash
# Emergency stop (immediate halt)
curl -X POST http://localhost:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": true, "reason": "Safety emergency"}'

# Activate emergency stop
curl -X POST http://localhost:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": true, "reason": "Manual emergency stop", "force": true}'

# Deactivate emergency stop
curl -X POST http://localhost:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": false, "reason": "Emergency resolved"}'
```

### n8n Webhook Integration

```bash
# Robot control webhook (for n8n workflows)
curl -X POST http://localhost:5000/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "forward"}'

# Emergency stop webhook
curl -X POST http://localhost:5000/webhook/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": true, "reason": "Workflow emergency"}'

# Pick and place webhook
curl -X POST http://localhost:5000/webhook/robot/pick_place \
  -H "Content-Type: application/json" \
  -d '{
    "pickup_location": {"position": {"x": 1.0, "y": 0.0, "z": 0.0}},
    "place_location": {"position": {"x": -1.0, "y": 0.0, "z": 0.0}}
  }'
```

### Status Monitoring

```bash
# Get comprehensive robot status (includes safety systems and sensor data)
curl http://localhost:5000/api/robot/status

# Get detailed sensor data
curl http://localhost:5000/api/robot/sensors

# Get active tasks
curl http://localhost:5000/api/robot/tasks

# Cancel a running task
curl -X POST http://localhost:5000/api/robot/tasks/task_123/cancel \
  -H "Content-Type: application/json" \
  -d '{"reason": "User requested cancellation"}'

# Get navigation status
curl http://localhost:5000/api/robot/navigation/status
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

The robot API is implemented in `ros2_ws/src/my_robot_automation/scripts/rest_api_server.py` and provides comprehensive REST endpoints for robot control and automation.

#### Complete API Endpoint Reference

| Endpoint                              | Method | Description                      | Parameters                                       |
| ------------------------------------- | ------ | -------------------------------- | ------------------------------------------------ |
| `/health`                           | GET    | System health check              | None                                             |
| `/api/robot/status`                 | GET    | Get comprehensive robot status   | None                                             |
| `/api/robot/sensors`                | GET    | Get detailed sensor data         | None                                             |
| `/api/robot/tasks`                  | GET    | Get active tasks                 | `task_id` (optional)                           |
| `/api/robot/tasks/{task_id}/cancel` | POST   | Cancel running task              | `reason`                                       |
| `/api/robot/navigation/status`      | GET    | Get navigation status            | `include_map`, `include_path`                |
| `/api/robot/mode`                   | POST   | Set robot operating mode         | `mode`, `reason`, `force`                  |
| `/api/robot/move`                   | POST   | Basic movement control           | `direction`, `speed`                         |
| `/api/robot/turn`                   | POST   | Rotation control                 | `direction`, `speed`                         |
| `/api/robot/stop`                   | POST   | Stop all movement                | None                                             |
| `/api/robot/picker/gripper`         | POST   | Control gripper open/close       | `command` ("open"/"close")                     |
| `/api/robot/picker/gripper_tilt`    | POST   | Control gripper tilt angle       | `angle` (0-180°)                              |
| `/api/robot/picker/gripper_neck`    | POST   | Control gripper neck position    | `position` (-1.0 to 1.0)                       |
| `/api/robot/picker/gripper_base`    | POST   | Control gripper base height      | `height` (0.0 to 1.0)                          |
| `/api/robot/containers/{id}`        | POST   | Control container operations     | `action` ("load"/"unload")                     |
| `/api/robot/pick-place`             | POST   | Execute pick and place operation | `pickup_location`, `place_location`, ...     |
| `/api/robot/patrol`                 | POST   | Execute autonomous patrol        | `waypoints`, `patrol_speed`, ...             |
| `/api/robot/obstacle-avoidance`     | POST   | Navigate with obstacle avoidance | `target_location`, `avoidance_distance`, ... |
| `/api/robot/emergency-stop`         | POST   | Emergency stop control           | `activate`, `reason`, `force`              |
| `/webhook/robot-control`            | POST   | n8n workflow integration         | `command`                                      |
| `/webhook/emergency-stop`           | POST   | n8n emergency stop webhook       | `activate`, `reason`                         |
| `/webhook/robot/pick_place`         | POST   | n8n pick and place webhook       | `pickup_location`, `place_location`          |

*Container IDs: `left_front`, `left_back`, `right_front`, `right_back`*

#### Response Format

All API endpoints return JSON responses with consistent structure:

```json
{
  "success": true|false,
  "message": "Description of the result",
  "data": { /* Response data */ },
  "...": "Additional response fields"
}
```

## Deployment

### Production Deployment (Ubuntu/Debian)

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

# Import and manage n8n workflows
./workflow_management_tools.sh import-enhanced  # Import enhanced control workflows
./workflow_management_tools.sh status           # Check N8N status
./workflow_management_tools.sh list             # List available workflows

# Complete workflow management guide: docs/workflows/WORKFLOW_MANAGEMENT_README.md
```

### Raspberry Pi Deployment

For comprehensive Raspberry Pi setup, see the dedicated guide:

**[Complete Raspberry Pi Setup Guide](docs/deployment/raspberry_pi_setup.md)**

#### Quick Raspberry Pi Setup

```bash
# On Raspberry Pi 5 with Ubuntu Server 22.04
# Run the automated setup script
./setup_raspberry_pi.sh

# Follow the prompts and reboot when instructed
sudo reboot

# After reboot, deploy the robot
cd Autonomous_Mobile_Manipulator
docker compose -f docker-compose.prod.yml up -d

# Access the robot
# n8n: http://raspberrypi:5678
# API: http://raspberrypi:5000
```

#### Manual Raspberry Pi Setup Steps

1. **OS Installation**: Ubuntu Server 22.04 LTS (64-bit) on Raspberry Pi 5
2. **Hardware Configuration**: GPIO, I2C, SPI, UART setup
3. **System Optimization**: Performance tuning for ARM64
4. **Docker Setup**: ARM64-compatible container deployment
5. **Project Deployment**: Automated service startup
6. **Hardware Testing**: GPIO, motors, sensors verification

**See: [docs/deployment/raspberry_pi_setup.md](docs/deployment/raspberry_pi_setup.md)**

## Documentation

Comprehensive documentation is available:

### Hardware Setup

- `docs/hardware/RASPBERRY_PI_PINOUTS.md` - Complete GPIO pinout configuration
- `docs/hardware/HARDWARE_ASSEMBLY_GUIDE.md` - Step-by-step hardware assembly
- `docs/hardware/gpio_test.py` - GPIO testing and validation script

### Raspberry Pi Deployment

- `docs/deployment/raspberry_pi_setup.md` - Complete Raspberry Pi ARM64 setup guide
- `setup_raspberry_pi.sh` - Automated Raspberry Pi configuration script

### Software & Workflows

#### **Core Documentation**

- **[Control Systems Documentation](docs/software/CONTROL_SYSTEMS.md)** - Complete PID control, motion control, navigation, and safety systems
- **[Software Configuration Guide](docs/software/README.md)** - Comprehensive software setup, ROS2 configuration, and system architecture
- **[Workflow Management Guide](docs/workflows/WORKFLOW_MANAGEMENT_README.md)** - Complete n8n workflow automation guide

#### **API & Development**

- **[API Documentation](docs/api/)** - Complete REST API reference with 25+ endpoints for robot control
- **[Development Guide](docs/development/)** - Development workflow, ROS2 programming, and best practices
- **[Installation Guide](docs/installation/)** - Detailed installation instructions for all platforms

#### **Hardware & Deployment**

- **[Hardware Guide](docs/hardware/)** - Complete hardware specifications, pinouts, and assembly guides
- **[Raspberry Pi Setup](docs/deployment/raspberry_pi_setup.md)** - ARM64 deployment guide for production hardware
- **[Deployment Guide](docs/deployment/)** - Production deployment procedures and containerization
- **[Troubleshooting Guide](docs/troubleshooting/)** - Common issues, solutions, and debugging procedures

- **[Workflows README](docs/README.md)** - Documentation overview and navigation guide

#### **n8n Workflow Automation**

The system includes **33+ pre-configured n8n workflows** covering:

**Combination Workflows:**

- Robot Simple Test, Emergency Stop, Pick & Place, Mobile Pick & Place
- Inspection Patrol, Material Transport, Search & Retrieve, Emergency Response
- System Calibration, Production Line, Object Recognition, Obstacle Avoidance
- Path Planning with sensor integration

**Individual Control Workflows:**

- Control Omni Wheels (3-wheel system), Control Picker System (4 components)
- Control Container System (4 containers), Hardware Controls, Sensor Monitoring
- All workflows use HTTP API calls to `http://localhost:5000` for full ROS2 integration

#### **Control System Features**

**PID Control Implementation:**

- **Wheel Motors**: Built-in PID velocity control with acceleration limiting
- **Lifter Motor**: High-precision PID positioning (Kp=100, Ki=0.01, Kd=10)
- **Servo Motors**: Precise angular control (Kp=50, Ki=0.01, Kd=5) for all 5 servos
- **Anti-windup Protection**: Prevents integral accumulation during saturation

**Motion Control:**

- **Omni-directional Movement**: 3-wheel kinematic control with lateral capability
- **Smooth Acceleration**: Velocity ramping prevents wheel slippage
- **Real-time Response**: 100Hz control loops for responsive operation

**Safety Systems:**

- **Emergency Stop**: <100ms response time with multi-layer protection
- **Obstacle Avoidance**: LIDAR-based reactive navigation
- **Velocity Limiting**: Configurable speed and acceleration limits

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
