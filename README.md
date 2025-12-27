# Autonomous Mobile Manipulator

A complete hexagonal-shaped autonomous mobile manipulator system built with ROS 2 Iron, featuring real-time automation workflows, advanced manipulation capabilities, and comprehensive robot control for industrial automation tasks.

## System Overview

The Autonomous Mobile Manipulator is a distributed robotics platform combining ROS2 high-level control on Raspberry Pi with real-time sensor/actuator management on Arduino Mega, designed for both simulation and real hardware deployment with a focus on reliable distributed control.

### Project Description

This document provides a high-level overview of the Autonomous Mobile Manipulator, a distributed robotics platform combining ROS2 high-level control on Raspberry Pi with real-time sensor/actuator control on Arduino Mega. The system is designed for both simulation and real hardware deployment, featuring a clear separation between high-level coordination and real-time control.

It integrates advanced robotics capabilities through a distributed architecture where the Raspberry Pi handles ROS2 navigation, web interfaces, and path planning, while the Arduino Mega manages real-time motor control, sensor acquisition, and actuator positioning.

### Key Features

- **Mobile Manipulation:** Combines 3-wheel omnidirectional base with servo-based picker system for object handling tasks.
- **Autonomous Navigation:** Custom path planning algorithms with obstacle avoidance and waypoint navigation.
- **Sensor Fusion:** Integrated sensor suite with IR distance, ultrasonic, LIDAR, IMU, and line sensors.
- **Distributed Architecture:** Raspberry Pi for high-level control + Arduino Mega for real-time motor control.
- **Flexible Control Interfaces:**
    - **Web UI (Primary Interface):** Complete robot control with path planning, real-time sensor monitoring, and system management.
    - **REST API:** HTTP interface for all robot functions with JSON responses.
    - **WebSocket Support:** Real-time data streaming for live monitoring.
    - **Serial Communication:** Robust Arduino Mega interface with automatic reconnection.
- **Hardware Control:** Direct GPIO control with simulation mode for development.

### Technology Stack

- **Robotics Framework:** ROS 2 (Iron)
- **Web Framework:** Flask (Python)
- **Hardware Control:** Arduino Mega (ATmega2560)
- **Serial Communication:** PySerial with auto-reconnection
- **Containerization:** Docker, Docker Compose
- **Navigation:** Custom path planning algorithms
- **Primary Language:** Python 3

### System Capabilities

This project provides a complete production-ready robotics platform featuring:

- **Hexagonal robot design** with 3-wheel omnidirectional movement system
- **Advanced manipulation** with servo-based picker system for precise object handling
- **Distributed sensor architecture** with real-time sensor fusion:
  - **Arduino Mega (Real-time):** IR Sharp distance sensors (6x), HC-SR04 ultrasonic sensors (2x), motor control, servo actuators, emergency stop, PID motor control
  - **Raspberry Pi (High-level):** TF-Luna LiDAR, MPU6050 IMU, line sensors (3x), path planning, ROS2 coordination, web interface
- **ROS 2 Iron** for robust robot control and navigation
- **Serial communication** between RPi and Arduino Mega for coordinated control
- **Web-based control interface** with real-time monitoring and manual control
- **Docker containerization** for reliable deployment
- **Hardware control systems** for safety and operational management

### Software Architecture

The architecture combines ROS 2 high-level control with Arduino Mega real-time control through a modular, distributed design.

#### ROS 2 Workspace (`ros2_ws`)

**Core ROS2 Packages:**
- **`my_robot_description`:** Robot physical structure (URDF) and controller configuration
- **`my_robot_bringup`:** Launch files for simulation/hardware deployment modes
- **`my_robot_navigation`:** Nav2 stack configuration for path planning and obstacle avoidance
- **`my_robot_manipulation`:** High-level manipulation coordination (ROS2 services)
- **`my_robot_automation`:** Bridge between ROS2 and external systems

#### Distributed Control Architecture

**Raspberry Pi Software Stack:**
- **Flask Web Interface**: REST API and web-based control (`scripts/app.py`)
- **ROS2 Interface**: ROS2 node coordination (`scripts/ros2_interface.py`)
- **Mega Serial Interface**: Bidirectional communication with Arduino Mega (`scripts/mega_interface.py`)
- **Sensor Manager**: High-level sensor processing (`scripts/sensor_manager.py`)
- **GPIO Controller**: Direct hardware control for RPi sensors (`scripts/gpio_controller.py`)
- **Path Planning**: Navigation algorithms (`scripts/path_planning.py`)

**Arduino Mega Software (Separate Repository):**
- **Motor Control**: Real-time PID control for 3 omnidirectional wheels
- **Sensor Acquisition**: IR sensors (6x), ultrasonic sensors (2x)
- **Actuator Control**: Servo positioning for gripper manipulation
- **Safety Systems**: Emergency stop and motor protection
- **Serial Communication**: JSON-based command protocol

#### Distributed Hardware Architecture

The system uses a distributed architecture with clear separation of responsibilities:

**Raspberry Pi 4 Responsibilities (High-level Control):**
- ROS 2 Iron framework and navigation stack coordination
- Web interface (Flask) and REST API server
- Path planning and waypoint navigation algorithms
- IMU sensor processing (MPU6050) for orientation tracking
- TF-Luna LiDAR processing for obstacle detection
- Line sensor processing for line following
- Serial communication management with Arduino Mega
- Docker container orchestration

**Arduino Mega Responsibilities (Real-time Control):**
- Real-time motor control for 3-wheel omnidirectional drive with PID velocity control
- IR Sharp distance sensors (6x) for wall alignment and obstacle detection
- HC-SR04 ultrasonic sensors (2x) for close-range obstacle detection
- Servo actuator control for gripper manipulation (open/close, tilt)
- Emergency stop handling with immediate motor shutdown
- Motor position tracking and odometry calculation
- Real-time sensor data acquisition and filtering
- Acceleration limiting and motor synchronization

**Communication Protocol:**
- Serial communication at 115200 baud (/dev/ttyACM0, /dev/ttyACM1, etc.)
- Single-character command protocol (f, b, l, r, q, e, z, x, c, w, t, y, a, j, 5-9, 0, u, d, s, p, 1-4, g, h)
- Real-time sensor data transmission from Arduino Mega
- Status messages and diagnostic feedback
- Automatic reconnection and error recovery

External control systems interact with the `my_robot_automation` package, which translates high-level commands into specific ROS 2 messages, service calls, and action goals.

### Control Interface Architecture

The system provides a unified web-based interface for complete robot control and monitoring:

#### Web UI - Primary Interface
The Web UI at http://localhost:8000 provides complete robot control and monitoring capabilities:

- **Movement Control:** Directional movement (forward, backward, strafe, turn) with variable speed
- **Individual Wheel Control:** Direct control of each omni wheel for testing and calibration
- **Gripper Control:** Servo-based manipulation (open/close, tilt positioning)
- **Path Planning:** Visual waypoint navigation with grid-based path planning
- **Real-time Monitoring:** Live sensor data from all sensors (IR, ultrasonic, LIDAR, IMU, line sensors)
- **System Diagnostics:** Connection status, sensor health, command history
- **Manual Testing:** Individual component testing and calibration tools

#### REST API
The Flask-based REST API provides programmatic access to all robot functions:

- **Movement Control:** `/api/robot/move`, `/api/robot/turn`, `/api/robot/stop`
- **Speed Control:** `/api/robot/speed`, `/api/robot/turbo`
- **Sensor Data:** `/api/robot/sensors`, `/api/status`
- **Individual Control:** `/api/robot/wheels/<id>`, `/api/robot/picker/*`
- **System Management:** `/health`, `/api/mega/status`

**Architecture Diagram:**
```
Operator → Web UI → REST API → Python Control Logic → Serial → Arduino Mega → Motors/Sensors
```

**Current Deployment Options:**
- **Development:** Simulation mode (no hardware required)
- **Testing:** Web UI with Arduino Mega hardware for validation
- **Production:** Full RPi + Arduino Mega deployment with web interface

**Future Deployment Options (Planned):**
- **Automated Production:** Advanced workflow automation integration
- **Multi-Robot Systems:** Coordinated multi-robot operations
- **Industrial Automation:** Production line integration

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
- [Contributing](#contributing)
- [License](#license)

## Hardware Specifications

### Robot Configuration

- **Shape**: Hexagonal shaped autonomous mobile robot
- **Drive System**: 3-wheel omnidirectional movement (Back, Front Left, Front Right)

### Sensors

**Arduino Mega (Real-time Sensors):**
- **IR Sharp Distance Sensors**: 6 units (GP2Y0A02YK0F, 20-150cm range)
  - Left front/back (wall alignment, obstacle detection)
  - Right front/back (wall alignment, obstacle detection)
  - Back left/right (rear obstacle detection)
- **HC-SR04 Ultrasonic Sensors**: 2 units for close-range detection
  - Front left/right ultrasonic sensors (0-400cm range)
- **Line Sensors**: 3 individual sensors for line following
  - Left, center, and right line detection
- **Motor Encoders**: Built-in quadrature encoders on all 4 motors
  - 28 counts per revolution (PG28 motors)
  - Real-time RPM calculation and PID feedback

**Raspberry Pi (High-level Sensors):**
- **TF-Luna LIDAR**: Single-point LIDAR sensor (I2C/UART interface, up to 8m range)
- **MPU6050 IMU**: 6-axis IMU for orientation tracking and motion sensing
- **Line Sensors**: 3 individual sensors for line following and alignment
  - Left, center, and right sensors with analog threshold detection
- **USB Camera**: Gripper-mounted camera for vision tasks (optional)

### Actuators & Manipulation

**Arduino Mega (Real-time Actuators):**
- **YFROBOT v2 Motor Driver Shield**:
  - 4x PG28 DC motors with built-in encoders (28 CPR)
  - I2C communication with Arduino Mega
  - PID velocity control for precise motor speed regulation
- **Omni Wheel Drive System**:
  - 3-wheel hexagonal configuration (135° wheel spacing)
  - Front Right (45°), Front Left (135°), Back (180°) wheel angles
  - Selective wheel usage based on movement type
- **Lifter Motor Control**:
  - Dedicated motor for vertical lifting operations
  - Encoder feedback with position tracking
  - Limit switch safety integration

**Raspberry Pi (High-level Control):**
- Path planning and trajectory generation for manipulation tasks
- ROS2 service coordination for complex manipulation sequences
- Web interface for manual servo positioning
- Automation workflow integration for pick-and-place operations

### Control Systems

**Arduino Mega (Real-time Control):**
- **PID Motor Control**: Individual PID controllers for each of 4 motors
  - Tunable Kp, Ki, Kd parameters for optimal performance
  - 10ms sample time for responsive control
  - Anti-windup protection and output limiting
- **Omni Kinematics**: Inverse kinematics for hexagonal 3-wheel robot
  - Selective wheel usage (2-3 wheels based on movement type)
  - Movement thresholds to prevent unwanted wheel activation
  - Speed normalization and acceleration limiting
- **Serial Command Interface**: Extensive command set for robot control
  - Single-character commands for immediate response
  - Speed multipliers (50%-100%) for fine control
  - Built-in test patterns and calibration routines
- **Safety Systems**: Emergency stop and motor protection
  - Immediate motor shutdown on command
  - Encoder monitoring and stall detection

**Raspberry Pi (High-level Control):**
- **ROS2 Navigation**: Path planning with obstacle avoidance
- **Web Interface**: REST API and web-based robot control
- **API Automation**: REST API for programmatic robot control
- **Sensor Integration**: IMU and LIDAR processing for localization
- **Path Planning**: Grid-based navigation with waypoint following

## System Architecture

The system consists of two main components:

### Python Control System

- **Robot coordination**: Main application managing all components
- **Hardware interface**: Serial communication with Arduino Mega
- **Sensor management**: Data collection and processing from all sensors
- **Path planning**: Grid-based navigation algorithms
- **REST API server**: Comprehensive HTTP API (8000) with 20+ endpoints
- **WebSocket support**: Real-time data streaming
- **GPIO control**: Direct hardware control for RPi components

### Professional Web Interface
#### WebRobotInterface Auto-Initialization

The `WebRobotInterface` class now supports automatic ROS 2 initialization:

```python
# Before (manual ROS 2 setup required)
import rclpy
rclpy.init()
from web_robot_interface import WebRobotInterface
interface = WebRobotInterface()

# After (automatic ROS 2 setup)
from web_robot_interface import WebRobotInterface
interface = WebRobotInterface()  # ROS 2 initialized automatically
interface.cleanup()  # ROS 2 shutdown automatically
```

**Features:**
- ✅ Automatic ROS 2 initialization and shutdown
- ✅ No manual `rclpy.init()` required
- ✅ Backward compatible with existing code
- ✅ Proper resource cleanup

**Test:** `python3 test_auto_ros_init.py`

- **User-friendly control center**: Modern web-based robot control interface
- **Real-time status monitoring**: Live system status and diagnostics
- **Tabbed control panels**: Organized access to all robot functions
- **Professional UI/UX**: Responsive design with intuitive controls
- **API integration**: Frontend for all ROS2 REST API endpoints

### Future Automation Integration

- **Workflow Automation**: Planned low-code automation platform (n8n)
- **HTTP API Foundation**: REST endpoints ready for automation integration
- **Programmatic Control**: Direct API access for custom automation
- **Sequence Execution**: Movement pattern automation via API
- **Event-Driven Control**: Sensor-based automation triggers
- **Multi-System Integration**: Ready for external automation platforms

## API Capabilities Overview

The Autonomous Mobile Manipulator provides a comprehensive REST API for complete robot control:

### **Core Control APIs**

- **Arduino Mega Serial Commands:**
  - `POST /api/serial/send` - Send commands directly to Arduino Mega
  - Supported commands: f, b, l, r, q, e, z, x, c, w, t, y, a, j, 5-9, 0, u, d, s, p, 1-4, g, h

- **Movement Commands:**
  - Forward: `f`, Backward: `b`, Left: `l`, Right: `r`
  - Diagonal: Forward-Left: `q`, Forward-Right: `e`, Backward-Left: `z`, Backward-Right: `x`
  - Rotation: Clockwise: `c`, Counter-clockwise: `w`
  - Turns: Left turn: `t`, Right turn: `y`
  - Arcs: Left arc: `a`, Right arc: `j`

- **Speed Control:**
  - Speed levels: 5(50%), 6(60%), 7(70%), 8(80%), 9(90%), 0(100%)
  - Send speed command before movement commands

- **Lifter Control:**
  - Lift up: `u`, Lift down: `d`, Stop: `s`

### **Manipulation APIs**

- **Gripper Control:**
  - `POST /api/robot/picker/gripper` - Open/close gripper
  - `POST /api/robot/picker/gripper_tilt` - Control gripper tilt angle

### **Monitoring & Diagnostics**

- **System Status:**
  - `GET /health` - Basic health check
  - `GET /api/status` - Comprehensive system status
  - `GET /api/mega/status` - Arduino Mega connection status
  - `POST /api/mega/reconnect` - Force reconnection to Arduino

- **Sensor Data:**
  - `GET /api/robot/sensors` - All sensor readings
  - `GET /api/robot/sensors/diagnostics` - Sensor health diagnostics

### **Advanced Features**

- **Serial Communication:**
  - `POST /api/serial/send` - Direct serial commands to Arduino
  - `GET /api/serial/monitor` - Serial communication monitoring

- **Emergency Systems:**
  - `POST /api/robot/emergency-stop` - Emergency stop with reason

- **Navigation (Development):**
  - `POST /api/robot/waypoints/navigate` - Waypoint-based navigation
  - `GET /api/map/canvas` - Navigation map data
  - `GET /api/robot/position` - Current position tracking

**Total: 20+ API endpoints** providing complete robotic control and monitoring capabilities.

## Quick Start

### Prerequisites

- Docker and Docker Compose
- Git

### Installation

#### Step 1: Clone Repository

```bash
git clone <repository-url>
cd lks_robot_project
```

#### Step 2: Run Setup

**For PC/Development:**
```bash
./setup --pc
```

**For Raspberry Pi:**
```bash
./setup --rpi
```

The setup script will:
- Install Docker and dependencies
- Configure system settings
- Build ROS2 workspace
- Set up monitoring scripts
- Configure hardware interfaces (Raspberry Pi only)

#### Step 3: Start the System

**Hardware Mode (Real Sensors on Raspberry Pi):**
```bash
./start --hw
```

**Simulation Mode (Development/Testing):**
```bash
./start --sim
```

**Test Mode (Verify System):**
```bash
./start --test
```

### Operating Modes

#### Hardware Mode (`--hw`)

- **Purpose**: Production operation with real sensors and actuators
- **Platform**: Raspberry Pi + Arduino Mega distributed system
- **Sensors**: Arduino Mega (IR + ultrasonic) + RPi (IMU, LIDAR, line sensors)
- **Actuators**: Arduino Mega (motors, servos) + RPi (GPIO control)
- **Usage**: `./start --hw`

#### Simulation Mode (`--sim`)

- **Purpose**: Development and testing without hardware
- **Platform**: Any PC with Docker
- **Sensors**: Simulated sensor data
- **Usage**: `./start --sim`

#### Test Mode (`--test`)

- **Purpose**: System verification and interactive testing
- **Features**: API testing, sensor verification, interactive control menu
- **Usage**: `./start --test`

### Access Points

**Primary Interface (Required):**
- **Web UI**: http://localhost:8000
  - Complete robot control (movement, gripper, containers)
  - Path planning with visual waypoint manager
  - Real-time sensor monitoring (all 6 laser, ultrasonic, LIDAR, IMU, line sensors)
  - System status and logs
  - IMU calibration
  - Hardware pinout reference

**REST API Access:**
- **API Endpoints**: http://localhost:8000/api/*
  - Programmatic robot control
  - Sensor data access
  - System diagnostics
  - Individual component control

**Developer Access:**
- **REST API**: http://127.0.0.1:8000/api/* (Backend for Web UI and automation)
- **ROS 2 Container**: `docker exec -it ros2_sim_container bash`

Note: Use `127.0.0.1` instead of `localhost` for API calls to ensure IPv4 connectivity.

### Quick Commands

The system provides two unified scripts for all operations:

#### Setup Script (`./setup`)

One-time system configuration:

```bash
./setup --pc          # PC/development setup
./setup --rpi         # Raspberry Pi setup
./setup --help        # Show all options
```

#### Start Script (`./start`)

All runtime operations:

```bash
# Starting the system
./start --hw          # Hardware mode
./start --sim         # Simulation mode
./start --test        # Test mode with interactive menu

# System management
./start --status      # Show system status
./start --logs        # View logs
./start --stop        # Stop all containers
./start --restart     # Restart containers
./start --shell       # Enter container shell
./start --clean       # Clean Docker system

# Advanced options
./start --hw --build  # Rebuild and start
./start --sim --foreground  # Run in foreground
./start --help        # Show all options
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

## Arduino Mega Command System

The Raspberry Pi communicates with the Arduino Mega via serial commands. The Arduino Mega implements a comprehensive command-based control system:

### Movement Commands

**Basic Directions (8-directional movement):**
- `f`/`F` - Forward
- `b`/`B` - Backward
- `l`/`L` - Left (strafe)
- `r`/`R` - Right (strafe)

**Diagonal Movements:**
- `q`/`Q` - Forward-Left
- `e`/`E` - Forward-Right
- `z`/`Z` - Backward-Left
- `x`/`X` - Backward-Right

**Rotation & Turning:**
- `c`/`C` - Rotate Clockwise (spin in place)
- `w`/`W` - Rotate Counter-Clockwise (spin in place)
- `t`/`T` - Turn Left (forward + left rotation)
- `y`/`Y` - Turn Right (forward + right rotation)

**Arc Movements:**
- `a`/`A` - Arc Left (gentle left curve)
- `j`/`J` - Arc Right (gentle right curve)

### Speed Control

- `5` - 50% speed
- `6` - 60% speed
- `7` - 70% speed
- `8` - 80% speed
- `9` - 90% speed
- `0` - 100% speed (full speed)

### Lifter Control

- `u`/`U` - Lift Up
- `d`/`D` - Lift Down

### System Commands

- `s`/`S` - Emergency Stop (all motors)
- `p`/`P` - Print Status (RPM, PID values)
- `1-4` - Test individual motors (Motor 1-4)
- `g`/`G` - Full calibration sequence
- `h`/`H` - Figure-8 test pattern

### Command Usage Examples

```bash
# Move forward
curl -X POST http://localhost:8000/api/serial/send \
  -H "Content-Type: application/json" \
  -d '{"command": "f"}'

# Set speed to 80% then move forward
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "8"}'
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "f"}'

# Emergency stop
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "s"}'

# Lifter operation
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "u"}'
```

## Automation Features

The system provides basic automation capabilities through the REST API and web interface. Advanced workflow automation is planned for future implementation:

### Current Automation Features

- **Arduino Mega Command Sequences**: Programmed movement patterns via serial commands
- **Speed Control Integration**: Variable speed control with command sequences
- **Emergency Protocols**: Automated safety responses and system recovery
- **Serial Command Automation**: Web interface for sending Arduino Mega commands

### Planned Automation Features (Future)

- **Workflow Automation**: Low-code automation platform for complex task sequences
- **Scheduled Operations**: Time-based autonomous operations
- **Event-Driven Automation**: Sensor-based conditional behaviors
- **Multi-Robot Coordination**: Distributed robot task management
- **Production Line Automation**: Manufacturing process orchestration

The REST API provides the foundation for future automation integration, with all robot functions accessible via HTTP endpoints for external automation systems.

## API Documentation

The robot provides a comprehensive REST API on port 8000 for external control and monitoring.

### System Health & Status

```bash
# Health check
curl http://127.0.0.1:5000/health

# Get comprehensive robot status
curl http://127.0.0.1:5000/api/robot/status
```

### Robot Mode Management

```bash
# Set robot operating mode
curl -X POST http://127.0.0.1:5000/api/robot/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "AUTONOMOUS", "reason": "Starting automated operation"}'

# Available modes: AUTONOMOUS, MANUAL, EMERGENCY, MAINTENANCE
```

### Basic Movement Control

```bash
# Move robot in cardinal directions
curl -X POST http://127.0.0.1:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn robot left/right
curl -X POST http://127.0.0.1:5000/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Strafe movement (omni-directional)
curl -X POST http://127.0.0.1:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "strafe_left", "speed": 0.4}'

# Stop all movement
curl -X POST http://127.0.0.1:5000/api/robot/stop
```

### Picker System Control

```bash
# Control gripper (open/close)
curl -X POST http://127.0.0.1:5000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command": "open"}'

# Control gripper tilt
curl -X POST http://127.0.0.1:5000/api/robot/picker/gripper_tilt \
  -H "Content-Type: application/json" \
  -d '{"angle": 15}'

# Control gripper neck position
curl -X POST http://127.0.0.1:5000/api/robot/picker/gripper_neck \
  -H "Content-Type: application/json" \
  -d '{"position": 0.5}'

# Control gripper base height
curl -X POST http://127.0.0.1:5000/api/robot/picker/gripper_base \
  -H "Content-Type: application/json" \
  -d '{"height": 0.3}'
```

### Container System Control

```bash
# Control left front container
curl -X POST http://127.0.0.1:5000/api/robot/containers/left_front \
  -H "Content-Type: application/json" \
  -d '{"action": "load"}'

# Control left back container
curl -X POST http://127.0.0.1:5000/api/robot/containers/left_back \
  -H "Content-Type: application/json" \
  -d '{"action": "unload"}'

# Control right front container
curl -X POST http://127.0.0.1:5000/api/robot/containers/right_front \
  -H "Content-Type: application/json" \
  -d '{"action": "load"}'

# Control right back container
curl -X POST http://127.0.0.1:5000/api/robot/containers/right_back \
  -H "Content-Type: application/json" \
  -d '{"action": "unload"}'
```

### Hardware Control

```bash
# Emergency stop
curl -X POST http://127.0.0.1:5000/api/robot/hardware/emergency \
  -H "Content-Type: application/json" \
  -d '{"action": "stop"}'

# Start/stop robot
curl -X POST http://127.0.0.1:5000/api/robot/hardware/start_stop \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'

# Set robot mode
curl -X POST http://127.0.0.1:5000/api/robot/hardware/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "run"}'
```

### Legacy Actuator Control (for compatibility)

```bash
# Legacy servo control (for compatibility)
curl -X POST http://127.0.0.1:5000/api/robot/servo \
  -H "Content-Type: application/json" \
  -d '{"servo": 1, "angle": 90}'

# Note: Lifter control is integrated into picker system
# Use /api/robot/picker/gripper_base for height control
```

### Advanced Automation Operations

```bash
# Execute pick and place operation
curl -X POST http://127.0.0.1:5000/api/robot/pick-place \
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
curl -X POST http://127.0.0.1:5000/api/robot/patrol \
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
curl -X POST http://127.0.0.1:5000/api/robot/obstacle-avoidance \
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
curl -X POST http://127.0.0.1:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": true, "reason": "Safety emergency"}'

# Activate emergency stop
curl -X POST http://127.0.0.1:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": true, "reason": "Manual emergency stop", "force": true}'

# Deactivate emergency stop
curl -X POST http://127.0.0.1:5000/api/robot/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"activate": false, "reason": "Emergency resolved"}'
```

### Automation Integration (Future)

The REST API is designed for easy integration with external automation systems:

```bash
# Programmatic movement control
curl -X POST http://localhost:8000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Arduino Mega command examples
# Move forward
curl -X POST http://localhost:8000/api/serial/send \
  -H "Content-Type: application/json" \
  -d '{"command": "f"}'

# Set speed to 80% and move forward
curl -X POST http://localhost:8000/api/serial/send \
  -H "Content-Type: application/json" \
  -d '{"command": "8"}'
curl -X POST http://localhost:8000/api/serial/send \
  -H "Content-Type: application/json" \
  -d '{"command": "f"}'

# Lift operation
curl -X POST http://localhost:8000/api/serial/send \
  -H "Content-Type: application/json" \
  -d '{"command": "u"}'
```

### Status Monitoring

```bash
# Get comprehensive robot status (includes safety systems and sensor data)
curl http://127.0.0.1:5000/api/robot/status

# Get detailed sensor data
curl http://127.0.0.1:5000/api/robot/sensors

# Get active tasks
curl http://127.0.0.1:5000/api/robot/tasks

# Cancel a running task
curl -X POST http://127.0.0.1:5000/api/robot/tasks/task_123/cancel \
  -H "Content-Type: application/json" \
  -d '{"reason": "User requested cancellation"}'

# Get navigation status
curl http://127.0.0.1:5000/api/robot/navigation/status

# Get IMU position (accessible all time as per specification)
curl http://127.0.0.1:5000/api/robot/imu/position

# Get robot log (accessible all time as per specification)
curl http://127.0.0.1:5000/api/robot/log

# Get last 3 commands (accessible all time as per specification)
curl http://127.0.0.1:5000/api/robot/commands/last
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

### API Integration Development

The REST API enables programmatic control for custom automation:

1. **REST API Access**: Use HTTP endpoints for robot control
2. **Python Integration**: Direct API calls from Python applications
3. **Sequence Creation**: Build movement sequences via API calls
4. **Sensor Monitoring**: Real-time sensor data access
5. **Future Automation**: Foundation for advanced workflow systems

### API Development

The robot API is implemented in `ros2_ws/src/my_robot_automation/scripts/app.py` and provides comprehensive REST endpoints that coordinate between ROS2 high-level control and Arduino Mega real-time execution.

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
| `/api/serial/send`                  | POST   | Send Arduino Mega commands       | `command` (f,b,l,r,q,e,z,x,c,w,t,y,a,j,5-9,0,u,d,s,p,1-4,g,h) |
| `/api/serial/monitor`               | GET    | Serial communication monitor     | None                                             |
| `/api/mega/status`                  | GET    | Arduino Mega connection status   | None                                             |
| `/api/mega/reconnect`               | POST   | Force Mega reconnection          | None                                             |
| `/api/robot/picker/gripper`         | POST   | Control gripper open/close       | `command` ("open"/"close")                     |
| `/api/robot/picker/gripper_tilt`    | POST   | Control gripper tilt angle       | `angle` (0-180°)                              |
| `/api/robot/picker/gripper_neck`    | POST   | Control gripper neck position    | `position` (-1.0 to 1.0)                       |
| `/api/robot/picker/gripper_base`    | POST   | Control gripper base height      | `height` (0.0 to 1.0)                          |
| `/api/robot/containers/{id}`        | POST   | Control container operations     | `action` ("load"/"unload")                     |
| `/api/robot/pick-place`             | POST   | Execute pick and place operation | `pickup_location`, `place_location`, ...     |
| `/api/robot/patrol`                 | POST   | Execute autonomous patrol        | `waypoints`, `patrol_speed`, ...             |
| `/api/robot/obstacle-avoidance`     | POST   | Navigate with obstacle avoidance | `target_location`, `avoidance_distance`, ... |
| `/api/robot/emergency-stop`         | POST   | Emergency stop control           | `activate`, `reason`, `force`              |
| `/api/robot/sequences/execute`      | POST   | Execute movement sequence        | `sequence` (array)                             |
| `/api/robot/sequences/save`         | POST   | Save movement sequence           | `name`, `sequence`                             |
| `/api/robot/sequences/load/<name>`  | GET    | Load saved sequence              | None                                            |
| `/api/robot/sequences/list`         | GET    | List saved sequences             | None                                            |

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

### PC/Development Deployment

```bash
# Install Python dependencies
pip3 install flask pyserial lgpio gpiozero

# Run in simulation mode (recommended for development)
python3 main.py --simulation

# Run with Arduino Mega hardware
python3 main.py --hardware

# Access interfaces
# Web UI: http://localhost:8000
# API: http://localhost:8000/api/*
```

### Raspberry Pi Production Deployment

For comprehensive Raspberry Pi setup, see the dedicated guide:

**[Complete Raspberry Pi Setup Guide](docs/deployment/raspberry_pi_setup.md)**

#### Quick Raspberry Pi Deployment

```bash
# On Raspberry Pi 5 with Ubuntu Server 22.04

# Step 1: Run automated setup (one-time)
./setup --rpi

# Follow prompts and reboot when instructed
sudo reboot

# Step 2: Start the robot
./start --hw

# Step 3: Verify system
./start --status
./start --test

# Access the robot
# Web UI: http://raspberrypi:8000
# API: http://raspberrypi:8000/api/*
```

#### Manual Raspberry Pi Setup Steps

1. **OS Installation**: Ubuntu Server 22.04 LTS (64-bit) on Raspberry Pi 5
2. **Run Setup Script**: `./setup --rpi` (handles all configuration)
3. **Reboot**: Allow hardware interfaces to initialize
4. **Start System**: `./start --hw`
5. **Test Hardware**: `./start --test`

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
- **[API Documentation](docs/api/README.md)** - Complete REST API reference and integration guide

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

#### **System Verification Reports**

- **[API Verification Report](docs/api/API_VERIFICATION_REPORT.md)** - Complete API verification against hardware specifications
- **[Workflow Cleanup Report](docs/workflow/WORKFLOW_CLEANUP_REPORT.md)** - Workflow management and corrections
- **[Connection Fix Report](docs/workflow/CONNECTION_FIX_REPORT.md)** - IPv4/IPv6 connectivity resolution

#### **REST API Automation**

The system provides comprehensive REST API automation covering:

**Core Control APIs:**
- Movement control (directional, speed, emergency stop)
- Individual wheel control and calibration
- Sensor data access and diagnostics
- System status monitoring

**Advanced Features:**
- Waypoint navigation and path planning
- Movement sequence creation and execution
- Real-time sensor fusion and monitoring
- Serial communication diagnostics

**Integration Ready:**
- HTTP-based API for easy automation integration
- JSON responses for programmatic control
- WebSocket support for real-time updates
- Foundation for future workflow automation systems

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

## Contributing

We welcome contributions to the Autonomous Mobile Manipulator project! Please see our [Contributing Guide](CONTRIBUTING.md) for detailed information on:

- Development environment setup
- Code contribution guidelines
- Testing requirements
- Pull request process
- Issue reporting

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
