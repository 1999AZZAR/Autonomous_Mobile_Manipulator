# Autonomous Mobile Manipulator

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Iron-blue)
![Docker](https://img.shields.io/badge/Docker-Ready-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi-5-red)
![GitHub Repo](https://img.shields.io/badge/GitHub-1999AZZAR%2FAutonomous__Mobile__Manipulator-blue?logo=github)

**Autonomous Mobile Manipulator Robot**

</div>

## Table of Contents

- [Project Synopsis](#project-synopsis)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Prerequisites](#software-prerequisites)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Detailed Usage](#detailed-usage)
- [Development Workflow](#development-workflow)
- [Examples &amp; Tutorials](#examples--tutorials)
- [Deployment to Raspberry Pi 5](#deployment-to-raspberry-pi-5)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project Synopsis

This repository contains the complete software stack for an autonomous mobile manipulator robot. The system integrates a mobile base with an articulated arm, leveraging the ROS 2 framework for robust navigation and manipulation.

The architecture is built on a containerized, multi-service platform using Docker and Docker Compose, ensuring a clean, repeatable, and portable development environment. This allows for a powerful "sim-to-real" workflow, where the entire software stack can be tested and validated in a realistic 3D simulation before being deployed to the physical Raspberry Pi 5.

**Core Technologies:**

- **Onboard Logic**: ROS 2 Iron, Navigation2, MoveIt2
- **Automation Engine**: n8n
- **Containerization**: Docker & Docker Compose
- **Simulation**: Gazebo Simulator
- **Development OS**: Debian / Ubuntu
- **Target OS**: Ubuntu Server 22.04 (64-bit) on Raspberry Pi 5

## Key Features

- **Sim-to-Real Transfer**: Seamless transition from simulation to physical robot deployment
- **Containerized Development**: Isolated, reproducible development environment
- **Autonomous Navigation**: Advanced path planning and obstacle avoidance with Navigation2
- **Manipulator Control**: Precise arm control and manipulation with MoveIt2
- **Workflow Automation**: High-level task automation with n8n
- **Real-time Monitoring**: Live system monitoring and visualization
- **Hot Reloading**: Live code updates without container restarts
- **Multi-platform**: Works on development PCs and Raspberry Pi 5

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Host Machine (Debian/Ubuntu)              │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐                          │
│  │   n8n       │  │   ROS 2     │                          │
│  │   Web UI    │  │ Container   │                          │
│  │ (Port 5678) │  │             │                          │
│  └─────────────┘  └─────────────┘                          │
│         │                 │                                │
│         └─────────────────┼────────────────────────────────┘
│                           │
├─────────────────────────────────────────────────────────────┤
│              Docker & Docker Compose                       │
└─────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┼───────────────┐
                │               │               │
        ┌───────▼───────┐ ┌─────▼──────┐ ┌─────▼──────┐
        │  Navigation2  │ │  MoveIt2   │ │  Gazebo    │
        │  (SLAM/AMCL)  │ │  (Arm IK)  │ │ Simulator  │
        └───────────────┘ └────────────┘ └────────────┘
```

## Hardware Requirements

### Development Machine (Host PC)

- **OS**: Debian 11/12 or Ubuntu 20.04/22.04 (64-bit)
- **RAM**: 16GB minimum, 32GB recommended
- **CPU**: Multi-core processor (i5/i7 or equivalent)
- **Storage**: 50GB free space for Docker images and containers
- **GPU**: Optional, for accelerated Gazebo simulation
- **Display**: For GUI applications (RViz, Gazebo)

### Target Platform (Raspberry Pi 5)

- **Model**: Raspberry Pi 5 (8GB RAM recommended)
- **OS**: Ubuntu Server 22.04 (64-bit)
- **Storage**: 32GB microSD card minimum
- **Power Supply**: 5V/3A USB-C power supply
- **Network**: Ethernet or WiFi connectivity

## Software Prerequisites

### Required Software

- **Git**: For cloning this repository
  ```bash
  sudo apt update && sudo apt install git
  ```
- **Docker Engine**: The core containerization platform
  - Instructions: [Install Docker Engine on Debian](https://docs.docker.com/engine/install/debian/)
- **Docker Compose**: Included with Docker Engine

### Optional but Recommended

- **Visual Studio Code**: For editing code with excellent Docker and ROS 2 integration
  - Download: [Visual Studio Code](https://code.visualstudio.com/)
  - Extensions: ROS, Docker, Python (if using Python nodes)

### X11 Configuration (for GUI applications)

If you plan to run GUI applications like RViz or Gazebo with graphical display:

```bash
# Allow Docker to access your display
xhost +local:docker

# Set DISPLAY variable (if not already set)
echo $DISPLAY  # Should show something like :0
```

**Note**: You do not need to install ROS 2 or Gazebo directly on your host PC. All robotics software runs inside the Docker container.

## Available Stack Files

This project includes a complete ROS 2 software stack with the following components:

### ROS 2 Packages

**my_robot_bringup** - Main robot bringup and orchestration
- **Launch Files**: robot.launch.py (hardware), gazebo_world.launch.py (simulation)
- **Configurations**: EKF localization, joystick teleop, navigation parameters
- **World Files**: Competition arena with obstacles and targets
- **RViz Config**: Complete visualization setup for monitoring

**my_robot_description** - Robot model and hardware interface
- **URDF Model**: my_robot.urdf.xacro - Complete 3-wheeled omnidirectional robot
- **Controllers**: ROS 2 Control configuration for differential drive and arm control
- **Gazebo Plugins**: Physics simulation, sensors, and camera integration

### Ready-to-Use Launch Commands

```bash
# Launch robot in simulation with full visualization
ros2 launch my_robot_bringup gazebo_world.launch.py

# Launch robot on real hardware
ros2 launch my_robot_bringup robot.launch.py

# Launch just RViz for monitoring
rviz2 -d ros2_ws/src/my_robot_bringup/rviz/robot_view.rviz

# Test robot movement manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Stack Features

- **Complete Robot Model**: 3-wheeled omnidirectional base with arm and sensors
- **Multiple Control Modes**: Joystick, keyboard, and autonomous control
- **Full Sensor Suite**: LiDAR, IMU, camera integration
- **Competition Ready**: Arena, obstacles, and target objects for testing
- **Production Configs**: Optimized controllers and navigation parameters

## Project Structure

This project uses a specific folder structure optimized for containerized ROS 2 development:

```
Autonomous_Mobile_Manipulator/
├── docker-compose.yml          # Master orchestration file for all services
├── LICENSE                     # MIT License file
├── n8n_data/                   # Auto-generated by Docker (n8n workflows & data)
│   ├── database.sqlite         # n8n workflow database (runtime)
│   ├── workflows/              # n8n workflow source files (tracked)
│   │   ├── robot_basic_control.json    # Basic movement control
│   │   ├── robot_patrol.json           # Autonomous patrol pattern
│   │   ├── robot_obstacle_avoidance.json # Reactive obstacle avoidance
│   │   ├── robot_pick_place.json       # Pick & place automation
│   │   ├── robot_emergency_stop.json   # Emergency stop system
│   │   ├── robot_test_verification.json # System testing
│   │   └── README.md                   # Workflow documentation
│   └── nodes/                  # Custom n8n nodes
│       └── package.json        # Node dependencies (tracked)
├── ros2_ws/                    # ROS 2 workspace
│   ├── Dockerfile             # Container blueprint with ROS 2 & dependencies
│   ├── package.xml            # Workspace package manifest
│   └── src/                   # Your custom ROS 2 packages (tracked)
│       ├── my_robot_bringup/  # Main bringup package
│       │   ├── package.xml    # Package dependencies & metadata
│       │   ├── CMakeLists.txt # CMake build configuration
│       │   ├── launch/        # Launch files
│       │   │   ├── robot.launch.py     # Main robot launch (hardware)
│       │   │   └── gazebo_world.launch.py # Gazebo simulation
│       │   ├── config/        # Configuration files
│       │   │   ├── ekf.yaml           # Extended Kalman Filter config
│       │   │   └── teleop_joy.yaml    # Joystick teleop config
│       │   ├── worlds/        # Gazebo world files
│       │   │   └── competition_arena.world # Competition environment
│       │   └── rviz/          # RViz configuration
│       │       └── robot_view.rviz    # Visualization config
│       └── my_robot_description/ # Robot description package
│           ├── package.xml    # Description package dependencies
│           ├── CMakeLists.txt # Build configuration
│           ├── urdf/          # Robot model files
│           │   └── my_robot.urdf.xacro # Complete robot URDF
│           └── config/        # Controller configurations
│               └── controllers.yaml   # ROS 2 Control config
└── README.md                  # This comprehensive documentation
```

### Directory Explanations

- **`LICENSE`**: MIT License file for the project
- **`docker-compose.yml`**: Defines the n8n and ROS 2 services with proper networking and volume mounts
- **`n8n_data/`**: n8n workflow automation data
  - `workflows/`: Source workflow files (tracked in Git) - ready-to-import automation workflows
  - `database.sqlite`: Runtime workflow database (ignored) - auto-generated runtime data
  - `nodes/`: Custom n8n nodes (tracked) - reusable workflow components
- **`ros2_ws/`**: Standard ROS 2 workspace structure following REP-0144
  - `src/`: Your custom ROS 2 packages (tracked in Git)
  - Each package follows ROS 2 structure with `package.xml`, `CMakeLists.txt`, launch files, configs, and URDF models

## Quick Start

### 1. Clone and Setup

```bash
# Clone the repository (or create the structure manually)
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git Autonomous_Mobile_Manipulator
cd Autonomous_Mobile_Manipulator

# Ensure docker-compose.yml and ros2_ws/Dockerfile are in place
ls -la
# Should show: docker-compose.yml, n8n_data/, ros2_ws/
```

### 2. Build and Launch

```bash
# Build images and start services in background
docker compose up --build -d

# Check that containers are running
docker compose ps
```

**Expected Output:**

```
NAME                 IMAGE                        STATUS
n8n_container        docker.io/n8nio/n8n:latest   Up
ros2_sim_container   Autonomous_Mobile_Manipulator-ros2-sim   Up
```

### 3. Verify Installation

```bash
# Access ROS 2 container
docker exec -it ros2_sim_container bash

# Inside container, verify ROS 2 installation
source /opt/ros/iron/setup.bash
ros2 --help
# Should show ROS 2 command help

# Exit container
exit
```

## Configuration

### Environment Variables

The system uses the following key environment variables:

```yaml
# docker-compose.yml
environment:
  - DISPLAY=${DISPLAY}           # X11 display for GUI apps
  - QT_X11_NO_MITSHM=1          # Disable MIT-SHM for containerized X11
  - GENERIC_TIMEZONE=Asia/Jakarta # Set timezone for n8n
```

### ROS 2 Workspace Configuration

Your ROS 2 workspace (`ros2_ws/`) should contain:

```bash
# Standard ROS 2 package structure
my_robot_bringup/
├── package.xml
├── CMakeLists.txt (or setup.py for Python)
├── launch/
│   ├── robot.launch.py       # Main robot launch file
│   └── gazebo_world.launch.py # Simulation world
└── config/
    ├── robot.urdf.xacro      # Robot description
    └── navigation.yaml       # Nav2 parameters
```

### n8n Workflow Configuration

Access n8n at `http://localhost:5678` to:

- Create workflows for task automation
- Set up triggers for robot behaviors
- Configure webhooks for external integrations
- Manage credentials for external services

## Detailed Usage

### Accessing System Components

#### 1. n8n Workflow Editor

The n8n service provides a web-based interface for creating automation workflows.

- **URL**: http://localhost:5678
- **Purpose**: Create high-level automation workflows for robot behaviors
- **Features**:
  - Visual workflow designer
  - 200+ pre-built nodes
  - Webhook triggers
  - External API integrations

**Quick Start Workflow Example**:

```bash
# In n8n web interface:
# 1. Create new workflow
# 2. Add "Manual Trigger" node
# 3. Add "ROS 2 Publisher" node (custom node needed)
# 4. Connect and configure for robot control
```

#### 2. ROS 2 Development Environment

Access the ROS 2 container for development work:

```bash
# Enter ROS 2 container
docker exec -it ros2_sim_container bash

# Source ROS 2 environment
source /opt/ros/iron/setup.bash

# Verify ROS 2 installation
ros2 --help
```

### Running ROS 2 Applications

#### Gazebo Simulation

```bash
# Launch simulation world (create this file first)
ros2 launch my_robot_bringup gazebo_world.launch.py

# In another terminal:
docker exec -it ros2_sim_container bash
# Then run the above command
```

#### Robot Bringup

```bash
# Launch robot hardware interface and controllers
ros2 launch my_robot_bringup robot.launch.py

# Components typically include:
# - Robot description (URDF/Xacro)
# - Joint controllers
# - Sensors (LiDAR, camera, IMU)
# - Navigation stack (SLAM, AMCL)
# - Manipulation stack (MoveIt2)
```

#### RViz Visualization

```bash
# Launch RViz for monitoring
rviz2

# Load configuration file:
# File → Open Config → select your config.rviz file
```

### Common ROS 2 Commands

```bash
# List available nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /scan

# Launch with debug info
ros2 launch my_robot_bringup robot.launch.py --debug

# Run specific node
ros2 run my_perception_package perception_node

# Build workspace
cd /root/ros2_ws && colcon build

# Source workspace after building
source /root/ros2_ws/install/setup.bash
```

## Development Workflow

### Live Development Setup

The system supports hot-reloading for efficient development:

1. **Edit Code**: Modify files in `ros2_ws/src/` on your host machine
2. **Volume Mounting**: Changes are automatically synced to the container
3. **Rebuild**: Run `colcon build` inside the container
4. **Test**: Launch updated nodes without container restart

### Development Cycle

```bash
# 1. Edit code on host
code ros2_ws/src/my_robot_bringup/launch/robot.launch.py

# 2. Access container and rebuild
docker exec -it ros2_sim_container bash
cd /root/ros2_ws
colcon build --packages-select my_robot_bringup

# 3. Source and test
source install/setup.bash
ros2 launch my_robot_bringup robot.launch.py
```

### Debugging Tips

```bash
# Enable ROS 2 logging
export ROS_LOG_LEVEL=debug

# Gazebo debugging
export GAZEBO_VERBOSE=1

# View ROS 2 computation graph
ros2 doctor --help  # Health check
rqt_graph           # Visual node graph
```

## Examples & Tutorials

### Example 1: Basic Robot Navigation

```python
# my_robot_navigation/nodes/simple_navigation.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class SimpleNavigation(Node):
    def __init__(self):
        super().__init__('simple_navigation')
        self.publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

    def navigate_to(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.publisher.publish(goal)

def main():
    rclpy.init()
    node = SimpleNavigation()
    node.navigate_to(2.0, 3.0)  # Navigate to (2, 3)
    rclpy.spin(node)
```

### Example 2: n8n Workflow for Task Automation

```bash
# In n8n: Create workflow that triggers on webhook
# 1. Webhook node (POST /robot/task)
# 2. Switch node (based on task type)
# 3. ROS 2 Publisher node (send goal to robot)
# 4. HTTP Request node (confirm task completion)
```

### Example 3: MoveIt2 Arm Control

```python
# my_robot_manipulation/nodes/arm_controller.py
from moveit2 import MoveIt2Interface
import rclpy

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.moveit2 = MoveIt2Interface()

    def move_to_pose(self, x, y, z):
        pose = PoseStamped()
        pose.pose.position = Point(x, y, z)
        self.moveit2.move_to_pose(pose)

# Usage in main competition code
```

## n8n Automation Examples

### Robot Control Workflows for 3-Omni-Wheel Robot

This section provides practical n8n workflow examples for controlling your 3-wheeled omnidirectional robot. These workflows demonstrate how to integrate high-level task automation with ROS 2 robot control.

#### Required Custom n8n Node: ROS 2 Publisher

First, you'll need to create a custom n8n node for ROS 2 communication. Here's the basic structure:

```javascript
// custom_nodes/ros2_publisher.js
const rosnodejs = require('rosnodejs');

class Ros2Publisher {
    async execute() {
        // Initialize ROS 2 node
        const nh = await rosnodejs.initNode('n8n_bridge');
        const pub = nh.advertise('/cmd_vel', 'geometry_msgs/Twist');

        // Send velocity command
        const twist = {
            linear: { x: this.linear_x || 0, y: this.linear_y || 0, z: 0 },
            angular: { x: 0, y: 0, z: this.angular_z || 0 }
        };

        pub.publish(twist);
        return [{ json: { success: true, command: twist } }];
    }
}
```

#### Example 1: Basic Movement Commands

**Manual Control Interface**

```javascript
// Workflow: Manual Robot Control
// Trigger: Manual (Button press)
// Nodes:
// 1. Manual Trigger
// 2. Switch (based on command)
// 3. ROS 2 Publisher (set velocity)

const movements = {
    forward: { linear_x: 0.5, linear_y: 0, angular_z: 0 },
    backward: { linear_x: -0.5, linear_y: 0, angular_z: 0 },
    left: { linear_x: 0, linear_y: 0.5, angular_z: 0 },
    right: { linear_x: 0, linear_y: -0.5, angular_z: 0 },
    rotate_cw: { linear_x: 0, linear_y: 0, angular_z: 0.5 },
    rotate_ccw: { linear_x: 0, linear_y: 0, angular_z: -0.5 },
    stop: { linear_x: 0, linear_y: 0, angular_z: 0 }
};
```

**n8n Workflow Configuration:**
- **Manual Trigger**: Add buttons for each movement
- **Set Node**: Map button selection to velocity values
- **ROS 2 Publisher**: Send `/cmd_vel` messages to robot

#### Example 2: Autonomous Patrol Pattern

**Automated Square Patrol**

```javascript
// Workflow: Autonomous Square Patrol
// Trigger: Schedule (every 30 seconds)
// Nodes:
// 1. Schedule Trigger
// 2. Loop (4 iterations)
// 3. Move Robot (forward 2m)
// 4. Delay (1 second)
// 5. Rotate Robot (90 degrees)
// 6. Loop back

const patrol_sequence = [
    { action: 'move_forward', distance: 2.0, duration: 4 },
    { action: 'rotate', angle: 90, duration: 2 },
    { action: 'move_forward', distance: 2.0, duration: 4 },
    { action: 'rotate', angle: 90, duration: 2 },
    { action: 'move_forward', distance: 2.0, duration: 4 },
    { action: 'rotate', angle: 90, duration: 2 },
    { action: 'move_forward', distance: 2.0, duration: 4 },
    { action: 'rotate', angle: 90, duration: 2 }
];
```

#### Example 3: Obstacle Avoidance Behavior

**Reactive Obstacle Avoidance**

```javascript
// Workflow: Reactive Obstacle Avoidance
// Trigger: Laser scan data (ROS topic subscription needed)
// Nodes:
// 1. ROS Topic Subscriber (/scan)
// 2. Process Laser Data
// 3. Decision Logic
// 4. ROS 2 Publisher (/cmd_vel)

// Process laser data to detect obstacles
function processLaserData(laserData) {
    const ranges = laserData.ranges;
    const minDistance = Math.min(...ranges);

    if (minDistance < 0.5) { // Obstacle detected
        return { linear_x: 0, linear_y: 0.3, angular_z: 0 }; // Strafe right
    } else if (minDistance < 1.0) {
        return { linear_x: 0.2, linear_y: 0, angular_z: 0 }; // Slow down
    } else {
        return { linear_x: 0.5, linear_y: 0, angular_z: 0 }; // Normal speed
    }
}
```

#### Example 4: Pick and Place Task Automation

**Complete Manipulation Workflow**

```javascript
// Workflow: Pick and Place Task
// Trigger: HTTP Webhook (POST /robot/pick_place)
// Nodes:
// 1. Webhook Trigger
// 2. Extract Task Parameters
// 3. Navigate to Object
// 4. Lower Arm & Grip
// 5. Navigate to Destination
// 6. Release Object
// 7. HTTP Response (task complete)

const task_workflow = {
    navigate_to_pickup: {
        linear_x: 0.3,
        linear_y: 0,
        angular_z: 0,
        duration: 5
    },
    lower_arm: {
        // MoveIt2 joint commands
        joint_positions: [0, -1.57, 0, 0, 0, 0],
        duration: 3
    },
    grip_object: {
        gripper_command: 'close',
        duration: 1
    },
    navigate_to_place: {
        linear_x: 0.3,
        linear_y: 0.5,
        angular_z: 0,
        duration: 4
    },
    release_object: {
        gripper_command: 'open',
        duration: 1
    }
};
```

#### Example 5: Emergency Stop System

**Safety Monitoring Workflow**

```javascript
// Workflow: Emergency Stop Monitor
// Trigger: Multiple triggers (buttons, sensors, timers)
// Nodes:
// 1. Multiple Triggers (OR logic)
// 2. Emergency Stop Publisher
// 3. Notification System
// 4. Log Emergency Event

const emergency_actions = {
    stop_robot: {
        linear_x: 0,
        linear_y: 0,
        angular_z: 0
    },
    retract_arm: {
        joint_positions: [0, 0, 0, 0, 0, 0],
        velocity: 0.1
    },
    send_notification: {
        message: 'Emergency stop activated',
        priority: 'high'
    }
};
```

### Integration with External Systems

#### Webhook Integration Example

```bash
# External system can control robot via HTTP
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{
    "command": "move_forward",
    "distance": 2.0,
    "speed": 0.3
  }'
```

#### Database Integration

```javascript
// Store mission logs in external database
// Workflow nodes:
// 1. Robot Command Trigger
// 2. Execute Robot Command
// 3. Database Insert (mission log)
// 4. Send Confirmation Email
```

### Testing and Debugging Workflows

#### Simulation Testing Workflow

```javascript
// Test workflows in simulation before real robot deployment
// 1. Manual Trigger (test mode)
// 2. Set Node (simulation parameters)
// 3. Gazebo Controller (virtual robot)
// 4. Monitor Node (check responses)
// 5. Validation Node (compare expected vs actual)
```

### Sample n8n Workflow Files

You can download sample workflow files to import into your n8n instance:

#### 1. Basic Movement Control (`robot_basic_control.json`)

```json
{
  "name": "Robot Basic Movement Control",
  "nodes": [
    {
      "id": "1",
      "type": "n8n-nodes-base.manualTrigger",
      "parameters": {
        "notice": "Select movement command"
      }
    },
    {
      "id": "2",
      "type": "n8n-nodes-base.set",
      "parameters": {
        "values": {
          "string": [
            {
              "name": "movement",
              "value": "={{ $node[\"Manual Trigger\"].json[\"body\"][\"command\"] }}"
            }
          ]
        }
      }
    },
    {
      "id": "3",
      "type": "custom.ros2Publisher",
      "parameters": {
        "topic": "/cmd_vel",
        "messageType": "geometry_msgs/Twist",
        "linearX": "={{ $node[\"Movement Mapper\"].json[\"linear_x\"] }}",
        "linearY": "={{ $node[\"Movement Mapper\"].json[\"linear_y\"] }}",
        "angularZ": "={{ $node[\"Movement Mapper\"].json[\"angular_z\"] }}"
      }
    }
  ],
  "connections": {
    "Manual Trigger": {
      "main": [
        [
          {
            "node": "Movement Mapper",
            "type": "main",
            "index": 0
          }
        ]
      ]
    }
  }
}
```

#### 2. Autonomous Patrol (`robot_patrol.json`)

```json
{
  "name": "Autonomous Square Patrol",
  "nodes": [
    {
      "id": "1",
      "type": "n8n-nodes-base.scheduleTrigger",
      "parameters": {
        "rule": "0,30 * * * * *"
      }
    },
    {
      "id": "2",
      "type": "n8n-nodes-base.set",
      "parameters": {
        "values": {
          "number": [
            {
              "name": "patrol_step",
              "value": "={{ ($node[\"Patrol Counter\"].json[\"patrol_step\"] || 0) + 1 }}"
            }
          ]
        }
      }
    },
    {
      "id": "3",
      "type": "n8n-nodes-base.if",
      "parameters": {
        "conditions": {
          "number": [
            {
              "value1": "={{ $node[\"Patrol Step\"].json[\"patrol_step\"] }}",
              "operation": "modulo",
              "value2": 2
            }
          ]
        }
      }
    }
  ]
}
```

### ROS 2 Topic Integration

For more advanced workflows, you can subscribe to ROS 2 topics:

```javascript
// Custom n8n node for ROS 2 topic subscription
class Ros2Subscriber {
    async execute() {
        const nh = await rosnodejs.initNode('n8n_subscriber');
        const sub = nh.subscribe('/scan', 'sensor_msgs/LaserScan');

        return new Promise((resolve) => {
            sub.on('message', (msg) => {
                resolve([{ json: { laser_data: msg } }]);
            });
        });
    }
}
```

### Workflow Deployment on Robot

```bash
# Export workflows from n8n web interface
# Copy workflow JSON files to robot's n8n_data/nodes directory
# Restart n8n container to load new workflows

# Test workflow execution
curl -X POST http://localhost:5678/webhook/test-patrol \
  -H "Content-Type: application/json" \
  -d '{"test": true}'
```

## Deployment to Raspberry Pi 5

### Preparation

1. **Install Ubuntu Server 22.04 on Raspberry Pi 5**

   ```bash
   # Use Raspberry Pi Imager
   # Select: Ubuntu Server 22.04 LTS (64-bit)
   ```
2. **Transfer Project Files**

   ```bash
   # Copy entire project to Raspberry Pi
   scp -r Autonomous_Mobile_Manipulator/ pi@raspberrypi5:/home/ubuntu/
   ```
3. **Install Docker on Raspberry Pi**

   ```bash
   # On Raspberry Pi
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker ubuntu
   ```

### Deployment Steps

```bash
# On Raspberry Pi
cd Autonomous_Mobile_Manipulator

# Pull n8n image (build ROS 2 image locally)
docker compose pull n8n

# Build ROS 2 image for ARM64
docker compose build --no-cache

# Run in production mode
docker compose up -d

# Verify deployment
docker compose ps
curl http://localhost:5678  # Should return n8n web interface
```

### Production Optimizations

```yaml
# docker-compose.prod.yml
version: '3.8'
services:
  ros2-sim:
    # ... existing config ...
    deploy:
      resources:
        limits:
          memory: 4G
        reservations:
          memory: 2G
  n8n:
    # ... existing config ...
    restart: unless-stopped
```

## Troubleshooting

### Common Issues

#### Docker Issues

```bash
# Check Docker service status
sudo systemctl status docker

# Free up disk space
docker system prune -a

# Reset Docker daemon
sudo systemctl restart docker
```

#### X11/GUI Issues

```bash
# Allow local Docker access
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Test X11 forwarding
xeyes  # Should show eyes following mouse
```

#### ROS 2 Issues

```bash
# Check node status
ros2 node list

# Check topic communication
ros2 topic list
ros2 topic hz /topic_name

# Debug parameter server
ros2 param list
```

#### Container Issues

```bash
# Check container logs
docker compose logs ros2-sim
docker compose logs n8n

# Restart specific service
docker compose restart ros2-sim

# Rebuild container
docker compose build --no-cache ros2-sim
```

### Performance Issues

```bash
# Monitor resource usage
docker stats

# Gazebo performance
export GAZEBO_GPU=0  # Use software rendering if GPU issues

# Reduce simulation complexity
# Edit world files to use simpler models
```

### Network Issues

```bash
# Check port availability
netstat -tuln | grep 5678

# Test n8n web interface
curl -I http://localhost:5678

# Check ROS 2 master
ros2 doctor
```

## Contributing

### Development Setup

1. Fork the repository at https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/fork
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Test thoroughly in both simulation and real hardware
5. Submit a pull request to the main repository

### Code Standards

- **ROS 2**: Follow REP-0144 for package structure
- **Python**: Use PEP 8 style, type hints where possible
- **C++**: Use ROS 2 C++ style guide, modern C++ (C++17+)
- **Documentation**: Update README for new features
- **Testing**: Add unit tests for new functionality

### Commit Guidelines

```bash
# Format: <type>(<scope>): <description>

feat(ros2-sim): Add SLAM integration for Navigation2
fix(dockerfile): Resolve Gazebo GPU acceleration issue
docs(README): Update installation instructions
test(navigation): Add path planning unit tests
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 1999AZZAR

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

<div align="center">

**Happy Robotics Development!**

*For questions or support, please open an issue on [GitHub](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues).*

</div>
