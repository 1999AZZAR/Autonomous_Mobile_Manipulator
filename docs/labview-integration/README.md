# LabVIEW Integration Guide

This guide provides comprehensive instructions for integrating the Autonomous Mobile Manipulator robot with National Instruments LabVIEW development environment.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
  - [Software Requirements](#software-requirements)
  - [Hardware Requirements](#hardware-requirements)
- [Integration Architecture](#integration-architecture)
  - [Communication Methods](#communication-methods)
- [LabVIEW VI Development](#labview-vi-development)
  - [Basic Robot Control VI](#basic-robot-control-vi)
  - [Sensor Data Acquisition VI](#sensor-data-acquisition-vi)
  - [Autonomous Behavior VIs](#autonomous-behavior-vis)
- [Network Communication Setup](#network-communication-setup)
  - [TCP/IP Socket Configuration](#tcpip-socket-configuration)
  - [HTTP API Integration](#http-api-integration)
- [Advanced Integration Examples](#advanced-integration-examples)
  - [Real-time Data Dashboard](#real-time-data-dashboard)
  - [Automated Testing Framework](#automated-testing-framework)
  - [Vision-Guided Manipulation](#vision-guided-manipulation)
- [Hardware Integration](#hardware-integration)
  - [NI DAQ Device Integration](#ni-daq-device-integration)
  - [Serial Communication](#serial-communication)
- [Development Workflow](#development-workflow)
  - [VI Development Best Practices](#vi-development-best-practices)
  - [Testing and Validation](#testing-and-validation)
- [Deployment and Distribution](#deployment-and-distribution)
  - [Application Building](#application-building)
- [Troubleshooting LabVIEW Integration](#troubleshooting-labview-integration)
  - [Common Issues](#common-issues)
- [Advanced Features](#advanced-features)
  - [Multi-Robot Coordination](#multi-robot-coordination)
  - [Machine Learning Integration](#machine-learning-integration)
- [Support and Resources](#support-and-resources)
  - [Documentation References](#documentation-references)
  - [Community Resources](#community-resources)
  - [Technical Support](#technical-support)
- [Best Practices](#best-practices)
  - [Development Guidelines](#development-guidelines)
  - [Safety Considerations](#safety-considerations)

## Overview

LabVIEW integration enables:
- **Visual Programming Interface**: Drag-and-drop robot control programming
- **Real-time Monitoring**: Live data visualization and control panels
- **Hardware Integration**: Direct interfacing with DAQ devices and sensors
- **Data Logging**: Automated data collection and analysis
- **HMI Development**: Custom operator interfaces for robot control

## Prerequisites

### Software Requirements
- **LabVIEW**: Version 2020 or later (64-bit recommended)
- **LabVIEW Real-Time Module**: For embedded control applications
- **LabVIEW Vision Development Module**: For computer vision tasks
- **NI-VISA**: For instrument communication
- **NI-DAQmx**: For data acquisition hardware

### Hardware Requirements
- **Development Computer**: Windows 10/11 with LabVIEW installed
- **Network Connection**: Ethernet or WiFi for robot communication
- **Optional**: NI DAQ device for additional sensor integration

## Integration Architecture

### Communication Methods

#### 1. TCP/IP Socket Communication
```bash
# Robot WebSocket server (already running via Foxglove Bridge)
# Port: 8765
# Protocol: WebSocket with JSON messages
```

#### 2. HTTP REST API
```bash
# n8n webhook endpoints
# Base URL: http://localhost:5678/webhook/
# Endpoints: robot-control, emergency-stop, pick_place
```

#### 3. ROS 2 Topic Bridge
```bash
# ROS 2 topics accessible via LabVIEW
# Topics: /cmd_vel, /scan, /imu/data, /joint_states
```

## LabVIEW VI Development

### Basic Robot Control VI

#### Front Panel Design

**Controls**:
- **Movement Buttons**: Forward, Backward, Left, Right, Rotate CW/CCW
- **Speed Slider**: Linear velocity control (0-1.0 m/s)
- **Rotation Slider**: Angular velocity control (0-2.0 rad/s)
- **Emergency Stop Button**: Red button for immediate halt

**Indicators**:
- **Connection Status**: LED indicator for robot connectivity
- **Battery Level**: Numeric indicator for battery voltage
- **Robot Position**: X, Y coordinate display
- **Sensor Data**: LiDAR distance readings, IMU orientation

#### Block Diagram Implementation

```labview
# Main control loop structure:
While Loop:
├── TCP Write (Send velocity commands)
├── TCP Read (Receive sensor data)
├── Update Indicators
└── Event Structure (Handle button presses)
```

### Sensor Data Acquisition VI

#### LiDAR Data Processing

```labview
# LiDAR scan processing:
TCP Read (LaserScan message)
├── JSON Parse
├── Extract ranges array
├── Find minimum distance
├── Calculate obstacle direction
└── Update polar plot display
```

#### IMU Data Processing

```labview
# IMU orientation processing:
TCP Read (IMU message)
├── JSON Parse
├── Extract quaternion data
├── Convert to Euler angles
├── Update orientation indicators
└── Calculate tilt compensation
```

### Autonomous Behavior VIs

#### Path Planning VI

```labview
# Simple waypoint navigation:
Array of waypoints (X, Y coordinates)
├── Calculate path segments
├── Generate velocity commands
├── Send to robot
└── Monitor progress with sensor feedback
```

#### Obstacle Avoidance VI

```labview
# Reactive obstacle avoidance:
LiDAR scan data input
├── Process scan ranges
├── Identify obstacle sectors
├── Calculate avoidance trajectory
├── Generate strafing commands
└── Update robot velocity
```

## Network Communication Setup

### TCP/IP Socket Configuration

#### LabVIEW TCP Client

```labview
# TCP Client VI Structure:
TCP Open Connection (IP: robot_ip, Port: 8765)
├── While Loop:
│   ├── TCP Write (JSON command)
│   ├── TCP Read (JSON response)
│   └── Parse response data
└── TCP Close Connection
```

#### Message Format

**Command Message**:
```json
{
  "type": "velocity_command",
  "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
}
```

**Sensor Data Response**:
```json
{
  "type": "sensor_data",
  "laser_scan": {"ranges": [2.1, 2.3, ...]},
  "imu": {"orientation": {"w": 0.98, "x": 0.0, "y": 0.0, "z": 0.17}},
  "odometry": {"pose": {"x": 1.2, "y": 0.8}}
}
```

### HTTP API Integration

#### REST API Calls

```labview
# HTTP POST for robot control:
HTTP POST Request:
├── URL: http://robot_ip:5678/webhook/robot-control
├── Headers: Content-Type: application/json
├── Body: {"command": "forward", "speed": 0.5}
└── Parse response
```

## Advanced Integration Examples

### Real-time Data Dashboard

#### Dashboard Components

**Numeric Indicators**:
- Robot position (X, Y, Theta)
- Battery voltage and current
- Motor temperatures
- Sensor status indicators

**Graphs and Charts**:
- Real-time position trajectory
- LiDAR scan visualization
- IMU orientation over time
- Motor velocity profiles

**Control Elements**:
- Joystick emulation for manual control
- Preset movement buttons
- Autonomous mode toggles
- Emergency stop button

### Automated Testing Framework

#### Test Sequence VI

```labview
# Automated test execution:
Test case array:
├── Movement accuracy tests
├── Sensor calibration verification
├── Battery discharge testing
├── Emergency stop response time
└── Communication latency measurement
```

#### Data Logging VI

```labview
# Comprehensive data logging:
Timestamp + Test data → CSV file
├── Robot state variables
├── Sensor readings
├── Control inputs
├── Performance metrics
└── Error conditions
```

### Vision-Guided Manipulation

#### Computer Vision Integration

```labview
# Vision processing pipeline:
Camera image acquisition
├── Color thresholding
├── Blob detection
├── Object classification
├── Position calculation
├── Coordinate transformation
└── Robot command generation
```

## Hardware Integration

### NI DAQ Device Integration

#### Analog Input Configuration

```labview
# DAQmx configuration for sensor readings:
DAQmx Create Channel (Analog Input)
├── Voltage range: -10V to 10V
├── Sampling rate: 1000 Hz
├── Input terminals: ai0, ai1, ai2
└── Configure timing and triggering
```

#### Digital Output Control

```labview
# DAQmx configuration for actuator control:
DAQmx Create Channel (Digital Output)
├── Output lines: port0/line0:7
├── Logic levels: 5V TTL
├── Update rate: 100 Hz
└── Enable hardware timing
```

### Serial Communication

#### Arduino Integration

```labview
# Serial communication with Arduino:
VISA Configure Serial Port:
├── Baud rate: 115200
├── Data bits: 8
├── Parity: None
├── Stop bits: 1
└── Flow control: None

VISA Write: Motor control commands
VISA Read: Sensor data responses
```

## Development Workflow

### VI Development Best Practices

#### Code Organization

```
Project Structure:
├── Main Application.vi
├── SubVIs/
│   ├── Robot Control.vi
│   ├── Sensor Processing.vi
│   ├── Data Logging.vi
│   └── Utilities.vi
├── Type Definitions/
│   ├── Robot State.ctl
│   └── Sensor Data.ctl
└── Configuration Files/
    └── System Config.ini
```

#### Error Handling

```labview
# Comprehensive error handling:
Error Cluster Wire:
├── Case Structure for error codes
├── Error logging to file
├── User notification dialogs
└── Graceful shutdown procedures
```

### Testing and Validation

#### Unit Testing

```labview
# VI unit testing framework:
Test Case Structure:
├── Input parameter sets
├── Expected output validation
├── Boundary condition testing
├── Performance benchmarking
└── Memory leak detection
```

#### Integration Testing

```labview
# System integration tests:
Hardware-in-the-loop testing:
├── Robot movement validation
├── Sensor data accuracy verification
├── Communication latency measurement
├── Emergency stop response testing
└── Battery life assessment
```

## Deployment and Distribution

### Application Building

#### EXE Creation

```labview
# Build application for distribution:
Tools → Build Application:
├── Destination directory
├── Application icon
├── VI inclusion settings
├── Runtime engine inclusion
└── Installer creation options
```

#### Shared Library Generation

```labview
# DLL creation for C++ integration:
Tools → Build Shared Library:
├── Exported VIs selection
├── Function prototype generation
├── Platform-specific compilation
└── Header file creation
```

## Troubleshooting LabVIEW Integration

### Common Issues

#### Network Communication Problems

**Issue**: TCP connection fails
```labview
# Diagnostic steps:
1. Ping robot IP address
2. Check firewall settings
3. Verify port accessibility (8765, 5678)
4. Test with simple TCP client
```

**Issue**: WebSocket connection drops
```labview
# Solution steps:
1. Implement connection retry logic
2. Add heartbeat/keepalive mechanism
3. Handle network timeouts gracefully
4. Monitor connection status continuously
```

#### Data Parsing Issues

**Issue**: JSON parsing errors
```labview
# Debugging approach:
1. Log raw received data
2. Validate JSON structure
3. Check for encoding issues
4. Implement robust error handling
```

#### Performance Issues

**Issue**: Slow VI execution
```labview
# Optimization steps:
1. Profile VI execution time
2. Optimize data flow
3. Reduce unnecessary operations
4. Implement asynchronous processing
```

## Advanced Features

### Multi-Robot Coordination

#### Fleet Management VI

```labview
# Multi-robot coordination:
Robot array management:
├── Individual robot status tracking
├── Task allocation algorithms
├── Collision avoidance coordination
├── Formation control implementation
└── Centralized command distribution
```

### Machine Learning Integration

#### Vision Classification

```labview
# Neural network integration:
Vision Assistant Express VI:
├── Image preprocessing
├── Feature extraction
├── Classification model loading
├── Real-time inference
└── Result visualization
```

## Support and Resources

### Documentation References
- [LabVIEW Help Documentation](https://www.ni.com/docs/en-US/bundle/labview/page/lvhelp.html)
- [NI Robotics Module](https://www.ni.com/docs/en-US/bundle/robotics-module/page/robotics-module.html)
- [ROS 2 LabVIEW Integration Examples](https://github.com/ni/ros2-labview)

### Community Resources
- [NI Community Forums](https://forums.ni.com/)
- [LabVIEW User Groups](https://forums.ni.com/t5/LabVIEW/bd-p/170)
- [ROS 2 Community](https://discourse.ros.org/)

### Technical Support
- [National Instruments Support](https://www.ni.com/support)
- [GitHub Issues](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues)
- [ROS Answers](https://answers.ros.org/)

## Best Practices

### Development Guidelines

1. **Modular Design**: Break complex applications into reusable subVIs
2. **Error Handling**: Implement comprehensive error checking and logging
3. **Performance Optimization**: Use profiling tools to identify bottlenecks
4. **Documentation**: Document all VIs with clear descriptions and usage examples
5. **Version Control**: Use Git for VI versioning and collaboration

### Safety Considerations

1. **Emergency Stop Integration**: Always include emergency stop functionality
2. **Hardware Protection**: Implement over-current and over-voltage protection
3. **Fail-Safe Design**: Ensure safe behavior during communication failures
4. **User Training**: Provide clear instructions for safe operation

### Maintenance Procedures

1. **Regular Calibration**: Periodic sensor and actuator calibration
2. **Software Updates**: Keep LabVIEW and robot software current
3. **Hardware Inspection**: Regular mechanical and electrical system checks
4. **Documentation Updates**: Maintain current integration documentation

---

*This LabVIEW integration guide provides comprehensive instructions for connecting National Instruments LabVIEW with the Autonomous Mobile Manipulator robot. The integration enables powerful visual programming capabilities for robot control, data analysis, and automation.*
