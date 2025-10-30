# Project Overview
**Version:** 1.0

## 1. Introduction

This document provides a high-level overview of the LKS Robot Project, a comprehensive and containerized ROS2-based platform for a versatile mobile manipulator. The project is designed for simulation in Gazebo and deployment on real hardware, with a focus on modularity, flexibility, and ease of use.

It integrates advanced robotics capabilities, including navigation, manipulation, and automation, all orchestrated through a variety of modern interfaces.

## 2. Key Features

- **Mobile Manipulation:** Combines a mobile base with a manipulator arm for pick-and-place tasks in a dynamic environment.
- **Autonomous Navigation:** Utilizes the ROS2 Navigation Stack (Nav2) for path planning, obstacle avoidance, and autonomous patrol missions.
- **Advanced Manipulation:** Employs MoveIt 2 for complex arm trajectory planning, kinematics, and gripper control.
- **Simulation & Reality:** Supports both realistic Gazebo simulation and real-world hardware deployment with a unified launch system.
- **Flexible Control Interfaces:** Offers multiple points of control and integration:
    - **n8n Workflow Automation:** A powerful, low-code platform for designing complex automation sequences (e.g., "patrol until an object is found, then pick and place it").
    - **REST API:** A simple HTTP interface for scripting and high-level control.
    - **WebSockets:** For real-time data streaming and interactive control.
    - **MQTT Bridge:** For integration with IoT ecosystems and devices.
- **Containerized Deployment:** Uses Docker and Docker Compose for consistent, reproducible, and isolated development and production environments.

## 3. Technology Stack

- **Robotics Framework:** ROS 2 (Humble)
- **Simulation:** Gazebo
- **Containerization:** Docker, Docker Compose
- **Navigation:** ROS 2 Navigation Stack (Nav2)
- **Manipulation:** MoveIt 2
- **Workflow Automation:** n8n.io
- **Primary Language:** Python 3
- **Robot Modeling:** URDF, xacro

## 4. Software Architecture

The architecture is centered around a ROS 2 workspace (`ros2_ws`) containing several modular packages.

- **`my_robot_description`:** Defines the robot's physical structure (URDF) and configuration for controllers.
- **`my_robot_bringup`:** Contains launch files to start the robot in various modes (simulation, real hardware, visualization).
- **`my_robot_navigation`:** Manages the Nav2 stack, including configuration for localization, planning, and obstacle avoidance.
- **`my_robot_manipulation`:** Manages the MoveIt 2 stack, including arm planning, gripper control, and pick-place execution.
- **`my_robot_automation`:** The core automation package. It acts as a bridge between ROS 2 and external systems, hosting the REST, WebSocket, and MQTT servers, and exposing ROS 2 Actions/Services for robot capabilities.

External control systems interact with the `my_robot_automation` package, which translates high-level commands into specific ROS 2 messages, service calls, and action goals.
