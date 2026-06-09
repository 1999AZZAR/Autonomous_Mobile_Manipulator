# Documentation Index

Complete documentation for the Autonomous Mobile Manipulator distributed system (Raspberry Pi + Arduino Mega).

## Quick Reference

- **[QUICK_COMMANDS.md](./QUICK_COMMANDS.md)** -- Essential commands reference

## Documentation Structure

### System Architecture

- **[SYSTEM_ARCHITECTURE.md](./SYSTEM_ARCHITECTURE.md)** -- System architecture, web interface, API, deployment configs, communication flows
- **[HARDWARE_ARCHITECTURE_UPDATE.md](./HARDWARE_ARCHITECTURE_UPDATE.md)** -- Hardware architecture changes and updates
- **[DOCUMENTATION_CLEANUP_SUMMARY.md](./DOCUMENTATION_CLEANUP_SUMMARY.md)** -- Documentation organization history

---

### Software Documentation

#### [software/web-interface/](./software/web-interface/)
Web UI documentation (primary control interface).

- Controls and features for all robot operations
- Path planning with waypoint management
- IMU calibration procedures
- Activity stream for real-time command feedback

#### [software/ros2/](./software/ros2/)
ROS2 high-level control and Arduino Mega integration.

- Distributed architecture: ROS2 + Arduino Mega coordination
- Performance optimization and tuning
- Serial communication protocols
- Sensor integration and data processing

#### [software/](./software/)
General software documentation.

- Control systems architecture
- Component integration details

---

### API Documentation

#### [api/](./api/)
REST API and service documentation.

- **[API_DOCUMENTATION.md](./api/API_DOCUMENTATION.md)** -- Complete API reference with endpoints and examples
- **[API_VERIFICATION_REPORT.md](./api/API_VERIFICATION_REPORT.md)** -- API testing and validation results

---

### Hardware Documentation

#### [hardware/](./hardware/)
Physical hardware specifications and assembly for the Pi-Mega distributed architecture.

- **[README.md](./hardware/README.md)** -- Hardware overview with Pi-Mega integration
- **[HARDWARE_ASSEMBLY_GUIDE.md](./hardware/HARDWARE_ASSEMBLY_GUIDE.md)** -- Assembly instructions
- **[SENSOR_WIRING.md](./hardware/SENSOR_WIRING.md)** -- Distributed sensor wiring (Mega + Pi)
- **[RASPBERRY_PI_PINOUTS.md](./hardware/RASPBERRY_PI_PINOUTS.md)** -- GPIO mapping and Pi-Mega communication
- **[PG23_MOTOR_CONNECTION_GUIDE.md](./hardware/PG23_MOTOR_CONNECTION_GUIDE.md)** -- Motor specifications and Mega control
- **[PG23_MOTOR_PROTOCOL.md](./hardware/PG23_MOTOR_PROTOCOL.md)** -- Motor communication protocol
- **[PG23_MOTOR_TROUBLESHOOTING.md](./hardware/PG23_MOTOR_TROUBLESHOOTING.md)** -- Motor troubleshooting
- **[MOTOR_PIN_ASSIGNMENTS.md](./hardware/MOTOR_PIN_ASSIGNMENTS.md)** -- Pin assignments
- **[MOTOR_SPECIFICATIONS.md](./hardware/MOTOR_SPECIFICATIONS.md)** -- Motor specs
- **[MOTOR_POWER_CONTROL_SOLUTION.md](./hardware/MOTOR_POWER_CONTROL_SOLUTION.md)** -- Power control
- **[MPU6050_SETUP.md](./hardware/MPU6050_SETUP.md)** -- IMU setup
- **[L298N_TROUBLESHOOTING.md](./hardware/L298N_TROUBLESHOOTING.md)** -- Motor driver troubleshooting

---

### Deployment Documentation

#### [deployment/](./deployment/)
Production deployment guides for the distributed system.

- **[raspberry_pi_setup.md](./deployment/raspberry_pi_setup.md)** -- Pi 5 + Arduino Mega setup guide
- **[ROS2_RELIABILITY_README.md](./deployment/ROS2_RELIABILITY_README.md)** -- ROS2 reliability and watchdog systems

---

### Installation Documentation

#### [installation/](./installation/)
System installation procedures.

- **[FIRST_RUN.md](./installation/FIRST_RUN.md)** -- First-time setup from scratch
- **[QUICK_START.md](./installation/QUICK_START.md)** -- Quick start guide
- **[QUICK_START_GPIO.md](./installation/QUICK_START_GPIO.md)** -- GPIO control quick start
- **[STARTUP_GUIDE.md](./installation/STARTUP_GUIDE.md)** -- Complete startup procedures
- **[SETUP_GUIDE.md](./installation/SETUP_GUIDE.md)** -- Detailed setup guide
- **[README_SETUP.md](./installation/README_SETUP.md)** -- Setup overview and portable paths

---

### Development Documentation

#### [development/](./development/)
Developer guides and workflows.

- **[PORTABLE_PATHS_UPDATE.md](./development/PORTABLE_PATHS_UPDATE.md)** -- Portable paths implementation
- **[STARTUP_SCRIPTS_UPDATE.md](./development/STARTUP_SCRIPTS_UPDATE.md)** -- Startup scripts updates

---

### Troubleshooting Documentation

#### [troubleshooting/](./troubleshooting/)
Problem diagnosis and solutions.

- **[IMU_TROUBLESHOOTING.md](./troubleshooting/IMU_TROUBLESHOOTING.md)** -- IMU sensor troubleshooting
- **[QUICK_IMU_TEST.md](./troubleshooting/QUICK_IMU_TEST.md)** -- Quick IMU testing procedures

---

### LabVIEW Integration

#### [labview-integration/](./labview-integration/)
LabVIEW system integration (if applicable).

---

## Quick Start Guides

### For Operators

1. **[Web Interface](./software/web-interface/)** -- Primary control interface
2. **[Hardware Reference](./hardware/RASPBERRY_PI_PINOUTS.md)** -- GPIO and connections
3. **[Troubleshooting](./troubleshooting/)** -- Problem resolution

### For Developers

1. **[System Architecture](./SYSTEM_ARCHITECTURE.md)** -- System design
2. **[ROS2 Optimization](./software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md)** -- Performance tuning
3. **[API Documentation](./api/)** -- Integration details
4. **[Development Guide](./development/)** -- Development setup

### For Deployment

1. **[Raspberry Pi Setup](./deployment/raspberry_pi_setup.md)** -- Hardware deployment
2. **[Installation Guide](./installation/)** -- Software installation

---

## Documentation by Feature

### Control and Monitoring

- Web Interface: [software/web-interface/](./software/web-interface/)
- API Endpoints: [api/](./api/)
- System Status: [SYSTEM_ARCHITECTURE.md](./SYSTEM_ARCHITECTURE.md)

### Automation

- API Automation: [api/](./api/)
- Path Planning: [software/web-interface/WEB_INTERFACE_PATH_PLANNING.md](./software/web-interface/WEB_INTERFACE_PATH_PLANNING.md)
- Autonomous Operations: [software/CONTROL_SYSTEMS.md](./software/CONTROL_SYSTEMS.md)

### Performance and Reliability

- ROS2 Optimization: [software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md](./software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md)
- Watchdog System: [software/ros2/](./software/ros2/)

### Hardware and Sensors

- GPIO Pinouts: [hardware/RASPBERRY_PI_PINOUTS.md](./hardware/RASPBERRY_PI_PINOUTS.md)
- Assembly Guide: [hardware/HARDWARE_ASSEMBLY_GUIDE.md](./hardware/HARDWARE_ASSEMBLY_GUIDE.md)
- IMU Calibration: [software/web-interface/IMU_CALIBRATION_FEATURE.md](./software/web-interface/IMU_CALIBRATION_FEATURE.md)

---

## Documentation Standards

All documentation follows these standards:

- **Markdown Format**: All docs are `.md` files
- **No Buzzwords**: Clear, technical language
- **No Emojis in Content**: Text-only for clarity
- **Complete Examples**: Working code snippets
- **Troubleshooting Sections**: Problem-solution format
- **Quick Links**: Fast navigation

---

## Contributing to Documentation

When adding new documentation:

1. **Choose the Right Directory**:
   - Software features: `software/`
   - Hardware specs: `hardware/`
   - API changes: `api/`
   - Deployment: `deployment/`

2. **Follow Naming Convention**:
   - Use UPPERCASE_WITH_UNDERSCORES.md
   - Be descriptive: `FEATURE_NAME_GUIDE.md`

3. **Include These Sections**:
   - Overview
   - Purpose
   - Usage examples
   - Troubleshooting
   - Related documentation links

4. **Update This README**:
   - Add link in appropriate section
   - Update recent changes
