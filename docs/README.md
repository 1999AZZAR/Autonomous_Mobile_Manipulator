# Documentation Index

Complete documentation for the Autonomous Mobile Manipulator system.

## 🚀 Quick Reference

- **[QUICK_COMMANDS.md](./QUICK_COMMANDS.md)** - Essential commands at your fingertips

## 📚 Documentation Structure

### 🏗️ System Architecture (Root Level)

- **[SYSTEM_ARCHITECTURE.md](./SYSTEM_ARCHITECTURE.md)**
  - Complete system architecture overview
  - Web UI vs N8N decision matrices
  - Deployment configurations
  - Operational modes
  - Communication flows

- **[ARCHITECTURE_UPDATE_SUMMARY.md](./ARCHITECTURE_UPDATE_SUMMARY.md)**
  - Recent architecture changes
  - Feature comparison tables
  - Migration path
  - Performance improvements

- **[DOCUMENTATION_UPDATE_SUMMARY.md](./DOCUMENTATION_UPDATE_SUMMARY.md)**
  - Documentation organization history
  - Update summaries
  - Change tracking

---

### 💻 Software Documentation

#### [software/web-interface/](./software/web-interface/)
Complete Web UI documentation (Primary Control Interface)

- **Controls & Features**: All available controls and operations
- **Path Planning**: Waypoint management and autonomous navigation
- **IMU Calibration**: Sensor calibration procedures
- **Activity Stream**: Real-time command feedback
- **Updates History**: Feature additions and improvements

#### [software/ros2/](./software/ros2/)
ROS2 implementation and optimization

- **Performance Optimization**: Complete tuning guide
- **QoS Configuration**: Quality of Service profiles
- **CycloneDDS Settings**: DDS optimization
- **Monitoring & Watchdog**: Reliability systems
- **Performance Testing**: Test procedures and benchmarks

#### [software/](./software/)
General software documentation

- **Control Systems**: Robot control architecture
- **Integration**: Component integration details

---

### 🔌 API Documentation

#### [api/](./api/)
REST API and service documentation

- **[API_DOCUMENTATION.md](./api/API_DOCUMENTATION.md)**: Complete API reference
- **[API_VERIFICATION_REPORT.md](./api/API_VERIFICATION_REPORT.md)**: API testing results
- **Endpoint Documentation**: All available endpoints
- **Request/Response Examples**: Usage examples
- **Integration Guide**: How to use the API

---

### 🔄 Workflow Documentation

#### [workflow/](./workflow/)
N8N workflow system documentation

- **Connection Fix Report**: IPv4/IPv6 issues resolved
- **Workflow Cleanup Report**: Organization and standardization
- **Workflow Fix Report**: Parameter fixes and improvements

#### [workflows/](./workflows/)
Workflow management and usage

- **Workflow Management**: Import/export/manage workflows
- **38 Pre-built Workflows**: Ready-to-use automation patterns

---

### 🔧 Hardware Documentation

#### [hardware/](./hardware/)
Physical hardware specifications and assembly

- **[GPIO_CONTROL_SETUP.md](./hardware/GPIO_CONTROL_SETUP.md)**: Direct GPIO control setup and usage
- **[SENSOR_WIRING.md](./hardware/SENSOR_WIRING.md)**: Complete sensor wiring guide
- **[MPU6050_SETUP.md](./hardware/MPU6050_SETUP.md)**: IMU sensor setup and configuration
- **[MODE_SELECTION.md](./hardware/MODE_SELECTION.md)**: Hardware vs simulation mode selection
- **[HARDWARE_MODE_UPDATE.md](./hardware/HARDWARE_MODE_UPDATE.md)**: Hardware mode feature updates
- **[HARDWARE_ASSEMBLY_GUIDE.md](./hardware/HARDWARE_ASSEMBLY_GUIDE.md)**: Complete assembly instructions
- **[RASPBERRY_PI_PINOUTS.md](./hardware/RASPBERRY_PI_PINOUTS.md)**: GPIO mapping and connections
- **GPIO Test Script**: Hardware testing utilities

---

### 🚀 Deployment Documentation

#### [deployment/](./deployment/)
Production deployment guides

- **[raspberry_pi_setup.md](./deployment/raspberry_pi_setup.md)**: Complete Pi 5 setup guide
- **[ROS2_RELIABILITY_README.md](./deployment/ROS2_RELIABILITY_README.md)**: ROS2 reliability and watchdog systems
- **Docker Configuration**: Container deployment
- **Network Setup**: Connectivity configuration
- **Security**: Production hardening

---

### 📦 Installation Documentation

#### [installation/](./installation/)
System installation procedures

- **[FIRST_RUN.md](./installation/FIRST_RUN.md)**: First-time setup from scratch
- **[QUICK_START.md](./installation/QUICK_START.md)**: Quick start guide
- **[QUICK_START_GPIO.md](./installation/QUICK_START_GPIO.md)**: GPIO control quick start (3 steps)
- **[STARTUP_GUIDE.md](./installation/STARTUP_GUIDE.md)**: Complete startup procedures
- **[SETUP_GUIDE.md](./installation/SETUP_GUIDE.md)**: Detailed setup guide
- **[README_SETUP.md](./installation/README_SETUP.md)**: Setup overview and portable paths
- **Prerequisites**: System requirements
- **Docker Setup**: Container installation
- **ROS2 Installation**: ROS2 Iron setup
- **Dependency Management**: Package installation

---

### 🛠️ Development Documentation

#### [development/](./development/)
Developer guides and workflows

#### Feature Updates & Changes:
- **[CHANGES_SUMMARY.md](./development/CHANGES_SUMMARY.md)**: GPIO control implementation summary
- **[IMU_FIX_SUMMARY.md](./development/IMU_FIX_SUMMARY.md)**: IMU data display fixes
- **[IMU_WEB_INTEGRATION_SUMMARY.md](./development/IMU_WEB_INTEGRATION_SUMMARY.md)**: IMU web integration
- **[MPU6050_INTEGRATION_SUMMARY.md](./development/MPU6050_INTEGRATION_SUMMARY.md)**: MPU6050 sensor integration
- **[SHARP_SENSOR_UPDATE_SUMMARY.md](./development/SHARP_SENSOR_UPDATE_SUMMARY.md)**: Sharp IR sensor updates
- **[WEB_INTERFACE_READY_SUMMARY.md](./development/WEB_INTERFACE_READY_SUMMARY.md)**: Web interface completion
- **[IMPLEMENTATION_SUMMARY.md](./development/IMPLEMENTATION_SUMMARY.md)**: Implementation details

#### System Updates:
- **[SYSTEM_STARTUP_COMPLETE_SUMMARY.md](./development/SYSTEM_STARTUP_COMPLETE_SUMMARY.md)**: Startup system implementation
- **[STARTUP_SCRIPTS_UPDATE.md](./development/STARTUP_SCRIPTS_UPDATE.md)**: Startup scripts updates
- **[RUN_SH_SUMMARY.md](./development/RUN_SH_SUMMARY.md)**: Run script improvements
- **[PORTABLE_PATHS_UPDATE.md](./development/PORTABLE_PATHS_UPDATE.md)**: Portable paths implementation
- **[PROXY_ARCHITECTURE_SUMMARY.md](./development/PROXY_ARCHITECTURE_SUMMARY.md)**: Proxy architecture
- **[GIT_RECOVERY_SUMMARY.md](./development/GIT_RECOVERY_SUMMARY.md)**: Git recovery procedures

#### General:
- **Development Environment**: Setup for developers
- **Code Structure**: Project organization
- **Testing**: Test procedures
- **Contributing**: Contribution guidelines

---

### ❓ Troubleshooting Documentation

#### [troubleshooting/](./troubleshooting/)
Problem diagnosis and solutions

- **[IMU_TROUBLESHOOTING.md](./troubleshooting/IMU_TROUBLESHOOTING.md)**: IMU sensor troubleshooting guide
- **[QUICK_IMU_TEST.md](./troubleshooting/QUICK_IMU_TEST.md)**: Quick IMU testing procedures
- **Common Issues**: Frequently encountered problems
- **Error Messages**: Error code reference
- **Diagnostic Tools**: Debugging procedures
- **FAQ**: Frequently asked questions

---

### 🔗 LabVIEW Integration

#### [labview-integration/](./labview-integration/)
LabVIEW system integration (if applicable)

- **Integration Guide**: Connecting LabVIEW systems
- **Communication Protocols**: Data exchange methods

---

## 🚦 Quick Start Guides

### For Operators

1. **[Web Interface](./software/web-interface/)** - Primary control interface
2. **[Hardware Reference](./hardware/RASPBERRY_PI_PINOUTS.md)** - GPIO and connections
3. **[Troubleshooting](./troubleshooting/)** - Problem resolution

### For Developers

1. **[System Architecture](./SYSTEM_ARCHITECTURE.md)** - System design
2. **[ROS2 Optimization](./software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md)** - Performance tuning
3. **[API Documentation](./api/)** - Integration details
4. **[Development Guide](./development/)** - Development setup

### For Deployment

1. **[Raspberry Pi Setup](./deployment/raspberry_pi_setup.md)** - Hardware deployment
2. **[Installation Guide](./installation/)** - Software installation
3. **[Workflow Setup](./workflows/)** - Automation configuration

---

## 📊 Documentation by Feature

### Control & Monitoring

- **Web Interface**: [software/web-interface/](./software/web-interface/)
- **API Endpoints**: [api/](./api/)
- **System Status**: [SYSTEM_ARCHITECTURE.md](./SYSTEM_ARCHITECTURE.md)

### Automation

- **N8N Workflows**: [workflow/](./workflow/) and [workflows/](./workflows/)
- **Path Planning**: [software/web-interface/WEB_INTERFACE_PATH_PLANNING.md](./software/web-interface/WEB_INTERFACE_PATH_PLANNING.md)
- **Autonomous Operations**: [software/CONTROL_SYSTEMS.md](./software/CONTROL_SYSTEMS.md)

### Performance & Reliability

- **ROS2 Optimization**: [software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md](./software/ros2/ROS2_PERFORMANCE_OPTIMIZATION.md)
- **Watchdog System**: [software/ros2/](./software/ros2/)
- **Performance Testing**: [software/ros2/README.md](./software/ros2/README.md)

### Hardware & Sensors

- **GPIO Pinouts**: [hardware/RASPBERRY_PI_PINOUTS.md](./hardware/RASPBERRY_PI_PINOUTS.md)
- **Assembly Guide**: [hardware/HARDWARE_ASSEMBLY_GUIDE.md](./hardware/HARDWARE_ASSEMBLY_GUIDE.md)
- **IMU Calibration**: [software/web-interface/IMU_CALIBRATION_FEATURE.md](./software/web-interface/IMU_CALIBRATION_FEATURE.md)

---

## 🎯 Documentation by Role

### Robot Operator

**Essential Reading:**
1. Web Interface documentation
2. Hardware pinout reference
3. Basic troubleshooting

**Optional:**
- Workflow management
- Advanced features

### System Administrator

**Essential Reading:**
1. Deployment guide
2. ROS2 optimization
3. Monitoring and watchdog
4. Troubleshooting

**Optional:**
- Development setup
- API documentation

### Software Developer

**Essential Reading:**
1. System architecture
2. API documentation
3. ROS2 optimization
4. Development guide

**Optional:**
- Workflow integration
- Hardware specifications

### Integration Engineer

**Essential Reading:**
1. API documentation
2. System architecture
3. Communication protocols

**Optional:**
- LabVIEW integration
- Custom workflow creation

---

## 📝 Documentation Standards

All documentation follows these standards:

- **Markdown Format**: All docs are `.md` files
- **No Buzzwords**: Clear, technical language
- **No Emojis in Content**: (except this README for navigation)
- **Complete Examples**: Working code snippets
- **Troubleshooting Sections**: Problem-solution format
- **Quick Links**: Fast navigation
- **Update Dates**: Last updated timestamps

---

## 🔄 Recent Updates

**2025-11-11:**
- ✅ All documentation moved to `docs/` folder
- ✅ Hardware docs: GPIO control, sensor wiring, IMU setup
- ✅ Installation docs: Quick starts, setup guides
- ✅ Development docs: All change summaries and update logs
- ✅ Troubleshooting docs: IMU troubleshooting guides
- ✅ API docs: Complete API documentation
- ✅ Updated main docs README with all new files
- ✅ Created QUICK_COMMANDS.md reference guide

**2025-11-10:**
- ✅ Documentation reorganized into logical folders
- ✅ Web Interface docs moved to `software/web-interface/`
- ✅ ROS2 docs moved to `software/ros2/`
- ✅ Created comprehensive README files for each section
- ✅ Added Activity Stream documentation
- ✅ Added ROS2 Performance Optimization guide
- ✅ Updated system architecture documentation

---

## 📖 Contributing to Documentation

When adding new documentation:

1. **Choose the Right Directory**:
   - Software features → `software/`
   - Hardware specs → `hardware/`
   - API changes → `api/`
   - Workflows → `workflow/` or `workflows/`
   - Deployment → `deployment/`

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

---

## 🔍 Finding Documentation

**Can't find what you need?**

1. Check the appropriate subfolder above
2. Search for keywords in file names
3. Review related documentation links
4. Check the troubleshooting section

**Still need help?**
- Review system logs
- Check Activity Stream in Web UI
- Examine API responses

---

**All documentation is organized, comprehensive, and kept up-to-date!**
