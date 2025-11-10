# Documentation Index

Complete documentation for the Autonomous Mobile Manipulator system.

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

- **API Verification Report**: Complete API testing results
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

- **Hardware Assembly Guide**: Complete assembly instructions
- **Raspberry Pi Pinouts**: GPIO mapping and connections
- **Component Specifications**: Sensor and actuator details
- **GPIO Test Script**: Hardware testing utilities

---

### 🚀 Deployment Documentation

#### [deployment/](./deployment/)
Production deployment guides

- **Raspberry Pi Setup**: Complete Pi 5 setup guide
- **Docker Configuration**: Container deployment
- **Network Setup**: Connectivity configuration
- **Security**: Production hardening

---

### 📦 Installation Documentation

#### [installation/](./installation/)
System installation procedures

- **Prerequisites**: System requirements
- **Docker Setup**: Container installation
- **ROS2 Installation**: ROS2 Iron setup
- **Dependency Management**: Package installation

---

### 🛠️ Development Documentation

#### [development/](./development/)
Developer guides and workflows

- **Development Environment**: Setup for developers
- **Code Structure**: Project organization
- **Testing**: Test procedures
- **Contributing**: Contribution guidelines

---

### ❓ Troubleshooting Documentation

#### [troubleshooting/](./troubleshooting/)
Problem diagnosis and solutions

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
