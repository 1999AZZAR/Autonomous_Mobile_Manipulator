# Hardware Setup Guide

This guide provides comprehensive instructions for assembling and configuring the hardware components of the Autonomous Mobile Manipulator robot.

## Table of Contents

- [Robot Overview](#robot-overview)
- [Bill of Materials](#bill-of-materials)
  - [Core Components](#core-components)
  - [Mechanical Components](#mechanical-components)
  - [Electrical Components](#electrical-components)
- [Assembly Instructions](#assembly-instructions)
  - [Step 1: Base Platform Assembly](#step-1-base-platform-assembly)
  - [Step 2: Electronics Installation](#step-2-electronics-installation)
  - [Step 3: Picker System Assembly](#step-3-picker-system-assembly)
  - [Step 4: Container System Assembly](#step-4-container-system-assembly)
  - [Step 5: Wiring and Connections](#step-5-wiring-and-connections)
- [Testing and Verification](#testing-and-verification)
  - [Motor Testing](#motor-testing)
  - [Sensor Verification](#sensor-verification)
  - [Picker System Testing](#picker-system-testing)
  - [Integration Testing](#integration-testing)
- [Safety Considerations](#safety-considerations)
  - [Electrical Safety](#electrical-safety)
  - [Mechanical Safety](#mechanical-safety)
  - [Operational Safety](#operational-safety)
- [Troubleshooting Hardware Issues](#troubleshooting-hardware-issues)
  - [Motor Control Problems](#motor-control-problems)
  - [Sensor Issues](#sensor-issues)
- [Maintenance Procedures](#maintenance-procedures)
  - [Regular Maintenance](#regular-maintenance)
  - [Component Replacement](#component-replacement)
  - [Calibration Procedures](#calibration-procedures)
- [Support and Resources](#support-and-resources)
  - [Documentation References](#documentation-references)
  - [Component Datasheets](#component-datasheets)
  - [Community Resources](#community-resources)

## Robot Overview

The LKS Autonomous Mobile Manipulator consists of:
- **Mobile Base**: 3-wheeled omnidirectional platform (hexagonal shape)
- **Manipulator System**: Servo-based picker with 4 components (gripper, tilt, neck, base) plus lifter
- **Sensors**: RPLIDAR A1 (380°), Microsoft USB camera, distance sensors (3x), line sensor, IMU
- **Computing**: Raspberry Pi 5 with Ubuntu Server and Docker
- **Container System**: 4-container material transport system
- **Power System**: Battery management and distribution

## Bill of Materials

### Core Components

| Component | Model/Specification | Quantity | Purpose |
|-----------|-------------------|----------|---------|
| Raspberry Pi 5 | 8GB RAM, Ubuntu Server | 1 | Main computer and ROS2 processing |
| Motors | PG23 built-in encoder, 12V, 15.5k RPM, 7 PPR | 4 | 3x omni wheels + 1x lifter (built-in encoders only, require L298N drivers) |
| L298N Motor Drivers | Dual H-Bridge motor driver modules | 4 | Motor control for PG23 motors |
| Omni Wheels | 75mm diameter | 3 | Omnidirectional movement (Back, Front Left, Front Right) |
| RPLIDAR A1 | 380° scanning | 1 | Laser-based obstacle detection and mapping |
| Microsoft USB Camera | Standard webcam | 1 | Object recognition and computer vision |
| Distance Sensors | Laser-based (3x) | 3 | Front, Back Left, Back Right obstacle detection |
| Line Sensor | IR-based line following | 1 | Line-based navigation |
| IMU Sensor | MPU6050/BNO055 | 1 | Orientation and motion sensing |
| Servo Motors | MG996R or similar | 5 | Picker system actuation |
| Gripper Servo | Standard servo | 1 | Object grasping |
| Lifter Motor | PG23 built-in encoder motor, 12V, 15.5k RPM, 7 PPR | 1 | Vertical lifting mechanism |
| Battery | 12V LiPo 5000mAh | 1 | Power supply |
| Voltage Regulator | 5V/12V buck converter | 2 | Power management |

| Container System | 4-compartment | 1 | Material transport and storage |

**Motor Documentation:**
- [PG23 Motor Connection Guide](PG23_MOTOR_CONNECTION_GUIDE.md) - Complete pinout, wiring, and connection instructions
- [Motor Specifications](MOTOR_SPECIFICATIONS.md) - Detailed motor and encoder specifications

### Mechanical Components

| Component | Specification | Quantity | Purpose |
|-----------|---------------|----------|---------|
| Base Plate | Hexagonal aluminum, 400mm diameter | 1 | Robot chassis |
| Motor Mounts | Aluminum brackets | 4 | 3x wheel motors + 1x lifter motor |
| Omni Wheel Hubs | Machined aluminum | 3 | Wheel attachment points |
| Lifter Frame | Aluminum extrusion, 300mm height | 1 | Vertical lifting mechanism |
| Picker Assembly | Servo bracket system | 1 | 5-servo manipulator mounting |
| Container Frame | 4-compartment aluminum | 1 | Material storage system |
| Sensor Mounts | Plastic/metal brackets | 8 | Sensor positioning (LiDAR, camera, IMU, distance sensors) |
| Battery Strap | Velcro/metal bracket | 1 | Battery secure mounting |

### Electrical Components

| Component | Specification | Quantity | Purpose |
|-----------|---------------|----------|---------|
| Power Distribution Board | Custom PCB with fuse protection | 1 | Power routing and protection |
| Wire Harness | 22AWG silicone wire | Various | Power and signal routing |
| Connectors | JST-XH and XT series | Various | Modular electrical connections |
| Fuse Holders | 10A automotive fuses | 4 | Circuit protection (3x motors + main) |
| Heat Sinks | Aluminum, driver-sized | 4 | Thermal management for L298N motor drivers |
| Terminal Blocks | 5.08mm pitch | 8 | Power distribution connections |
| GPIO Header | Raspberry Pi compatible | 1 | Hardware control interface |
| I2C/SPI Interfaces | Standard bus | 2 | IMU and sensor communication |

## Assembly Instructions

### Step 1: Base Platform Assembly

#### Materials Needed:
- Hexagonal base plate (400mm diameter)
- 3x omni wheel motor assemblies
- 1x lifter motor assembly
- Motor mounting brackets and hardware

#### Assembly Procedure:

1. **Position Motor Mounts on Hexagonal Base**:
   ```
   Omni wheel motors positioned at 120° intervals on hexagon perimeter:
   - Back Motor: Center rear position
   - Front Left Motor: 120° from back motor
   - Front Right Motor: 240° from back motor
   - Lifter Motor: Center of base plate
   ```

2. **Install Motor Mounts**:
   ```bash
   # Secure motor brackets to base plate with M4/M5 screws
   # Ensure motors are oriented correctly for omni-directional movement
   # Verify motor shaft heights are level
   ```

3. **Mount Omni Wheels and Lifter**:
   ```bash
   # Attach omni wheels to motor shafts with set screws
   # Install lifter column at center position
   # Test free rotation of all components
   ```

### Step 2: Electronics Installation

#### Power System Setup:

1. **Power Distribution Board Installation**:
   ```bash
   # Mount power distribution board centrally on base plate
   # Connect battery input with main fuse protection
   # Wire 12V outputs to PG23 motors (M+ terminals - 4 motors)
   # Wire 5V outputs to Raspberry Pi and servo power
   ```

2. **Motor Configuration**:
   ```bash
   # Connect 4 PG23 motors (3x omni wheels + 1x lifter)
   # Motors have built-in encoders only - require L298N motor drivers
   # Connect motor M+ to L298N OUT1, M- to L298N OUT2
   # Connect L298N VS to 12V power supply, VCC to 5V (logic)
   # Connect L298N DIR and PWM pins to Raspberry Pi GPIO
   # Connect encoder DATA(A) and DATA(B) pins to Raspberry Pi GPIO (read-only inputs)
   # Connect VIN to 5V for encoder power only
   ```

3. **Servo Power Distribution**:
   ```bash
   # Install servo power regulator (6V for servos)
   # Connect to 5 servo motors and gripper servo
   # Ensure proper grounding and decoupling capacitors
   ```

#### Sensor Integration:

1. **RPLIDAR A1 Installation**:
   ```bash
   # Mount RPLIDAR at 200mm height above base center
   # Connect USB cable to Raspberry Pi USB port
   # Ensure 380° scanning clearance around robot
   ```

2. **Microsoft USB Camera Setup**:
   ```bash
   # Mount camera on front-facing bracket
   # Connect USB cable to Raspberry Pi
   # Position for optimal object recognition field of view
   ```

3. **Distance Sensors (3x Laser)**:
   ```bash
   # Mount front sensor at 0° (forward)
   # Mount back-left sensor at 120° from front
   # Mount back-right sensor at 240° from front
   # Connect analog/digital interfaces to Raspberry Pi GPIO
   ```

4. **Line Sensor and IMU Installation**:
   ```bash
   # Mount line sensor at bottom center of base
   # Mount IMU at robot center of gravity
   # Connect I2C interfaces to Raspberry Pi
   ```

### Step 3: Picker System Assembly

#### Picker Components (4 main components + lifter):

1. **Lifter Installation**:
   ```bash
   # Mount lifter motor assembly to base center
   # Install vertical guide rails (300mm travel)
   # Attach picker bracket to lifter carriage
   # Connect motor driver and encoder feedback
   ```

2. **Gripper Servo Installation**:
   ```bash
   # Mount gripper servo to picker bracket
   # Install gripper fingers with spring return
   # Connect servo signal wire to Raspberry Pi PWM
   # Test open/close operation (0-180° range)
   ```

3. **Gripper Tilt Servo Installation**:
   ```bash
   # Mount tilt servo above gripper servo
   # Connect tilt bracket for angle adjustment
   # Wire PWM signal for tilt control
   # Calibrate tilt range (±45° from vertical)
   ```

4. **Gripper Neck Servo Installation**:
   ```bash
   # Install continuous rotation servo for neck
   # Mount on tilt bracket for forward/backward movement
   # Connect PWM signal for position control
   # Test extension/retraction movement
   ```

5. **Gripper Base Servo Installation**:
   ```bash
   # Mount base servo at top of picker assembly
   # Connect to neck servo for rotation
   # Wire PWM for 360° rotation capability
   # Verify smooth rotational movement
   ```

### Step 4: Container System Assembly

#### Container Installation:

1. **Container Frame Mounting**:
   ```bash
   # Mount 4-compartment container frame to base plate
   # Position around lifter column (left front, left back, right front, right back)
   # Secure with appropriate fasteners
   ```

2. **Container Mechanism Setup**:
   ```bash
   # Install load/unload mechanisms for each container
   # Wire solenoid or servo actuators to Raspberry Pi GPIO
   # Test load detection sensors
   ```

### Step 5: Wiring and Connections

#### Power Wiring:

```
Battery (+) → Main Fuse (15A) → Power Distribution Board
Power Distribution Board → PG23 Motors (12V, M+ terminals - 4 motors)
Power Distribution Board → Servo Power Regulator (6V)
Power Distribution Board → Raspberry Pi (5V)
Power Distribution Board → Sensors (5V)
All Grounds → Common Ground Bus
```

#### Motor Control Wiring:

```
Raspberry Pi GPIO → L298N Motor Drivers:
- Front Left: DIR=GPIO17, PWM=GPIO27
- Front Right: DIR=GPIO24, PWM=GPIO25
- Back: DIR=GPIO5, PWM=GPIO6
- Lifter: DIR=GPIO13, PWM=GPIO12

Raspberry Pi GPIO → Motor Encoders (read-only):
- Front Left: Encoder A=GPIO22, Encoder B=GPIO23
- Front Right: Encoder A=GPIO16, Encoder B=GPIO26
- Back: Encoder A=GPIO7, Encoder B=GPIO9
- Lifter: Encoder A=GPIO20, Encoder B=GPIO21
```

#### Sensor Wiring:

```
Raspberry Pi Interfaces:
- USB Port 1 → RPLIDAR A1
- USB Port 2 → Microsoft USB Camera
- GPIO 12, 13, 16 → Distance Sensors (Front, Back Left, Back Right)
- GPIO 20 → Line Sensor
- I2C Bus (GPIO 3, 5) → IMU (MPU6050/BNO055)
- GPIO 4, 5, 6, 7 → Container Load Sensors
```

#### Servo Control Wiring:

```
Raspberry Pi PWM Pins → Servo Motors:
- GPIO 14 (PWM0) → Gripper Servo
- GPIO 15 (PWM1) → Gripper Tilt Servo
- GPIO 18 (PWM2) → Gripper Neck Servo (continuous)
- GPIO 19 (PWM3) → Gripper Base Servo
- GPIO 21 (PWM4) → Container Actuators
```

## Testing and Verification

### Motor Testing

```bash
# Test individual omni wheel motors
ros2 run my_robot_automation motor_test --wheel 1
ros2 run my_robot_automation motor_test --wheel 2
ros2 run my_robot_automation motor_test --wheel 3

# Test lifter motor
ros2 run my_robot_automation motor_test --lifter

# Expected behavior:
# - Omni wheels rotate smoothly in both directions
# - Lifter moves up/down with encoder feedback
# - No unusual vibrations or resistance
```

### Sensor Verification

```bash
# Test RPLIDAR A1 scanning
ros2 topic echo /scan

# Test Microsoft USB camera
ros2 topic echo /camera/image_raw

# Test distance sensors
ros2 topic echo /distance/front
ros2 topic echo /distance/back_left
ros2 topic echo /distance/back_right

# Test IMU data
ros2 topic echo /imu/data

# Test line sensor
ros2 topic echo /line_sensor/raw
```

### Picker System Testing

```bash
# Test individual servo motors
ros2 run my_robot_automation servo_test --gripper
ros2 run my_robot_automation servo_test --tilt
ros2 run my_robot_automation servo_test --neck
ros2 run my_robot_automation servo_test --base

# Test complete picker sequence
ros2 run my_robot_automation picker_test

# Expected behavior:
# - All servos move smoothly within ranges
# - Gripper opens/closes properly
# - Lifter raises/lowers picker assembly
```

### Integration Testing

```bash
# Test complete system integration
ros2 launch my_robot_bringup robot.launch.py

# Verify:
# - All sensors publishing data at correct rates
# - Omni wheel movement in all directions
# - Picker system responds to commands
# - Container system functions properly
# - Emergency stop halts all operations
```

## Safety Considerations

### Electrical Safety
- Use proper wire gauge for current requirements
- Install fuses on all power lines
- Use heat sinks on L298N motor drivers and voltage regulators
- Avoid short circuits during assembly

### Mechanical Safety
- Secure all mechanical fasteners
- Verify structural integrity
- Test emergency stop functionality
- Implement collision detection

### Operational Safety
- Implement emergency stop buttons
- Use proper power-down procedures
- Monitor battery voltage levels
- Implement over-current protection

## Troubleshooting Hardware Issues

### Motor Control Problems

**Issue**: Omni wheels not responding
```bash
# Check motor power (12V supply to L298N VS pin)
# Verify L298N connections: VS=12V, VCC=5V, GND=common ground
# Verify L298N DIR and PWM pin connections to GPIO
# Check that PWM pins are set correctly (0=stop, 1=run)
# Verify encoder DATA(A) and DATA(B) pin connections (read-only inputs)
# Test L298N with simple GPIO write commands
# Verify encoder readings from built-in encoders
```

**Issue**: Wheels running in wrong direction
```bash
# Reverse DIR signal polarity in software configuration
# Check motor wiring phase (A+/A-/B+/B-)
# Verify omni wheel orientation on motor shafts
# Update kinematic calculations if needed
```

**Issue**: Lifter motor not moving
```bash
# Check lifter motor power (12V to M+, GND to M-) and serial control connections
# Verify encoder feedback wiring
# Test limit switches and safety interlocks
# Check mechanical binding in guide rails
```

### Sensor Issues

**Issue**: RPLIDAR A1 not detected
```bash
# Check USB connection and power supply
# Verify /dev/ttyUSB* device creation
# Test with lsusb and usb-devices commands
# Check USB port functionality with other devices
```

**Issue**: Distance sensors inaccurate
```bash
# Verify sensor power supply (5V)
# Check analog/digital signal connections to GPIO
# Calibrate sensor thresholds and ranges
# Test sensors individually away from robot
```

**Issue**: IMU data incorrect
```bash
# Check I2C bus connectivity (i2cdetect command)
# Verify sensor mounting orientation and calibration
# Test I2C communication with i2c-tools
# Recalibrate IMU offsets and scaling
```

**Issue**: Camera not working
```bash
# Check USB connection and power
# Verify camera device with v4l2-ctl --list-devices
# Test camera with cheese or guvcview applications
# Check USB bandwidth and interference
```

## Maintenance Procedures

### Regular Maintenance
- Clean debris from wheels and sensors
- Check electrical connections for corrosion
- Verify battery charge levels
- Update firmware as needed

### Component Replacement
- Document all component specifications
- Maintain spare parts inventory
- Follow proper ESD procedures for electronics

### Calibration Procedures
- Motor PID tuning
- Sensor offset calibration
- Kinematic parameter verification
- Coordinate system alignment

## Support and Resources

### Documentation References
- [Raspberry Pi 5 Documentation](https://www.raspberrypi.com/documentation/)
- [ROS 2 Hardware Setup](https://docs.ros.org/en/iron/Tutorials/Advanced/Sim-to-Real.html)
- [Gazebo Simulation Setup](https://gazebosim.org/docs/)

### Component Datasheets
- RPLidar A1/A2 Technical Specifications
- MPU6050/BNO055 IMU Datasheets
- PG23 Motor Datasheet (built-in encoder specifications)
- L298N Motor Driver Datasheet
- Servo Motor Specifications
- PG23 Motor Specifications (see MOTOR_SPECIFICATIONS.md)
- PG23 Motor Connection Guide (see PG23_MOTOR_CONNECTION_GUIDE.md)

### Community Resources
- [ROS Discourse](https://discourse.ros.org/)
- [Raspberry Pi Forums](https://forums.raspberrypi.com/)
- [GitHub Issues](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues)

---

*This hardware setup guide ensures proper assembly and configuration of the Autonomous Mobile Manipulator robot. Follow each step carefully and verify functionality before proceeding to software integration.*
