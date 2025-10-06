# Hardware Setup Guide

This guide provides comprehensive instructions for assembling and configuring the hardware components of the Autonomous Mobile Manipulator robot.

## Robot Overview

The Autonomous Mobile Manipulator consists of:
- **Mobile Base**: 3-wheeled omnidirectional platform
- **Manipulator Arm**: 6-DOF robotic arm with gripper
- **Sensors**: LiDAR, IMU, camera, and contact sensors
- **Computing**: Raspberry Pi 5 with Ubuntu Server
- **Power System**: Battery management and distribution

## Bill of Materials

### Core Components

| Component | Model/Specification | Quantity | Purpose |
|-----------|-------------------|----------|---------|
| Raspberry Pi 5 | 8GB RAM | 1 | Main computer |
| Motor Driver | TB6600 or similar | 3 | Omni wheel control |
| DC Motors | 12V, high torque | 3 | Wheel actuation |
| Omni Wheels | 100mm diameter | 3 | Omnidirectional movement |
| LiDAR Sensor | RPLidar A1/A2 | 1 | 360° scanning |
| IMU Sensor | MPU6050/BNO055 | 1 | Orientation sensing |
| Camera | Raspberry Pi Camera Module 3 | 1 | Computer vision |
| Servo Motors | MG996R or similar | 6 | Arm joints |
| Gripper | Robotic gripper kit | 1 | Object manipulation |
| Battery | 12V LiPo 5000mAh | 1 | Power supply |
| Voltage Regulator | 5V/12V buck converter | 2 | Power management |

### Mechanical Components

| Component | Specification | Quantity | Purpose |
|-----------|---------------|----------|---------|
| Base Plate | 300mm x 300mm aluminum | 1 | Robot chassis |
| Support Posts | M8 threaded rod, 200mm | 4 | Structure support |
| Motor Mounts | 3D printed or aluminum | 3 | Motor attachment |
| Wheel Hubs | Custom machined | 3 | Wheel attachment |
| Arm Base | Servo bracket system | 1 | Arm mounting |
| Gripper Mount | Custom bracket | 1 | End effector |

### Electrical Components

| Component | Specification | Quantity | Purpose |
|-----------|---------------|----------|---------|
| Power Distribution Board | Custom PCB or breadboard | 1 | Power management |
| Wire Harness | 22AWG silicone wire | Various | Interconnections |
| Connectors | JST-XH series | Various | Modular connections |
| Fuse Holders | 10A automotive fuses | 3 | Circuit protection |
| Heat Sinks | For motor drivers | 3 | Thermal management |
| Cable Management | Zip ties, cable sleeves | Various | Organization |

## Assembly Instructions

### Step 1: Base Platform Assembly

#### Materials Needed:
- Base plate (300mm x 300mm)
- 4x M8 threaded rods (200mm length)
- 8x M8 nuts and washers
- Motor mounting brackets

#### Assembly Procedure:

1. **Mark Motor Positions**:
   ```
   Place motors at 120° intervals:
   - Motor 1: (150mm, 86.6mm) from center
   - Motor 2: (-150mm, 86.6mm) from center
   - Motor 3: (0mm, -173.2mm) from center
   ```

2. **Install Motor Mounts**:
   ```bash
   # Secure each motor mount with M4 screws
   # Ensure proper alignment for 120° spacing
   # Verify motor shaft alignment
   ```

3. **Mount Omni Wheels**:
   ```bash
   # Attach wheels to motor shafts
   # Ensure wheels are perpendicular to base
   # Test rotation freedom
   ```

### Step 2: Electronics Installation

#### Power System Setup:

1. **Battery Installation**:
   ```bash
   # Mount battery securely to base plate
   # Connect to power distribution board
   # Install fuse protection (10A)
   ```

2. **Voltage Regulation**:
   ```bash
   # Connect 12V battery to buck converter
   # Output 5V for Raspberry Pi and sensors
   # Output 12V for motor drivers
   ```

3. **Motor Driver Configuration**:
   ```bash
   # Set microstepping (1/16 recommended)
   # Configure current limits (1.5A per phase)
   # Connect enable, direction, and step pins
   ```

#### Sensor Integration:

1. **LiDAR Mounting**:
   ```bash
   # Mount RPLidar at 200mm height above base
   # Connect USB to Raspberry Pi
   # Ensure 360° clearance
   ```

2. **IMU Installation**:
   ```bash
   # Mount IMU at robot center of gravity
   # Connect I2C interface to Raspberry Pi
   # Calibrate orientation
   ```

3. **Camera Setup**:
   ```bash
   # Mount camera with forward-facing orientation
   # Connect CSI ribbon cable to Raspberry Pi
   # Configure camera parameters
   ```

### Step 3: Manipulator Arm Assembly

#### Arm Structure:

1. **Base Joint Installation**:
   ```bash
   # Mount base servo to arm bracket
   # Connect to Raspberry Pi PWM pin
   # Set servo limits (0-180°)
   ```

2. **Joint Assembly**:
   ```bash
   # Connect shoulder, elbow, wrist joints
   # Install servo horns and linkages
   # Verify joint movement range
   ```

3. **Gripper Installation**:
   ```bash
   # Mount gripper to wrist joint
   # Connect servo for gripper actuation
   # Test grip and release functions
   ```

### Step 4: Wiring and Connections

#### Power Wiring:

```
Battery (+) → Fuse → Power Distribution Board → Buck Converters
Battery (-) → Power Distribution Board → All Grounds

Buck Converter 1 (12V) → Motor Driver Power
Buck Converter 2 (5V) → Raspberry Pi, Sensors, Servos
```

#### Signal Wiring:

```
Raspberry Pi GPIO:
- PWM Pins (18, 19, 21) → Motor Driver STEP inputs
- GPIO Pins (22, 24, 26) → Motor Driver DIR inputs
- GPIO Pin 17 → Motor Driver ENABLE (all)
- I2C Pins (3, 5) → IMU sensor
- UART Pins (8, 10) → Optional serial devices
- CSI Port → Camera module
```

#### Motor Driver Connections:

```
Motor Driver 1 (Wheel 1):
- STEP → GPIO 18
- DIR → GPIO 22
- ENABLE → GPIO 17

Motor Driver 2 (Wheel 2):
- STEP → GPIO 19
- DIR → GPIO 24
- ENABLE → GPIO 17

Motor Driver 3 (Wheel 3):
- STEP → GPIO 21
- DIR → GPIO 26
- ENABLE → GPIO 17
```

## Testing and Verification

### Motor Testing

```bash
# Test individual motor control
ros2 run my_robot_bringup motor_test

# Expected behavior:
# - Each motor rotates forward/backward
# - Proper direction control
# - Consistent speed across all motors
```

### Sensor Verification

```bash
# Test LiDAR functionality
ros2 topic echo /scan

# Test IMU data
ros2 topic echo /imu/data

# Test camera stream
ros2 run image_view image_view image:=/camera/image_raw
```

### Integration Testing

```bash
# Test complete system integration
ros2 launch my_robot_bringup robot.launch.py

# Verify:
# - All sensors publishing data
# - Motor controllers responding
# - Robot maintains stable operation
```

## Safety Considerations

### Electrical Safety
- Use proper wire gauge for current requirements
- Install fuses on all power lines
- Use heat sinks on motor drivers
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

**Issue**: Motors not responding
```bash
# Check motor driver power
# Verify GPIO pin connections
# Test with simple PWM script
```

**Issue**: Motors running in wrong direction
```bash
# Reverse DIR pin logic in software
# Swap motor phase connections
# Update coordinate system mapping
```

### Sensor Issues

**Issue**: LiDAR not detected
```bash
# Check USB connection and power
# Verify /dev/ttyUSB0 exists
# Test with lsusb command
```

**Issue**: IMU data incorrect
```bash
# Check I2C bus connectivity
# Verify sensor mounting orientation
# Recalibrate sensor offsets
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
- TB6600 Motor Driver Manual
- Servo Motor Specifications

### Community Resources
- [ROS Discourse](https://discourse.ros.org/)
- [Raspberry Pi Forums](https://forums.raspberrypi.com/)
- [GitHub Issues](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues)

---

*This hardware setup guide ensures proper assembly and configuration of the Autonomous Mobile Manipulator robot. Follow each step carefully and verify functionality before proceeding to software integration.*
