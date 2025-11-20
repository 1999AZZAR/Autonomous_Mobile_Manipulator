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

The LKS Autonomous Mobile Manipulator uses a distributed control architecture:

### Computing Systems
- **Raspberry Pi 5**: High-level control, ROS2 framework, web interfaces, sensor fusion
- **Arduino Mega 2560**: Real-time motor control, PID algorithms, sensor acquisition

### Mechanical Systems
- **Mobile Base**: 3-wheeled omnidirectional platform (hexagonal shape) with PG23 motors
- **Manipulator System**: 5-servo picker (gripper, tilt, neck, base, lifter)
- **Container System**: 4-compartment material transport with individual actuators

### Sensor Suite (Distributed)
- **Arduino Mega Sensors**: IR distance (6x), ultrasonic (2x), IMU, line sensors (3x), motor encoders (4x)
- **Raspberry Pi Sensors**: RPLIDAR A1 (380°), Microsoft USB camera, TF-Luna LIDAR (optional)
- **Communication**: Serial UART (115200 baud) for Pi-Mega coordination

## Bill of Materials

### Core Components

| Component | Model/Specification | Quantity | Purpose |
|-----------|-------------------|----------|---------|
| **Computing** | | | |
| Raspberry Pi 5 | 8GB RAM, Ubuntu Server | 1 | High-level control, ROS2, web interfaces |
| Arduino Mega 2560 | With YFROBOT shield | 1 | Real-time motor control, PID, sensor acquisition |
| **Motors & Drive** | | | |
| PG23 Motors | Built-in encoder, 12V, 15.5k RPM, 7 PPR | 4 | 3x omni wheels + 1x lifter (Mega-controlled) |
| Omni Wheels | 75mm diameter | 3 | Omnidirectional movement |
| **Sensors (Mega-controlled)** | | | |
| IR Distance Sensors | Laser-based | 6 | Wall alignment and obstacle detection |
| Ultrasonic Sensors | HC-SR04 | 2 | Front obstacle detection |
| IMU Sensor | MPU6050/BNO055 | 1 | Orientation and motion sensing |
| Line Sensors | IR-based | 3 | Line following navigation |
| Motor Encoders | Built-in PG23 | 4 | Speed feedback and odometry |
| **Sensors (Pi-controlled)** | | | |
| RPLIDAR A1 | 380° scanning | 1 | Laser-based obstacle detection and mapping |
| Microsoft USB Camera | Standard webcam | 1 | Object recognition and computer vision |
| TF-Luna LIDAR | Single-point ranging | 1 | Optional distance measurement |
| **Actuators** | | | |
| Servo Motors | MG996R | 5 | Picker system (gripper, tilt, neck, base, lifter) |
| Container Actuators | Solenoid/servos | 4 | Individual container control |
| **Power System** | | | |
| Battery | 12V LiPo 5000mAh | 1 | Power supply for all systems |
| Voltage Regulators | 5V/12V buck converters | 2 | Power management and distribution |

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
   # Connect battery input with main fuse protection (15A)
   # Wire 12V outputs to Arduino Mega (motor power)
   # Wire 5V outputs to Raspberry Pi, Arduino Mega, and servo power
   # Include power monitoring for battery level sensing
   ```

2. **Arduino Mega Motor Control Setup**:
   ```bash
   # Connect Arduino Mega with YFROBOT motor shield
   # Shield provides integrated motor drivers and PID control
   # Connect 4 PG23 motors to shield motor terminals:
   # - M1: Back omni wheel
   # - M2: Front left omni wheel
   # - M3: Front right omni wheel
   # - M4: Lifter motor
   # Connect 12V power to shield VIN terminal
   # Connect 5V logic power to shield VCC
   ```

3. **Pi-Mega Communication Setup**:
   ```bash
   # Connect Raspberry Pi UART to Arduino Mega UART
   # Pi GPIO 14 (TX) → Mega RX (pin 0)
   # Pi GPIO 15 (RX) ← Mega TX (pin 1)
   # Common ground connection
   # Set baud rate to 115200 on both devices
   ```

3. **Servo Power Distribution**:
   ```bash
   # Install servo power regulator (6V for servos)
   # Connect to 5 servo motors and gripper servo
   # Ensure proper grounding and decoupling capacitors
   ```

#### Sensor Integration (Arduino Mega):

1. **IR Distance Sensors (6x)**:
   ```bash
   # Mount sensors for wall alignment:
   # - 2 on left side (left front, left back)
   # - 2 on right side (right front, right back)
   # - 2 on back (back left, back right)
   # Connect analog outputs to Arduino Mega analog pins A0-A5
   # Provide 5V power and common ground
   ```

2. **Ultrasonic Sensors (2x)**:
   ```bash
   # Mount front left and front right ultrasonic sensors
   # Connect trigger/echo pins to Arduino Mega digital pins
   # Use HC-SR04 sensors with 5V power supply
   ```

3. **IMU and Line Sensors**:
   ```bash
   # Mount MPU6050 IMU at robot center of gravity
   # Connect I2C bus to Arduino Mega (SDA/SCL pins)
   # Mount 3 IR line sensors at bottom center (left, center, right)
   # Connect line sensor outputs to Arduino Mega digital pins
   ```

#### Sensor Integration (Raspberry Pi):

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

3. **Optional TF-Luna LIDAR**:
   ```bash
   # Mount single-point LIDAR for specific applications
   # Connect UART interface to Raspberry Pi GPIO serial
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

#### Arduino Mega Motor Control:

```
YFROBOT Shield Connections:
- Motor 1 (Back): Shield motor terminals M1
- Motor 2 (Front Left): Shield motor terminals M2
- Motor 3 (Front Right): Shield motor terminals M3
- Motor 4 (Lifter): Shield motor terminals M4
- Power: 12V to VIN, 5V to VCC
- Encoders: Connected internally to shield encoder inputs
```

#### Pi-Mega Communication:

```
Serial UART Connection:
- Raspberry Pi GPIO 14 (TX) → Arduino Mega pin 0 (RX)
- Raspberry Pi GPIO 15 (RX) ← Arduino Mega pin 1 (TX)
- Common ground connection between Pi and Mega
- Baud rate: 115200 (configured in both devices)
```

#### Arduino Mega Sensor Wiring:

```
IR Distance Sensors (Analog):
- Left Front: Arduino A0
- Left Back: Arduino A1
- Right Front: Arduino A2
- Right Back: Arduino A3
- Back Left: Arduino A4
- Back Right: Arduino A5

Ultrasonic Sensors (Digital):
- Front Left: Trigger=22, Echo=23
- Front Right: Trigger=24, Echo=25

Line Sensors (Digital):
- Left: Arduino pin 26
- Center: Arduino pin 27
- Right: Arduino pin 28

IMU (I2C):
- SDA: Arduino pin 20
- SCL: Arduino pin 21
```

#### Raspberry Pi Sensor Wiring:

```
USB Interfaces:
- USB Port 1 → RPLIDAR A1
- USB Port 2 → Microsoft USB Camera

Optional TF-Luna (UART):
- TX: Pi GPIO 14 (software UART)
- RX: Pi GPIO 15 (software UART)

Container Load Sensors:
- Sensor 1: Pi GPIO 4
- Sensor 2: Pi GPIO 5
- Sensor 3: Pi GPIO 6
- Sensor 4: Pi GPIO 7
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

### Arduino Mega Testing

#### Motor Control Verification
```bash
# Test Mega motor control via serial commands
# Connect to Mega serial port and send commands:
f    # Forward
b    # Backward
l    # Strafe left
r    # Strafe right
s    # Stop

# Expected behavior:
# - Motors respond to commands with smooth PID control
# - Encoder feedback provides accurate speed control
# - Emergency stop (v) immediately halts all motors
```

#### Mega Sensor Testing
```bash
# Request sensor data from Mega
sr   # Request sensor readings

# Expected response format:
# SENSORS:IR_LF:245.5,IR_LB:238.2,US_FL:85.4,US_FR:92.1,...

# Test individual sensors:
# - IR sensors should show distance readings (0-4000mm)
# - Ultrasonic sensors should detect obstacles (0-400cm)
# - IMU should provide orientation data
# - Line sensors should detect line boundaries
```

### Raspberry Pi Testing

#### ROS2 Serial Interface Testing
```bash
# Start Mega serial interface
ros2 run my_robot_automation mega_serial_interface

# Test command transmission
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"

# Monitor sensor data topics
ros2 topic echo /distance/left_front
ros2 topic echo /ultrasonic/front_left
ros2 topic echo /imu/data
```

#### Pi Sensor Verification
```bash
# Test RPLIDAR A1 scanning
ros2 topic echo /scan

# Test Microsoft USB camera
ros2 topic echo /camera/image_raw

# Test optional TF-Luna LIDAR
ros2 topic echo /tf_luna/range
```

### Servo and Picker System Testing

```bash
# Test servo control via Mega commands
ta90  # Tilt to 90 degrees
ga45  # Gripper to 45 degrees
no    # Gripper open
nc    # Gripper close

# Test lifter control
u     # Lifter up
d     # Lifter down

# Expected behavior:
# - All servos move smoothly to commanded positions
# - Gripper opens/closes reliably
# - Lifter moves with controlled speed
```

### Pi-Mega Integration Testing

```bash
# Test complete system integration
ros2 launch my_robot_bringup robot.launch.py

# Run integration test script
python3 /home/azzar/project/robotic/test_pi_mega_integration.py

# Verify:
# - Serial communication established (115200 baud)
# - Commands flow from Pi to Mega successfully
# - Sensor data flows from Mega to Pi ROS topics
# - Emergency stop works across both systems
# - All safety interlocks function properly
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

**Issue**: Motors not responding to commands
```bash
# Check Arduino Mega power (12V to VIN, 5V to VCC)
# Verify YFROBOT shield is properly seated
# Test Mega serial connection: screen /dev/ttyACM0 115200
# Send direct commands: f (forward), s (stop)
# Check Mega LED indicators for power and activity
```

**Issue**: Motors running erratically or with poor PID control
```bash
# Check encoder connections on YFROBOT shield
# Verify motor wiring polarity (swap if running backward)
# Test individual motor control via Mega serial
# Check for mechanical binding or wheel interference
# Verify PID tuning parameters in Mega code
```

**Issue**: Pi-Mega communication failure
```bash
# Check serial connection: Pi GPIO 14→Mega RX, Pi GPIO 15←Mega TX
# Verify baud rate (115200) matches in both devices
# Test serial loopback: short TX to RX on one device
# Check for serial port permissions: ls -la /dev/ttyACM0
# Monitor serial traffic: cat /dev/ttyACM0
```

**Issue**: Lifter motor not moving
```bash
# Check lifter motor power (12V to M+, GND to M-) and serial control connections
# Verify encoder feedback wiring
# Test limit switches and safety interlocks
# Check mechanical binding in guide rails
```

### Sensor Issues

**Issue**: Arduino Mega sensors not responding
```bash
# Check Mega power and sensor connections
# Test Mega serial: screen /dev/ttyACM0 115200
# Send sensor request: sr
# Check analog pin connections (A0-A5 for IR sensors)
# Verify digital pin connections for ultrasonic/line sensors
# Test I2C: i2cdetect -y 1 (should show IMU at 0x68 or 0x28)
```

**Issue**: IR/Ultrasonic sensors showing incorrect readings
```bash
# Check sensor power supply (5V from Mega)
# Verify sensor mounting (clear line of sight)
# Test sensors individually with Mega serial commands
# Calibrate distance thresholds in Mega code
# Check for electrical interference or crosstalk
```

**Issue**: IMU data incorrect or noisy
```bash
# Check I2C bus connectivity: i2cdetect -y 1
# Verify IMU mounting (secure, level, away from motors)
# Test I2C communication speed and pull-up resistors
# Recalibrate IMU offsets via Mega serial commands
# Check for magnetic interference near motors
```

**Issue**: RPLIDAR A1 not detected (Pi sensor)
```bash
# Check USB connection and permissions: ls -la /dev/ttyUSB*
# Verify RPLIDAR power supply (sufficient current)
# Test USB port with other devices
# Check ROS2 RPLIDAR driver configuration
# Monitor USB traffic: lsusb -v | grep RPLIDAR
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
