# Raspberry Pi 5 Pinout Configuration - LKS Robot Project

## Complete Hardware Pinout for Hexagonal Robot

This document provides the complete GPIO pinout configuration for the LKS Robot Project running on Raspberry Pi 5 with Ubuntu Server, ROS2, and N8N automation.

## Raspberry Pi 5 GPIO Layout

```
Raspberry Pi 5 GPIO Header (40-pin)
┌─────────────────────────────────────┐
│  3V3 (1)   5V (2)                   │
│ GPIO2 (3)  5V (4)                   │
│ GPIO3 (5) GND (6)                   │
│ GPIO4 (7) GPIO14 (8)                │
│ GND (9)   GPIO15 (10)               │
│GPIO17(11) GPIO18(12)                │
│GPIO27(13) GND (14)                  │
│GPIO22(15) GPIO23(16)                │
│  3V3(17)  GPIO24(18)                │
│GPIO10(19) GND (20)                  │
│ GPIO9(21) GPIO25(22)                │
│GPIO11(23) GPIO8 (24)                │
│ GND (25)  GPIO7 (26)                │
│ GPIO0(27) GPIO1 (28)                │
│ GPIO5(29) GND (30)                  │
│ GPIO6(31) GPIO12(32)                │
│GPIO13(33) GND (34)                  │
│GPIO19(35) GPIO16(36)                │
│GPIO26(37) GPIO20(38)                │
│ GND (39)  GPIO21(40)                │
└─────────────────────────────────────┘
```

## Power Distribution

### Main Power Supply

- **Input Voltage**: 12V DC (recommended for motors)
- **Current Rating**: 5A minimum (for all motors + servos)
- **Connector**: DC Barrel Jack (5.5mm x 2.1mm)

### Power Rails

- **5V Rail**: For Raspberry Pi, sensors, and logic circuits
- **3.3V Rail**: For low-power sensors and I2C devices
- **12V Rail**: For motors and high-power actuators

### Voltage Regulators Needed

- **12V → 5V**: DC-DC converter (5A minimum) for motor drivers
- **5V → 3.3V**: AMS1117-3.3 or similar for sensors

## GPIO Pin Assignments

### **Motor Drivers (4x TB6600 or similar)**

```bash
# Omni Wheel Motors (3x)
GPIO17 (11) - All Motors ENABLE
GPIO18 (12) - Wheel 1 STEP
GPIO22 (15) - Wheel 1 DIR
GPIO19 (35) - Wheel 2 STEP
GPIO24 (18) - Wheel 2 DIR
GPIO21 (40) - Wheel 3 STEP
GPIO26 (37) - Wheel 3 DIR

# Lifter Motor (1x)
GPIO25 (22) - Lifter ENABLE
GPIO27 (13) - Lifter STEP
GPIO23 (16) - Lifter DIR
```

### **Servo Motors (5x)**

```bash
# Picker System Servos
GPIO14 (8)  - Gripper Servo (Open/Close)
GPIO15 (10) - Gripper Tilt Servo
GPIO18 (12) - Gripper Neck Servo (Continuous)
GPIO19 (35) - Gripper Base Servo (Rotation)
GPIO21 (40) - Container Actuators
```

### **Distance Sensors (3x Laser)**

```bash
# Front Distance Sensor
GPIO12 (32) - Front Sensor Input

# Back Left Distance Sensor
GPIO13 (33) - Back Left Sensor Input

# Back Right Distance Sensor
GPIO16 (36) - Back Right Sensor Input
```

### **Line Sensor (Digital)**

```bash
GPIO20 (38) - Line Sensor Raw Data
```

### **Container Load Sensors (4x)**

```bash
GPIO4  (7)  - Left Front Container
GPIO5  (29) - Left Back Container
GPIO6  (31) - Right Front Container
GPIO7  (26) - Right Back Container
```

### **IMU Sensor (MPU6050/BNO055)**

```bash
# I2C Interface (Bus 1)
GPIO2  (3)  - SDA (I2C Data)
GPIO3  (5)  - SCL (I2C Clock)
```

### **USB Interfaces**

```bash
# USB Ports (Hardware)
USB1 - RPLIDAR A1 (380° LiDAR)
USB2 - Microsoft USB Camera
```

## Hardware Connections

### **Motor Driver Connections (L298N or similar)**

```bash
# Motor Driver 1 (Back Wheel)
IN1: GPIO17 (11)
IN2: GPIO27 (13) - Direction control
ENA: GPIO27 (13) - PWM Speed (ENA pin)

# Motor Driver 2 (Front Left Wheel)
IN3: GPIO24 (18)
IN4: GPIO25 (22) - Direction control
ENB: GPIO25 (22) - PWM Speed (ENB pin)

# Motor Driver 3 (Front Right Wheel)
IN1: GPIO16 (36)
IN2: GPIO26 (37) - Direction control
ENA: GPIO26 (37) - PWM Speed (ENA pin)

# Lifter Motor Driver
IN1: GPIO12 (32)
IN2: GPIO13 (33) - Direction control
ENA: GPIO13 (33) - PWM Speed (ENA pin)

# Power Connections (12V)
All motor drivers connect to 12V supply
Logic pins connect to 5V from Raspberry Pi
```

### **Sensor Connections**

#### **Ultrasonic Sensors (HC-SR04)**

```bash
# Power: 5V, GND
# Front Sensor:
TRIG: GPIO4 (7)  - Connect to Trig pin
ECHO: GPIO14 (8) - Connect to Echo pin (with voltage divider)

# Back Left Sensor:
TRIG: GPIO15 (10) - Connect to Trig pin
ECHO: GPIO17 (11) - Connect to Echo pin (voltage divider)

# Back Right Sensor:
TRIG: GPIO18 (12) - Connect to Trig pin
ECHO: GPIO27 (13) - Connect to Echo pin (voltage divider)

# Voltage Divider for Echo Pins (5V → 3.3V)
# 1kΩ resistor from Echo to GPIO
# 2kΩ resistor from Echo to GND
```

#### **IR Proximity Sensors (Sharp GP2Y0A21YK)**

```bash
# Analog output connects to MCP3008 ADC
# MCP3008 Connections:
VDD:  3.3V
VREF: 3.3V
AGND: GND
CLK:  GPIO11 (23)
DOUT: GPIO9  (21)
DIN:  GPIO10 (19)
CS:   GPIO8  (24)

# Sensor Connections to MCP3008:
CH0: Front IR Sensor
CH1: Left IR Sensor
CH2: Right IR Sensor
```

#### **Line Sensor Array**

```bash
# 8-sensor line following array
# Uses 74HC165 shift register for parallel-to-serial conversion

# 74HC165 Connections:
QH:  GPIO10 (19) - Serial output to Raspberry Pi
CLK: GPIO9  (21) - Clock signal
SH/LD: GPIO8 (24) - Shift/Load control
CLK_INH: GPIO11 (23) - Clock inhibit

# Sensor inputs (QH to A) connect to individual IR sensors
# Each sensor: 3-pin (VCC, GND, OUT)
# OUT: Digital signal (HIGH = line detected, LOW = no line)
```

#### **IMU Sensor (MPU6050)**

```bash
# I2C Connection (default Raspberry Pi I2C bus)
SDA: GPIO2 (3)  - Connect to MPU6050 SDA
SCL: GPIO3 (5)  - Connect to MPU6050 SCL
VCC: 3.3V or 5V (check module specifications)
GND: GND
INT: GPIO4 (7)  - Interrupt pin (optional)
```

## Software Configuration

### **Enable I2C Interface**

```bash
sudo raspi-config
# Interfacing Options > I2C > Enable
```

### **Enable SPI Interface (for ADC)**

```bash
sudo raspi-config
# Interfacing Options > SPI > Enable
```

### **Install Required Libraries**

```bash
sudo apt update
sudo apt install python3-pip python3-gpiozero python3-smbus python3-spidev
pip3 install adafruit-circuitpython-mpu6050 adafruit-circuitpython-mcp3008
```

### **GPIO Permissions**

```bash
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER
sudo usermod -a -G spi $USER
```

## Pin Usage Summary

### **PWM Pins Used (Hardware PWM)**

- GPIO12 (32) - Servo 1
- GPIO13 (33) - Servo 2
- GPIO18 (12) - Servo 3
- GPIO19 (35) - Servo 4

### **Interrupt Pins (Encoder Inputs)**

- GPIO22 (15) - Back wheel encoder A
- GPIO23 (16) - Back wheel encoder B
- GPIO5 (29)  - Front left encoder A
- GPIO6 (31)  - Front left encoder B
- GPIO20 (38) - Front right encoder A
- GPIO21 (40) - Front right encoder B

### **I2C Pins**

- GPIO2 (3) - SDA
- GPIO3 (5) - SCL

### **SPI Pins (MCP3008 ADC)**

- GPIO8 (24)  - CS
- GPIO9 (21)  - CLK
- GPIO10 (19) - DOUT
- GPIO11 (23) - DIN

### **Digital I/O Pins**

- GPIO4 (7)   - Ultrasonic TRIG
- GPIO14 (8)  - Ultrasonic ECHO
- GPIO15 (10) - Ultrasonic TRIG
- GPIO16 (36) - Motor direction
- GPIO17 (11) - Motor direction
- GPIO24 (18) - Motor direction
- GPIO25 (22) - Motor PWM
- GPIO26 (37) - Motor PWM
- GPIO27 (13) - Motor PWM

## Important Notes

### **Pin Conflicts Resolution**

Some pins are assigned to multiple functions. Prioritize based on hardware capabilities:

1. Use hardware PWM pins for servo control
2. Use interrupt-capable pins for encoders
3. Redistribute conflicting pins as needed

### **Power Considerations**

- **Separate power supplies** for motors and logic to avoid noise
- **Current monitoring** for motor drivers
- **Voltage regulators** must handle peak current loads
- **Heat sinks** for motor drivers and voltage regulators

### **Sensor Calibration**

- **Ultrasonic sensors**: Calibrate for temperature and humidity
- **IR sensors**: Calibrate distance-to-voltage curves
- **Line sensors**: Calibrate threshold values for different surfaces
- **IMU**: Calibrate gyroscope and accelerometer offsets

### **Safety Features**

- **Emergency stop button** connected to GPIO (not assigned yet)
- **Motor current monitoring** to prevent overheating
- **Software limits** for all actuators
- **Watchdog timer** for system health monitoring

## Hardware Shopping List

### **Core Components**

- Raspberry Pi 5
- 5x MG996R Servo Motors
- 3x DC Motors with Encoders (for omni wheels)
- 1x DC Motor for Lifter
- 3x L298N Motor Drivers
- 3x HC-SR04 Ultrasonic Sensors
- 3x Sharp GP2Y0A21YK IR Sensors
- 8x IR Line Sensors
- MPU6050 IMU Sensor
- MCP3008 ADC Converter
- 74HC165 Shift Register
- Power Supply (12V, 5A)

### **Supporting Components**

- Jumper wires, breadboards
- Voltage regulators (12V→5V, 5V→3.3V)
- Resistors for voltage dividers
- Capacitors for noise filtering
- Heat sinks and cooling fans

## Ready for Implementation

This pinout configuration provides a complete hardware interface for the LKS Robot Project. All sensors and actuators referenced in the ROS2 code and N8N workflows are properly mapped to Raspberry Pi GPIO pins.

**The robot is now ready for physical assembly and testing!**
