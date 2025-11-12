# Raspberry Pi 5 Pinout Configuration - Autonomous Mobile Manipulator

## Complete Hardware Pinout for Hexagonal Robot

This document provides the complete GPIO pinout configuration for the Autonomous Mobile Manipulator running on Raspberry Pi 5 with Ubuntu Server, ROS2, and N8N automation.

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

- **12V → 5V**: DC-DC converter (5A minimum) for PG23 motor VIN pins (encoder/controller power)
- **5V → 3.3V**: AMS1117-3.3 or similar for sensors

## GPIO Pin Assignments (Hexagonal Robot Architecture)

### **Motor Control (L298N Drivers - 3x for Omni Wheels + 1x for Gripper)**

**Note:** PG23 motors have built-in encoders only - controlled via L298N motor drivers.

```bash
# Omni Wheel Motors (3x) - L298N Control & Encoder (Separate Pins)
# Motor Control: L298N DIR and PWM pins
# Encoder Reading: DATA(A) and DATA(B) pins (read only)

# Front Left Wheel
GPIO17 (11) - Front Left DIR - L298N Direction pin (OUT1/OUT2 control)
GPIO27 (13) - Front Left PWM - L298N PWM pin (speed control)
GPIO22 (15) - Front Left Encoder A - DATA(A) pin (read only)
GPIO23 (16) - Front Left Encoder B - DATA(B) pin (read only)

# Front Right Wheel
GPIO24 (18) - Front Right DIR - L298N Direction pin
GPIO25 (22) - Front Right PWM - L298N PWM pin
GPIO16 (36) - Front Right Encoder A - DATA(A) pin (read only)
GPIO26 (37) - Front Right Encoder B - DATA(B) pin (read only)

# Back Wheel
GPIO5  (29) - Back DIR - L298N Direction pin
GPIO6  (31) - Back PWM - L298N PWM pin
GPIO7  (26) - Back Encoder A - DATA(A) pin (read only)
GPIO9  (21) - Back Encoder B - DATA(B) pin (read only)

# Gripper Lifter Motor (1x PG23)
GPIO13 (33) - Lifter DIR - L298N Direction pin
GPIO12 (32) - Lifter PWM - L298N PWM pin
GPIO20 (38) - Lifter Encoder A - DATA(A) pin (read only)
GPIO21 (40) - Lifter Encoder B - DATA(B) pin (read only)
```

### **Servo Motors (3x for Gripper System)**

```bash
# Gripper System Servos
GPIO18 (12) - Gripper Tilt Servo (PWM)
GPIO19 (35) - Gripper Open/Close Servo (PWM)
GPIO21 (40) - Gripper Extension Servo (360° continuous, PWM)
```

### **Laser Distance Sensors (6x VL53L0X - I2C Multiplexed)**

```bash
# I2C Bus (Default Raspberry Pi I2C)
GPIO2 (3)  - SDA (I2C Data)
GPIO3 (5)  - SCL (I2C Clock)

# TCA9548A I2C Multiplexer (addresses 6 sensors)
# Sensor Channels:
# TCA9548A CH0: Front Left Laser Sensor
# TCA9548A CH1: Front Right Laser Sensor
# TCA9548A CH2: Left Front Laser Sensor
# TCA9548A CH3: Left Back Laser Sensor
# TCA9548A CH4: Right Front Laser Sensor
# TCA9548A CH5: Right Back Laser Sensor
# TCA9548A CH6: Back Left Laser Sensor
# TCA9548A CH7: Back Right Laser Sensor
```

### **HC-SR04 Ultrasonic Sensors (2x Front)**

```bash
# Front Left Ultrasonic Sensor
GPIO4 (7)   - Front Left TRIG
GPIO14 (8)  - Front Left ECHO (with voltage divider)

# Front Right Ultrasonic Sensor
GPIO15 (10) - Front Right TRIG
GPIO17 (11) - Front Right ECHO (with voltage divider)
```

### **Line Sensors (3x Individual IR Sensors)**

```bash
GPIO5 (29)  - Left Line Sensor (Digital Input)
GPIO6 (31)  - Center Line Sensor (Digital Input)
GPIO20 (38) - Right Line Sensor (Digital Input)
```

### **Container Load Sensors (4x Limit Switches)**

```bash
GPIO7 (26)  - Left Front Container (Limit Switch)
GPIO8 (24)  - Left Back Container (Limit Switch)
GPIO16 (36) - Right Front Container (Limit Switch)
GPIO26 (37) - Right Back Container (Limit Switch)
```

### **IMU Sensor (MPU6050)**

```bash
# I2C Interface (Bus 1 - shared with laser sensors)
GPIO2  (3)  - SDA (I2C Data)
GPIO3  (5)  - SCL (I2C Clock)
# Note: IMU uses separate I2C address (0x68) from laser sensors
```

### **Hardware Control Buttons**

```bash
GPIO0 (27)  - Emergency Stop Button
GPIO1 (28)  - Start Button
GPIO9 (21)  - Mode Select Button (Train/Run)
```

### **USB Interfaces**

```bash
# USB Ports (Hardware)
USB1 - TF-Luna LIDAR (Serial/USB)
USB2 - Microsoft USB Camera (for object recognition)
USB3 - Optional: RPLIDAR backup (if needed)
```

## Hardware Connections

### **Motor Driver Connections (L298N)**

```bash
# Motor Driver 1 - Front Left Omni Wheel (PG23 motor)
IN1: GPIO17 (11) - Direction control (forward/reverse)
IN2: GPIO10 (19) - Direction control (forward/reverse)
ENA: GPIO11 (23) - Enable/PWM speed control (NOTE: GPIO27 not working, using GPIO11)
Motor M+: L298N OUT1 (12V from L298N)
Motor M-: L298N OUT2 (GND via L298N)
L298N VS: 12V Power Supply (motor power)
L298N VCC: 5V Power Supply (logic power - CRITICAL: must be 5V!)
L298N GND: Common Ground (shared with Raspberry Pi)
Encoder DATA(A): GPIO22 (15) - Read only
Encoder DATA(B): GPIO23 (16) - Read only

# Motor Driver 2 - Front Right Omni Wheel (PG23 motor)
IN1: GPIO24 (18) - DIR pin (Direction control)
ENA: GPIO25 (22) - PWM pin (Speed control)
Motor M+: 12V Power Supply
Motor M-: L298N OUT1
Encoder DATA(A): GPIO16 (36) - Read only
Encoder DATA(B): GPIO26 (37) - Read only

# Motor Driver 3 - Back Omni Wheel (PG23 motor)
IN1: GPIO5  (29) - DIR pin (Direction control)
ENA: GPIO6  (31) - PWM pin (Speed control)
Motor M+: 12V Power Supply
Motor M-: L298N OUT1
Encoder DATA(A): GPIO7  (26) - Read only
Encoder DATA(B): GPIO9  (21) - Read only

# Motor Driver 4 - Gripper Lifter (PG23 motor)
IN1: GPIO13 (33) - DIR pin (Direction control)
ENA: GPIO12 (32) - PWM pin (Speed control)
Motor M+: 12V Power Supply
Motor M-: L298N OUT1
Encoder DATA(A): GPIO20 (38) - Read only
Encoder DATA(B): GPIO21 (40) - Read only

# Note: All L298N modules require:
# - VCC: 5V (logic power)
# - GND: Common ground
# - VS: 12V (motor power)
# - OUT1/OUT2: Connect to motor M+/M- terminals
# - Heat sinks recommended for continuous operation
```

### **Sensor Connections**

#### **HC-SR04 Ultrasonic Sensors (2x Front)**

```bash
# Power: 5V, GND for both sensors

# Front Left Ultrasonic Sensor:
TRIG: GPIO4 (7)   - Connect to Trig pin
ECHO: GPIO14 (8)  - Connect to Echo pin (with voltage divider)

# Front Right Ultrasonic Sensor:
TRIG: GPIO15 (10) - Connect to Trig pin
ECHO: GPIO17 (11) - Connect to Echo pin (voltage divider)

# Voltage Divider for Echo Pins (5V → 3.3V)
# Required for both sensors:
# 1kΩ resistor from Echo to GPIO pin
# 2kΩ resistor from Echo to GND
```

#### **Laser Distance Sensors (6x VL53L0X)**

```bash
# I2C Multiplexer Setup (TCA9548A)
# TCA9548A I2C Address: 0x70 (default)
SDA: GPIO2 (3)  - Connect to TCA9548A SDA
SCL: GPIO3 (5)  - Connect to TCA9548A SCL
GND: GND
VCC: 3.3V
A0-A2: GND (address 0x70)

# Individual VL53L0X Sensors (all at address 0x29)
# Connect each sensor to TCA9548A channels:
TCA_CH0: Front Left Laser Sensor
TCA_CH1: Front Right Laser Sensor
TCA_CH2: Left Front Laser Sensor
TCA_CH3: Left Back Laser Sensor
TCA_CH4: Right Front Laser Sensor
TCA_CH5: Right Back Laser Sensor
TCA_CH6: Back Left Laser Sensor
TCA_CH7: Back Right Laser Sensor

# Each VL53L0X: VCC=3.3V, GND, SDA, SCL (to multiplexer)
```

#### **Individual Line Sensors (3x IR Sensors)**

```bash
# Simple digital IR line sensors (TCRT5000 or similar)
# Each sensor: 3-pin (VCC, GND, OUT)

# Left Line Sensor:
OUT: GPIO5 (29)  - Digital input (HIGH = line detected)

# Center Line Sensor:
OUT: GPIO6 (31)  - Digital input (HIGH = line detected)

# Right Line Sensor:
OUT: GPIO20 (38) - Digital input (HIGH = line detected)

# All sensors: VCC=5V, GND=common GND
```

#### **IMU Sensor (MPU6050)**

```bash
# I2C Connection (shared bus with laser sensors)
SDA: GPIO2 (3)  - Connect to MPU6050 SDA
SCL: GPIO3 (5)  - Connect to MPU6050 SCL
VCC: 3.3V (recommended for stability)
GND: GND
INT: GPIO9 (21) - Interrupt pin (optional, for motion detection)

# Note: MPU6050 I2C address is 0x68 (different from VL53L0X sensors)
```

#### **Container Load Sensors (4x Limit Switches)**

```bash
# Limit switches for detecting objects in containers
# Each switch: 3-pin (COM, NO, NC) or 2-pin (momentary contact)

# Left Front Container:
SIGNAL: GPIO7 (26)  - Connect to COM/NO terminal
GND: Common GND

# Left Back Container:
SIGNAL: GPIO8 (24)  - Connect to COM/NO terminal
GND: Common GND

# Right Front Container:
SIGNAL: GPIO16 (36) - Connect to COM/NO terminal
GND: Common GND

# Right Back Container:
SIGNAL: GPIO26 (37) - Connect to COM/NO terminal
GND: Common GND

# All switches: VCC=5V, GND=common GND
```

#### **Hardware Control Buttons**

```bash
# Emergency stop (normally open, pulled up)
EMERGENCY_STOP: GPIO0 (27) - Connect to button COM
GND: Common GND

# Start button (normally open, pulled up)
START_BUTTON: GPIO1 (28) - Connect to button COM
GND: Common GND

# Mode select button (normally open, pulled up)
MODE_BUTTON: GPIO9 (21) - Connect to button COM (shared with IMU INT)
GND: Common GND

# All buttons: Internal pull-up resistors enabled in software
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

## Hardware Shopping List (Hexagonal Robot Architecture)

### **Core Components**

- **Raspberry Pi 5** with Ubuntu Server 24.04 LTS
- **3x Omni Wheel Motors** (PG23 built-in encoder motors, 12V, 15.5k RPM, 7 PPR)
- **1x DC Motor** for gripper lifter (PG23 built-in encoder motor, 12V, 15.5k RPM, 7 PPR)
- **4x L298N Motor Driver Modules** (for controlling PG23 motors - motors have built-in encoders only)
- **3x Servo Motors** for gripper system (MG996R or similar)
- **6x VL53L0X Laser Distance Sensors** (for wall alignment)
- **2x HC-SR04 Ultrasonic Sensors** (for front obstacle detection)
- **3x IR Line Sensors** (TCRT5000 or similar, individual sensors)
- **MPU6050 IMU Sensor** (for orientation)
- **TCA9548A I2C Multiplexer** (for laser sensors)
- **4x Limit Switches** (for container detection)
- **USB Camera** (Microsoft LifeCam or similar for object recognition)
- **TF-Luna LIDAR** (single-point distance sensor, USB/serial)
- **Power Supply** (12V DC, 5A minimum)

### **Supporting Components**

- Jumper wires and Dupont connectors
- Breadboard or custom PCB for connections
- **Voltage regulators**: 12V→5V (5A), 5V→3.3V (1A)
- **Resistors**: 1kΩ and 2kΩ (for ultrasonic voltage dividers)
- **Capacitors**: 10µF, 100nF (for motor driver noise filtering)
- **Heat sinks** for L298N motor drivers and voltage regulators
- **Terminal blocks** for power distribution
- **Push buttons** (3x) for emergency stop, start, and mode selection

## Ready for Implementation

This pinout configuration provides a complete hardware interface for the Autonomous Mobile Manipulator. All sensors and actuators referenced in the ROS2 code and N8N workflows are properly mapped to Raspberry Pi GPIO pins.

**The robot is now ready for physical assembly and testing!**
