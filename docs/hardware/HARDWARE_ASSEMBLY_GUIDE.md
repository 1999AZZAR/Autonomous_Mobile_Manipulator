# 🔧 LKS Robot - Hardware Assembly Guide

## 📋 Complete Hardware Setup for Raspberry Pi Integration

This guide provides step-by-step instructions for assembling and connecting all hardware components for the LKS Robot Project.

## 🛠️ Required Tools & Materials

### **Tools**
- Soldering iron and solder
- Wire cutters/strippers
- Multimeter
- Screwdrivers (Phillips and flathead)
- Heat shrink tubing
- Breadboard for testing
- Jumper wires

### **Power Supplies**
- 12V DC power supply (5A minimum)
- 5V DC-DC converter (5A)
- 3.3V voltage regulator
- Power distribution board

### **Cables & Connectors**
- Dupont jumper wires (male-male, male-female, female-female)
- Servo extension cables
- Motor power cables (16-18 AWG)
- Sensor cables
- Terminal blocks for power distribution

## 🔌 Step-by-Step Assembly

### **Step 1: Power Distribution Setup**

#### **Main Power Supply Connections**
```bash
12V Power Supply → Power Distribution Board
├── 12V Rail → All Motor Drivers (L298N modules)
├── 12V → DC-DC Converter (12V→5V)
└── 5V → Raspberry Pi Power Input
```

#### **Voltage Regulator Setup**
```bash
# 12V → 5V DC-DC Converter
Input: 12V from main supply
Output: 5V → Raspberry Pi GPIO header (5V pins)
        5V → Motor driver logic pins
        5V → Servo power rails
        5V → Sensor power

# 5V → 3.3V Regulator (AMS1117-3.3)
Input: 5V from DC-DC converter
Output: 3.3V → MCP3008 ADC
        3.3V → MPU6050 IMU
        3.3V → Logic level shifting
```

### **Step 2: Raspberry Pi GPIO Header Connections**

#### **Servo Motor Connections**
```bash
# Servo 1: Gripper Open/Close
Signal: GPIO12 (Pin 32) → Servo 1 Signal Wire
Power: 5V → Servo 1 Power (Red)
Ground: GND → Servo 1 Ground (Brown/Black)

# Servo 2: Gripper Tilt
Signal: GPIO13 (Pin 33) → Servo 2 Signal Wire
Power: 5V → Servo 2 Power
Ground: GND → Servo 2 Ground

# Servo 3: Gripper Neck (Continuous)
Signal: GPIO18 (Pin 12) → Servo 3 Signal Wire
Power: 5V → Servo 3 Power
Ground: GND → Servo 3 Ground

# Servo 4: Gripper Base
Signal: GPIO19 (Pin 35) → Servo 4 Signal Wire
Power: 5V → Servo 4 Power
Ground: GND → Servo 4 Ground

# Servo 5: Auxiliary (Software PWM)
Signal: GPIO21 (Pin 40) → Servo 5 Signal Wire
Power: 5V → Servo 5 Power
Ground: GND → Servo 5 Ground
```

#### **Motor Driver Connections (L298N Modules)**

```bash
# Motor Driver 1: Back Wheel
IN1: GPIO17 (Pin 11) → Direction Control
IN2: GPIO27 (Pin 13) → Direction Control
ENA: GPIO27 (Pin 13) → PWM Speed Control

Motor Connections:
- Motor + → OUT1
- Motor - → OUT2
Power: 12V → VS, GND → GND
Logic: 5V → VSS

# Motor Driver 2: Front Left Wheel
IN1: GPIO24 (Pin 18) → Direction Control
IN2: GPIO25 (Pin 22) → Direction Control
ENA: GPIO25 (Pin 22) → PWM Speed Control

Motor Connections:
- Motor + → OUT1
- Motor - → OUT2
Power: 12V → VS, GND → GND
Logic: 5V → VSS

# Motor Driver 3: Front Right Wheel
IN1: GPIO16 (Pin 36) → Direction Control
IN2: GPIO26 (Pin 37) → Direction Control
ENA: GPIO26 (Pin 37) → PWM Speed Control

Motor Connections:
- Motor + → OUT1
- Motor - → OUT2
Power: 12V → VS, GND → GND
Logic: 5V → VSS

# Lifter Motor Driver
IN1: GPIO12 (Pin 32) → Direction Control ⚠️ CONFLICT with Servo 1
IN2: GPIO13 (Pin 33) → Direction Control ⚠️ CONFLICT with Servo 2
ENA: GPIO13 (Pin 33) → PWM Speed Control ⚠️ CONFLICT with Servo 2

Motor Connections:
- Motor + → OUT1
- Motor - → OUT2
Power: 12V → VS, GND → GND
Logic: 5V → VSS
```

### **Step 3: Encoder Connections**

#### **Wheel Encoders**
```bash
# Back Wheel Encoder
Encoder A: GPIO22 (Pin 15) → Encoder Channel A
Encoder B: GPIO23 (Pin 16) → Encoder Channel B
Power: 5V → Encoder VCC
Ground: GND → Encoder GND

# Front Left Wheel Encoder
Encoder A: GPIO5 (Pin 29) → Encoder Channel A
Encoder B: GPIO6 (Pin 31) → Encoder Channel B
Power: 5V → Encoder VCC
Ground: GND → Encoder GND

# Front Right Wheel Encoder
Encoder A: GPIO20 (Pin 38) → Encoder Channel A
Encoder B: GPIO21 (Pin 40) → Encoder Channel B ⚠️ CONFLICT with Servo 5
Power: 5V → Encoder VCC
Ground: GND → Encoder GND

# Lifter Encoder
Encoder A: GPIO19 (Pin 35) → Encoder Channel A ⚠️ CONFLICT with Servo 4
Encoder B: GPIO16 (Pin 36) → Encoder Channel B ⚠️ CONFLICT with Motor FR Dir
Power: 5V → Encoder VCC
Ground: GND → Encoder GND
```

### **Step 4: Sensor Connections**

#### **Ultrasonic Sensors (HC-SR04)**
```bash
# Front Ultrasonic Sensor
TRIG: GPIO4 (Pin 7) → HC-SR04 TRIG
ECHO: GPIO14 (Pin 8) → HC-SR04 ECHO (through voltage divider)
VCC: 5V → HC-SR04 VCC
GND: GND → HC-SR04 GND

# Voltage Divider for ECHO Pin (5V → 3.3V)
HC-SR04 ECHO → 1kΩ Resistor → GPIO14
HC-SR04 ECHO → 2kΩ Resistor → GND

# Back Left Ultrasonic Sensor
TRIG: GPIO15 (Pin 10) → HC-SR04 TRIG
ECHO: GPIO17 (Pin 11) → HC-SR04 ECHO (voltage divider) ⚠️ CONFLICT
VCC: 5V → HC-SR04 VCC
GND: GND → HC-SR04 GND

# Back Right Ultrasonic Sensor
TRIG: GPIO18 (Pin 12) → HC-SR04 TRIG ⚠️ CONFLICT with Servo 3
ECHO: GPIO27 (Pin 13) → HC-SR04 ECHO (voltage divider) ⚠️ CONFLICT
VCC: 5V → HC-SR04 VCC
GND: GND → HC-SR04 GND
```

#### **IR Proximity Sensors (Sharp GP2Y0A21YK)**
```bash
# MCP3008 ADC Connections
VDD: 3.3V → MCP3008 VDD
VREF: 3.3V → MCP3008 VREF
AGND: GND → MCP3008 AGND
CLK: GPIO11 (Pin 23) → MCP3008 CLK ⚠️ CONFLICT with Line Sensors
DOUT: GPIO9 (Pin 21) → MCP3008 DOUT ⚠️ CONFLICT with Line Sensors
DIN: GPIO10 (Pin 19) → MCP3008 DIN ⚠️ CONFLICT with Line Sensors
CS: GPIO8 (Pin 24) → MCP3008 CS ⚠️ CONFLICT with Line Sensors

# IR Sensor Connections
Front IR → MCP3008 CH0
Left IR → MCP3008 CH1
Right IR → MCP3008 CH2

# Individual Sensor Pins
VCC: 5V → All IR Sensors
GND: GND → All IR Sensors
VO → ADC Channels (through MCP3008)
```

#### **Line Sensor Array**
```bash
# 74HC165 Shift Register Connections
QH: GPIO10 (Pin 19) → 74HC165 QH ⚠️ CONFLICT with ADC
CLK: GPIO9 (Pin 21) → 74HC165 CLK ⚠️ CONFLICT with ADC
SH/LD: GPIO8 (Pin 24) → 74HC165 SH/LD ⚠️ CONFLICT with ADC
CLK_INH: GPIO11 (Pin 23) → 74HC165 CLK_INH ⚠️ CONFLICT with ADC
VCC: 5V → 74HC165 VCC
GND: GND → 74HC165 GND

# Line Sensor Inputs (QH to A)
QH → Sensor 1 OUT
Q → Sensor 2 OUT
... (8 sensors total)
A → Sensor 8 OUT

# Individual IR Line Sensors
VCC: 5V → All line sensors
GND: GND → All line sensors
OUT → 74HC165 inputs
```

#### **IMU Sensor (MPU6050)**
```bash
# I2C Connections (Default Raspberry Pi I2C bus)
SDA: GPIO2 (Pin 3) → MPU6050 SDA
SCL: GPIO3 (Pin 5) → MPU6050 SCL
VCC: 3.3V → MPU6050 VCC (check your module specs)
GND: GND → MPU6050 GND
INT: GPIO4 (Pin 7) → MPU6050 INT (optional)
```

## ⚠️ Pin Conflict Resolution

### **Critical Conflicts to Resolve**

#### **Lifter Motor vs Servos**
```
Current: GPIO12/13 used for both Servo 1/2 AND Lifter
Solution: Move lifter to GPIO20/21 (currently shared with encoders)
New Assignment:
- Lifter DIR: GPIO20 (Pin 38)
- Lifter PWM: GPIO21 (Pin 40)
- Front Right Encoder B: GPIO26 (Pin 37)
```

#### **Ultrasonic Sensors**
```
Current: Conflicts with motor and servo pins
Solution: Use GPIO0/1 (I2C ID pins) for additional sensors
New Assignment:
- Back Left ECHO: GPIO0 (Pin 27)
- Back Right TRIG: GPIO1 (Pin 28)
- Back Right ECHO: GPIO6 (Pin 31)
```

#### **ADC vs Line Sensors**
```
Current: Same pins used for both
Solution: Use software SPI for ADC or separate SPI bus
Alternative: Use GPIO16/20/26 for ADC (requires code changes)
```

### **Recommended Final Pinout**

```bash
# Servos (Hardware PWM)
GPIO12 (32) - Servo 1: Gripper
GPIO13 (33) - Servo 2: Gripper Tilt
GPIO18 (12) - Servo 3: Gripper Neck
GPIO19 (35) - Servo 4: Gripper Base
GPIO21 (40) - Servo 5: Auxiliary

# Motors
GPIO17 (11) - Back Wheel DIR
GPIO27 (13) - Back Wheel PWM
GPIO24 (18) - Front Left DIR
GPIO25 (22) - Front Left PWM
GPIO16 (36) - Front Right DIR
GPIO26 (37) - Front Right PWM
GPIO20 (38) - Lifter DIR
GPIO21 (40) - Lifter PWM ⚠️ CONFLICT - use GPIO6 (31) instead

# Encoders
GPIO22 (15) - Back Wheel A
GPIO23 (16) - Back Wheel B
GPIO5  (29) - Front Left A
GPIO6  (31) - Front Left B
GPIO20 (38) - Front Right A ⚠️ CONFLICT - use GPIO12 (32) instead
GPIO26 (37) - Front Right B ⚠️ CONFLICT - use GPIO13 (33) instead
GPIO19 (35) - Lifter A ⚠️ CONFLICT - use GPIO18 (12) instead
GPIO16 (36) - Lifter B ⚠️ CONFLICT - use GPIO27 (13) instead

# Ultrasonic
GPIO4  (7)  - Front TRIG
GPIO14 (8)  - Front ECHO
GPIO15 (10) - Back Left TRIG
GPIO0  (27) - Back Left ECHO
GPIO1  (28) - Back Right TRIG
GPIO6  (31) - Back Right ECHO ⚠️ CONFLICT - use GPIO25 (22) instead

# IR Proximity (ADC)
GPIO8  (24) - ADC CS ⚠️ CONFLICT - use GPIO7 (26)
GPIO9  (21) - ADC CLK ⚠️ CONFLICT - use GPIO0 (27) - CONFLICT!
GPIO10 (19) - ADC DOUT ⚠️ CONFLICT - use GPIO1 (28) - CONFLICT!
GPIO11 (23) - ADC DIN ⚠️ CONFLICT - use GPIO5 (29) - CONFLICT!

# Line Sensors (74HC165)
GPIO8  (24) - SH/LD ⚠️ CONFLICT - use GPIO7 (26)
GPIO9  (21) - CLK ⚠️ CONFLICT - use GPIO0 (27) - CONFLICT!
GPIO10 (19) - QH ⚠️ CONFLICT - use GPIO1 (28) - CONFLICT!
GPIO11 (23) - CLK_INH ⚠️ CONFLICT - use GPIO5 (29) - CONFLICT!

# IMU (I2C)
GPIO2  (3)  - SDA
GPIO3  (5)  - SCL
```

## 🔧 Software Setup

### **Enable Interfaces**
```bash
sudo raspi-config
# Interfacing Options:
# I2C: Enable
# SPI: Enable
# Serial: Enable (if needed)
# 1-Wire: Disable
```

### **Install Dependencies**
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

### **Test Hardware**
```bash
python3 docs/hardware/gpio_test.py          # Test all GPIO pins
python3 docs/hardware/gpio_test.py --quick  # Quick GPIO test
```

## 🔍 Testing & Calibration

### **Individual Component Tests**

#### **Servo Test**
```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)  # Servo 1
pwm = GPIO.PWM(12, 50)    # 50Hz for servos
pwm.start(7.5)            # 90 degrees (neutral)
time.sleep(1)
pwm.ChangeDutyCycle(5)    # 0 degrees
time.sleep(1)
pwm.ChangeDutyCycle(10)   # 180 degrees
time.sleep(1)
pwm.stop()
GPIO.cleanup()
```

#### **Motor Test**
```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Direction
GPIO.setup(27, GPIO.OUT)  # PWM
pwm = GPIO.PWM(27, 1000)  # 1kHz

# Forward
GPIO.output(17, GPIO.HIGH)
pwm.start(50)  # 50% speed
time.sleep(2)

# Stop
pwm.stop()
GPIO.cleanup()
```

#### **Ultrasonic Test**
```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 4
ECHO = 14

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, GPIO.LOW)
time.sleep(0.1)

GPIO.output(TRIG, GPIO.HIGH)
time.sleep(0.00001)
GPIO.output(TRIG, GPIO.LOW)

while GPIO.input(ECHO) == 0:
    pulse_start = time.time()

while GPIO.input(ECHO) == 1:
    pulse_end = time.time()

pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150  # Speed of sound / 2
print(f"Distance: {distance:.2f} cm")
GPIO.cleanup()
```

### **System Integration Test**

#### **ROS2 Launch Test**
```bash
# Terminal 1: Launch ROS2
ros2 launch my_robot_bringup robot.launch.py

# Terminal 2: Test web interface
python3 ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py
# Open browser: http://localhost:5000

# Terminal 3: Test API
curl http://localhost:5000/api/robot/status
curl -X POST http://localhost:5000/api/robot/move -H "Content-Type: application/json" -d '{"direction":"forward","speed":0.3}'
```

## 🔧 Troubleshooting

### **Common Issues**

#### **GPIO Permission Denied**
```bash
sudo usermod -a -G gpio $USER
sudo reboot
```

#### **I2C/SPI Not Working**
```bash
ls /dev/i2c*  # Check I2C devices
ls /dev/spi*  # Check SPI devices
sudo i2cdetect -y 1  # Scan I2C bus
```

#### **PWM Not Working**
- Check if pin supports hardware PWM
- Use software PWM as fallback: `GPIO.PWM(pin, frequency)`

#### **Motor Not Moving**
- Check power supply voltage/current
- Verify motor driver connections
- Test motor driver with simple script

#### **Sensors Not Responding**
- Check power connections (5V/3.3V)
- Verify signal connections
- Test with multimeter
- Check sensor documentation for pinouts

### **Debug Commands**
```bash
# Check GPIO status
gpio readall

# Monitor I2C
sudo i2cdetect -y 1

# Test SPI
ls /dev/spidev*

# Check system logs
dmesg | grep gpio
dmesg | grep i2c
dmesg | grep spi
```

## 📋 Final Checklist

### **Hardware Assembly**
- [ ] Power distribution board assembled
- [ ] Voltage regulators tested
- [ ] Raspberry Pi GPIO connections verified
- [ ] All sensors connected with correct pinouts
- [ ] Motors connected to drivers
- [ ] Servo motors connected
- [ ] I2C/SPI devices connected

### **Software Setup**
- [ ] Interfaces enabled (I2C, SPI)
- [ ] Dependencies installed
- [ ] GPIO permissions set
- [ ] Docker containers running
- [ ] ROS2 nodes launching
- [ ] N8N workflows imported

### **Testing & Calibration**
- [ ] Individual component tests passed
- [ ] System integration test successful
- [ ] Sensor calibration completed
- [ ] Motor PID tuning done
- [ ] Emergency stop tested

## 🚀 Ready for Operation

Once all components are assembled and tested, the LKS Robot will be fully operational with:

- **🤖 ROS2 low-level control** for real hardware
- **🌐 Web interface** for manual control
- **🔄 N8N workflows** for automation
- **📊 Real-time monitoring** of all systems

**The robot is now ready for advanced autonomous operations!** 🎯🤖⚙️
