# Arduino Mega Pinout Reference - Hexagonal 3-Wheel Omni Robot

## Overview
Complete pinout documentation for the Arduino Mega-based omni robot control system with motor control and sensors.

---

## Arduino Mega Pin Assignments

### Digital Pins (0-53)

#### Motor Encoder Inputs (Direct Connection from Motors)
```
D2  → Motor 2 (Front Right) Encoder A
D3  → Motor 1 (Lifter) Encoder A
D4  → Motor 2 (Front Right) Encoder B
D5  → Motor 1 (Lifter) Encoder B
D6  → Motor 3 (Front Left) Encoder B
D7  → Motor 3 (Front Left) Encoder A
D8  → Motor 4 (Back) Encoder B
D9  → Motor 4 (Back) Encoder A
```

#### Ultrasonic Sensor Control (HC-SR04)
```
D22 → Ultrasonic Front Left TRIGGER
D23 → Ultrasonic Front Left ECHO
D24 → Ultrasonic Front Right TRIGGER
D25 → Ultrasonic Front Right ECHO
```

#### Lifter Limit Switches
```
D26 → Lifter Top Limit Switch (Normally Open)
D27 → Lifter Bottom Limit Switch (Normally Open)
```

#### Servo Control Channels (YFROBOT Shield)
```
Channel 08 → Tilt Servo
Channel 09 → Gripper Servo
```
*Servos controlled through YFROBOT shield microcontroller via I2C*

#### Unused Digital Pins (Available for Expansion)
```
D0, D1   → Serial RX/TX (Reserved)
D10-D13  → SPI (Available)
D14-D21  → I2C/Timer (D20=D20/SDA, D21=D21/SCL - Reserved for I2C)
D26-D53  → Available for expansion
```

---

### Analog Pins (A0-A15)

#### IR Distance Sensors (Sharp GP2Y0A02YK0F)
```
A0  → IR Distance Left 1   (shield header - direct connection)
A1  → IR Distance Left 2   (shield header - direct connection)
A2  → IR Distance Right 1  (shield header - direct connection)
A9  → IR Distance Right 2  (direct connection - jumper wire needed)
A10 → IR Distance Back 1   (direct connection - jumper wire needed)
A11 → IR Distance Back 2   (direct connection - jumper wire needed)
```

#### Line Sensors (Analog Input)
```
A6  → Line Sensor Left
A7  → Line Sensor Center
A8  → Line Sensor Right
```

#### Unused Analog Pins (Available for Expansion)
```
A9-A15 → Available for expansion
```

---

## YFROBOT v2 Motor Driver Shield Connections

### I2C Communication (Required)
```
YFROBOT SDA → Arduino D20 (SDA)
YFROBOT SCL → Arduino D21 (SCL)
YFROBOT GND → Arduino GND
YFROBOT VCC → Arduino 5V
```

**Important Notes:**
- Motor control is handled entirely via I2C communication
- No direct PWM or DIR pin connections needed
- Shield has onboard microcontroller for motor control
- Ensure proper I2C pull-up resistors if needed

---

## Motor Connections (PG28 Motors with Encoders)

### Motor 1 - Lifter Motor
```
Motor Terminals → YFROBOT Shield Motor 1 (M1+/M1-)
Encoder d3 → Arduino D3 (Data A)
Encoder d5 → Arduino D5 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 2 - Front Right Wheel
```
Motor Terminals → YFROBOT Shield Motor 2 (M2+/M2-)
Encoder d2 → Arduino D2 (Data A)
Encoder d4 → Arduino D4 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 3 - Front Left Wheel
```
Motor Terminals → YFROBOT Shield Motor 3 (M3+/M3-)
Encoder d7 → Arduino D7 (Data A)
Encoder d6 → Arduino D6 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 4 - Back Wheel
```
Motor Terminals → YFROBOT Shield Motor 4 (M4+/M4-)
Encoder d9 → Arduino D9 (Data A)
Encoder d8 → Arduino D8 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

---

## Sensor Connections Summary

### IR Distance Sensors (Sharp GP2Y0A02YK0F)
```
Sensor  | Arduino Pin | Purpose
---------|-------------|---------
IR Left 1  | A0         | Left wall alignment
IR Left 2  | A1         | Left wall alignment (redundant)
IR Right 1 | A2         | Right wall alignment
IR Right 2 | A9         | Right wall alignment (redundant)
IR Back 1  | A10        | Back wall alignment
IR Back 2  | A11        | Back wall alignment (redundant)
```

**IR Sensor Specifications:**
- Range: 20-150cm (200-1500mm)
- Output: Analog voltage 0.4V-2.7V
- Power: Connect to Arduino 5V/GND

### Ultrasonic Sensors (HC-SR04)
```
Sensor          | Trigger Pin | Echo Pin | Purpose
----------------|-------------|----------|---------
Front Left      | D22        | D23     | Front left obstacle detection
Front Right     | D24        | D25     | Front right obstacle detection
```

**Ultrasonic Sensor Specifications:**
- Range: 2-400cm
- Trigger: 10μs pulse to start measurement
- Echo: Pulse width proportional to distance
- Power: Connect to Arduino 5V/GND

### Line Sensors
```
Sensor  | Arduino Pin | Purpose
---------|-------------|---------
Left    | A6         | Left line detection
Center  | A7         | Center line detection
Right   | A8         | Right line detection
```

**Line Sensor Specifications:**
- Input: Analog voltage (0-5V)
- Output: Higher values = darker surface
- Threshold: Configurable (default 512)
- Power: Connect to Arduino 5V/GND

---

## Power Connections

### Main Power (Arduino Mega)
```
VIN → External power supply (7-12V recommended)
GND → Power ground
5V → Logic power output (for sensors)
3.3V → Low voltage output (for compatible sensors)
```

### Motor Power (Via YFROBOT Shield)
```
Motor Power → YFROBOT Shield motor terminals (M+/M-)
Logic Power → Arduino 5V/GND (via I2C)
```

### Sensor Power
```
All Sensors → Arduino 5V/GND
IR Sensors → Arduino 5V/GND
Ultrasonic → Arduino 5V/GND
Line Sensors → Arduino 5V/GND
```

---

## Pin Usage Summary

### Used Pins (37 total)
```
Digital: D2, D3, D4, D5, D6, D7, D8, D9, D20, D21, D22, D23, D24, D25, D26, D27
Analog: A0, A1, A2, A6, A7, A8, A9, A10, A11
```
*Servo control uses YFROBOT shield channels (no additional Arduino pins)*

### Reserved Pins (Serial/I2C)
```
D0, D1 → Serial communication
D20, D21 → I2C (SDA/SCL) - motor driver communication
```

### Available Pins (16 digital + 7 analog = 23 total)
```
Digital: D10-D19, D28-D53  (D10-D12 now available - analog sensors use A9-A11)
Analog: A3, A4, A5, A12, A13, A14, A15
```

---

## Connection Checklist

### Required Connections
- [ ] YFROBOT Shield I2C (SDA, SCL, GND, 5V)
- [ ] Motor encoder wires to Arduino digital pins
- [ ] Motor power to YFROBOT shield terminals
- [ ] Arduino main power (VIN/GND)

### Optional Connections
- [ ] IR Distance sensors (A0-A5, 5V, GND)
- [ ] Ultrasonic sensors (D22-D25, 5V, GND)
- [ ] Line sensors (A6-A8, 5V, GND)

### Verification Steps
1. Check all encoder connections (8 wires total)
2. Verify I2C connections (4 wires for shield)
3. Confirm motor power connections
4. Test sensor power (5V for all sensors)
5. Verify no pin conflicts

---

## Troubleshooting Pin Issues

### Common Problems
- **Motor not moving**: Check encoder connections and I2C wiring
- **Invalid sensor readings**: Check sensor power and pin assignments

### Testing Commands
- `1-4`: Individual motor tests
- `p`: Status display (includes sensor readings)
- `!`: Detailed sensor readings

---

## Expansion Options

### Available Digital Pins (D10-D19, D26-D53)
- Additional ultrasonic sensors
- Servo motors for mechanisms
- LED indicators
- Button inputs
- Additional motor encoders

### Available Analog Pins (A9-A15)
- Additional IR distance sensors
- Additional line sensors
- Potentiometers for manual tuning
- Battery voltage monitoring
- Temperature sensors

---

## Wiring Diagram Summary

```
Arduino Mega
├── Digital Pins
│   ├── D2-D9: Motor encoders (8 pins)
│   ├── D20-D21: I2C bus (shared)
│   ├── D22-D25: Ultrasonic control (4 pins)
│   └── D0-D1, D10-D19, D26-D53: Available
├── Analog Pins
│   ├── A0-A5: IR distance sensors (6 pins)
│   ├── A6-A8: Line sensors (3 pins)
│   └── A9-A15: Available (7 pins)
└── Power
    ├── VIN: Main power input
    ├── 5V: Sensor power output
    ├── 3.3V: Low voltage output
    └── GND: Common ground
```

---
*Last updated: November 13, 2025*
*Total pins used: 37/69 (54% utilization)*
