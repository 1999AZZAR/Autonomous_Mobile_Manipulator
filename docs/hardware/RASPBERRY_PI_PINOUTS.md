# Arduino Mega Pinout Reference

All GPIO-dependent components connect to the Arduino Mega. The Raspberry Pi communicates with the Mega over serial (115200 baud) and has no direct GPIO dependencies — it can be swapped for any mini PC.

Full canonical reference: [for_the_mega/PINOUT.md](../../for_the_mega/PINOUT.md)

## Pin Summary

### Motor Encoders (Digital)

| Pin | Function |
|-----|----------|
| D2 | Motor 2 (Front Right) Encoder A |
| D3 | Motor 1 (Lifter) Encoder A |
| D4 | Motor 2 (Front Right) Encoder B |
| D5 | Motor 1 (Lifter) Encoder B |
| D6 | Motor 3 (Front Left) Encoder B |
| D7 | Motor 3 (Front Left) Encoder A |
| D8 | Motor 4 (Back) Encoder B |
| D9 | Motor 4 (Back) Encoder A |

### Ultrasonic Sensors (Digital)

| Pin | Function |
|-----|----------|
| D22 | Ultrasonic Front Left TRIGGER |
| D23 | Ultrasonic Front Left ECHO |
| D24 | Ultrasonic Front Right TRIGGER |
| D25 | Ultrasonic Front Right ECHO |

### Lifter Limit Switches (Digital)

| Pin | Function |
|-----|----------|
| D26 | Lifter Top Limit Switch (NO) |
| D27 | Lifter Bottom Limit Switch (NO) |

### IR Distance Sensors (Analog)

| Pin | Sensor | Purpose |
|-----|--------|---------|
| A0 | IR Left 1 | Left wall alignment |
| A1 | IR Left 2 | Left wall alignment (redundant) |
| A2 | IR Right 1 | Right wall alignment |
| A9 | IR Right 2 | Right wall alignment (redundant) |
| A10 | IR Back 1 | Back wall alignment |
| A11 | IR Back 2 | Back wall alignment (redundant) |

### Line Sensors (Analog)

| Pin | Sensor |
|-----|--------|
| A6 | Line Sensor Left |
| A7 | Line Sensor Center |
| A8 | Line Sensor Right |

### I2C Bus

| Pin | Function |
|-----|----------|
| D20 | SDA (to YFROBOT shield) |
| D21 | SCL (to YFROBOT shield) |

### Servo Control

Servos connect through the YFROBOT shield (not directly to Arduino pins):

| Channel | Servo |
|---------|-------|
| 08 | Tilt servo |
| 09 | Gripper servo |

### Unused Pins (Available for Expansion)

- Digital: D0-D1 (serial), D10-D19, D28-D53
- Analog: A3-A5, A12-A15

## Power Distribution

```
External Supply (7-12V)
├── Arduino VIN ──► Arduino logic (5V, 3.3V regulators)
│   └── 5V pin ──► Sensors (IR, ultrasonic, line)
└── YFROBOT Shield motor terminals ──► 4x DC motors
    └── Shield VCC ──► Servos (via onboard regulator)
```

## Serial Protocol

Pi sends commands, Mega responds. Single characters for movement, two characters for sensors/actuators.

| Category | Examples | Response |
|----------|----------|----------|
| Movement | `f`, `b`, `l`, `r`, `s` | Motor action |
| Rotation | `c`, `w`, `t`, `y` | Motor action |
| Speed | `5`-`0` (50%-100%) | Speed set |
| Lifter | `u`, `d` | Motor action (limit switches enforced) |
| Servo | `mc`, `nc`, `ta45`, `ga90` | Servo position |
| Sensors | `sr` | `SENSORS:LF:245,LB:312,...` |
| Status | `p` | Full status dump |
| Safety | `se`, `sd` | Perimeter enable/disable |

Full command list: [for_the_mega/COMMANDS.md](../../for_the_mega/COMMANDS.md)
