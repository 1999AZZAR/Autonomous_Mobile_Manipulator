# Hardware Overview

Hexagonal 3-wheel omnidirectional mobile manipulator. All GPIO-dependent sensors and actuators live on Arduino Mega. Raspberry Pi communicates over serial and runs the higher-level control stack.

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Chassis | Hexagonal, 3 omni wheels in triangle + 1 lifter wheel |
| Drive | True holonomic (omnidirectional), 8 directions + rotation |
| MCU | Arduino Mega 2560 |
| SBC | Raspberry Pi 5 (swappable — any mini PC works) |
| Motor Driver | YFROBOT v2 DC Motor Driver Shield (I2C, not L298N) |
| Motors | 4x PG28 DC motors with built-in encoders (3 omni + 1 lifter) |
| Servos | 2x (gripper + tilt, controlled via YFROBOT shield I2C) |
| IMU | MPU6050 (on Raspberry Pi via I2C) |

## Motor Layout

```
        Front Left (Motor 3)
            /
           /
  Back (Motor 4) ---- [Center] ---- Front Right (Motor 2)
           \
            \
        Lifter (Motor 1) — vertical gripper assembly
```

- **Motor 2, 3, 4**: Omni wheels at 120-degree spacing. Pure linear movement uses 2 wheels; diagonal uses 1-2.
- **Motor 1**: Lifter. Moves gripper assembly up/down. Hardware limit switches at top (D26) and bottom (D27).

## Sensors

### Distance Sensors (on Arduino Mega)

| Sensor | Type | Pin | Range | Purpose |
|--------|------|-----|-------|---------|
| IR Left 1 | Sharp GP2Y0A02YK0F | A0 | 200-1500mm | Left wall alignment |
| IR Left 2 | Sharp GP2Y0A02YK0F | A1 | 200-1500mm | Left wall alignment (redundant) |
| IR Right 1 | Sharp GP2Y0A02YK0F | A2 | 200-1500mm | Right wall alignment |
| IR Right 2 | Sharp GP2Y0A02YK0F | A9 | 200-1500mm | Right wall alignment (redundant) |
| IR Back 1 | Sharp GP2Y0A02YK0F | A10 | 200-1500mm | Back wall alignment |
| IR Back 2 | Sharp GP2Y0A02YK0F | A11 | 200-1500mm | Back wall alignment (redundant) |
| Ultrasonic FL | HC-SR04 | D22/D23 | 2-400cm | Front left obstacle |
| Ultrasonic FR | HC-SR04 | D24/D25 | 2-400cm | Front right obstacle |

### Line Sensors (on Arduino Mega)

| Sensor | Pin | Purpose |
|--------|-----|---------|
| Left | A6 | Line detection |
| Center | A7 | Line detection |
| Right | A8 | Line detection |

Threshold default: 512. Higher analog value = darker surface.

### Other Sensors

| Sensor | Location | Interface | Purpose |
|--------|----------|-----------|---------|
| MPU6050 IMU | Raspberry Pi | I2C (GPIO2/GPIO3) | Heading, acceleration |
| USB Camera | Raspberry Pi | USB | Object recognition (AI mode only) |
| TF-Luna LiDAR | Raspberry Pi | USB serial | Single-point distance |

## Actuators

### Gripper Assembly

The gripper, tilt servo, camera, and TF-Luna LiDAR are mounted on a shared tilt axis. One servo controls the tilt angle for the entire assembly.

- **Tilt servo**: Channel 8 on YFROBOT shield. 0 degrees = up, 180 degrees = down.
- **Gripper servo**: Channel 9 on YFROBOT shield. 0 degrees = open, 180 degrees = closed.
- **Lifter motor**: Motor 1 on YFROBOT shield. Moves the entire gripper assembly vertically.

### YFROBOT v2 Shield

The motor driver shield handles:
- 4x DC motor control (I2C commands, no direct PWM/DIR wiring needed)
- 2x servo control (channels 8 and 9)
- Onboard microcontroller for motor commutation

I2C connection to Arduino Mega: SDA (D20), SCL (D21), GND, 5V.

## Power

- **Input**: 7-12V DC via Arduino VIN
- **Logic**: 5V from Arduino/regulator (sensors, encoders)
- **Motors**: Via YFROBOT shield motor terminals (from main supply)
- **Separate supplies** recommended for motors and logic to reduce noise.

## Communication Architecture

```
Raspberry Pi                    Arduino Mega
┌──────────────┐   Serial     ┌──────────────┐
│  Flask API   │──(115200)───►│   Mega FW    │
│  ROS2 nodes  │◄────────────│  Sensors     │
│  AI engine   │              │  Motors      │
│  Frontend    │              │  Servos      │
└──────────────┘              └──────────────┘
```

The Pi sends single-character movement commands and two-character sensor/actuator commands over serial. The Mega responds with sensor data as key:value pairs.

Full command reference: [for_the_mega/COMMANDS.md](../../for_the_mega/COMMANDS.md)
Full pinout reference: [for_the_mega/PINOUT.md](../../for_the_mega/PINOUT.md)
