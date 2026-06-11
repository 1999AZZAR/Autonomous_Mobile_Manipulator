# Command Reference - Triangular 3-Wheel Omni Robot

## Overview
Complete command reference for controlling the Arduino Mega-based omni robot with direct motor combinations and IMU integration.

---

## Movement Commands (8 Base Movements)

### Forward / Backward (2 Wheels)
```
f / F  → Forward         (FR + FL wheels forward)
b / B  → Backward        (FR + FL wheels backward)
```

### Diagonal Movements (2 Wheels Each)
```
q / Q  → Forward-Left    (FR + Back wheels forward)
e / E  → Forward-Right   (FL + Back wheels forward)
z / Z  → Backward-Left   (FR forward + Back backward)
x / X  → Backward-Right  (FL forward + Back backward)
```

### Rotation (All 3 Wheels)
```
t     → Turn Left        (All 3 wheels forward — CCW spin)
y     → Turn Right       (All 3 wheels backward — CW spin)
```

**Rotation Control:**
- Arduino spins all 3 wheels continuously
- Raspi monitors IMU heading in real-time
- When target angle reached (±3°), Raspi sends stop command

---

## Motor Mapping

| Motor | Wheel | Location |
|-------|-------|----------|
| Motor 1 | FR (Front Right) | 45° position |
| Motor 2 | FL (Front Left) | 135° position |
| Motor 3 | Back | 180° position |
| Motor 0 | Lifter | Vertical lift |

### Movement to Motor Matrix

| Command | FR (M1) | FL (M2) | Back (M3) | Description |
|---------|---------|---------|-----------|-------------|
| `f` | Forward | Forward | Idle | Forward |
| `b` | Backward | Backward | Idle | Backward |
| `q` | Forward | Idle | Forward | Forward-Left |
| `e` | Idle | Forward | Forward | Forward-Right |
| `z` | Forward | Idle | Backward | Backward-Left |
| `x` | Idle | Forward | Backward | Backward-Right |
| `t` | Forward | Forward | Forward | Rotate CCW (all forward) |
| `y` | Backward | Backward | Backward | Rotate CW (all backward) |
| `s` | Stop | Stop | Stop | Stop all |

---

## Control Commands

### Basic Controls
```
s / S  → Stop            (Stop all motors immediately)
p / P  → Status Display  (Show comprehensive system status)
v / V  → Emergency Stop  (Force stop with state reset)
```

### Speed Control
```
5  → Speed 50%   (0.5 × BASE_SPEED)
6  → Speed 60%   (0.6 × BASE_SPEED)
7  → Speed 70%   (0.7 × BASE_SPEED)
8  → Speed 80%   (0.8 × BASE_SPEED)
9  → Speed 90%   (0.9 × BASE_SPEED)
0  → Speed 100%  (1.0 × BASE_SPEED - maximum)
```

### Special Modes
```
o / O  → Turbo Mode Toggle (Permanent fast rotation mode)
```

---

## Lifter Commands

### Lifter Motor Control (Motor 1)
```
u / U  → Lift Up          (Lifter motor forward)
d / D  → Lift Down        (Lifter motor reverse)
```

**Limit Switch Safety:**
- **Top Limit Switch** (D26): Prevents lifting beyond maximum height
- **Bottom Limit Switch** (D27): Prevents lowering beyond minimum height
- **Safety Timeout**: 5-second maximum movement time prevents stalls

---

## Safety & System Control Commands

### Perimeter Safety System
```
se → Enable Perimeter Safety     (Virtual Bumper Enabled)
sd → Disable Perimeter Safety    (Virtual Bumper Disabled)
```

---

## Testing & Diagnostic Commands

### Individual Motor Tests
```
1  → Test Motor 1 (FR Wheel)   @ 97% speed for 3 seconds
2  → Test Motor 2 (FL Wheel)   @ 97% speed for 3 seconds
3  → Test Motor 3 (Back Wheel) @ 97% speed for 3 seconds
4  → Test Motor 4 (Lifter)     @ variable speed with limit switch safety
```

### Pattern Tests
```
g / G  → Figure-8 Pattern (Test diagonal movements and motor coordination)
h / H  → Continuous Rotation (360° rotation test)
```

---

## Sensor Commands

### Servo Control Commands
```
mu  → Tilt Up             (Set tilt servo to up position)
md  → Tilt Down           (Set tilt servo to down position)
mc  → Tilt Center         (Set tilt servo to center position)
no  → Gripper Open        (Set gripper servo to open position)
nc  → Gripper Close       (Set gripper servo to closed position)
nh  → Gripper Half-Open   (Set gripper servo to half-open position)
ta<angle> → Tilt Angle     (Set tilt servo to specific angle 0-180°)
ga<angle> → Gripper Angle  (Set gripper servo to specific angle 0-180°)
```

### Sensor Readings Command
```
sr  → Sensor Readings     (Detailed readings from all sensors)
```

**Sensor Data Includes:**
- **IR Distance Sensors**: 6 readings (Left1/2, Right1/2, Back1/2) in mm
- **Ultrasonic Sensors**: 2 readings (Front Left/Right) in cm
- **Line Sensors**: 3 readings (Left/Center/Right) with threshold detection

---

## Command Reference Table

| Command | Type | Description | Wheels | Speed |
|---------|------|-------------|--------|-------|
| f/F | Movement | Forward | FR+FL | BASE_SPEED |
| b/B | Movement | Backward | FR+FL | BASE_SPEED |
| q/Q | Movement | Forward-Left | FR+Back | BASE_SPEED |
| e/E | Movement | Forward-Right | FL+Back | BASE_SPEED |
| z/Z | Movement | Backward-Left | FR+Back | BASE_SPEED |
| x/X | Movement | Backward-Right | FL+Back | BASE_SPEED |
| t | Rotation | Turn Left (CCW) | All 3 | TURN_SPEED |
| y | Rotation | Turn Right (CW) | All 3 | TURN_SPEED |
| s/S | Control | Stop | All | 0 |
| p/P | Control | Status | N/A | N/A |
| v/V | Control | Emergency Stop | All | 0 |
| o/O | Control | Turbo Mode Toggle | N/A | N/A |
| 5-9,0 | Speed | Speed Control (50%-100%) | N/A | Variable |
| u/U | Lifter | Lift Up | Lifter | LIFT_SPEED |
| d/D | Lifter | Lift Down | Lifter | -LIFT_SPEED |
| 1-4 | Testing | Individual Motor Test | Single | 97% |
| g/G | Testing | Figure-8 Pattern | All | Variable |
| h/H | Testing | Continuous Rotation | All 3 | Variable |
| m[u/d/c] | Servo | Tilt Control | N/A | N/A |
| n[o/c/h] | Servo | Gripper Control | N/A | N/A |
| ta<angle> | Servo | Tilt Angle (0-180°) | N/A | N/A |
| ga<angle> | Servo | Gripper Angle (0-180°) | N/A | N/A |
| sr | Sensors | Sensor Readings | N/A | N/A |
| se | Safety | Enable Perimeter Safety | N/A | N/A |
| sd | Safety | Disable Perimeter Safety | N/A | N/A |

---

## Keyboard Shortcuts (Frontend)

| Key | Action | Command | Wheels |
|-----|--------|---------|--------|
| W / ↑ | Forward | `f` | FR + FL |
| S / ↓ | Backward | `b` | FR + FL |
| A / ← | Rotate Left | `t` | All 3 |
| D / → | Rotate Right | `y` | All 3 |
| Q | Forward-Left | `q` | FR + Back |
| E | Forward-Right | `e` | FL + Back |
| Z | Backward-Left | `z` | FR + Back (reversed) |
| X | Backward-Right | `x` | FL + Back (reversed) |
| Space | Stop | `s` | All |

---

## IMU Integration (Raspberry Pi)

### Rotation Control Flow
1. Raspi sends `t` or `y` command to Arduino Mega
2. Arduino spins all 3 wheels continuously at TURN_SPEED
3. Raspi monitors IMU heading in real-time
4. When target angle reached (±3°), Raspi sends `s` (stop) to Arduino
5. Arduino stops all motors

### Python API
```python
# Turn robot to specific heading
mega.turn_robot(direction='left', target_angle=90, get_heading_fn=get_heading)

# Rotate continuously (Raspi controls stop)
mega.rotate_robot(direction='cw', speed=60, get_heading_fn=get_heading)
```

---

*Last updated: June 2026*
*Total Commands: 30+ (20 single + 10 two-character)*
*Note: IMU (MPU6050) is on Raspberry Pi, not Arduino Mega*