# Command Reference - Hexagonal 3-Wheel Omni Robot

## Overview
Complete command reference for controlling the Arduino Mega-based omni robot with motor control, sensors, and IMU integration.

---

## Command Structure

### Single Character Commands (Motor Control)
All motor-related operations use single character commands for quick access and control.

### Two Character Commands (Sensor Operations)
All sensor and IMU operations use two-character commands for clear separation.

---

## Movement Commands

### Cardinal Directions (2 Wheels Each)
```
f / F  → Forward         (FL + FR wheels)
b / B  → Backward       (FL + FR wheels)
l / L  → Strafe Left    (FL + Back wheels)
r / R  → Strafe Right   (FR + Back wheels)
```

**Movement Characteristics:**
- **Speed**: BASE_SPEED (60 RPM default)
- **Wheels Used**: Always 2 wheels for optimal efficiency
- **Direction**: Pure linear movement without rotation

### Diagonal Movements (2 Wheels Each)
```
q / Q  → Forward-Left   (FL wheel only)
e / E  → Forward-Right  (FR wheel only)
z / Z  → Backward-Left  (Back wheel only)
x / X  → Backward-Right (FR + Back wheels)
```

**Movement Characteristics:**
- **Speed**: BASE_SPEED (60 RPM default)
- **Wheels Used**: 1-2 wheels depending on direction
- **Direction**: 45-degree diagonal movement
- **Efficiency**: Optimized for smooth diagonal motion

### Rotation Commands
```
c / C  → Rotate Clockwise      (All 3 omni wheels)
w / W  → Rotate Counter-clockwise (All 3 omni wheels)
```

**Rotation Characteristics:**
- **Speed**: BASE_SPEED (60 RPM default)
- **Wheels Used**: All 3 omni wheels (FR, FL, Back)
- **Mode**: Fast rotation mode activated for 5 seconds
- **Synchronization**: Enhanced motor synchronization

### Turning Commands
```
t     → Turn Left        (FR + Back wheels forward, FL backward)
y     → Turn Right       (FL + Back wheels forward, FR backward)
a / A → Arc Left         (FL + Back forward, FR partial speed)
j / J → Arc Right        (FR + Back forward, FL partial speed)
```

**Turning Characteristics:**
- **Speed**: TURN_SPEED (40 RPM default)
- **Wheels Used**: 2-3 wheels depending on turn radius
- **Radius**: Variable turn radius based on command
- **Application**: Point turns and curved movements

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

**Speed Characteristics:**
- **Range**: 50% to 100% of BASE_SPEED
- **Application**: Fine-tune movement speed for precision
- **Persistence**: Speed multiplier maintained until changed

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

**Lifter Characteristics:**
- **Speed**: LIFT_SPEED (50 RPM default)
- **Motor**: Dedicated lifter motor (Motor 1)
- **Independence**: Separate from omni wheel control
- **Safety**: Hardware limit switches prevent over-travel

**Limit Switch Safety:**
- **Top Limit Switch** (D26): Prevents lifting beyond maximum height
- **Bottom Limit Switch** (D27): Prevents lowering beyond minimum height
- **Safety Timeout**: 5-second maximum movement time prevents stalls
- **Automatic Stop**: Motor stops immediately when limit is reached

**Compact Safety Codes:**
See `LOGGING_CODES.md` for complete documentation of all compact codes used by the system.

**Movement Logic:**
- When at TOP limit: Can only move DOWN (away from limit)
- When at BOTTOM limit: Can only move UP (away from limit)
- When not at limits: Can move in both directions

---

## Safety & System Control Commands

### Perimeter Safety System
```
se → Enable Perimeter Safety     (Virtual Bumper Enabled)
sd → Disable Perimeter Safety    (Virtual Bumper Disabled)
```

**Safety Characteristics:**
- **Virtual Bumper**: Artificial potential field obstacle avoidance
- **Force Field**: Repulsive forces from IR and ultrasonic sensors
- **Auto-Brake**: Automatic emergency stop when obstacles detected
- **Default State**: Enabled on startup for maximum safety

---

## Testing & Diagnostic Commands

### Individual Motor Tests
```
1  → Test Motor 1 (Lifter)     @ variable speed with limit switch safety
2  → Test Motor 2 (FR Wheel)   @ 97% speed for 3 seconds
3  → Test Motor 3 (FL Wheel)   @ 97% speed for 3 seconds
4  → Test Motor 4 (Back Wheel) @ 97% speed for 3 seconds
```

**Test Characteristics:**
- **Duration**: 3 seconds forward, 1 second pause, 3 seconds reverse
- **Speed**: ~4000 PWM (97% of maximum)
- **Purpose**: Verify motor connections and direction
- **Safety**: Individual motor isolation

### Pattern Tests
```
g / G  → Figure-8 Pattern (Test synchronization and coordination)
h / H  → Continuous Rotation (360° rotation test)
```

**Pattern Characteristics:**
- **Figure-8**: Tests diagonal movements and motor coordination
- **Rotation**: Tests continuous rotation and stability
- **Duration**: Continuous until stopped
- **Purpose**: System integration testing

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
- **Validity**: Each sensor shows VALID/INVALID status

### Servo Characteristics:
- **Tilt Servo**: Channel 8 (SERVO 01), controls vertical tilt (0°=up, 180°=down)
- **Gripper Servo**: Channel 9 (SERVO 02), controls gripper opening (0°=open, 180°=closed)
- **Control**: I2C communication through YFROBOT shield microcontroller with 50Hz PWM
- **Angle Range**: 0-180° for both servos
- **Initialization**: PWM frequency set to 50Hz during setup for proper servo control
- **Feedback**: Detailed serial output shows channel numbers and angles
- **Troubleshooting**: Check channel numbers if servos don't respond (try 0,1,10,11,etc.)

---

## Command Categories Summary

### Navigation (8 Commands)
```
Cardinal: f, b, l, r
Diagonal: q, e, z, x
Purpose: 8-directional movement control
```

### Rotation (6 Commands)
```
Pure Spin: c, w
Point Turns: t, y
Arc Turns: a, j
Purpose: Rotational and turning maneuvers
```

### Speed Control (6 Commands)
```
Levels: 5, 6, 7, 8, 9, 0
Purpose: Fine speed adjustment (50%-100%)
```

### Utility (4 Commands)
```
Control: s, p, v
Mode: o
Purpose: System control and status
```

### Lifter (2 Commands)
```
Movement: u, d
Purpose: Vertical lifting operations
```

### Testing (6 Commands)
```
Motors: 1, 2, 3, 4
Patterns: g, h
Purpose: Diagnostics and system testing
```

### Sensors (1 Command)
```
Readings: sr
Purpose: Comprehensive sensor data
```

---

## Command Response Examples

### Status Display (`p`)
```
=== Enhanced Motor Status ===
Omni Motors Stopped: NO
Lifter Active: NO
Lifter Status: Top=OK Bottom=OK Timeout=NO
Servo Status: Tilt=90° Gripper=90°
Synchronization: ACTIVE (Motors: 2 3 4 active) | FastMode: OFF | ForceStop: none

=== Sensor Status ===
IR Distance Sensors (mm):
  Left: 245 | 238
  Right: INVALID | 320
  Back: 180 | 175
Ultrasonic Sensors (cm):
  Front Left: 85.4 | Front Right: 92.1
Line Sensors:
  Left: OFF_LINE (234) | Center: ON_LINE (678) | Right: OFF_LINE (345)

Motor 1: Setpoint=0.0 RPM, Smoothed=0.0 RPM, SyncErr=0.0, Output=0.0
Motor 2: Setpoint=60.0 RPM, Smoothed=59.8 RPM, SyncErr=0.2, Output=25.5
Motor 3: Setpoint=60.0 RPM, Smoothed=60.1 RPM, SyncErr=-0.1, Output=24.8
Motor 4: Setpoint=60.0 RPM, Smoothed=59.9 RPM, SyncErr=0.1, Output=25.2
==================
```

### Sensor Readings (`sr`)
```
=== Detailed Sensor Readings ===
IR Distance Sensors (Sharp GP2Y0A02YK0F):
  Left 1: 245.5mm (2.35V) - VALID
  Left 2: 238.2mm (2.38V) - VALID
  Right 1: 0.0mm (0.00V) - INVALID
  Right 2: 320.8mm (1.85V) - VALID
  Back 1: 180.3mm (2.65V) - VALID
  Back 2: 175.9mm (2.67V) - VALID

HC-SR04 Ultrasonic Sensors:
  Front Left: 85.4cm (4978us) - VALID
  Front Right: 92.1cm (5392us) - VALID

Line Sensors (Threshold: 512):
  Left: 234 - OFF LINE
  Center: 678 - ON LINE
  Right: 345 - OFF LINE
========================
```

---

## Command Timing and Behavior

### Immediate Commands
- **Single Character**: Execute immediately (f, b, l, r, s, p, etc.)
- **Response Time**: < 10ms from command receipt
- **Motor Action**: Direct setpoint changes

### Buffered Commands
- **Two Character**: Require complete command before execution
- **Timeout**: 500ms for incomplete commands
- **Error Handling**: "Command timeout" message for incomplete input

### Continuous Commands
- **Movement Commands**: Continuous until stopped or new command
- **Test Commands**: Fixed duration (3 seconds + pause)
- **Pattern Commands**: Continuous until interrupted

### State Persistence
- **Speed Multiplier**: Maintained until changed (5-0 commands)
- **Fast Mode**: Temporary (rotation) or permanent (o command)

---

## Command Combinations

### Basic Movement Sequences
```
f + p → Move forward and check status
f + sr → Move forward with sensor monitoring
```

### Testing Sequences
```
1 → Test motor 1
2 → Test motor 2
3 → Test motor 3
4 → Test motor 4
g → Test coordination
```

### Emergency Procedures
```
v → Emergency stop (force all motors off)
p → Check system status
sr → Verify sensor readings
```

---

## Command Error Handling

### Invalid Commands
```
Unknown command. Available: f,b,l,r,t,y,c,w,u,d,s,p,1-4(motor test), sr(sensor readings), ls(limit switch test), v(emergency stop)
```

### Incomplete Commands
```
Command timeout - incomplete command cleared
```

### Sensor Errors
```
[Sensor]: INVALID - Range/Timeout errors displayed
```

---

## Raspberry Pi Integration Examples

### Python Serial Control
```python
import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Basic movement
ser.write(b'f')  # Forward
time.sleep(2)
ser.write(b's')  # Stop

# Servo control
ser.write(b'mc')  # Center tilt servo
time.sleep(0.5)
ser.write(b'nc')  # Close gripper
time.sleep(0.5)

# Sensor monitoring
ser.write(b'sr')  # Get sensor readings
response = ser.readline().decode()
```

### Simple Movement Sequences
```python
def move_forward(distance_seconds):
    ser.write(b'f')
    time.sleep(distance_seconds)
    ser.write(b's')

def rotate_90_clockwise():
    ser.write(b'c')
    time.sleep(1.5)  # Adjust timing for 90° rotation
    ser.write(b's')

# Figure-8 pattern
def figure_eight():
    ser.write(b'g')  # Start figure-8 pattern
    time.sleep(10)   # Run for 10 seconds
    ser.write(b's')  # Stop
```

### Sensor-Based Navigation
```python
def avoid_obstacle():
    ser.write(b'sr')  # Get sensor readings
    # Parse ultrasonic readings
    if front_distance < 30:  # cm
        ser.write(b'b')  # Backup
        time.sleep(1)
        ser.write(b'c')  # Turn
        time.sleep(0.5)
        ser.write(b's')
```

---

## Command Reference Table

| Command | Type | Description | Wheels | Speed |
|---------|------|-------------|--------|-------|
| f/F | Movement | Forward | FL+FR | BASE_SPEED |
| b/B | Movement | Backward | FL+FR | BASE_SPEED |
| l/L | Movement | Strafe Left | FL+Back | BASE_SPEED |
| r/R | Movement | Strafe Right | FR+Back | BASE_SPEED |
| q/Q | Movement | Forward-Left | FL | BASE_SPEED |
| e/E | Movement | Forward-Right | FR | BASE_SPEED |
| z/Z | Movement | Backward-Left | Back | BASE_SPEED |
| x/X | Movement | Backward-Right | FR+Back | BASE_SPEED |
| c/C | Rotation | Clockwise | All 3 | BASE_SPEED |
| w/W | Rotation | Counter-clockwise | All 3 | BASE_SPEED |
| t | Turning | Turn Left | FR+Back | TURN_SPEED |
| y | Turning | Turn Right | FL+Back | TURN_SPEED |
| a/A | Turning | Arc Left | FL+Back | Variable |
| j/J | Turning | Arc Right | FR+Back | Variable |
| s/S | Control | Stop | All | 0 |
| p/P | Control | Status | N/A | N/A |
| v/V | Control | Emergency Stop | All | 0 |
| o/O | Control | Turbo Mode Toggle | N/A | N/A |
| 5-9,0 | Speed | Speed Control (50%-100%) | N/A | Variable |
| u/U | Lifter | Lift Up | Lifter | LIFT_SPEED |
| d/D | Lifter | Lift Down | Lifter | -LIFT_SPEED |
| 1-4 | Testing | Individual Motor Test | Single | 97% |
| 1 (lifter) | Testing | Lifter Safety Test | Lifter | Variable |
| g/G | Testing | Figure-8 Pattern | All | Variable |
| h/H | Testing | Continuous Rotation | All 3 | Variable |
|| m[u/d/c] | Servo | Tilt Control (Up/Down/Center) | N/A | N/A |
|| n[o/c/h] | Servo | Gripper Control (Open/Close/Half) | N/A | N/A |
|| ta<angle> | Servo | Tilt Angle (0-180°) | N/A | N/A |
|| ga<angle> | Servo | Gripper Angle (0-180°) | N/A | N/A |
| sr | Sensors | Sensor Readings | N/A | N/A |
| ls | Lifter | Limit Switch Test | N/A | N/A |
| se | Safety | Enable Perimeter Safety | N/A | N/A |
| sd | Safety | Disable Perimeter Safety | N/A | N/A |

---

## ⚠️ **Known Issues & Missing Commands**

### Commands Listed in Help But Not Implemented
The firmware help text incorrectly lists these commands as available, but they are **NOT implemented**:

```
ir → (Not Implemented) - Listed but no functionality
ic → (Not Implemented) - Listed but no functionality
im → (Not Implemented) - Listed but no functionality
ih → (Not Implemented) - Listed but no functionality
```

**Note**: These commands appear in the startup help message but will return "Unknown command" if used.

---

## Command Groups by Function

### Primary Movement (Most Used)
**Forward/Backward**: `f`, `b`
**Strafing**: `l`, `r`
**Rotation**: `c`, `w`
**Stop**: `s`

### Precision Control
**Speed Adjustment**: `5`, `6`, `7`, `8`, `9`, `0`
**Turning**: `t`, `y`, `a`, `j`
**Turbo Mode**: `o`

### Diagnostics
**Status**: `p`
**Servo**: `m`, `n`, `ta`, `ga`
**Sensors**: `sr`
**Testing**: `1`, `2`, `3`, `4`, `g`, `h`

### Specialized
**Lifter**: `u`, `d`
**Emergency**: `v`

---

## Command Quick Reference

### Movement Control
```
   q     w     e
   ↑     ↑     ↑
a ← l  s/f  r → j
   ↓     ↓     ↓
   z     b     x
```

### Speed Control
```
5=50%  6=60%  7=70%  8=80%  9=90%  0=100%
```

### Sensor Publishing Control
```
spe = Enable sensor publishing (IR, ultrasonic data)
spd = Disable sensor publishing
```

### Individual Wheel Control
```
w[0-3][-100-100] = Set individual wheel speed (-100 to 100 = -100% to 100%)
w0-100 = Lifter motor full speed forward
w150 = Front Right motor 50% forward
w2-75 = Front Left motor 75% reverse
wstop = Stop all individual wheel control
```

### Testing Sequence
```
1 → 2 → 3 → 4 → g → h → p
```

---
*Last updated: November 22, 2025*
*Total Commands: 42 (25 single + 17 two-character)*
*Command Structure: Motor (single) | Sensors/Servo (two-character) | Publishing (two-character) | Wheels (w+)*
*Sensor Publishing: DIST:values, US:values (100ms intervals)*
*Individual Wheel Control: w[wheel][speed] for precise motor control*
*Note: IMU (MPU6050) is on Raspberry Pi, not Arduino Mega*
