# Triangular 3-Wheel Omni Robot Control with 4 PG28 Motors

Arduino Mega code for controlling a **triangular 3-wheel omni robot** using **YFROBOT v2 Motor Driver Shield** with PID control and direct motor combinations.

## Hardware Setup

### YFROBOT v2 Shield Connections

**Important**: The YFROBOT v2 shield uses **I2C communication** with an onboard microcontroller. Motor control is handled via I2C, not direct PWM pins!

**Connections Required:**

**I2C Communication (Shield ↔ Arduino Mega):**

- **SDA**: Arduino SDA (Pin 20)
- **SCL**: Arduino SCL (Pin 21)
- **GND**: Connect to Arduino GND
- **5V/VCC**: Connect to Arduino 5V

**Encoder Connections (Motor → Arduino Direct):**

- **Motor 1 (FR - Front Right)**: Data A (d2) → Arduino D2, Data B (d4) → Arduino D4
- **Motor 2 (FL - Front Left)**: Data A (d7) → Arduino D7, Data B (d6) → Arduino D6
- **Motor 3 (Back)**: Data A (d9) → Arduino D9, Data B (d8) → Arduino D8
- **Motor 0 (Lifter)**: Data A (d3) → Arduino D3, Data B (d5) → Arduino D5

**Power Connections:**

- **Motor Power**: Connect to YFROBOT v2 shield motor terminals (M+, M-)
- **Motor Signal**: Connect motor encoder pins (d#) to Arduino pins as above
- **Main Power**: Shield gets power from Arduino 5V/GND via I2C

**Note**: No PWM or DIR pins need to be connected - the shield handles motor control internally via I2C!

### Important Notes

- The shield uses I2C communication - ensure SDA/SCL connections are secure
- Encoder feedback provides closed-loop PID control for precise movement
- If motor directions are reversed, adjust the `motorConfig()` call in setup()
- Ensure proper power supply for motors (YFROBOT v2 supports the current draw)

## Features

### Movement System (Direct Motor Combinations)

The robot uses **8 base movements** with direct motor combinations for simpler, more predictable control:

| Command | Description | Motors Used | Direction |
|---------|-------------|-------------|-----------|
| `f` | Forward | FR + FL | Both forward |
| `b` | Backward | FR + FL | Both backward |
| `q` | Forward-Left | FR + Back | Both forward |
| `e` | Forward-Right | FL + Back | Both forward |
| `z` | Backward-Left | FR + Back | FR forward, Back backward |
| `x` | Backward-Right | FL + Back | FL forward, Back backward |
| `t` | Turn Left (CCW) | All 3 | All forward |
| `y` | Turn Right (CW) | All 3 | All backward |

### Motor Mapping

| Motor | Wheel | Location | Angle |
|-------|-------|----------|-------|
| Motor 1 | FR (Front Right) | 45° | Forward-Right |
| Motor 2 | FL (Front Left) | 135° | Forward-Left |
| Motor 3 | Back | 180° | Rear |
| Motor 0 | Lifter | N/A | Vertical lift |

### IMU Integration (Raspberry Pi)

Rotation is controlled by the Raspberry Pi using IMU feedback:
1. Raspi sends `t` or `y` to Arduino
2. Arduino spins all 3 wheels continuously
3. Raspi monitors IMU heading in real-time
4. When target angle reached (±3°), Raspi sends `s` (stop)

## Sensor Integration

### Sensor Hardware Setup

**Important**: The YFROBOT v2 Motor Driver Shield does NOT have built-in sensor interfaces. All sensors must be connected directly to Arduino pins.

**6x IR Distance Sensors (Sharp GP2Y0A02YK0F)** - Wall Alignment

- **Range**: 20-150cm (200-1500mm)
- **Output**: Analog voltage 0.4V-2.7V (linear relationship with distance)
- **Pins**: A0-A2, A9-A11 (Left1, Left2, Right1, Right2, Back1, Back2)
- **Purpose**: Wall alignment and obstacle detection on left/right/back sides
- **Connection**: Connect to Arduino analog pins (A0-A2 direct on shield, A9-A11 via jumper wires)
- **Power**: Connect sensor VCC to Arduino 5V, GND to Arduino GND
- **Note**: Shield headers only expose A0-A2; remaining sensors need jumper wires to A9-A11

**2x HC-SR04 Ultrasonic Sensors** - Front Distance

- **Range**: 2-400cm
- **Pins**: Trig(22,24), Echo(23,25) for Front Left/Right
- **Purpose**: Front obstacle detection and precise front distance measurement
- **Connection**: Connect directly to Arduino digital pins as specified
- **Power**: Connect sensor VCC to Arduino 5V, GND to Arduino GND

**3x Line Sensors** - Line Following/Navigation

- **Pins**: A6-A8 (Left, Center, Right)
- **Purpose**: Line following, alignment with floor lines, navigation assistance
- **Connection**: Connect sensor output pins directly to Arduino analog pins A6-A8
- **Power**: Connect sensor VCC to Arduino 5V, GND to Arduino GND

### Sensor Features

#### IR Distance Sensor Processing

- **Voltage-to-Distance Conversion**: Automatic conversion using Sharp GP2Y0A02YK0F formula
- **Validity Checking**: Only valid readings (within 200-1500mm range) are used
- **Real-time Updates**: Sensors updated every 100ms in main loop
- **Wall Alignment**: 2 sensors per side provide redundancy and better alignment

#### Ultrasonic Sensor Processing

- **Pulse Timing**: Standard HC-SR04 pulse timing with 30ms timeout
- **Distance Calculation**: Automatic conversion using speed of sound (343 m/s)
- **Timeout Handling**: Invalid readings flagged when no echo received
- **Front Detection**: Left/right front sensors for obstacle avoidance

#### Line Sensor Processing

- **Threshold Detection**: Configurable threshold (default 512) for line detection
- **Raw Value Reading**: Analog readings provide sensitivity information
- **Three-Sensor Array**: Left/Center/Right for precise line following
- **Navigation Aid**: Can be used for both line following and general navigation

#### Sensor Integration Features

- **Automatic Updates**: All sensors updated periodically in main loop
- **Data Structures**: Organized sensor data with validity flags
- **Status Display**: Sensor readings shown in `p` (status) command
- **Detailed Readings**: `sr` command provides comprehensive sensor data
- **Error Handling**: Invalid readings properly flagged and handled

### Sensor Commands

#### Detailed Sensor Readings (`sr` command)

```
=== Detailed Sensor Readings ===
IR Distance Sensors (Sharp GP2Y0A02YK0F):
  Left 1: 450.5mm (2.10V) - VALID
  Left 2: 432.1mm (2.15V) - VALID
  Right 1: 0.0mm (0.00V) - INVALID
  ...

HC-SR04 Ultrasonic Sensors:
  Front Left: 125.4cm (7352us) - VALID
  Front Right: 0.0cm (0us) - TIMEOUT

Line Sensors (Threshold: 512):
  Left: 234 - OFF LINE
  Center: 678 - ON LINE
  Right: 345 - OFF LINE
========================
```

### Sensor Applications

#### Wall Alignment

- **Left/Right IR Sensors**: Maintain consistent distance from walls
- **Redundant Sensors**: 2 sensors per side provide reliable alignment
- **Range**: 20-150cm optimal for wall following in corridors

#### Obstacle Avoidance

- **Front Ultrasonic**: Detect obstacles before collision
- **Wide Coverage**: Left/right front sensors cover approach angles
- **Long Range**: Up to 4m detection range for navigation planning

#### Line Following/Navigation

- **Three-Sensor Array**: Precise line detection and centering
- **Floor Navigation**: Follow lines or navigate to marked positions
- **Dual Purpose**: Can be used for line following or general navigation

### Sensor Troubleshooting

#### IR Sensor Issues

- **Invalid Readings**: Check voltage range (0.4-2.7V expected)
- **Noisy Readings**: Add filtering or averaging if needed
- **Range Limits**: Sensors may give invalid readings outside 20-150cm

#### Ultrasonic Sensor Issues

- **No Echo**: Check power, connections, and obstacle angles
- **Timeout Errors**: May occur with absorbent surfaces or extreme angles
- **Interference**: Multiple ultrasonics may interfere - use timing offsets

#### Line Sensor Issues

- **Threshold Tuning**: Adjust threshold (default 512) for your surface
- **Lighting**: Consistent lighting important for reliable detection
- **Surface Contrast**: Ensure good contrast between line and floor

### Complete Command Set for Raspberry Pi Integration

#### Movement Commands (8 Base Movements)

| Command | Description | Motors Used | Speed |
|---------|-------------|-------------|-------|
| `f`/`F` | Forward | FR + FL | BASE_SPEED |
| `b`/`B` | Backward | FR + FL | BASE_SPEED |
| `q`/`Q` | Forward-Left | FR + Back | BASE_SPEED |
| `e`/`E` | Forward-Right | FL + Back | BASE_SPEED |
| `z`/`Z` | Backward-Left | FR + Back | BASE_SPEED |
| `x`/`X` | Backward-Right | FL + Back | BASE_SPEED |
| `t` | Turn Left (CCW) | All 3 | TURN_SPEED |
| `y` | Turn Right (CW) | All 3 | TURN_SPEED |
| `s`/`S` | Stop | All | 0 |

#### Motor Usage Matrix

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

#### Control Commands

- **`s`/`S`**: Emergency Stop (all motors)
- **`p`/`P`**: Print Status (motor RPM, PID values)
- **`u`/`U`**: Lift Up
- **`d`/`D`**: Lift Down
- **`5`-`0`**: Speed Control (50%-100%)
- **`o`/`O`**: Turbo Mode Toggle

#### Sensor Commands

- **`sr`**: Detailed sensor readings (IR, ultrasonic, line sensors)
- **`mu`/`md`/`mc`**: Tilt Up/Down/Center
- **`no`/`nc`/`nh`**: Gripper Open/Close/Half
- **`ta<angle>`**: Set tilt angle (0-180°)
- **`ga<angle>`**: Set gripper angle (0-180°)

#### Testing Commands

- **`1`-`4`**: Test individual motors (FR, FL, Back, Lifter)
- **`g`/`G`**: Figure-8 pattern test
- **`h`/`H`**: Continuous rotation test

### Raspberry Pi Integration Examples

#### Python Serial Control:

```python
import serial
import time

# Connect to Arduino Mega
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Send commands
def send_command(cmd):
    ser.write(cmd.encode())
    time.sleep(0.1)  # Small delay

# Movement examples
send_command('f')  # Forward
time.sleep(2)
send_command('s')  # Stop

# Speed control
send_command('7')  # 70% speed
send_command('f')  # Forward at 70% speed

# Diagonal movement
send_command('q')  # Forward-left
time.sleep(1.5)
send_command('s')  # Stop

# Lifter control
send_command('u')  # Lift up
time.sleep(3)
send_command('d')  # Lift down
```

#### IMU-Based Rotation:

```python
# Turn robot 90 degrees left
mega.turn_robot(direction='left', target_angle=90, get_heading_fn=get_heading)

# Rotate continuously (Raspi controls stop)
mega.rotate_robot(direction='cw', speed=60, get_heading_fn=get_heading)
```

#### Simple Movement Sequences:

```python
# Square pattern
moves = ['f', 'y', 'f', 'y', 'f', 'y', 'f', 'y', 's']
for move in moves:
    send_command(move)
    time.sleep(2 if move != 's' else 0.5)

# Figure-8 autonomous
send_command('h')  # Built-in figure-8 pattern
```

## Configuration

### Motor Parameters

```cpp
const double MAX_RPM = 100.0;        // Maximum RPM for motors
const double BASE_SPEED = 60.0;      // Base speed for movements
const double TURN_SPEED = 40.0;      // Speed for turning movements
const double LIFT_SPEED = 50.0;      // Speed for lifter motor
```

### Encoder Settings

```cpp
const int ENCODER_CPR = 28;          // Counts per revolution for PG28
const double GEAR_RATIO = 1.0;       // Adjust if motors are geared
```

### Motor Angles

```cpp
const double MOTOR_ANGLES[4] = {0, 45, 135, 180};  // Lifter, FR, FL, Back
```

## Usage

1. **Upload the code** to Arduino Mega
2. **Open Serial Monitor** (115200 baud)
3. **Send commands** using single characters:
   - `f` - Forward
   - `b` - Backward
   - `q` - Forward-Left
   - `e` - Forward-Right
   - `z` - Backward-Left
   - `x` - Backward-Right
   - `t` - Turn Left (CCW)
   - `y` - Turn Right (CW)
   - `u` - Lift Up
   - `d` - Lift Down
   - `s` - Stop
   - `p` - Print Status

## Troubleshooting

### Common Issues

1. **Motors not moving**: Check power connections and motor driver enable pins
2. **Erratic movement**: Verify encoder connections and polarities
3. **Poor PID performance**: Tune PID parameters for your specific motors
4. **Wrong direction**: Check motor wiring and `MOTOR_ANGLES` configuration

### Debug Information

Use the `p` command to print current motor status including:

- Target RPM (setpoint)
- Current RPM (input)
- PID output values

## Dependencies

- **PID_v1 library**: Install via Arduino IDE Library Manager
- **YFROBOT Motor Driver Library**: [https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library](https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library) - Required for YFROBOT v2 Motor Driver Shield control
- Arduino Mega board
- R27889 motor driver shield (YFROBOT v2 Shield)
- 4x PG28 motors with encoders

## License

This code is provided as-is for educational and robotics projects.

---

*Last updated: June 2026*
