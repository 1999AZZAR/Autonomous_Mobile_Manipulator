# Hexagonal 3-Wheel Omni Robot Control with 4 PG28 Motors

Arduino Mega code for controlling a **hexagonal-shaped 3-wheel omni robot** using **YFROBOT v2 Motor Driver Shield** with PID control and pre-built movement patterns.

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

- **Motor 1 (Lifter)**: Data A (d3) → Arduino D3, Data B (d5) → Arduino D5
- **Motor 2 (Front Right)**: Data A (d2) → Arduino D2, Data B (d4) → Arduino D4
- **Motor 3 (Front Left)**: Data A (d7) → Arduino D7, Data B (d6) → Arduino D6
- **Motor 4 (Back)**: Data A (d9) → Arduino D9, Data B (d8) → Arduino D8

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

### PID Control

- Each motor has independent PID control for precise speed regulation
- Encoder feedback for closed-loop control
- Adjustable PID parameters (Kp, Ki, Kd)
- **Separated Control**: Lifter and omni motors are controlled independently
- **Stop Protection**: PID control is disabled when motors are stopped to ensure true stopping
- **State Management**: System tracks motor states (omni active/stopped, lifter active/inactive)
- **Acceleration Limiting**: Smooth motor speed transitions prevent jerky movements
- **Lifter Safety**: Hardware limit switches prevent over-travel with automatic motor shutdown

### Lifter Safety System

The lifter mechanism includes comprehensive safety features to prevent damage and ensure reliable operation:

- **Hardware Limit Switches**: Top and bottom limit switches (D26, D27) prevent over-travel
- **Automatic Motor Shutdown**: Motor stops immediately when limit switch is triggered
- **Movement Timeout**: 5-second safety timeout prevents motor stalls
- **Smart Movement Logic**: When at a limit, only allows movement away from that limit
- **Continuous Monitoring**: Limit switches are checked during movement for immediate response
- **Virtual Bumper Exclusion**: Lifter operates independently of obstacle avoidance system

### System Logging

The robot uses compact logging codes for memory efficiency. All logging codes are documented in `LOGGING_CODES.md` with detailed explanations and examples.

### Compact Logging Codes

The system uses memory-efficient compact logging codes for minimal memory usage. All codes are documented in detail in `LOGGING_CODES.md`.

**Quick Reference:**
- `VFF:x,y,m|vx,vy` = Virtual Force Field forces and resulting velocity
- `LFT:c` = Lifter status and safety codes
- `LSD:t,b` = Limit Switch Debug (raw values)
- `LS:t,b` = Limit Switch status check

### Servo Integration

**Servo control is implemented** through the YFROBOT shield's dedicated servo channels (08 and 09). Servos are controlled via I2C communication with the shield's microcontroller.

#### Servo Hardware Setup

- **Channels**: Tilt servo on Channel 8 (SERVO 01), Gripper servo on Channel 9 (SERVO 02)
- **Control**: I2C communication through YFROBOT shield microcontroller with 50Hz PWM
- **No Arduino Pins Used**: Control handled entirely by shield (no PWM pins required)
- **Power**: Servos powered directly from YFROBOT shield
- **Angle Range**: 0-180° for both servos
- **PWM Frequency**: Set to 50Hz during initialization for proper servo operation
- **Real-time Updates**: Servo angles displayed in status output with channel info

#### Servo Features

- **Tilt Control**: Vertical tilt servo for camera/lidar positioning
- **Gripper Control**: Gripper opening/closing for manipulation tasks
- **Angle Feedback**: Current angles displayed in status commands
- **Preset Positions**: Quick commands for common positions (up/down/center/open/close)
- **Direct Angle Control**: Precise angle setting with `ta<angle>` and `ga<angle>` commands
- **Integration**: Servo status integrated into main status (`p`) command

#### Servo Commands

- **`mu`**: Tilt Up (0°)
- **`md`**: Tilt Down (180°)
- **`mc`**: Tilt Center (90°)
- **`no`**: Gripper Open (0°)
- **`nc`**: Gripper Close (180°)
- **`nh`**: Gripper Half-Open (90°)
- **`ta<angle>`**: Set tilt angle (0-180°)
- **`ga<angle>`**: Set gripper angle (0-180°)

#### Servo Integration Details

- **Channel-based Control**: Uses YFROBOT shield's dedicated servo channels
- **I2C Communication**: All servo control routed through shield microcontroller
- **Status Display**: Servo angles shown in main status (`p`) command
- **No Arduino Resources**: Doesn't consume PWM pins or timers
- **Reliable Control**: Shield handles PWM generation for stable servo operation

#### Servo Troubleshooting

If servo commands are inconsistent or don't respond:

1. **PWM Frequency**: Ensure `setPWMFreq(50)` is called during setup (50Hz required for servos)
2. **Channel Numbers**: Check your shield labeling - may be channels 0/1, 10/11, or others
3. **Power Supply**: Verify YFROBOT shield has adequate power for servo operation
4. **I2C Connection**: Test SDA/SCL connections and I2C bus health
5. **Servo Compatibility**: Ensure servos are standard hobby servos (1500μs center pulse)
6. **Debug Output**: Check serial output for "Setting servo (channel X) to Y°" messages
7. **Alternative Channels**: Try different channel numbers if 8/9 don't work:
   ```cpp
   const int TILT_SERVO_CHANNEL = 0;      // Try 0, 1, 10, 11, etc.
   const int GRIPPER_SERVO_CHANNEL = 1;
   ```
8. **Test Commands**: Start with `mc` (center) and `nc` (close) to verify basic function

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

#### Basic Movements (Cardinal Directions - 2 Wheels Each)

- **`f`/`F`**: Forward (FL & FR wheels only)
- **`b`/`B`**: Backward (FL & FR wheels only)
- **`l`/`L`**: Strafe Left (FL & Back wheels only)
- **`r`/`R`**: Strafe Right (FR & Back wheels only)

#### Motor Usage & Power Matrix

| Command               | Movement Type       | Motors Used                            | Power Level                 | Notes                                    |
| --------------------- | ------------------- | -------------------------------------- | --------------------------- | ---------------------------------------- |
| **`f`/`F`** | Pure Forward        | M2(FR) + M3(FL)                        | 50%-100% × speedMultiplier | 2-wheel cardinal                         |
| **`b`/`B`** | Pure Backward       | M2(FR) + M3(FL)                        | 50%-100% × speedMultiplier | 2-wheel cardinal                         |
| **`l`/`L`** | Strafe Left         | M3(FL) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel cardinal                         |
| **`r`/`R`** | Strafe Right        | M2(FR) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel cardinal                         |
| **`q`/`Q`** | Forward-Left        | M2(FR) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel diagonal                         |
| **`e`/`E`** | Forward-Right       | M3(FL) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel diagonal                         |
| **`z`/`Z`** | Backward-Left       | M3(FL) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel diagonal                         |
| **`x`/`X`** | Backward-Right      | M2(FR) + M4(Back)                      | 50%-100% × speedMultiplier | 2-wheel diagonal                         |
| **`t`/`T`** | Turn Left           | M2(FR) + M3(FL) ± M4(Back)            | 35%-80% × speedMultiplier  | Forward + left turn                      |
| **`y`/`Y`** | Turn Right          | M2(FR) + M3(FL) ± M4(Back)            | 35%-80% × speedMultiplier  | Forward + right turn                     |
| **`a`/`A`** | Arc Left            | M2(FR) + M3(FL) ± M4(Back)            | 40%-80% × speedMultiplier  | Forward + left curve                     |
| **`j`/`J`** | Arc Right           | M2(FR) + M3(FL) ± M4(Back)            | 40%-80% × speedMultiplier  | Forward + right curve                    |
| **`c`/`C`** | Rotate CW           | M2(FR) + M3(FL) + M4(Back)             | 50%-100% × speedMultiplier | All wheels spin                          |
| **`w`/`W`** | Rotate CCW          | M2(FR) + M3(FL) + M4(Back)             | 50%-100% × speedMultiplier | All wheels spin                          |
| **`u`/`U`** | Lift Up             | M1(Lifter) only                        | 100%                        | Independent of omni motors               |
| **`d`/`D`** | Lift Down           | M1(Lifter) only                        | 100%                        | Independent of omni motors               |
| **`1-4`**     | Motor Test          | Single motor                           | 97% max (4000/4096)         | Direct control, no PID                   |
| **`g`/`G`** | Encoder Calibration | Reset all encoder data                 | None                        | Zero positions & smoothing               |
| **`h`/`H`** | Figure-8            | M2(FR) + M3(FL) ± M4(Back)            | 30%-80% × speedMultiplier  | Smooth encoder-controlled                |
| **`o`/`O`** | Turbo Mode Toggle   | Enable/disable permanent fast response | None                        | Maximum responsiveness for all movements |

### Motor Speed Calculations

**Inverse Kinematics Formula (CORRECTED for physical motor mounting):**

```
motor_speed = (-vx) × cos(θ) × speedMultiplier + vy × sin(θ) × speedMultiplier + ω × R × speedMultiplier
```

**Motor Angles (hexagonal robot):**

- **M2 (FR)**: θ = 45° (0.785 rad)
- **M3 (FL)**: θ = 135° (2.356 rad)
- **M4 (Back)**: θ = 180° (3.14 rad)

**Example Calculations (speedMultiplier = 1.0) - CORRECTED FOR PHYSICAL MOTOR MOUNTING:**

| Command         | vx   | vy   | ω   | M2 Speed | M3 Speed | M4 Speed | Back Used? |
| --------------- | ---- | ---- | ---- | -------- | -------- | -------- | ---------- |
| **`f`** | 1.0  | 0.0  | 0.0  | -0.707   | +0.707   | 0        | No         |
| **`b`** | -1.0 | 0.0  | 0.0  | +0.707   | -0.707   | 0        | No         |
| **`l`** | 0.0  | -1.0 | 0.0  | +0.707   | -0.707   | +1.0     | Yes        |
| **`r`** | 0.0  | 1.0  | 0.0  | -0.707   | +0.707   | -1.0     | Yes        |
| **`c`** | 0.0  | 0.0  | 1.0  | +1.0     | +1.0     | +1.0     | Yes        |
| **`w`** | 0.0  | 0.0  | -1.0 | -1.0     | -1.0     | -1.0     | Yes        |

**Note:** Motor mounting direction required vx negation in inverse kinematics formula

### Enhanced Encoder Processing, Motor Synchronization & Fast Rotation

The system now includes advanced encoder processing, **motor synchronization**, and **fast rotation response** for **smoother and more uniform motor control**:

- **Moving Average Filter**: 5-sample RPM averaging reduces noise
- **Exponential Smoothing**: α=0.6 smoothing factor for responsive velocity tracking
- **Encoder-based Acceleration Limiting**: Prevents jerky movements (200 RPM normal, 500 RPM rotation)
- **Position Tracking**: Monitors motor position in revolutions
- **Motor Synchronization**: PID-based speed matching for uniform RPM across active motors
- **Fast Rotation Mode**: Bypasses acceleration limits for 2 seconds after rotation commands
- **Optimized PID Gains**: Higher gains (Kp=4.0, Ki=1.5, Kd=0.5) for omni motors
- **Real-time Status**: Enhanced `p` command shows smoothed RPM, sync errors, and position data

**Fast Rotation Features:**

- **TURBO MODE**: Permanent fast response toggle with `o` command
- **Extended Fast Response**: 5 seconds of unlimited acceleration for rotation commands
- **Aggressive PID Gains**: Kp=8.0, Ki=3.0, Kd=0.2 for omni motors (4x more responsive)
- **Adaptive Smoothing**: α=0.9 during fast rotation, α=0.6 normal
- **Synchronization Bypass**: Disabled during fast rotation for maximum speed
- **Higher Acceleration Limits**: 1000 RPM/s for rotation vs 200 RPM/s normal

**Synchronization Features:**

- **Intended Active Motor Tracking**: Explicit flags track which motors are intended to move
- **Strict 2/3-Wheel Enforcement**: Only intended motors participate in synchronization
- **Rotation-to-Translation Fix**: Prevents motors from staying active after 3-wheel rotation commands
- **Average RPM Calculation**: Computes target synchronized RPM from intended active motors
- **PID Correction**: Applies proportional correction (Kp=0.3) to maintain uniformity
- **Error Monitoring**: Displays synchronization errors and active motors in status output

**Benefits:**

- ✅ **Smoother Motor Response**: Filtered encoder feedback reduces oscillations
- ✅ **Uniform Speed Control**: Motors maintain consistent RPM through PID synchronization
- ✅ **Fast Rotation Response**: Immediate acceleration for spin commands
- ✅ **Better Acceleration**: Gradual speed changes prevent wheel slip
- ✅ **Position Awareness**: Track motor positions for advanced control
- ✅ **Noise Reduction**: Multiple filtering layers eliminate encoder jitter
- ✅ **Motor Synchronization**: Active motors automatically match speeds for perfect uniformity

### Speed Control System

**Global Speed Multiplier:** `speedMultiplier` (default: 1.0 = 100%)

- **`5`**: 0.5× speed (50%)
- **`6`**: 0.6× speed (60%)
- **`7`**: 0.7× speed (70%)
- **`8`**: 0.8× speed (80%)
- **`9`**: 0.9× speed (90%)
- **`0`**: 1.0× speed (100%)

**Formula:** `final_speed = base_speed × speedMultiplier`

- Example: Forward at 70% speed = `f` command + `7` command

### Back Wheel Activation Rules

**STRICT RULES:** Back wheel (M4) is **idle** unless:

- `|vy| > 0.1` (significant sideways movement >10%)
- `|ω| > 0.1` (significant rotation >10%)

**Examples:**

- **`f` (1.0, 0.0, 0.0)**: `|vy|=0, |ω|=0` → Back wheel idle ✅
- **`l` (0.0, -1.0, 0.0)**: `|vy|=1.0, |ω|=0` → Back wheel active ✅
- **`c` (0.0, 0.0, 1.0)**: `|vy|=0, |ω|=1.0` → Back wheel active ✅
- **`t` (0.5, 0.0, -0.8)**: `|vy|=0, |ω|=0.8` → Back wheel active ✅

#### Diagonal Movements (2-wheel combinations)

- **`q`/`Q`**: Forward-Left (FR + Back wheels)
- **`e`/`E`**: Forward-Right (FL + Back wheels)
- **`z`/`Z`**: Backward-Left (FL + Back wheels)
- **`x`/`X`**: Backward-Right (FR + Back wheels)

#### Turning & Rotation

- **`t`**: Turn Left (forward + left rotation)
- **`y`**: Turn Right (forward + right rotation)
- **`c`/`C`**: Rotate Clockwise (spin in place)
- **`w`/`W`**: Rotate Counter-Clockwise (spin in place)

#### Arc Movements (Curved paths)

- **`a`/`A`**: Arc Left (gentle left curve)
- **`j`/`J`**: Arc Right (gentle right curve)

#### Speed Control (50%-100%)

- **`5`**: 50% speed
- **`6`**: 60% speed
- **`7`**: 70% speed
- **`8`**: 80% speed
- **`9`**: 90% speed
- **`0`**: 100% speed (full speed)

#### Lifter Control

- **`u`/`U`**: Lift Up
- **`d`/`D`**: Lift Down

#### System Commands

- **`s`/`S`**: Emergency Stop (all motors)
- **`p`/`P`**: Print Status (motor RPM, PID values)
- **`1-4`**: Test individual motors (FULL POWER - 4000/4096 max speed)
- **`g`/`G`**: Full calibration sequence
- **`h`/`H`**: Figure-8 test pattern

### Command Categories for Easy Integration:

**Navigation**: `f,b,l,r,q,e,z,x` (8 directional movements - all use 2 wheels)
**Rotation**: `t,y,c,w,a,j` (6 rotation options - turns use 2-3 wheels, pure spin uses 3)
**Precision**: `5,6,7,8,9,0` (6 speed levels)
**Utility**: `s,p,u,d` (essential controls)
**Servo Control**: `mu,md,mc,no,nc,nh,ta<angle>,ga<angle>` (servo tilt and gripper control)
**Sensor Control**: `sr` (detailed sensor readings)
**Testing**: `1,2,3,4,g,h,o` (diagnostics and turbo mode)

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

# Arc movement
send_command('j')  # Arc right
time.sleep(2)
send_command('s')  # Stop

# Lifter control
send_command('u')  # Lift up
time.sleep(3)
send_command('d')  # Lift down
```

#### Simple Movement Sequences:

```python
# Square pattern
moves = ['f', 'r', 'f', 'r', 'f', 'r', 'f', 'r', 's']
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

### PID Settings

```cpp
// Default PID values (adjust based on your system)
PID pid[4] = {
  PID(&input[0], &output[0], &setpoint[0], 2.0, 0.5, 0.1, DIRECT),  // Kp=2.0, Ki=0.5, Kd=0.1
  // ... other motors with same settings
}
```

### Inverse Kinematics Tuning

The code uses **strict selective wheel usage inverse kinematics** optimized for your hexagonal robot:

- **Forward/Backward Dominant**: Only uses Front Left & Front Right wheels (Back wheel idle)
- **Sideways/Rotation Dominant**: Uses all three wheels for full omnidirectional movement
- **Movement Threshold**: Back wheel activates only when sideways or rotation > 10% of max speed

**Strict Rules:**

- If `|vy| > 0.1` OR `|ω| > 0.1`: Use all 3 wheels (omnidirectional)
- Otherwise: Use only FL & FR wheels (efficient forward/backward)

If movement directions are incorrect, adjust the `MOTOR_ANGLES` array in the code:

```cpp
const double MOTOR_ANGLES[4] = {0, 45, 135, 180};  // Lifter, FR, FL, Back
```

- **FR (Front Right)**: 45° (positive for forward movement)
- **FL (Front Left)**: 135° (negative for forward movement - counter-rotates with FR)
- **Back**: 180° (activated only for significant sideways/rotation movement)

**Key Improvements:**

- ✅ **Strict Selective Usage**: Back wheel idle unless significant sideways/rotation needed
- ✅ **Threshold-Based Control**: 10% threshold prevents unwanted wheel activation
- ✅ **Proper Inverse Kinematics**: Uses `vx * cos(θ) + vy * sin(θ) + ω * R` formula
- ✅ **Speed Normalization**: Prevents motor saturation when combining movements
- ✅ **Acceleration Limiting**: Smooth transitions (adjust `max_acceleration` in code)
- ✅ **Hexagonal Optimization**: Tuned for 135° wheel spacing

**Motor Testing:** Use individual motor tests (`1-4` commands) to verify each wheel spins in the correct direction. Tests run at **near-maximum power (4000/4096)** for 3 seconds each direction with 1-second pauses.

## Omni Wheel Configuration

The code is configured for your **3-wheel hexagonal omni robot**:

- **Motor 1 (Lifter)**: Not used for navigation (angle = 0°)
- **Motor 2 (Front Right)**: 45° from forward
- **Motor 3 (Front Left)**: 135° from forward (for counter-rotation with FR)
- **Motor 4 (Back)**: 180° from forward

**Wheel Spacing**: 135° between adjacent wheels (hexagonal robot geometry)
**Note**: This is a non-standard 3-wheel omni configuration. The inverse kinematics are calculated accordingly.

Adjust `MOTOR_ANGLES[]` array if your physical wheel placement differs.

## Usage

1. **Upload the code** to Arduino Mega
2. **Open Serial Monitor** (115200 baud)
3. **Send commands** using single characters:
   - `f` - Forward
   - `b` - Backward
   - `l` - Left
   - `r` - Right
   - `t` - Turn Left
   - `y` - Turn Right
   - `c` - Rotate CW
   - `w` - Rotate CCW
   - `u` - Lift Up
   - `d` - Lift Down
   - `s` - Stop
   - `p` - Print Status

## Tuning

### PID Tuning

1. Start with Kp=2.0, Ki=0.0, Kd=0.0
2. Increase Ki until steady-state error is eliminated
3. Add Kd to reduce overshoot
4. Fine-tune Kp for faster response

### Speed Tuning

Adjust `BASE_SPEED`, `TURN_SPEED`, and `LIFT_SPEED` constants based on your requirements and motor capabilities.

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

*Last updated: November 13, 2025*
