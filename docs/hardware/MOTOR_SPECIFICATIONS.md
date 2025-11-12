# Motor Specifications

## Motor Type: PG23 Built-in Encoder Motor

### General Specifications

- **Model**: PG23
- **Type**: DC Motor with Built-in Encoder
- **No-load Speed**: 15,500 RPM (15.5k RPM)
- **Encoder Resolution**: 7 PPR (Pulses Per Revolution)
- **Voltage**: 12V DC
- **Usage**: Omni wheels (3x) and Gripper lifter (1x)

### Encoder Specifications

- **PPR**: 7 pulses per revolution
- **Quadrature Encoding**: Standard A/B channel encoder
- **Effective Resolution**: 7 PPR × 4 (quadrature) = 28 counts per revolution
- **Encoder Type**: Built-in optical or magnetic encoder

### Motor Applications

#### Omni Wheel Motors (3x)
- **Location**: Front Left, Front Right, Back
- **Purpose**: Omnidirectional movement
- **Control**: L298N motor driver modules (motors have built-in encoders only)
- **Control Interface**: L298N DIR and PWM pins for motor control
- **Encoder Interface**: DATA(A) and DATA(B) pins for encoder feedback (read only)
- **GPIO Pins**: 
  - Front Left: DIR=GPIO17, PWM=GPIO27, Encoder A=GPIO22, Encoder B=GPIO23
  - Front Right: DIR=GPIO24, PWM=GPIO25, Encoder A=GPIO16, Encoder B=GPIO26
  - Back: DIR=GPIO5, PWM=GPIO6, Encoder A=GPIO7, Encoder B=GPIO9

#### Gripper Lifter Motor (1x)
- **Location**: Center of base plate
- **Purpose**: Vertical lifting mechanism for gripper assembly
- **Control**: L298N motor driver module (motor has built-in encoder only)
- **Control Interface**: L298N DIR and PWM pins for motor control
- **Encoder Interface**: DATA(A) and DATA(B) pins for encoder feedback (read only)
- **GPIO Pins**: DIR=GPIO13, PWM=GPIO12, Encoder A=GPIO20, Encoder B=GPIO21

### PG23 Motor Pinout

Each PG23 motor has 6 pins:

| Pin | Name | Description | Connection |
|-----|------|-------------|------------|
| 1 | M+ | Motor Positive Terminal | L298N OUT1 (12V from L298N) |
| 2 | M- | Motor Negative Terminal | L298N OUT2 (GND via L298N) |
| 3 | GND | Ground | Common Ground Bus |
| 4 | VIN | Encoder Power Supply | 5V Power Rail (for encoder only) |
| 5 | DATA(A) | Encoder Channel A | GPIO Pin (Input - read only) |
| 6 | DATA(B) | Encoder Channel B | GPIO Pin (Input - read only) |

**Wiring Notes:**
- M+ and M- connect to L298N motor driver OUT1 and OUT2 terminals
- L298N requires: VCC=5V (logic), VS=12V (motor power), GND=common ground
- L298N control pins: IN1=DIR pin (GPIO), ENA=PWM pin (GPIO)
- GND must be connected to common ground
- VIN requires 5V for encoder operation only
- DATA(A) and DATA(B) are encoder outputs only (read-only inputs on Raspberry Pi)
- External L298N motor driver required - motor has built-in encoder only, not driver

### Encoder Connections

All motors use built-in encoders with A/B channel outputs (read-only):

#### Omni Wheel Encoders
- **Front Left Wheel Encoder**:
  - Channel A: GPIO22 (Pin 15) - DATA(A) pin
  - Channel B: GPIO23 (Pin 16) - DATA(B) pin
  
- **Front Right Wheel Encoder**:
  - Channel A: GPIO16 (Pin 36) - DATA(A) pin
  - Channel B: GPIO26 (Pin 37) - DATA(B) pin
  
- **Back Wheel Encoder**:
  - Channel A: GPIO7 (Pin 26) - DATA(A) pin
  - Channel B: GPIO9 (Pin 21) - DATA(B) pin

#### Gripper Lifter Encoder
- **Channel A**: GPIO20 (Pin 38) - DATA(A) pin
- **Channel B**: GPIO21 (Pin 40) - DATA(B) pin

### Encoder Reading Calculation

For position and velocity feedback:

```python
# Encoder counts per wheel revolution
ENCODER_PPR = 7  # Pulses per revolution
QUADRATURE_MULTIPLIER = 4  # A/B channel quadrature
COUNTS_PER_REVOLUTION = ENCODER_PPR * QUADRATURE_MULTIPLIER  # 28 counts/rev

# Wheel radius (meters)
WHEEL_RADIUS = 0.075  # 75mm

# Distance per encoder count
DISTANCE_PER_COUNT = (2 * math.pi * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION
# Approximately 0.0168 meters per count (16.8 mm per count)
```

### Velocity Calculation

```python
# Calculate wheel velocity from encoder counts
def calculate_velocity(encoder_counts, time_delta):
    """
    Calculate wheel velocity in m/s
    
    Args:
        encoder_counts: Change in encoder counts
        time_delta: Time interval in seconds
    
    Returns:
        Velocity in m/s
    """
    revolutions = encoder_counts / COUNTS_PER_REVOLUTION
    angular_velocity = revolutions / time_delta  # rev/s
    linear_velocity = angular_velocity * 2 * math.pi * WHEEL_RADIUS  # m/s
    return linear_velocity
```

### PID Control Parameters

Recommended PID parameters for smooth motor control:

```yaml
motor_control:
  encoder_ppr: 7
  quadrature_multiplier: 4
  counts_per_revolution: 28
  
  # PID gains (may need tuning)
  pid:
    kp: 0.5    # Proportional gain
    ki: 0.01   # Integral gain
    kd: 0.1    # Derivative gain
    
  # Velocity limits
  max_velocity: 1.0      # m/s
  max_acceleration: 2.0  # m/s²
```

### Wiring Notes

- **Power**: 12V DC supply required for motor operation
- **Encoder Power**: 5V for encoder logic (if separate power required)
- **Ground**: Common ground between motor driver, encoder, and Raspberry Pi
- **Signal Levels**: Encoder outputs are typically 5V logic, use voltage dividers if needed for 3.3V GPIO

### Troubleshooting

**Issue**: Encoder not reading correctly
- Verify encoder A/B channel connections
- Check encoder power supply (5V)
- Verify GPIO pin assignments
- Test encoder output with oscilloscope or logic analyzer

**Issue**: Motor speed inconsistent
- Check encoder PPR setting in software (should be 7)
- Verify quadrature decoding is enabled
- Calibrate PID parameters for smooth control

**Issue**: Position drift
- Verify encoder connections are secure
- Check for electrical noise on encoder lines
- Ensure proper grounding
- Consider adding encoder filtering in software

### References

- Motor datasheet: PG23 specifications
- Encoder documentation: 7 PPR quadrature encoder
- GPIO pin assignments: See RASPBERRY_PI_PINOUTS.md
- Motor control: See GPIO_CONTROL_SETUP.md

