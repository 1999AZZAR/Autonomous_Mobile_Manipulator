# Motor Pin Assignments - Verified Working Configuration

## Front Left Motor (PG23 with L298N Driver)

### Verified Working GPIO Pins

| Signal | GPIO Pin | Physical Pin | Status | Notes |
|--------|----------|--------------|--------|-------|
| IN1 | GPIO17 | Pin 11 | ✓ Working | Direction control |
| IN2 | GPIO10 | Pin 19 | ✓ Working | Direction control |
| ENA | GPIO11 | Pin 23 | ✓ Working | Enable/PWM control |
| Encoder A | GPIO22 | Pin 15 | ✓ Working | Read only |
| Encoder B | GPIO23 | Pin 16 | ✓ Working | Read only |

### Non-Working Pins

| GPIO Pin | Physical Pin | Issue |
|----------|--------------|-------|
| GPIO27 | Pin 13 | Stays HIGH, cannot be set LOW |

### L298N Power Connections

| L298N Pin | Connection | Voltage | Critical |
|-----------|------------|---------|----------|
| VS | 12V Power Supply | ~12V | Yes - Motor power |
| VCC | 5V Power Supply | ~5V | **CRITICAL** - Logic power (must be 5V, not 3.3V!) |
| GND | Common Ground | 0V | Yes - Shared with Raspberry Pi |

### Motor Connections

| PG23 Pin | L298N Connection | Notes |
|----------|------------------|-------|
| M+ | L298N OUT1 | Motor positive |
| M- | L298N OUT2 | Motor negative |
| GND | Common Ground | Shared ground |
| VIN | 5V Power Rail | Encoder power only |
| DATA(A) | GPIO22 | Encoder feedback (read only) |
| DATA(B) | GPIO23 | Encoder feedback (read only) |

## Control Logic

### Forward Motion
- IN1 = HIGH (1)
- IN2 = LOW (0)
- ENA = HIGH (1)

### Reverse Motion
- IN1 = LOW (0)
- IN2 = HIGH (1)
- ENA = HIGH (1)

### Stop
- ENA = LOW (0)
- IN1 and IN2 can be any state

### Brake
- IN1 = HIGH (1)
- IN2 = HIGH (1)
- ENA = HIGH (1)

## Testing

Use the dedicated test script:
```bash
python3 ros2_ws/src/my_robot_automation/scripts/test_motor_dedicated.py
```

This script tests all motor control combinations and verifies GPIO pin states.

## Important Notes

1. **L298N VCC must be 5V**: Without 5V on VCC, the L298N will not recognize 3.3V GPIO signals from Raspberry Pi.

2. **GPIO27 doesn't work**: GPIO27 cannot be used for ENA control as it stays HIGH. Use GPIO11 instead.

3. **GPIO10 and GPIO11**: These pins are normally used for SPI (MOSI and SCLK). They can be used for motor control if SPI is not needed, or if SPI is configured to use different pins.

4. **Common Ground**: All components must share a common ground connection.

5. **Power Supply**: Ensure 12V power supply can provide sufficient current for motor operation.

## Files Updated

- `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py`
- `ros2_ws/src/my_robot_automation/scripts/actuator_control_server.py`
- `ros2_ws/src/my_robot_automation/scripts/test_motor_dedicated.py`
- `docs/hardware/PG23_MOTOR_CONNECTION_GUIDE.md`
- `docs/hardware/RASPBERRY_PI_PINOUTS.md`
- `docs/hardware/MOTOR_PIN_ASSIGNMENTS.md` (this file)

## Last Updated

Configuration verified and documented on current date. All pin assignments tested and confirmed working.

