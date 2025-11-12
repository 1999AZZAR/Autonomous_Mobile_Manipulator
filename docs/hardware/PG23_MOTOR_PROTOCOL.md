# PG23 Motor Serial Communication Protocol

## Overview

This document describes the serial communication protocol implementation for PG23 motors. The protocol may need adjustment based on the actual motor datasheet specifications.

## Current Implementation

The motor controller uses software serial communication (bit-banging) on GPIO pins configured as DATA(A) and DATA(B).

### Serial Parameters

- **Baud Rate**: 9600 (default, configurable)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### Command Protocol

The current implementation uses simple byte commands:

| Command | Byte Value | Description |
|---------|------------|-------------|
| STOP | `0x00` | Stop motor immediately (brake mode) |
| ENABLE | `0x01` | Enable motor controller |
| DISABLE | `0x02` | Disable motor controller (free spin) |
| FORWARD | `0x03` | Set forward direction |
| REVERSE | `0x04` | Set reverse direction |

### Speed Control

Speed is sent as a second byte after the direction command:
- Speed value: 0-255 (0 = stop, 255 = max speed)
- Speed is calculated from 0.0 to 1.0 float value: `speed_byte = int(speed * 255)`

### Example Commands

**Stop Motor:**
```
Send: 0x00
```

**Forward at 50% speed:**
```
Send: 0x03 (FORWARD)
Send: 0x80 (128 = 50% of 255)
```

**Reverse at 25% speed:**
```
Send: 0x04 (REVERSE)
Send: 0x40 (64 = 25% of 255)
```

## Important Notes

1. **Protocol May Vary**: The actual PG23 motor protocol may differ. Check the motor datasheet for:
   - Exact command byte values
   - Required initialization sequence
   - Baud rate specifications
   - Checksum or CRC requirements
   - Response/acknowledgment protocol

2. **Motor Default Behavior**: 
   - When DATA pins are not connected: Motor spins freely
   - When DATA pins are connected but not initialized: Motor may still spin freely
   - **Solution**: Send STOP command immediately on initialization

3. **Initialization Sequence**:
   - GPIO pins configured (TX as output, RX as input)
   - TX pin set to idle state (HIGH)
   - STOP command sent immediately
   - Motor should now be stopped/braked

## Testing and Debugging

### Verify Motor Control

1. **Check GPIO Configuration**:
   ```python
   # Verify pins are configured correctly
   # DATA(A) should be output
   # DATA(B) should be input
   ```

2. **Test STOP Command**:
   ```python
   motor = motor_manager['front_left']
   motor.stop()  # Should stop motor immediately
   ```

3. **Test Speed Control**:
   ```python
   motor.forward(0.5)  # 50% speed forward
   time.sleep(2)
   motor.stop()
   ```

4. **Monitor Serial Signals**:
   - Use oscilloscope or logic analyzer on DATA(A) pin
   - Verify UART signals are being sent
   - Check baud rate matches motor specifications

## Protocol Adjustment

If the motor doesn't respond, you may need to adjust:

1. **Baud Rate**: Change `DEFAULT_BAUD_RATE` in `pg23_motor_controller.py`
2. **Command Bytes**: Update command constants based on datasheet
3. **Protocol Format**: Modify `_send_command()` method if protocol differs
4. **Initialization**: Add required initialization sequence if needed

## References

- PG23 Motor Datasheet (check for serial protocol specifications)
- `ros2_ws/src/my_robot_automation/scripts/pg23_motor_controller.py` - Implementation file
- `ros2_ws/src/my_robot_automation/scripts/actuator_control_server.py` - Integration file

