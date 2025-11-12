# PG23 Motor Troubleshooting Guide

## Issue: Motor Spinning Freely / Not Responding to STOP Commands

### Problem Description
The PG23 motor continues to spin even after sending STOP/DISABLE commands via serial communication.

### Root Cause
The current serial communication protocol implementation uses **placeholder command bytes** that may not match the actual PG23 motor protocol. The motor controller defaults to free-spin mode when it doesn't receive valid commands.

### Current Workaround
The motor controller now **keeps DATA(A) pin LOW** to physically disable the motor. This prevents free spinning but may not allow proper motor control.

### Solution: Get Correct Protocol from Datasheet

**CRITICAL:** You need to obtain the PG23 motor datasheet to get the correct:
1. **Command byte values** (STOP, ENABLE, DISABLE, FORWARD, REVERSE)
2. **Baud rate** (currently set to 9600, may need to be different)
3. **Protocol format** (command structure, checksums, etc.)
4. **Initialization sequence** (required setup commands)

### Steps to Fix

1. **Find Motor Datasheet:**
   - Check motor manufacturer website
   - Look for model number on motor label
   - Contact motor supplier/manufacturer

2. **Update Command Bytes:**
   Edit `ros2_ws/src/my_robot_automation/scripts/pg23_motor_controller.py`:
   ```python
   # Update these values based on datasheet:
   CMD_STOP = b'\x??'      # Replace ?? with actual STOP command byte
   CMD_ENABLE = b'\x??'    # Replace ?? with actual ENABLE command byte
   CMD_DISABLE = b'\x??'   # Replace ?? with actual DISABLE command byte
   CMD_FORWARD = b'\x??'   # Replace ?? with actual FORWARD command byte
   CMD_REVERSE = b'\x??'   # Replace ?? with actual REVERSE command byte
   ```

3. **Update Baud Rate:**
   ```python
   DEFAULT_BAUD_RATE = 9600  # Change to correct baud rate from datasheet
   ```

4. **Test Motor Control:**
   ```python
   from pg23_motor_controller import PG23MotorController
   import lgpio
   
   h = lgpio.gpiochip_open(0)
   motor = PG23MotorController(h, 17, 27, "test")
   motor.stop()  # Should stop motor
   motor.forward(0.5)  # Should move forward at 50% speed
   ```

### Temporary Workaround (Current Implementation)

The current code keeps DATA(A) pin LOW to disable the motor. This prevents free spinning but:
- ⚠️ Motor cannot be controlled (no speed/direction control)
- ⚠️ This is a temporary fix until correct protocol is implemented

### Testing Serial Communication

To verify if serial communication is working:

1. **Use Oscilloscope/Logic Analyzer:**
   - Connect to DATA(A) pin (GPIO17 for front left motor)
   - Send a command and verify UART signals are being transmitted
   - Check baud rate matches motor specifications

2. **Test Different Baud Rates:**
   ```python
   # Try common baud rates:
   motor = PG23MotorController(h, 17, 27, "test", baud_rate=115200)
   motor = PG23MotorController(h, 17, 27, "test", baud_rate=57600)
   motor = PG23MotorController(h, 17, 27, "test", baud_rate=19200)
   ```

3. **Check Motor Response:**
   - Some motors have status LEDs that indicate command reception
   - Check motor controller for error codes or diagnostic indicators

### Motor Model Information Needed

Please provide:
- Motor manufacturer name
- Full model number (e.g., "PG23-XXX-XXX")
- Datasheet link or PDF
- Any documentation about serial communication protocol

### Files to Update After Getting Protocol

1. `ros2_ws/src/my_robot_automation/scripts/pg23_motor_controller.py`
   - Update command bytes
   - Update baud rate
   - Update protocol format if needed

2. `docs/hardware/PG23_MOTOR_PROTOCOL.md`
   - Document actual protocol
   - Add command examples
   - Add initialization sequence

### Current Status

- ✅ GPIO pins configured correctly
- ✅ Serial communication code implemented
- ✅ Motor initialization sequence in place
- ⚠️ **Command protocol needs verification from datasheet**
- ⚠️ Motor currently disabled by keeping DATA(A) LOW (temporary workaround)

