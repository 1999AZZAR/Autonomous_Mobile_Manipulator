# Compact Logging Codes Reference

This document explains all compact logging codes used in the Hexagonal 3-Wheel Omni Robot system. The compact codes minimize memory usage while maintaining full debugging and monitoring capabilities.

---

## Virtual Force Field (VFF)

Format: `VFF:x,y,m|vx,vy`

**Parameters:**
- `x` = Force X component (-1.0 to 1.0)
- `y` = Force Y component (-1.0 to 1.0)
- `m` = Force magnitude (0.0 to 1.0)
- `vx` = Resulting velocity X (-1.5 to 1.5)
- `vy` = Resulting velocity Y (-1.5 to 1.5)

**Example:** `VFF:0.23,-0.16,0.28|0.15,0.83`

**Meaning:** Virtual force field applied with force vector (0.23, -0.16), magnitude 0.28, resulting in velocity (0.15, 0.83)

---

## Lifter Status (LFT)

Format: `LFT:code`

### Movement Codes
- `U` = Lifting Up (normal operation)
- `D` = Lifting Down (normal operation)
- `T` = Testing (motor test initiated)

### Safety Codes
- `1` = TOP_BLOCK (attempted to move up when already at top limit)
- `2` = BOTTOM_BLOCK (attempted to move down when already at bottom limit)
- `3` = TIMEOUT (movement exceeded 5-second safety timeout)
- `4` = HIT_TOP (hit top limit switch while moving up)
- `5` = HIT_BOTTOM (hit bottom limit switch while moving down)

### Test Codes
- `T` = Testing (motor test initiated)
- `UT` = UP Test (starting up direction test)
- `UC` = UP Complete (up direction test finished)
- `DT` = DOWN Test (starting down direction test)
- `DC` = DOWN Complete (down direction test finished)
- `TC` = Test Complete (all motor tests finished)

**Examples:**
- `LFT:U` = Normal lifting up operation
- `LFT:1` = Cannot move up - already at top limit
- `LFT:4` = Hit top limit while moving up - emergency stop

---

## Limit Switch Debug (LSD)

Format: `LSD:top,bottom`

**Parameters:**
- `top` = Raw digital read value from top limit switch (0=pressed, 1=open)
- `bottom` = Raw digital read value from bottom limit switch (0=pressed, 1=open)

**Example:** `LSD:1,0`

**Meaning:** Top switch open (1), bottom switch pressed (0)

---

## Limit Switch Status (LS)

Format: `LS:top,bottom`

**Parameters:** Same as LSD format

**Example:** `LS:1,1`

**Meaning:** Both switches open (not pressed)

---

## System Status (STS)

Format: `STS:sync,vb,ctrl`

**Parameters:**
- `sync` = Synchronization status (1=ACTIVE, 0=INACTIVE)
- `vb` = Virtual bumper status (1=ENABLED, 0=DISABLED)
- `ctrl` = Control status (1=ACTIVE, 0=DISABLED)

**Example:** `STS:1,1,1`

**Meaning:** Sync active, virtual bumper enabled, control active

---

## Turbo Mode (TURBO)

Format: `TURBO:c`

**Parameters:**
- `c` = Turbo mode status (1=ENABLED, 0=DISABLED)

**Example:** `TURBO:1`

**Meaning:** Turbo mode enabled

---

## Virtual Bumper Control (VB/VBE/VBD)

Format: `VB:c`, `VBE:c`, `VBD:c`

**Parameters:**
- `c` = Virtual bumper status (1=ENABLED, 0=DISABLED)

**Examples:**
- `VB:1` = Virtual bumper currently enabled
- `VBE:1` = Virtual bumper enabled (command confirmation)
- `VBD:0` = Virtual bumper disabled (command confirmation)

---

## Speed Control (SPD)

Format: `SPD:percentage`

**Parameters:**
- `percentage` = Speed multiplier as percentage (50, 60, 70, 80, 90, 100)

**Examples:**
- `SPD:100` = Speed set to 100%
- `SPD:50` = Speed set to 50%

---

## Synchronization (SYNC)

Format: `SYNC:target,count`

**Parameters:**
- `target` = Target synchronization RPM
- `count` = Number of active motors in synchronization

**Example:** `SYNC:95.2,3`

**Meaning:** Synchronizing 3 motors to 95.2 RPM target

---

## Motor Status (MTR)

Format: `MTR:id,setpoint,rpm,syncerr,position,output`

**Parameters:**
- `id` = Motor ID (1-4)
- `setpoint` = Target setpoint RPM
- `rpm` = Smoothed measured RPM
- `syncerr` = Synchronization error
- `position` = Motor position in revolutions
- `output` = PID output value

**Example:** `MTR:1,100.0,95.2,4.8,12.34,45.6`

**Meaning:** Motor 1: setpoint 100.0 RPM, measured 95.2 RPM, sync error 4.8, position 12.34 rev, output 45.6

---

## I2C Bus Status (I2C)

Format: `I2C:status`

**Parameters:**
- `status` = I2C bus health (1=OK, 0=ERROR)

**Example:** `I2C:1`

**Meaning:** I2C bus is healthy

---

## Emergency Brake (EB)

Format: `EB:code`

### Codes
- `1` = Ultrasonic critical distance
- `2` = IR critical distance
- `3` = Multiple sensor critical
- `4` = System emergency stop

**Example:** `EB:2`

**Meaning:** Emergency brake triggered by IR sensor critical distance

---

## Motor Synchronization (SYNC)

Format: `SYNC:active_motors,target_rpm`

**Parameters:**
- `active_motors` = Binary mask of active motors (1=M1, 2=M2, 4=M3)
- `target_rpm` = Synchronization target RPM

**Example:** `SYNC:6,85.5`

**Meaning:** Motors 2 and 3 active, synchronizing to 85.5 RPM

---

## Servo Status (SERVO)

Format: `SERVO:tilt_angle,gripper_angle`

**Parameters:**
- `tilt_angle` = Current tilt servo angle (0-180°)
- `gripper_angle` = Current gripper servo angle (0-180°)

**Example:** `SERVO:90,45`

**Meaning:** Tilt servo at 90° (centered), gripper servo at 45° (partially open)

---

## Sensor Readings (SEN)

Format: `SEN:left,right,back,front`

**Parameters:**
- `left` = Left IR distance (mm)
- `right` = Right IR distance (mm)
- `back` = Back IR distance (mm)
- `front` = Front ultrasonic distance (mm)

**Example:** `SEN:245,180,320,450`

**Meaning:** Left=245mm, Right=180mm, Back=320mm, Front=450mm

---

## System Status (SYS)

Format: `SYS:mode,active,speed`

**Parameters:**
- `mode` = System mode (0=STOPPED, 1=MOVING, 2=LIFTER_ACTIVE, 3=TESTING)
- `active` = Active components bitmask
- `speed` = Current speed multiplier (0.0-1.0)

**Example:** `SYS:1,6,0.75`

**Meaning:** Moving mode, motors 2&3 active, 75% speed

---

## Error Codes (ERR)

Format: `ERR:code,component`

**Parameters:**
- `code` = Error code number
- `component` = Affected component (0=GENERAL, 1=MOTOR1, 2=MOTOR2, 3=MOTOR3, 4=MOTOR4, 5=SENSORS)

### Common Error Codes
- `1` = Communication timeout
- `2` = Sensor failure
- `3` = Motor stall detected
- `4` = Limit switch malfunction
- `5` = Memory allocation failed

**Example:** `ERR:3,2`

**Meaning:** Motor stall detected on motor 2

---

## Performance Metrics (PERF)

Format: `PERF:cpu,mem,temp`

**Parameters:**
- `cpu` = CPU usage percentage (0-100)
- `mem` = Memory usage percentage (0-100)
- `temp` = System temperature (°C)

**Example:** `PERF:45,87,32`

**Meaning:** 45% CPU usage, 87% memory usage, 32°C temperature

---

## Usage Guidelines

### For Firmware Developers
1. **Use compact codes only** - no verbose explanations in Arduino code
2. **Refer to this document** for code meanings during development/debugging
3. **Add new codes here** when extending system functionality
4. **Test logging output** with serial monitor to verify readability
5. **Keep codes consistent** with existing naming conventions

### For System Operators
1. **Use this reference** to interpret serial output from the robot
2. **Codes are designed** to be parsed by monitoring software
3. **All codes are documented** for effective troubleshooting
4. **Regular updates** - check this file when firmware is updated

### For External Developers
1. **Integrate with monitoring systems** using these codes
2. **Parse compact format** for efficient data transmission
3. **Use error codes** for automated system health monitoring
4. **Reference this document** for API-like code interpretation

### Memory Impact
- **Compact codes reduce string literals** in firmware significantly
- **Each verbose message replaced** saves ~20-50 bytes of PROGMEM
- **Total memory savings**: ~15-20% of logging overhead
- **Faster transmission**: Shorter messages improve serial communication speed
- **Professional appearance**: Clean, uniform logging output

---

## Adding New Codes

When adding new compact logging codes:

1. Choose a 2-3 letter prefix (e.g., VFF, LFT, SEN)
2. Use colon separator: `PREFIX:value1,value2,value3`
3. Document in this file with parameter explanations
4. Add inline comment in code: `// PREFIX: description`
5. Test with serial monitor to verify readability

---

*Last updated: November 13, 2025*
*Total codes documented: 16 categories*
