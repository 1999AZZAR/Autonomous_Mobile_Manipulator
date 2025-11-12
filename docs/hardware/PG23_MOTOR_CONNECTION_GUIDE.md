# PG23 Motor Connection Guide

## Overview

The PG23 is a DC motor with built-in encoder and motor driver. This guide provides detailed connection instructions, pinout information, and wiring diagrams for integrating PG23 motors into the robot system.

## Motor Specifications

### General Specifications
- **Model**: PG23 Built-in Encoder Motor
- **Type**: DC Motor with Built-in Driver and Encoder
- **Voltage**: 12V DC (motor power)
- **No-load Speed**: 15,500 RPM (15.5k RPM)
- **Encoder Resolution**: 7 PPR (Pulses Per Revolution)
- **Effective Encoder Resolution**: 28 counts per revolution (7 PPR × 4 quadrature)
- **Control Interface**: Serial Communication (UART/SPI) via DATA pins
- **Built-in Driver**: Yes (no external motor driver required)

### Power Requirements
- **Motor Power**: 12V DC (M+ terminal)
- **Controller/Encoder Power**: 5V DC (VIN terminal)
- **Current**: Check motor datasheet for specific current requirements
- **Ground**: Common ground connection required

## Pinout Diagram

### Physical Pinout

```
    ┌─────────────────┐
    │   PG23 Motor    │
    │                 │
    │  [1] M+         │ ← Motor Positive (12V)
    │  [2] M-         │ ← Motor Negative (Ground)
    │  [3] GND        │ ← Ground (Common)
    │  [4] VIN        │ ← Encoder/Controller Power (5V)
    │  [5] DATA(A)    │ ← Serial TX / Encoder Channel A
    │  [6] DATA(B)    │ ← Serial RX / Encoder Channel B
    └─────────────────┘
```

### Pin Description Table

| Pin # | Name | Type | Description | Connection |
|-------|------|------|-------------|------------|
| 1 | M+ | Power | Motor Positive Terminal | 12V Power Supply (direct connection) |
| 2 | M- | Power | Motor Negative Terminal | Ground |
| 3 | GND | Ground | Common Ground | Common Ground Bus |
| 4 | VIN | Power | Encoder/Controller Power Supply | 5V Power Rail |
| 5 | DATA(A) | I/O | Serial Control TX / Encoder Channel A | GPIO Pin (TX Output / Encoder Input) |
| 6 | DATA(B) | I/O | Serial Control RX / Encoder Channel B | GPIO Pin (RX Input / Encoder Input) |

## Connection Guide

### Power Connections

#### Motor Power (12V)
```
12V Power Supply (+) ──→ PG23 M+ (Pin 1)
12V Power Supply (-) ──→ PG23 M- (Pin 2)
```

**Important Notes:**
- Connect M+ directly to 12V power supply (no external driver needed)
- Ensure power supply can provide sufficient current for motor operation
- Use appropriate wire gauge for current requirements
- Install fuse protection on power supply line

#### Encoder/Controller Power (5V)
```
5V Power Rail (+) ──→ PG23 VIN (Pin 4)
Common Ground ──→ PG23 GND (Pin 3)
```

**Important Notes:**
- VIN requires 5V for encoder and built-in controller operation
- Ensure stable 5V supply with proper filtering
- Common ground connection is critical for proper operation

### Control and Encoder Connections

**Important:** The PG23 motor has only 6 pins total. DATA(A) and DATA(B) pins serve dual purpose:
1. Serial communication for motor control (TX/RX)
2. Encoder feedback (A/B channels)

The motor's built-in controller handles both functions on the same pins. You connect DATA(A) and DATA(B) to GPIO pins, and the motor controller manages both serial communication and encoder output.

**Connection:**
```
Raspberry Pi GPIO ──→ PG23 DATA(A) (Pin 5) - Serial TX / Encoder A
Raspberry Pi GPIO ──→ PG23 DATA(B) (Pin 6) - Serial RX / Encoder B
```

**GPIO Pin Assignments:**

| Motor Location | DATA(A) Pin | DATA(B) Pin | GPIO Numbers | Notes |
|----------------|-------------|-------------|--------------|-------|
| Front Left | GPIO17 | GPIO27 | Pin 11, Pin 13 | Same pins for control & encoder |
| Front Right | GPIO22 | GPIO23 | Pin 15, Pin 16 | Same pins for control & encoder |
| Back | GPIO24 | GPIO25 | Pin 18, Pin 22 | Same pins for control & encoder |
| Gripper Lifter | GPIO13 | GPIO12 | Pin 33, Pin 32 | Same pins for control & encoder |

**How It Works:**
- The motor's built-in controller uses DATA(A) and DATA(B) for serial communication (motor control commands)
- The same pins also output quadrature encoder signals (A/B channels) for position feedback
- The controller multiplexes or handles both functions automatically
- Your software reads encoder signals from the same GPIO pins used for serial control

## Complete Wiring Diagram

### Single Motor Connection Example (Front Left Wheel)

```
                    ┌─────────────────┐
                    │  Raspberry Pi 5 │
                    │                 │
     GPIO17 (TX) ───┤ Pin 11          │
     GPIO27 (RX) ───┤ Pin 13          │
     GPIO5 (ENC A) ─┤ Pin 29          │
     GPIO6 (ENC B) ─┤ Pin 31          │
                    └─────────────────┘
                            │
                            │
                    ┌───────┴────────┐
                    │                │
                    ▼                ▼
            ┌───────────────┐  ┌──────────────┐
            │  12V Supply   │  │   5V Supply   │
            │               │  │               │
            │  (+)          │  │  (+)          │
            └───────┬───────┘  └───────┬───────┘
                    │                  │
                    │                  │
                    ▼                  ▼
                    ┌───────────────────────────────┐
            │      PG23 Motor              │
            │                              │
            │  [1] M+  ←── 12V            │
            │  [2] M-  ←── GND            │
            │  [3] GND ←── Common GND     │
            │  [4] VIN ←── 5V             │
            │  [5] DATA(A) ←── GPIO17     │
            │              (TX/Encoder A) │
            │  [6] DATA(B) ←── GPIO27     │
            │              (RX/Encoder B) │
            │                              │
            │  Note: DATA(A) and DATA(B)  │
            │  handle both serial control │
            │  and encoder feedback       │
            └───────────────────────────────┘
```

## All Motor Connections Summary

### Front Left Motor
```
Power:
  M+  → 12V Power Supply
  M-  → Ground
  GND → Common Ground Bus
  VIN → 5V Power Rail

Control & Encoder (Same Pins):
  DATA(A) → GPIO17 (Pin 11) - Serial TX / Encoder A
  DATA(B) → GPIO27 (Pin 13) - Serial RX / Encoder B
```

### Front Right Motor
```
Power:
  M+  → 12V Power Supply
  M-  → Ground
  GND → Common Ground Bus
  VIN → 5V Power Rail

Control & Encoder (Same Pins):
  DATA(A) → GPIO22 (Pin 15) - Serial TX / Encoder A
  DATA(B) → GPIO23 (Pin 16) - Serial RX / Encoder B
```

### Back Motor
```
Power:
  M+  → 12V Power Supply
  M-  → Ground
  GND → Common Ground Bus
  VIN → 5V Power Rail

Control & Encoder (Same Pins):
  DATA(A) → GPIO24 (Pin 18) - Serial TX / Encoder A
  DATA(B) → GPIO25 (Pin 22) - Serial RX / Encoder B
```

### Gripper Lifter Motor
```
Power:
  M+  → 12V Power Supply
  M-  → Ground
  GND → Common Ground Bus
  VIN → 5V Power Rail

Control & Encoder (Same Pins):
  DATA(A) → GPIO13 (Pin 33) - Serial TX / Encoder A
  DATA(B) → GPIO12 (Pin 32) - Serial RX / Encoder B
```

## Encoder Specifications

### Encoder Details
- **Type**: Built-in Quadrature Encoder
- **PPR**: 7 Pulses Per Revolution (on one channel)
- **Quadrature Multiplier**: 4 (for A and B channels, rising and falling edges)
- **Counts Per Revolution (CPR)**: 7 PPR × 4 = 28 counts per motor shaft revolution
- **Output**: A and B channels (digital square waves, 5V logic)
- **Resolution**: Suitable for position and velocity feedback

### Encoder Reading
The encoder provides quadrature-encoded signals on DATA(A) and DATA(B) pins. With quadrature decoding (reading both rising and falling edges of both channels), the effective resolution is 4 times the PPR.

**Calculation:**
```
Counts Per Revolution (CPR) = PPR × 4 = 7 × 4 = 28 counts/revolution
```

### Velocity Calculation
```
Angular Velocity (rad/s) = (Δcounts / Δtime) × (2π / Counts Per Revolution)
Linear Velocity (m/s) = (Δcounts / Δtime) × Distance Per Count
```

## Serial Communication Protocol

### Control Interface
The PG23 motor uses serial communication for motor control commands. The exact protocol depends on the motor's built-in controller specifications.

**Typical Serial Parameters:**
- **Baud Rate**: Check motor datasheet (commonly 9600, 115200, etc.)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

**Note:** Refer to PG23 motor datasheet for specific serial protocol details, command format, and baud rate settings.

## Power Distribution

### Power Supply Requirements
```
Main Power Supply (12V)
├── Motor 1 (Front Left) M+
├── Motor 2 (Front Right) M+
├── Motor 3 (Back) M+
└── Motor 4 (Gripper Lifter) M+

5V Power Rail
├── Motor 1 VIN
├── Motor 2 VIN
├── Motor 3 VIN
├── Motor 4 VIN
└── Other 5V components

Common Ground Bus
├── All motor GND pins
├── All motor M- terminals
├── Raspberry Pi ground
└── Power supply ground
```

### Fuse Protection
- Install appropriate fuses on 12V power lines
- Recommended: 5A fuse per motor or 20A fuse for all motors combined
- Use fast-blow fuses for protection

## Wiring Checklist

### Before Powering On
- [ ] All M+ terminals connected to 12V power supply
- [ ] All M- terminals connected to ground
- [ ] All GND pins connected to common ground bus
- [ ] All VIN pins connected to 5V power rail
- [ ] DATA(A) and DATA(B) pins connected to correct GPIO pins (same pins for control & encoder)
- [ ] Common ground verified between all components
- [ ] Power supply voltage verified (12V and 5V)
- [ ] Fuses installed and checked
- [ ] Wire gauge appropriate for current requirements
- [ ] Connections secure and properly insulated

## Troubleshooting

### Motor Not Moving
**Symptoms:** Motor does not respond to control commands

**Checklist:**
1. Verify 12V power supply is connected to M+ terminal
2. Verify ground connection to M- terminal
3. Check 5V supply to VIN pin (encoder/controller power)
4. Verify serial communication pins (TX/RX) are connected correctly
5. Check serial communication settings (baud rate, protocol)
6. Verify GPIO pins are configured correctly in software
7. Test serial communication with diagnostic tool
8. Check for loose connections

### Encoder Not Reading
**Symptoms:** No encoder feedback, position readings are zero or incorrect

**Checklist:**
1. Verify 5V power to VIN pin
2. Check DATA(A) and DATA(B) pin connections (same pins used for encoder feedback)
3. Verify GPIO pins configured correctly (DATA A as output for TX, DATA B as input for RX/encoder)
4. Check for proper pull-up resistors if required
5. Test encoder signals with oscilloscope or logic analyzer (read from DATA pins)
6. Verify quadrature decoding in software
7. Check for interference or noise on DATA lines
8. Note: Encoder signals come from same DATA pins used for serial control

### Motor Runs Erratically
**Symptoms:** Motor speed varies unexpectedly, jerky motion

**Checklist:**
1. Check power supply stability (voltage ripple)
2. Verify adequate current capacity of power supply
3. Check for loose connections
4. Verify serial communication integrity
5. Check for electromagnetic interference
6. Verify PID control parameters are tuned correctly
7. Check encoder readings for noise or errors

### Serial Communication Errors
**Symptoms:** Cannot communicate with motor, timeout errors

**Checklist:**
1. Verify DATA(A) and DATA(B) pin connections (DATA A = TX, DATA B = RX)
2. Check baud rate matches motor specifications
3. Verify serial protocol settings (data bits, parity, stop bits)
4. Check for proper ground connection (critical for serial)
5. Test with known-good serial device
6. Verify GPIO pins are configured correctly (DATA A as output, DATA B as input)
7. Check for interference on DATA lines
8. Note: Same pins handle both serial control and encoder feedback

## Safety Considerations

### Electrical Safety
- Always disconnect power before making wiring changes
- Use appropriate wire gauge for current requirements
- Install fuses for overcurrent protection
- Ensure proper insulation on all connections
- Verify voltage levels before connecting
- Use proper tools and techniques for connections

### Mechanical Safety
- Ensure motor is securely mounted
- Verify mechanical connections are tight
- Check for binding or mechanical interference
- Test motor operation before final assembly
- Use appropriate mounting hardware

### Operational Safety
- Test motors individually before system integration
- Verify emergency stop functionality
- Test encoder feedback before autonomous operation
- Monitor motor temperature during operation
- Check for unusual sounds or vibrations

## Additional Resources

### Related Documentation
- `MOTOR_SPECIFICATIONS.md` - Detailed motor specifications
- `RASPBERRY_PI_PINOUTS.md` - Complete GPIO pin assignments
- `HARDWARE_ASSEMBLY_GUIDE.md` - Assembly instructions
- `motor_config.yaml` - Software configuration file

### External Resources
- PG23 Motor Datasheet (refer to manufacturer documentation)
- Serial Communication Protocol Documentation
- Encoder Reading Best Practices

## Quick Reference

### Pin Summary
```
Pin 1 (M+)     → 12V Power Supply
Pin 2 (M-)     → Ground
Pin 3 (GND)    → Common Ground Bus
Pin 4 (VIN)    → 5V Power Rail
Pin 5 (DATA A) → GPIO Pin (Serial TX / Encoder A - same pin)
Pin 6 (DATA B) → GPIO Pin (Serial RX / Encoder B - same pin)
```

### GPIO Quick Reference
```
Front Left:  DATA(A)=GPIO17, DATA(B)=GPIO27 (control & encoder on same pins)
Front Right: DATA(A)=GPIO22, DATA(B)=GPIO23 (control & encoder on same pins)
Back:        DATA(A)=GPIO24, DATA(B)=GPIO25 (control & encoder on same pins)
Lifter:      DATA(A)=GPIO13, DATA(B)=GPIO12 (control & encoder on same pins)
```

### Power Quick Reference
```
Motor Power:  12V DC (M+ terminal)
Controller:   5V DC (VIN terminal)
Ground:       Common ground required
Current:      Check motor datasheet
```

---

**Last Updated:** 2025-01-XX  
**Document Version:** 1.0

