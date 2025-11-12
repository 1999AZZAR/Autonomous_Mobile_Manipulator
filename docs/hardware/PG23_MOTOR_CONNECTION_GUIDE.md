# PG23 Motor Connection Guide

## Overview

The PG23 is a DC motor with built-in encoder. This guide provides detailed connection instructions, pinout information, and wiring diagrams for integrating PG23 motors into the robot system using L298N motor drivers.

## Motor Specifications

### General Specifications
- **Model**: PG23 Built-in Encoder Motor
- **Type**: DC Motor with Built-in Encoder (encoder only, not driver)
- **Voltage**: 12V DC (motor power)
- **No-load Speed**: 15,500 RPM (15.5k RPM)
- **Encoder Resolution**: 7 PPR (Pulses Per Revolution)
- **Effective Encoder Resolution**: 28 counts per revolution (7 PPR × 4 quadrature)
- **Control Interface**: L298N Motor Driver (DIR and PWM pins)
- **Encoder Interface**: DATA(A) and DATA(B) pins (read-only encoder feedback)
- **External Driver Required**: Yes (L298N motor driver module)

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
| 1 | M+ | Power | Motor Positive Terminal | L298N OUT1 (12V from L298N) |
| 2 | M- | Power | Motor Negative Terminal | L298N OUT2 (GND via L298N) |
| 3 | GND | Ground | Common Ground | Common Ground Bus |
| 4 | VIN | Power | Encoder Power Supply | 5V Power Rail (encoder only) |
| 5 | DATA(A) | Input | Encoder Channel A | GPIO Pin (Input - read only) |
| 6 | DATA(B) | Input | Encoder Channel B | GPIO Pin (Input - read only) |

## Connection Guide

### Power Connections

#### Motor Power (12V via L298N)
```
12V Power Supply (+) ──→ L298N VS (Motor Power)
12V Power Supply (-) ──→ Common Ground
L298N OUT1 ──→ PG23 M+ (Pin 1)
L298N OUT2 ──→ PG23 M- (Pin 2)
```

**Important Notes:**
- Motor connects to L298N OUT1 and OUT2 terminals
- L298N VS pin connects to 12V power supply
- L298N VCC pin connects to 5V (logic power)
- Ensure power supply can provide sufficient current for motor operation
- Use appropriate wire gauge for current requirements
- Install fuse protection on power supply line
- Heat sinks recommended on L298N modules

#### Encoder Power (5V)
```
5V Power Rail (+) ──→ PG23 VIN (Pin 4)
Common Ground ──→ PG23 GND (Pin 3)
```

**Important Notes:**
- VIN requires 5V for encoder operation only
- Ensure stable 5V supply with proper filtering
- Common ground connection is critical for proper operation

### Motor Control Connections (L298N)

**L298N Motor Driver Connections:**
```
Raspberry Pi GPIO ──→ L298N IN1 (Direction control: 1=forward, 0=reverse)
Raspberry Pi GPIO ──→ L298N ENA (PWM speed control: 0=stop, 1=full speed)
L298N OUT1 ──→ PG23 M+ (Pin 1)
L298N OUT2 ──→ PG23 M- (Pin 2)
L298N VCC ──→ 5V (Logic power)
L298N VS ──→ 12V (Motor power)
L298N GND ──→ Common Ground
```

### Encoder Connections

**Important:** DATA(A) and DATA(B) pins are encoder outputs only (read-only on Raspberry Pi).

**Connection:**
```
PG23 DATA(A) (Pin 5) ──→ Raspberry Pi GPIO (Input - Encoder A)
PG23 DATA(B) (Pin 6) ──→ Raspberry Pi GPIO (Input - Encoder B)
```

**GPIO Pin Assignments:**

| Motor Location | DIR Pin | PWM Pin | Encoder A | Encoder B | GPIO Numbers |
|----------------|---------|---------|-----------|-----------|--------------|
| Front Left | GPIO17 | GPIO27 | GPIO22 | GPIO23 | Pin 11, 13, 15, 16 |
| Front Right | GPIO24 | GPIO25 | GPIO16 | GPIO26 | Pin 18, 22, 36, 37 |
| Back | GPIO5 | GPIO6 | GPIO7 | GPIO9 | Pin 29, 31, 26, 21 |
| Lifter | GPIO13 | GPIO12 | GPIO20 | GPIO21 | Pin 33, 32, 38, 40 |

**How It Works:**
- Motor control: L298N DIR pin controls direction (1=forward, 0=reverse)
- Motor speed: L298N PWM pin controls speed (0=stop, 1=full speed, or use PWM for variable speed)
- Encoder feedback: DATA(A) and DATA(B) pins output quadrature encoder signals (read-only on Raspberry Pi)
- Motor power: L298N OUT1/OUT2 connect to motor M+/M- terminals

## Complete Wiring Diagram

### Single Motor Connection Example (Front Left Wheel)

```
                    ┌─────────────────┐
                    │  Raspberry Pi 5 │
                    │                 │
     GPIO17 (DIR) ──┤ Pin 11          │
     GPIO27 (PWM) ──┤ Pin 13          │
     GPIO22 (ENC A) ┤ Pin 15          │
     GPIO23 (ENC B) ┤ Pin 16          │
                    └─────────────────┘
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
                    ▼                  ▼
            ┌───────────────┐  ┌──────────────┐
            │  L298N Driver │  │              │
            │               │  │              │
            │  VS ←── 12V   │  │              │
            │  VCC ←── 5V   │  │              │
            │  IN1 ←── DIR  │  │              │
            │  ENA ←── PWM  │  │              │
            │  OUT1 ──┐     │  │              │
            │  OUT2 ──┘     │  │              │
            └───────────────┘  │              │
                    │          │              │
                    ▼          ▼              ▼
                    ┌───────────────────────────────┐
            │      PG23 Motor              │
            │                              │
            │  [1] M+  ←── L298N OUT1     │
            │  [2] M-  ←── L298N OUT2     │
            │  [3] GND ←── Common GND     │
            │  [4] VIN ←── 5V             │
            │  [5] DATA(A) → GPIO22 (ENC A)│
            │  [6] DATA(B) → GPIO23 (ENC B)│
            │                              │
            │  Note: Motor controlled via │
            │  L298N DIR/PWM pins         │
            │  Encoder read from DATA pins │
            └───────────────────────────────┘
```

## All Motor Connections Summary

### Front Left Motor
```
Power:
  M+  → L298N OUT1 (12V from L298N)
  M-  → L298N OUT2 (GND via L298N)
  GND → Common Ground Bus
  VIN → 5V Power Rail (encoder only)

L298N Control:
  IN1 → GPIO17 (Pin 11) - Direction control (forward/reverse)
  IN2 → GPIO10 (Pin 19) - Direction control (forward/reverse)
  ENA → GPIO11 (Pin 23) - Enable/PWM speed control
  VS → 12V Power Supply (motor power)
  VCC → 5V Power Supply (logic power - CRITICAL: must be 5V, not 3.3V!)
  GND → Common Ground (shared with Raspberry Pi)

Encoder (Read Only):
  DATA(A) → GPIO22 (Pin 15) - Encoder A
  DATA(B) → GPIO23 (Pin 16) - Encoder B
```

### Front Right Motor
```
Power:
  M+  → L298N OUT1 (12V from L298N)
  M-  → L298N OUT2 (GND via L298N)
  GND → Common Ground Bus
  VIN → 5V Power Rail (encoder only)

L298N Control:
  IN1 (DIR) → GPIO24 (Pin 18) - Direction control
  ENA (PWM) → GPIO25 (Pin 22) - Speed control
  VS → 12V Power Supply
  VCC → 5V (logic power)
  GND → Common Ground

Encoder (Read Only):
  DATA(A) → GPIO16 (Pin 36) - Encoder A
  DATA(B) → GPIO26 (Pin 37) - Encoder B
```

### Back Motor
```
Power:
  M+  → L298N OUT1 (12V from L298N)
  M-  → L298N OUT2 (GND via L298N)
  GND → Common Ground Bus
  VIN → 5V Power Rail (encoder only)

L298N Control:
  IN1 (DIR) → GPIO5 (Pin 29) - Direction control
  ENA (PWM) → GPIO6 (Pin 31) - Speed control
  VS → 12V Power Supply
  VCC → 5V (logic power)
  GND → Common Ground

Encoder (Read Only):
  DATA(A) → GPIO7 (Pin 26) - Encoder A
  DATA(B) → GPIO9 (Pin 21) - Encoder B
```

### Gripper Lifter Motor
```
Power:
  M+  → L298N OUT1 (12V from L298N)
  M-  → L298N OUT2 (GND via L298N)
  GND → Common Ground Bus
  VIN → 5V Power Rail (encoder only)

L298N Control:
  IN1 (DIR) → GPIO13 (Pin 33) - Direction control
  ENA (PWM) → GPIO12 (Pin 32) - Speed control
  VS → 12V Power Supply
  VCC → 5V (logic power)
  GND → Common Ground

Encoder (Read Only):
  DATA(A) → GPIO20 (Pin 38) - Encoder A
  DATA(B) → GPIO21 (Pin 40) - Encoder B
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

