# Motor Power Control Solution

## Problem
PG23 motors continue spinning even when DATA pins are set LOW or serial commands are sent. This is because:
1. Motor receives 12V power directly on M+ terminal
2. DATA pins don't control motor power - they only send control commands
3. Without correct protocol, motor controller defaults to free-spin mode

## Immediate Solution: Add Power Relay/Switch

### Option 1: Add Relay to Control 12V Power (Recommended)

Add a relay module to control 12V power supply to motors:

```
12V Power Supply (+) ──→ Relay COM
Relay NO (Normally Open) ──→ Motor M+ (12V)
Relay Control ──→ GPIO Pin (e.g., GPIO20)
```

**GPIO Pin Assignment:**
- GPIO20 (or any available GPIO) → Relay control pin
- When GPIO20 = LOW: Relay OFF → Motor power disconnected → Motor stops
- When GPIO20 = HIGH: Relay ON → Motor power connected → Motor can run

**Implementation:**
```python
import lgpio

h = lgpio.gpiochip_open(0)
RELAY_PIN = 20  # Motor power relay control

# Stop motor by disconnecting power
lgpio.gpio_claim_output(h, RELAY_PIN)
lgpio.gpio_write(h, RELAY_PIN, 0)  # LOW = Relay OFF = Motor power OFF

# Enable motor power
lgpio.gpio_write(h, RELAY_PIN, 1)  # HIGH = Relay ON = Motor power ON
```

### Option 2: Manual Power Switch (Temporary)

For immediate testing, add a physical switch/breaker on the 12V power line:
- Install switch between 12V power supply and motor M+ terminals
- Turn OFF to stop motors
- Turn ON to enable motors

### Option 3: Disconnect Power Physically

**SAFETY:** To stop motor immediately:
1. Disconnect 12V power supply from motor M+ terminal
2. Motor will stop immediately
3. Reconnect when ready to test again

## Long-term Solution: Get Motor Datasheet

**CRITICAL:** You need the PG23 motor datasheet to implement correct serial protocol:

1. **Find Motor Model Number:**
   - Check motor label/sticker
   - Look for model like "PG23-XXX" or similar

2. **Get Datasheet:**
   - Search manufacturer website
   - Contact supplier/manufacturer
   - Check motor documentation

3. **Required Information:**
   - Serial communication protocol
   - Command byte values (STOP, ENABLE, DISABLE, FORWARD, REVERSE)
   - Baud rate (currently using 9600, may need different)
   - Protocol format (command structure, checksums)
   - Initialization sequence

4. **Update Code:**
   - Edit `ros2_ws/src/my_robot_automation/scripts/pg23_motor_controller.py`
   - Update command bytes with correct values
   - Update baud rate if needed
   - Test motor control

## Current Status

- ✅ GPIO pins configured correctly
- ✅ Serial communication code implemented
- ⚠️ **Command protocol is wrong (placeholder bytes)**
- ⚠️ **Motor spins freely because protocol doesn't work**
- ⚠️ **Need relay or datasheet to properly control motor**

## Recommended Action Plan

1. **Immediate (Safety):**
   - Add relay to control 12V power (Option 1)
   - OR disconnect 12V power manually when not testing

2. **Short-term:**
   - Get PG23 motor datasheet
   - Find correct serial protocol
   - Update code with correct command bytes

3. **Long-term:**
   - Implement proper motor control with correct protocol
   - Test all motor functions (forward, reverse, speed control)
   - Add safety features (emergency stop, power monitoring)

