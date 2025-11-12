# L298N Motor Driver Troubleshooting Guide

## Problem: Motor Doesn't Move When Controlled by Raspberry Pi

### Symptoms
- Motor spins when connected directly to 12V power supply ✓
- Motor does NOT move when controlled via L298N from Raspberry Pi ✗
- GPIO pins are set correctly (verified with test scripts)
- L298N power connections appear correct

### Root Cause
**L298N VCC pin must be connected to 5V** for the module to recognize 3.3V logic signals from Raspberry Pi GPIO pins.

### Solution

#### Step 1: Verify L298N Power Connections

**Required Connections:**
```
L298N VS  → 12V Power Supply (motor power)
L298N VCC → 5V Power Supply  ← CRITICAL: Must be 5V, not 3.3V!
L298N GND → Common Ground (shared with Raspberry Pi)
```

**Measure with Multimeter:**
- VS pin: Should read ~12V
- VCC pin: Should read ~5V (if reading 3.3V or 0V, this is the problem!)
- GND: Should be common with Pi ground

#### Step 2: Connect L298N VCC to 5V

**Option A: Use Raspberry Pi 5V Pin**
```
L298N VCC → Raspberry Pi Pin 2 (5V) or Pin 4 (5V)
```

**Option B: Use External 5V Power Supply**
```
L298N VCC → 5V Power Supply (must share common ground with Pi)
```

**Option C: Use Voltage Regulator**
```
12V Power Supply → LM7805 Voltage Regulator → L298N VCC (5V)
```

#### Step 3: Verify Logic Level Compatibility

**Most L298N modules:**
- Can accept 3.3V logic signals IF VCC=5V
- Will NOT work if VCC=3.3V or VCC=0V

**If VCC=5V but still not working:**
- L298N module might require 5V logic levels
- Use level shifter (3.3V → 5V) between Pi GPIO and L298N IN1/IN2/ENA
- Or try different L298N module that supports 3.3V logic

### Complete Wiring Checklist

**L298N Power:**
- [ ] VS → 12V power supply (measured: _____V)
- [ ] VCC → 5V power supply (measured: _____V) ← CRITICAL
- [ ] GND → Common ground with Raspberry Pi

**Motor Connections:**
- [ ] Motor M+ → L298N OUT1
- [ ] Motor M- → L298N OUT2

**Control Signals (Raspberry Pi):**
- [ ] L298N IN1 → GPIO17 (Pin 11)
- [ ] L298N IN2 → GPIO10 (Pin 19)
- [ ] L298N ENA → GPIO11 (Pin 23)

**Encoder (PG23 Motor):**
- [ ] Motor DATA(A) → GPIO22 (Pin 15) - Read only
- [ ] Motor DATA(B) → GPIO23 (Pin 16) - Read only
- [ ] Motor VIN → 5V (encoder power only)
- [ ] Motor GND → Common ground

### Testing Procedure

1. **Test Motor Directly:**
   ```bash
   # Connect motor M+ and M- directly to 12V power supply
   # Motor should spin - confirms motor is working ✓
   ```

2. **Test GPIO Outputs:**
   ```bash
   # Run GPIO test script
   python3 test_gpio_output_voltage.py
   # Verify GPIO pins output HIGH (~3.3V) and LOW (~0V)
   ```

3. **Test L298N Control:**
   ```bash
   # Run motor control test
   python3 test_fl_motor_gpio11_ena.py
   # Motor should move forward/reverse
   ```

4. **Measure L298N Pin Voltages:**
   - With multimeter, measure voltage on L298N IN1, IN2, ENA pins
   - Should see voltage change when GPIO pins toggle
   - If no voltage change: Check wiring connections

### Common Issues

**Issue 1: VCC Not Connected**
- **Symptom:** Motor doesn't respond to GPIO signals
- **Solution:** Connect L298N VCC to 5V power supply

**Issue 2: VCC Connected to 3.3V**
- **Symptom:** L298N doesn't recognize GPIO signals
- **Solution:** Change VCC connection to 5V (not 3.3V)

**Issue 3: No Common Ground**
- **Symptom:** Unpredictable behavior, motor might not respond
- **Solution:** Ensure L298N GND and Raspberry Pi GND are connected

**Issue 4: L298N Jumpers on ENA/ENB**
- **Symptom:** Motor runs continuously, can't be stopped
- **Solution:** Remove jumpers from ENA/ENB pins (if present)

**Issue 5: Wrong Motor Connections**
- **Symptom:** Motor doesn't move or moves erratically
- **Solution:** Verify M+ → OUT1, M- → OUT2 (try swapping if needed)

### Verification

After connecting VCC to 5V:

1. Measure VCC pin voltage: Should read ~5V
2. Run motor test script: Motor should respond to GPIO commands
3. Check GPIO outputs: Should toggle between 0V and 3.3V
4. Motor should move forward when IN1=HIGH, IN2=LOW, ENA=HIGH
5. Motor should move reverse when IN1=LOW, IN2=HIGH, ENA=HIGH
6. Motor should stop when ENA=LOW

### Additional Notes

- Raspberry Pi GPIO outputs 3.3V logic levels
- L298N needs VCC=5V to properly recognize 3.3V signals
- If L298N VCC is not connected or connected to wrong voltage, control signals won't work
- Always verify power connections with multimeter before troubleshooting software

