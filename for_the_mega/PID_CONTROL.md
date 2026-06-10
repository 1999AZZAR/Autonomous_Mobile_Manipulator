# PID Control System - Hexagonal 3-Wheel Omni Robot

## Overview
Comprehensive PID control implementation for precise motor speed regulation in a hexagonal 3-wheel omni robot using Arduino Mega and YFROBOT v2 motor driver shield.

---

## PID Architecture

### Motor Control Structure
```
Encoder Input → PID Controller → Motor Driver → Motor → Encoder Feedback Loop
      ↑                                                                ↓
   Target RPM ← PID Output (PWM) ← Motor Response ← Measured RPM
```

### PID Controllers (4 Independent Controllers)
- **Motor 1 (Lifter)**: Conservative tuning for smooth lifting operations
- **Motors 2-4 (Omni Wheels)**: Aggressive tuning for instant response and precise movement

---

## PID Configuration

### PID Parameters (Defined in config.h)

#### Lifter Motor (Motor 1) - Conservative Tuning
```cpp
const double Lifter_Kp = 2.0;    // Proportional gain
const double Lifter_Ki = 0.5;    // Integral gain
const double Lifter_Kd = 0.1;    // Derivative gain
```

#### Omni Motors (Motors 2-4) - Aggressive Tuning
```cpp
const double Omni_Kp = 8.0;      // Proportional gain
const double Omni_Ki = 3.0;      // Integral gain
const double Omni_Kd = 0.2;      // Derivative gain
```

#### PID System Parameters
```cpp
const double PID_SAMPLE_TIME = 100;     // PID update interval (ms)
const int PID_OUTPUT_LIMIT_MIN = -255;  // Minimum PWM output
const int PID_OUTPUT_LIMIT_MAX = 255;   // Maximum PWM output
```

---

## PID Control Algorithm

### Standard PID Formula
```
Output = Kp × Error + Ki × ∫Error × dt + Kd × dError/dt

Where:
- Error = Target_RPM - Current_RPM
- ∫Error = Accumulated error over time
- dError/dt = Rate of error change
```

### Implementation Details

#### Error Calculation
```cpp
double error = setpoint[i] - input[i];  // setpoint = target RPM, input = measured RPM
```

#### Integral Windup Protection
```cpp
// Automatic integral reset when motors are stopped
if (motorsStopped) {
  // Reset integral terms to prevent windup
}
```

#### Derivative Filtering
```cpp
// Exponential smoothing on RPM measurements reduces derivative noise
double smoothedRPM = rpmAlpha * filteredRPM + (1 - rpmAlpha) * smoothedRPM;
```

---

## Motor Synchronization System

### Overview
Advanced motor synchronization ensures uniform RPM across active motors during movement operations.

#### Synchronization Parameters
```cpp
const double SYNC_KP = 0.3;  // Synchronization PID gain (conservative)
```

### Synchronization Algorithm

#### Active Motor Detection
```cpp
// Only synchronize motors that are intended to be active
bool motorIntendedActive[4] = {false, false, false, false};
```

#### Target RPM Calculation
```cpp
double targetSyncRPM = 0;
int activeMotorCount = 0;

for (int i = 1; i < 4; i++) {  // Skip lifter motor
  if (motorIntendedActive[i]) {
    targetSyncRPM += smoothedRPM[i];
    activeMotorCount++;
  }
}
targetSyncRPM /= activeMotorCount;
```

#### Synchronization Correction
```cpp
double syncError = targetSyncRPM - smoothedRPM[motorIndex];
double syncCorrection = syncKp * syncError;

// Apply correction only to intended active motors
if (motorIntendedActive[motorIndex]) {
  setpoint[motorIndex] += syncCorrection;
}
```

---

## Acceleration Limiting

### Purpose
Prevents jerky movements and motor damage by limiting RPM changes per control cycle.

#### Acceleration Limits
```cpp
const double MAX_RPM_CHANGE = 200.0;        // Normal acceleration limit
const double MAX_RPM_CHANGE_ROTATION = 1000.0; // Fast rotation limit
```

### Implementation
```cpp
double rpmChange = setpoint[i] - prev_setpoint[i];

if (fastRotationMode) {
  rpmChange = constrain(rpmChange, -MAX_RPM_CHANGE_ROTATION, MAX_RPM_CHANGE_ROTATION);
} else {
  rpmChange = constrain(rpmChange, -MAX_RPM_CHANGE, MAX_RPM_CHANGE);
}

setpoint[i] = prev_setpoint[i] + rpmChange;
prev_setpoint[i] = setpoint[i];
```

---

## Fast Rotation Mode

### Overview
Turbo mode for rapid rotation responses with relaxed acceleration limits and enhanced filtering.

#### Fast Rotation Parameters
```cpp
const double RPM_ALPHA_FAST = 0.9;  // Ultra-responsive smoothing
// Acceleration limits increased to 1000 RPM/s during fast rotation
```

### Activation Conditions
```cpp
// Activated by rotation commands (c, w) for 5 seconds
unsigned long lastRotationCommand = millis();
bool fastRotationMode = (millis() - lastRotationCommand < 5000);

// Or permanently activated by 'o' command
// fastRotationMode = true (permanent turbo)
```

---

## Encoder Processing

### Encoder Specifications
```cpp
const int ENCODER_CPR = 28;          // Counts per revolution
const double GEAR_RATIO = 1.0;       // No gearing
const double PID_SAMPLE_TIME = 100;  // 100ms sample interval
```

### RPM Calculation
```cpp
// Encoder count difference over time
long countDiff = encoderCount - lastEncoderCount;
double timeDiff = (currentTime - lastEncoderTime) / 1000000.0; // microseconds to seconds

// RPM calculation
double revolutions = countDiff / (ENCODER_CPR * GEAR_RATIO);
double rpm = (revolutions / timeDiff) * 60.0;
```

### Filtering Pipeline
```
Raw Encoder → Median Filter → Exponential Smoothing → PID Input
     ↓             ↓                ↓              ↓
   Counts      Filtered RPM    Smoothed RPM    Control Signal
```

#### Exponential Smoothing
```cpp
const double RPM_ALPHA = 0.6;        // Normal smoothing factor
const double RPM_ALPHA_FAST = 0.9;   // Fast rotation smoothing

smoothedRPM = alpha * filteredRPM + (1 - alpha) * smoothedRPM;
```

---

## PID Tuning Guide

### Basic Tuning Principles

#### Proportional (Kp)
- **Too Low**: Slow response, steady-state error
- **Too High**: Oscillations, overshoot, instability
- **Optimal**: Quick response without oscillations

#### Integral (Ki)
- **Too Low**: Steady-state error persists
- **Too High**: Windup, oscillations, instability
- **Optimal**: Eliminates steady-state error without oscillations

#### Derivative (Kd)
- **Too Low**: Overshoot, oscillations
- **Too High**: Noise sensitivity, jerky response
- **Optimal**: Reduces overshoot and oscillations

### Tuning Methodology

#### Step 1: Set Ki = 0, Kd = 0 (P-only control)
```
1. Increase Kp until oscillations start
2. Reduce Kp by 20-50% from oscillation point
3. Note the Kp value for proportional-only response
```

#### Step 2: Add Integral (PI control)
```
1. Start with Ki = Kp / 100
2. Gradually increase Ki until steady-state error is eliminated
3. Reduce Ki if oscillations occur
4. Note the optimal Ki value
```

#### Step 3: Add Derivative (PID control)
```
1. Start with Kd = Ki / 10
2. Increase Kd to reduce overshoot
3. Reduce Kd if response becomes too sensitive to noise
4. Fine-tune all three parameters together
```

### Motor-Specific Tuning

#### Lifter Motor (Conservative)
- **Application**: Smooth lifting operations
- **Requirements**: No oscillations, steady movement
- **Tuning**: Lower Kp, moderate Ki, low Kd

#### Omni Motors (Aggressive)
- **Application**: Fast, precise movement control
- **Requirements**: Quick response, tight speed control
- **Tuning**: Higher Kp, moderate Ki, low Kd

---

## PID Monitoring and Diagnostics

### Real-time Status (`p` command)
```
Motor 1: Setpoint=50.0 RPM, Smoothed=49.8 RPM, SyncErr=0.2, Output=25.5
Motor 2: Setpoint=60.0 RPM, Smoothed=59.5 RPM, SyncErr=-0.5, Output=30.2
Motor 3: Setpoint=60.0 RPM, Smoothed=60.2 RPM, SyncErr=0.2, Output=29.8
Motor 4: Setpoint=60.0 RPM, Smoothed=59.8 RPM, SyncErr=-0.2, Output=30.5
```

### Key Metrics to Monitor
- **Setpoint vs Smoothed RPM**: Tracking accuracy
- **SyncErr**: Motor synchronization quality
- **Output**: PWM duty cycle (0-255 range)
- **RPM Stability**: Consistency over time

### Common Issues and Solutions

#### Oscillations
```
Cause: Kp too high, Kd too low
Solution: Reduce Kp, increase Kd
```

#### Steady-State Error
```
Cause: Ki too low
Solution: Increase Ki gradually
```

#### Noise Sensitivity
```
Cause: Kd too high, encoder noise
Solution: Reduce Kd, improve encoder filtering
```

#### Slow Response
```
Cause: Kp too low
Solution: Increase Kp, check acceleration limits
```

---

## Advanced PID Features

### Motor State Management
```cpp
// PID processing only for intended active motors
if (!motorIntendedActive[i]) {
  motorDriver.stopMotor(i + 1);  // Force stop idle motors
  setpoint[i] = 0;              // Reset PID setpoint
  continue;                     // Skip PID computation
}
```

### Speed Multiplier Integration
```cpp
// Global speed scaling affects all setpoints
setpoint[i] *= speedMultiplier;  // 0.5 to 1.0 range
```

---

## Configuration File (config.h)

```cpp
// PID Tunings
const double Lifter_Kp = 2.0;
const double Lifter_Ki = 0.5;
const double Lifter_Kd = 0.1;

const double Omni_Kp = 8.0;
const double Omni_Ki = 3.0;
const double Omni_Kd = 0.2;

// System Parameters
const double PID_SAMPLE_TIME = 100;
const int PID_OUTPUT_LIMIT_MIN = -255;
const int PID_OUTPUT_LIMIT_MAX = 255;

// Synchronization
const double SYNC_KP = 0.3;

// Acceleration Limits
const double MAX_RPM_CHANGE = 200.0;
const double MAX_RPM_CHANGE_ROTATION = 1000.0;

// Filtering
const double RPM_ALPHA = 0.6;
const double RPM_ALPHA_FAST = 0.9;
```

---

## Performance Optimization

### Sample Time Considerations
- **100ms**: Good balance of responsiveness and stability
- **Lower values**: More responsive but may cause oscillations
- **Higher values**: More stable but less responsive

### Memory Usage
- **PID Objects**: 4 PID controllers × ~60 bytes each
- **Filter Arrays**: 4 motors × 5 samples × 8 bytes
- **Variables**: Minimal additional RAM usage

### CPU Usage
- **PID Calculations**: ~2ms per control cycle
- **Filtering**: ~1ms per control cycle
- **Synchronization**: ~0.5ms per control cycle
- **Total**: ~3.5ms per 100ms cycle (< 4% CPU usage)

---

## Troubleshooting PID Issues

### Motor Not Reaching Target Speed
```
1. Check encoder connections and CPR setting
2. Verify PID tuning (increase Kp)
3. Check acceleration limits (may be too restrictive)
4. Verify motor power and driver configuration
```

### Oscillations or Instability
```
1. Reduce Kp gain
2. Increase Kd gain
3. Check for mechanical issues (loose belts, worn gears)
4. Verify encoder filtering (increase smoothing)
```

### Poor Synchronization
```
1. Adjust SYNC_KP (0.1 - 0.5 range)
2. Check motor encoder alignment
3. Verify motor specifications are identical
4. Check for mechanical binding or friction differences
```

---

## PID Testing Commands

### Individual Motor Testing
```
1, 2, 3, 4  → Test individual motors at 50% speed
g            → Figure-8 pattern for synchronization testing
h            → Continuous rotation test
```

### PID Monitoring
```
p            → Real-time PID status and motor data
```

### Tuning Commands
```
o            → Toggle fast rotation mode (affects PID response)
5-9, 0       → Speed multiplier (affects PID setpoints)
```

---
*Last updated: November 12, 2025*
*PID Sample Time: 100ms*
*Control Frequency: 10Hz*
