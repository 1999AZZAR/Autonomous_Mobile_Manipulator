# Hardware vs Simulation Mode Update

## Problem Fixed

The web interface was getting stuck in simulation mode even when hardware was connected, because it defaulted to simulation if hardware detection failed.

## Solution

Added explicit mode control with command-line flags.

## New Features

### 1. Command-Line Arguments

The web interface now accepts these flags:

```bash
python3 web_robot_interface.py --hardware     # Force hardware mode
python3 web_robot_interface.py --simulation  # Force simulation mode
python3 web_robot_interface.py --auto        # Auto-detect (default)
```

### 2. Convenience Scripts

#### For Real Hardware
```bash
./start_hardware.sh
```
- Checks I2C permissions
- Checks for MPU6050
- Forces hardware mode
- Fails if sensors not available

#### For Testing Without Hardware
```bash
./start_simulation.sh
```
- No hardware checks
- Forces simulation mode
- Works anywhere

### 3. Docker Support

Updated `run.sh` to pass mode:
```bash
./run.sh start       # Production mode → hardware
./run.sh start --dev # Development mode → simulation
```

## Mode Behavior

### Hardware Mode (`--hardware`)
- **Requires:** Real sensors connected
- **Fails if:** No hardware available
- **IMU:** Real MPU6050 data
- **Sensors:** Real IR distance readings
- **Use when:** Running on actual robot

### Simulation Mode (`--simulation`)
- **Requires:** Nothing
- **Never fails:** Always works
- **IMU:** Simulated smooth data
- **Sensors:** Simulated distance readings
- **Use when:** Testing, development, CI/CD

### Auto-Detect Mode (default)
- **Tries:** Hardware first
- **Falls back:** To simulation if hardware fails
- **Flexible:** Works in any environment
- **Use when:** Not sure what's available

## How To Use

### On Real Robot (Raspberry Pi)

```bash
# Run diagnostics first
python3 diagnose_imu.py

# Fix any issues it reports

# Start in hardware mode
./start_hardware.sh
```

### On Development Machine

```bash
# Start in simulation mode
./start_simulation.sh
```

### Flexible (Works Either Way)

```bash
# Let it detect automatically
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

## Verification

### Check Mode via Web UI
1. Open http://localhost:8000
2. Click "Status" tab
3. Look for `simulation_mode: false` (hardware) or `true` (simulation)

### Check Mode via API
```bash
curl http://localhost:8000/api/robot/status | python3 -m json.tool
```

### Check Mode via Startup Logs
Look for these messages:
- ✅ `Starting in HARDWARE mode (forced)` - Using real sensors
- ✅ `Starting in SIMULATION mode (forced)` - Simulated data
- ✅ `Starting in AUTO-DETECT mode` - Will detect automatically

## Troubleshooting

### "Stuck in simulation mode"

**Before (Problem):**
```bash
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py
# Auto-detects → fails to find hardware → simulation mode
```

**After (Solution):**
```bash
./start_hardware.sh
# Forces hardware mode → will error if hardware missing
```

### "Can't test without hardware"

**Before:**
Had to have hardware to test

**After:**
```bash
./start_simulation.sh
# Works anywhere, no hardware needed
```

## Files Updated

### Modified
- `web_robot_interface.py` - Added argparse for mode control
- `run.sh` - Sets ROBOT_MODE environment variable
- `STARTUP_GUIDE.md` - Documented new startup options
- `QUICK_COMMANDS.md` - Added mode commands
- `notes.txt` - Updated with new features

### Created
- `start_hardware.sh` - Easy hardware mode startup
- `start_simulation.sh` - Easy simulation mode startup
- `MODE_SELECTION.md` - Comprehensive mode guide
- `HARDWARE_MODE_UPDATE.md` - This file

## Quick Reference

| Want to... | Command |
|------------|---------|
| Run on real robot | `./start_hardware.sh` |
| Test without hardware | `./start_simulation.sh` |
| Let it auto-detect | `python3 web_robot_interface.py` |
| Check current mode | `curl localhost:8000/api/robot/status` |
| Diagnose issues | `python3 diagnose_imu.py` |

## Summary

**Before:** Web interface would auto-detect and often get stuck in simulation mode even with hardware present.

**After:** You explicitly choose:
- `./start_hardware.sh` for real robot
- `./start_simulation.sh` for testing
- Or let it auto-detect (default)

This prevents the "stuck in simulation" issue and makes mode selection explicit and controllable.

