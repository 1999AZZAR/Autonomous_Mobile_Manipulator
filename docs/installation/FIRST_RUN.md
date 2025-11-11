# First Run Guide

## Complete Setup from Scratch

This guide walks you through setting up and running the robot for the first time.

## Prerequisites

### On Raspberry Pi:

1. **ROS2 Installed**
   ```bash
   # Check if ROS2 is installed
   ls /opt/ros/
   ```
   
   Should show `humble` or `foxy`. If not installed:
   ```bash
   # Install ROS2 Humble (recommended)
   # Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
   ```

2. **Python 3**
   ```bash
   python3 --version  # Should be 3.8 or higher
   ```

3. **Build Tools**
   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-pip
   ```

4. **I2C Tools (for sensors)**
   ```bash
   sudo apt install i2c-tools
   ```

5. **I2C Permissions**
   ```bash
   sudo usermod -a -G i2c $USER
   # Logout and login again
   ```

## Step-by-Step Setup

### 1. Clone/Navigate to Project

```bash
cd ~/Autonomous_Mobile_Manipulator  # Or wherever your project is
```

### 2. Build the Workspace

```bash
./build_workspace.sh
```

**What this does:**
- Checks ROS2 installation
- Installs Python dependencies (flask, requests, mpu6050-raspberrypi)
- Builds the ROS2 workspace with colcon
- Takes 2-5 minutes

**Expected output:**
```
========================================
  Building ROS2 Workspace
========================================

Checking ROS2 installation...
✓ ROS2 Humble found
Checking colcon...
✓ colcon found
Checking Python dependencies...
✓ All Python dependencies installed

Building workspace...
  This may take a few minutes on first build

[... build output ...]

========================================
✓ Workspace built successfully!
========================================
```

### 3. Connect Hardware (Skip if Simulation)

**For Hardware Mode:**

Connect the sensors:
- MPU6050 IMU to I2C (pins 3, 5)
- Servo to GPIO18 (tilt) and GPIO19 (gripper)
- Optional: IR sensors, line sensors, etc.

Verify IMU:
```bash
i2cdetect -y 1
```

Should show `68` in the grid.

### 4. Run Diagnostics (Optional)

```bash
python3 diagnose_imu.py
```

This checks everything and tells you what needs to be fixed.

### 5. Start the Robot

**For Real Hardware:**
```bash
./start_hardware.sh
```

**For Simulation/Testing:**
```bash
./start_simulation.sh
```

**What happens:**
- Checks I2C permissions
- Detects MPU6050
- Sources ROS2 workspace (or builds if needed)
- Starts web interface

**Expected output (Hardware):**
```
========================================
  Starting Robot in HARDWARE Mode
========================================

Checking I2C permissions...
✓ User is in i2c group
Checking for MPU6050 IMU...
✓ MPU6050 detected at 0x68

Starting web interface in HARDWARE mode...

✓ Sourcing ROS2 workspace
Starting in HARDWARE mode (forced)
MPU6050 initialized successfully at address 0x68
SPI interface initialized successfully
Professional Robot Web Interface initialized
Access the interface at: http://localhost:8000
```

### 6. Access Web Interface

Open your browser:
```
http://localhost:8000
```

Or from another device on the network:
```
http://<raspberry-pi-ip>:8000
```

**You should see:**
- Modern blue/purple gradient interface
- Sidebar with navigation tabs
- Sensor readings (real data in hardware mode)
- Movement controls
- Gripper controls
- IMU data

## Troubleshooting First Run

### Build Fails

**Error:** `colcon: command not found`

**Fix:**
```bash
sudo apt install python3-colcon-common-extensions
```

**Error:** `No module named 'setuptools'`

**Fix:**
```bash
pip3 install setuptools
```

### Hardware Detection Fails

**Error:** `MPU6050 not detected`

**Check:**
1. Wiring: VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3
2. I2C enabled: `sudo raspi-config` → Interface Options → I2C
3. Permissions: `groups | grep i2c`
4. Device: `i2cdetect -y 1`

### Module Not Found

**Error:** `ModuleNotFoundError: No module named 'rclpy'`

**Fix:**
Workspace not built. Run:
```bash
./build_workspace.sh
```

### Permission Denied

**Error:** `Permission denied: '/dev/i2c-1'`

**Fix:**
```bash
sudo usermod -a -G i2c $USER
# Logout and login
```

## Quick Start Summary

```bash
# 1. Build (first time only)
./build_workspace.sh

# 2. Start robot
./start_hardware.sh      # For real hardware
# OR
./start_simulation.sh    # For testing

# 3. Open browser
# http://localhost:8000

# Done!
```

## What's Next?

After first successful run:

1. **Explore Web Interface**
   - Try movement controls
   - Check sensor readings
   - Test gripper controls
   - View hardware tab for pinout

2. **Calibrate IMU**
   - Go to Sensors tab
   - Click "Calibrate IMU" button
   - Place robot on level surface first

3. **Test Path Planning**
   - Go to Path Planning tab
   - Add waypoints
   - Execute patrol

4. **Read Documentation**
   - `SENSOR_WIRING.md` - Complete wiring guide
   - `MODE_SELECTION.md` - Understanding modes
   - `API_DOCUMENTATION.md` - REST API reference

## Common First-Run Issues

| Issue | Solution |
|-------|----------|
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
| `ModuleNotFoundError: No module named 'rclpy'` | Run `./build_workspace.sh` |
| IMU not detected | Check wiring and run `i2cdetect -y 1` |
| Permission denied on I2C | Add to group: `sudo usermod -a -G i2c $USER` then logout/login |
| Web interface stuck in simulation | Use `./start_hardware.sh` to force hardware mode |
| Port 8000 already in use | Kill existing: `pkill -f web_robot_interface.py` |

## Success Checklist

After first run, you should have:

- [✓] Workspace built successfully
- [✓] Web interface accessible at http://localhost:8000
- [✓] Hardware mode active (if using real sensors)
- [✓] IMU data showing real values (not smooth sine waves)
- [✓] Movement controls responsive
- [✓] Activity stream showing feedback

## Getting Help

If stuck:

1. Run diagnostics: `python3 diagnose_imu.py`
2. Check logs in terminal output
3. Review `IMU_TROUBLESHOOTING.md`
4. Check `STARTUP_GUIDE.md`

## Next Steps

Once everything works:

- Explore automation features
- Set up n8n workflows
- Configure path planning
- Calibrate sensors
- Test pick-and-place sequences

