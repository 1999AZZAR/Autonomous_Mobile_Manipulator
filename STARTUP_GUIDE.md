# Robot System Startup Guide

## Quick Start

### Step 1: Run IMU Diagnostics

This will check all components and tell you exactly what's wrong:

```bash
# From the project root directory
python3 diagnose_imu.py
```

The diagnostic will check:
- I2C permissions
- MPU6050 sensor detection
- Library installation
- Direct sensor access
- Web interface status
- API endpoints

Follow the recommendations it provides.

### Step 2: Start Web Interface

You have THREE ways to start the web interface:

#### Option A: Hardware Mode (Recommended for Real Robot)

```bash
# From the project root
./start_hardware.sh
```

This forces HARDWARE mode and requires:
- MPU6050 IMU connected
- I2C permissions
- Real sensor hardware

#### Option B: Simulation Mode (For Testing)

```bash
# From the project root
./start_simulation.sh
```

This forces SIMULATION mode:
- No hardware required
- All sensors simulated
- Good for development/testing

#### Option C: Manual Start (Advanced)

```bash
# From the project root
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py --hardware  # Force hardware
# OR
python3 src/my_robot_automation/scripts/web_robot_interface.py --simulation  # Force simulation
# OR
python3 src/my_robot_automation/scripts/web_robot_interface.py  # Auto-detect
```

**Watch for startup messages:**
- ✓ `Starting in HARDWARE mode (forced)` - Using real sensors
- ✓ `Starting in SIMULATION mode (forced)` - Using simulated data
- ✓ `Starting in AUTO-DETECT mode` - Will detect hardware automatically
- ✓ `MPU6050 IMU initialized successfully` - IMU is working
- ✗ `Running in SIMULATION mode` - Fell back to simulation

### Step 3: Verify Web Interface

Open browser to: http://localhost:8000

Or test from command line:
```bash
python3 test_imu_endpoint.py
```

## Common Issues

### Issue 1: "ModuleNotFoundError: No module named 'rclpy'"

**Cause:** Trying to run ROS2 scripts without sourcing the workspace

**Solution:** 
ROS2 scripts need the workspace sourced first:

```bash
# From the project root
cd ros2_ws
source install/setup.bash
python3 src/my_robot_automation/scripts/YOUR_SCRIPT.py
```

Or use the helper script:
```bash
# From the project root
./run_ros2_script.sh ros2_ws/src/my_robot_automation/scripts/YOUR_SCRIPT.py
```

**Note:** You CANNOT install rclpy via pip. It's part of ROS2.

### Issue 2: IMU Shows No Data or Wrong Data

**Symptom:** Web UI shows "--" or simulated sine wave data

**Possible Causes:**

1. **I2C Permissions**
   ```bash
   # Check if you're in i2c group
   groups | grep i2c
   
   # If not, add yourself:
   sudo usermod -a -G i2c $USER
   # Then logout and login
   ```

2. **MPU6050 Not Detected**
   ```bash
   # Check I2C bus
   i2cdetect -y 1
   # Should show device at 0x68
   
   # If not found, check wiring
   ```

3. **Web Interface Started Before Sensor Connected**
   - Stop the web interface (Ctrl+C)
   - Connect the MPU6050
   - Start web interface again

4. **Library Not Installed**
   ```bash
   pip3 install mpu6050-raspberrypi
   ```

### Issue 3: Different Data from Different Tests

**If:**
- `mpu6050_reader.py` shows real data
- `test_imu_endpoint.py` shows simulated data
- Web UI shows no data

**Then:** Web interface is running in simulation mode because IMU didn't initialize.

**Fix:**
1. Run diagnostics: `python3 diagnose_imu.py`
2. Fix any issues it reports
3. Restart web interface
4. Check startup logs for "MPU6050 initialized successfully"

## Running ROS2 Scripts

### Method 1: Manual (Recommended)
```bash
# From the project root
cd ros2_ws
source install/setup.bash
python3 src/my_robot_automation/scripts/SCRIPT_NAME.py
```

### Method 2: Helper Script
```bash
# From the project root
./run_ros2_script.sh ros2_ws/src/my_robot_automation/scripts/SCRIPT_NAME.py
```

## System Architecture

```
Hardware Layer:
  MPU6050 (I2C @ 0x68)
  ↓
Software Stack:
  mpu6050 library
  ↓
  mpu6050_reader.py (wrapper)
  ↓
  web_robot_interface.py (Flask API)
  ↓
  Web UI (Browser)
```

## Verification Checklist

Before starting, verify:

- [ ] MPU6050 connected to I2C pins
- [ ] User in i2c group: `groups | grep i2c`
- [ ] Device detected: `i2cdetect -y 1`
- [ ] Library installed: `pip3 list | grep mpu6050`
- [ ] Direct test works: `python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py`

Then start system:

- [ ] Web interface started from ros2_ws directory
- [ ] Startup log shows "MPU6050 initialized successfully"
- [ ] API test returns real data: `python3 test_imu_endpoint.py`
- [ ] Web UI shows live IMU data at http://localhost:8000

## Troubleshooting Commands

```bash
# 1. Run full diagnostics
python3 diagnose_imu.py

# 2. Check I2C device
i2cdetect -y 1

# 3. Test sensor directly
python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py

# 4. Test API endpoint
python3 test_imu_endpoint.py

# 5. Check web interface status
curl http://localhost:8000/health

# 6. Check system status
curl http://localhost:8000/api/robot/status
```

## Files Reference

- `diagnose_imu.py` - Comprehensive diagnostic tool
- `test_imu_endpoint.py` - API endpoint tester
- `run_ros2_script.sh` - Helper to run ROS2 scripts
- `mpu6050_reader.py` - MPU6050 hardware interface
- `web_robot_interface.py` - Web interface and API server
- `IMU_TROUBLESHOOTING.md` - Detailed troubleshooting guide
- `QUICK_IMU_TEST.md` - Quick testing reference

## Getting Help

If issues persist after following this guide:

1. Run full diagnostics: `python3 diagnose_imu.py`
2. Check the recommendations it provides
3. Review IMU_TROUBLESHOOTING.md for detailed solutions
4. Check wiring against SENSOR_WIRING.md

## Hardware Specifications

**MPU6050 Wiring:**
```
MPU6050          Raspberry Pi 5
VCC      ----->  3.3V (Pin 1)
GND      ----->  GND (Pin 6)
SDA      ----->  GPIO2/SDA (Pin 3)
SCL      ----->  GPIO3/SCL (Pin 5)
```

**Servo Connections:**
- Gripper Servo: GPIO19 (Pin 35)
- Tilt Servo: GPIO18 (Pin 12)

See SENSOR_WIRING.md for complete wiring diagrams.

