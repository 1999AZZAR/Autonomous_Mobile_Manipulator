# Quick IMU Testing Guide

## Test Sequence

### Step 1: Verify Hardware
```bash
python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py
```

Expected: Should display accelerometer, gyroscope, temperature, and orientation data for 5 seconds.

### Step 2: Start Web Interface
```bash
cd ros2_ws
source install/setup.bash
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

Expected: Should see "MPU6050 IMU initialized successfully" in the output.

### Step 3: Test API Endpoint
```bash
# In a new terminal
cd /home/azzar/project/robotic/lks_robot_project
python3 test_imu_endpoint.py
```

Expected: Should display JSON response with IMU data including orientation, angular velocity, and acceleration.

### Step 4: Test Web UI
1. Open browser to: http://localhost:8000
2. Click on "Sensors" tab in the left sidebar
3. Scroll to "Line Sensors & IMU" section
4. IMU data should update every 500ms

Expected values when robot is stationary:
- Orientation X (Roll): ~10-11° (based on your test output)
- Orientation Y (Pitch): ~47-48° (based on your test output)
- Orientation Z (Yaw): 0°
- Angular Velocity: Near 0 deg/s when stationary
- Linear Accel X: ~-1.2 m/s²
- Linear Accel Y: ~7.2 m/s²

## What Was Fixed

1. Added robust error handling in IMU data reading
2. Safe dictionary access to prevent KeyError exceptions
3. Default values for missing data
4. Debug logging for troubleshooting
5. Better None checking

## Files Modified

- `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py`
  - Enhanced `read_imu_data()` method (lines 3055-3128)

## New Files Created

1. `test_imu_endpoint.py` - API endpoint testing tool
2. `IMU_TROUBLESHOOTING.md` - Comprehensive troubleshooting guide
3. `QUICK_IMU_TEST.md` - This file

## Troubleshooting

If IMU data still doesn't show:

1. Check I2C connection:
```bash
i2cdetect -y 1
```
Should show device at 0x68.

2. Check permissions:
```bash
groups | grep i2c
```
User should be in i2c group.

3. Check web interface logs for errors:
```bash
# Look for "Error reading IMU" messages
```

4. Run continuous API test:
```bash
python3 test_imu_endpoint.py
# When prompted, type 'y' for continuous test
```

## Remote Access

To access from another device on the network:
1. Find Raspberry Pi IP: `hostname -I`
2. Access web UI: http://[PI_IP]:8000
3. Run test from remote: `python3 test_imu_endpoint.py [PI_IP] 8000`

