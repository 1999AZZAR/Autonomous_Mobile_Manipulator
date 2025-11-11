# MPU6050 IMU Integration Troubleshooting Guide

## Overview

This guide helps troubleshoot IMU data display issues in the web interface.

## Architecture

```
MPU6050 Sensor (I2C @ 0x68)
    ↓
mpu6050_reader.py (Hardware Interface)
    ↓
web_robot_interface.py (Web API)
    ↓
Web UI (JavaScript @ /api/robot/imu/position)
```

## Common Issues and Solutions

### Issue 1: IMU Works Directly but Not in Web UI

**Symptoms:**
- Direct test of mpu6050_reader.py shows data
- Web UI shows "--" or no data for IMU readings

**Diagnosis Steps:**

1. Test the MPU6050 sensor directly:
```bash
cd /home/azzar/project/robotic/lks_robot_project
python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py
```

Expected output: Accelerometer, Gyroscope, Temperature, and Orientation data

2. Test the web interface API endpoint:
```bash
python3 test_imu_endpoint.py
```

This will query the web interface API and show the response.

3. Check web interface logs:
```bash
# Look for IMU initialization messages
# Should see: "MPU6050 IMU initialized successfully"
```

**Solutions:**

1. Verify I2C permissions:
```bash
sudo usermod -a -G i2c $USER
# Logout and login again
```

2. Check I2C bus:
```bash
i2cdetect -y 1
# Should show device at 0x68
```

3. Restart web interface after sensor initialization

### Issue 2: Web Interface Shows Simulated Data

**Symptoms:**
- IMU data appears but values are obviously simulated (smooth sine waves)

**Cause:**
- Web interface is running in simulation mode
- IMU not initialized when web interface started

**Solution:**
1. Ensure MPU6050 is connected before starting web interface
2. Check initialization logs for errors
3. Restart web interface

### Issue 3: Browser Console Errors

**Symptoms:**
- IMU section shows errors in browser console
- 503 Service Unavailable errors

**Diagnosis:**
Open browser developer tools (F12) and check Console tab for errors

**Solutions:**

1. If seeing 503 errors:
   - Web interface may not be able to read IMU
   - Check server logs for Python exceptions
   - Verify IMU is initialized: check `/api/robot/status` endpoint

2. If seeing CORS errors:
   - Ensure you're accessing from correct hostname
   - Try accessing via localhost first

### Issue 4: Data Format Issues

**Symptoms:**
- API returns data but UI displays incorrectly
- Values show as "NaN" or "undefined"

**Cause:**
- Data structure mismatch between API and UI
- JavaScript parsing errors

**Solution:**
The fix implemented includes:
- Safe dictionary access with `.get()` methods
- Default values for missing keys
- Proper None checking before data access

## API Endpoint Reference

### GET /api/robot/imu/position

Returns IMU sensor data in the following format:

```json
{
  "success": true,
  "data": {
    "orientation": {
      "x": 10.3,    // Roll in degrees
      "y": 46.7,    // Pitch in degrees
      "z": 0.0      // Yaw in degrees
    },
    "angular_velocity": {
      "x": -0.06,   // deg/s
      "y": 1.44,    // deg/s
      "z": 0.33     // deg/s
    },
    "linear_acceleration": {
      "x": -1.32,   // m/s²
      "y": 7.26,    // m/s²
      "z": 6.29     // m/s²
    },
    "temperature": 44.4  // Celsius
  },
  "timestamp": 1234567890.123
}
```

## Testing Procedure

1. Test hardware directly:
```bash
python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py
```

2. Start web interface:
```bash
cd ros2_ws
source install/setup.bash
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

3. Test API endpoint:
```bash
python3 test_imu_endpoint.py
```

4. Access web UI:
- Open browser to: http://localhost:8000
- Navigate to Sensors tab
- Check IMU section for live data

## Recent Fixes (2025-11-11)

1. Enhanced error handling in `read_imu_data()` method
2. Added safe dictionary access with `.get()` and default values
3. Added debug logging for data flow tracking
4. Added None checking for orientation data
5. Created test script for API endpoint verification

## Expected Sensor Readings

When robot is stationary on level surface:
- Roll (X): ~0° (±2°)
- Pitch (Y): ~0° (±2°)
- Yaw (Z): 0° (requires gyroscope integration)
- Accel Z: ~9.8 m/s² (gravity)
- Accel X, Y: ~0 m/s²
- Gyro X, Y, Z: ~0 deg/s

## Hardware Specifications

- Sensor: MPU6050 6-axis IMU
- Interface: I2C
- I2C Address: 0x68 (default)
- Update Rate: 500ms (2Hz) in web UI
- Power: 3.3V from Raspberry Pi

## Wiring

```
MPU6050          Raspberry Pi 5
VCC      ----->  3.3V (Pin 1)
GND      ----->  GND (Pin 6)
SDA      ----->  GPIO2/SDA (Pin 3)
SCL      ----->  GPIO3/SCL (Pin 5)
```

## Further Debugging

Enable debug logging in web interface:
```python
# Add to web_robot_interface.py initialization
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

Monitor logs in real-time:
```bash
# Watch IMU data flow
tail -f /path/to/ros2/logs | grep -i imu
```

## Contact

For additional support, check:
- Project documentation: `/home/azzar/project/robotic/lks_robot_project/notes.txt`
- Sensor wiring: `SENSOR_WIRING.md`
- Hardware specs in web UI: Hardware tab

