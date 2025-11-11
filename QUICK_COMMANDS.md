# Quick Command Reference

## Diagnostics (Run This First!)

```bash
# From the project root directory
python3 diagnose_imu.py
```

This will tell you exactly what's wrong and how to fix it.

## Start Web Interface

### Hardware Mode (Real Sensors)
```bash
./start_hardware.sh
```

### Simulation Mode (No Hardware)
```bash
./start_simulation.sh
```

### Manual Control
```bash
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py --hardware     # Force hardware
python3 src/my_robot_automation/scripts/web_robot_interface.py --simulation  # Force simulation
python3 src/my_robot_automation/scripts/web_robot_interface.py               # Auto-detect
```

**Watch for:**
- "Starting in HARDWARE mode" - Using real sensors
- "Starting in SIMULATION mode" - Using simulated data  
- "MPU6050 IMU initialized successfully" - IMU working

## Test IMU

```bash
# Test sensor directly
python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py

# Test API endpoint
python3 test_imu_endpoint.py
```

## Run ROS2 Scripts (Correct Way)

**DO NOT:** `python3 script.py` ✗

**DO THIS:**
```bash
# From the project root
cd ros2_ws
source install/setup.bash
python3 src/my_robot_automation/scripts/SCRIPT_NAME.py
```

## Fix I2C Permissions

```bash
sudo usermod -a -G i2c $USER
# Logout and login again
```

## Check I2C Device

```bash
i2cdetect -y 1
# Should show device at 0x68
```

## Access Web UI

Browser: http://localhost:8000
- Click "Sensors" tab
- Scroll to "Line Sensors & IMU" section

## Check System Status

```bash
curl http://localhost:8000/health
curl http://localhost:8000/api/robot/status
curl http://localhost:8000/api/robot/imu/position
```

## Troubleshooting Workflow

1. Run diagnostics → `python3 diagnose_imu.py`
2. Fix reported issues
3. Test sensor directly → `python3 ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py`
4. Start web interface (from ros2_ws directory)
5. Test API → `python3 test_imu_endpoint.py`
6. Check web UI → http://localhost:8000

## Quick Checks

```bash
# Am I in i2c group?
groups | grep i2c

# Is MPU6050 detected?
i2cdetect -y 1 | grep 68

# Is mpu6050 library installed?
pip3 list | grep mpu6050

# Is web interface running?
curl -s http://localhost:8000/health | python3 -m json.tool
```

## Common Errors & Fixes

| Error | Fix |
|-------|-----|
| `ModuleNotFoundError: No module named 'rclpy'` | Source workspace: `cd ros2_ws && source install/setup.bash` |
| `ERROR: No matching distribution found for rclpy` | Don't use pip! rclpy is part of ROS2 |
| IMU shows "--" in UI | Run `python3 diagnose_imu.py` |
| Web interface shows simulated data | IMU didn't initialize, restart web interface |
| Permission denied on /dev/i2c-1 | Add to i2c group: `sudo usermod -a -G i2c $USER` |

## File Locations (relative to project root)

- Diagnostics: `diagnose_imu.py`
- API Test: `test_imu_endpoint.py`
- Web Interface: `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py`
- MPU6050 Reader: `ros2_ws/src/my_robot_automation/scripts/mpu6050_reader.py`
- ROS2 Helper: `run_ros2_script.sh`

