# MPU6050 IMU Sensor Setup and Integration

## Overview
The MPU6050 is a 6-axis IMU (Inertial Measurement Unit) that provides 3-axis accelerometer and 3-axis gyroscope data. It's used for measuring orientation, acceleration, and angular velocity.

## Hardware Specifications

### MPU6050 Sensor
- **Communication Protocol**: I2C
- **I2C Address**: 0x68 (default) or 0x69 (if AD0 pin is HIGH)
- **Supply Voltage**: 3.3V or 5V
- **Accelerometer Range**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope Range**: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Temperature Sensor**: Built-in

## Wiring

### I2C Connection to Raspberry Pi

```
MPU6050      Raspberry Pi 5
---------------------------------
VCC    -->   Pin 1  (3.3V) or Pin 2 (5V)
GND    -->   Pin 6  (GND)
SCL    -->   Pin 5  (GPIO3 - I2C1_SCL)
SDA    -->   Pin 3  (GPIO2 - I2C1_SDA)
```

### Pin Details
- **VCC**: Power supply (3.3V or 5V)
- **GND**: Ground
- **SCL**: I2C Serial Clock Line (GPIO3)
- **SDA**: I2C Serial Data Line (GPIO2)
- **AD0**: Address select (leave floating for 0x68, connect to VCC for 0x69)
- **INT**: Interrupt pin (optional, not used in basic setup)

## Software Setup

### 1. Enable I2C Interface

I2C should already be enabled on your Raspberry Pi. Verify with:

```bash
ls -l /dev/i2c*
```

You should see `/dev/i2c-1` (and possibly others).

### 2. Install Required Tools

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus
```

### 3. Detect MPU6050

Scan the I2C bus to verify the sensor is detected:

```bash
i2cdetect -y 1
```

You should see `68` (or `69`) in the output grid.

### 4. Install Python Libraries

```bash
pip3 install --break-system-packages smbus2 mpu6050-raspberrypi
```

Or add to `requirements.txt`:
```
smbus2
mpu6050-raspberrypi
```

## Usage

### Basic Test

```bash
cd ~/Autonomous_Mobile_Manipulator/ros2_ws/src/my_robot_automation/scripts
python3 mpu6050_reader.py
```

### In Python Code

```python
from mpu6050_reader import MPU6050Reader

# Initialize the sensor
imu = MPU6050Reader(address=0x68)

if imu.initialized:
    # Read all sensor data
    data = imu.read_all()
    if data:
        print(f"Accel X: {data['accelerometer']['x']:.2f} m/s²")
        print(f"Gyro Z: {data['gyroscope']['z']:.2f} deg/s")
        print(f"Temp: {data['temperature']:.1f}°C")
    
    # Get calculated orientation
    orientation = imu.get_orientation()
    if orientation:
        print(f"Pitch: {orientation['pitch']:.1f}°")
        print(f"Roll: {orientation['roll']:.1f}°")
```

## Data Interpretation

### Accelerometer
- **Units**: m/s²
- **Axes**: 
  - X: Forward/Backward
  - Y: Left/Right
  - Z: Up/Down (should read ~9.8 m/s² when stationary)

### Gyroscope
- **Units**: degrees/second
- **Axes**:
  - X: Rotation around X-axis (pitch)
  - Y: Rotation around Y-axis (roll)
  - Z: Rotation around Z-axis (yaw)

### Orientation
- **Pitch**: Rotation around Y-axis (forward/backward tilt)
- **Roll**: Rotation around X-axis (left/right tilt)
- **Yaw**: Rotation around Z-axis (requires gyroscope integration)

## Calibration

For accurate readings, the sensor should be calibrated:

1. Place the robot on a flat, level surface
2. Let the sensor warm up for 30 seconds
3. Record the offset values for all axes
4. Subtract these offsets from future readings

## Troubleshooting

### Sensor Not Detected
```bash
# Check I2C bus
i2cdetect -y 1

# Check I2C kernel modules
lsmod | grep i2c

# Check user permissions
groups  # Should include 'i2c' group
```

### Incorrect Readings
- Check wiring connections
- Verify power supply voltage (3.3V or 5V)
- Ensure sensor is firmly mounted
- Calibrate the sensor

### Library Errors
```bash
# Reinstall libraries
pip3 install --break-system-packages --upgrade smbus2 mpu6050-raspberrypi
```

## Integration Status

### Current Status
- Hardware detected at I2C address 0x68
- Libraries installed and working
- MPU6050Reader module created and tested
- Successfully reading accelerometer, gyroscope, and temperature

### Next Steps
1. Integrate into ROS2 node for real-time publishing
2. Add calibration routine
3. Implement sensor fusion for accurate orientation
4. Add data to web interface dashboard

## Testing Results

Test conducted on 2025-11-11:
```
MPU6050 initialized successfully at address 0x68

Sample reading:
Accel: X=0.24, Y=0.86, Z=10.01 m/s²
Gyro:  X=0.15, Y=2.24, Z=0.46 deg/s
Temp:  44.6°C
Orient: Pitch=4.8°, Roll=-1.1°
```

Status: WORKING CORRECTLY

## Additional Resources

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [mpu6050-raspberrypi Library](https://pypi.org/project/mpu6050-raspberrypi/)

