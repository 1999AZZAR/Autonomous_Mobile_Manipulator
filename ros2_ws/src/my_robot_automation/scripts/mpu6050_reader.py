#!/usr/bin/env python3
'''
MPU6050 IMU Sensor Reader for Robot
Reads accelerometer, gyroscope, and temperature data
'''

import time
import math

# Try to import smbus2 (compatible with smbus)
try:
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False
        print("WARNING: smbus/smbus2 not available. IMU will not work.")

# MPU6050 registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

class MPU6050Reader:
    def __init__(self, address=0x68, bus=1):
        '''
        Initialize MPU6050 sensor
        Args:
            address: I2C address (default 0x68, alternative 0x69)
            bus: I2C bus number (default 1 for Raspberry Pi)
        '''
        self.address = address
        self.bus_num = bus
        self.initialized = False
        
        if not SMBUS_AVAILABLE:
            print('SMBUS not available - cannot initialize MPU6050')
            return
        
        try:
            self.bus = SMBus(bus)
            # Wake up the MPU6050 (set sleep bit to 0)
            self.bus.write_byte_data(address, PWR_MGMT_1, 0)
            time.sleep(0.1)  # Wait for sensor to wake up
            
            # Test read to verify sensor is responding
            test = self.bus.read_byte_data(address, PWR_MGMT_1)
            self.initialized = True
            print(f'MPU6050 initialized successfully at address 0x{address:02x} on bus {bus}')
        except Exception as e:
            self.initialized = False
            print(f'Failed to initialize MPU6050: {e}')
    
    def _read_word_2c(self, addr):
        '''Read a 16-bit signed value from two consecutive registers'''
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def read_all(self):
        '''
        Read all sensor data
        Returns:
            dict with accel, gyro, temp data, or None if error
        '''
        if not self.initialized:
            return None
        
        try:
            # Read accelerometer (in g units, need to convert to m/s²)
            accel_x = self._read_word_2c(ACCEL_XOUT_H) / 16384.0  # LSB/g for ±2g range
            accel_y = self._read_word_2c(ACCEL_XOUT_H + 2) / 16384.0
            accel_z = self._read_word_2c(ACCEL_XOUT_H + 4) / 16384.0
            
            # Read gyroscope (in deg/s)
            gyro_x = self._read_word_2c(GYRO_XOUT_H) / 131.0  # LSB/(deg/s) for ±250deg/s range
            gyro_y = self._read_word_2c(GYRO_XOUT_H + 2) / 131.0
            gyro_z = self._read_word_2c(GYRO_XOUT_H + 4) / 131.0
            
            # Read temperature (in °C)
            temp_raw = self._read_word_2c(TEMP_OUT_H)
            temperature = temp_raw / 340.0 + 36.53
            
            return {
                'accelerometer': {
                    'x': accel_x * 9.81,  # Convert g to m/s²
                    'y': accel_y * 9.81,
                    'z': accel_z * 9.81
                },
                'gyroscope': {
                    'x': gyro_x,
                    'y': gyro_y,
                    'z': gyro_z
                },
                'temperature': temperature,
                'timestamp': time.time()
            }
        except Exception as e:
            print(f'Error reading MPU6050: {e}')
            return None
    
    def get_orientation(self):
        '''
        Calculate orientation from accelerometer
        Returns pitch and roll in degrees
        '''
        if not self.initialized:
            return None
        
        try:
            data = self.read_all()
            if not data:
                return None
                
            accel = data['accelerometer']
            
            # Calculate pitch and roll from accelerometer
            pitch = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)) * 180 / math.pi
            roll = math.atan2(-accel['x'], accel['z']) * 180 / math.pi
            
            return {
                'pitch': pitch,
                'roll': roll,
                'yaw': 0.0  # Gyroscope integration needed for yaw
            }
        except Exception as e:
            print(f'Error calculating orientation: {e}')
            return None

if __name__ == '__main__':
    # Test the MPU6050 reader
    imu = MPU6050Reader()
    
    if imu.initialized:
        print('\nReading IMU data for 5 seconds...\n')
        for i in range(10):
            data = imu.read_all()
            if data:
                print(f'Accel: X={data["accelerometer"]["x"]:.2f}, Y={data["accelerometer"]["y"]:.2f}, Z={data["accelerometer"]["z"]:.2f} m/s²')
                print(f'Gyro:  X={data["gyroscope"]["x"]:.2f}, Y={data["gyroscope"]["y"]:.2f}, Z={data["gyroscope"]["z"]:.2f} deg/s')
                print(f'Temp:  {data["temperature"]:.1f}°C')
                
                orient = imu.get_orientation()
                if orient:
                    print(f'Orient: Pitch={orient["pitch"]:.1f}°, Roll={orient["roll"]:.1f}°\n')
            
            time.sleep(0.5)
    else:
        print('Failed to initialize MPU6050')
