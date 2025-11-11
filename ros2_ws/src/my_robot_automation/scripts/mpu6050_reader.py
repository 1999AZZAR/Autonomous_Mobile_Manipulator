#!/usr/bin/env python3
'''
MPU6050 IMU Sensor Reader for Robot
Reads accelerometer, gyroscope, and temperature data
'''

import time
from mpu6050 import mpu6050

class MPU6050Reader:
    def __init__(self, address=0x68):
        '''
        Initialize MPU6050 sensor
        Args:
            address: I2C address (default 0x68, alternative 0x69)
        '''
        try:
            self.sensor = mpu6050(address)
            self.initialized = True
            print(f'MPU6050 initialized successfully at address 0x{address:02x}')
        except Exception as e:
            self.initialized = False
            print(f'Failed to initialize MPU6050: {e}')
    
    def read_all(self):
        '''
        Read all sensor data
        Returns:
            dict with accel, gyro, temp data, or None if error
        '''
        if not self.initialized:
            return None
        
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            temp = self.sensor.get_temp()
            
            return {
                'accelerometer': {
                    'x': accel['x'],
                    'y': accel['y'],
                    'z': accel['z']
                },
                'gyroscope': {
                    'x': gyro['x'],
                    'y': gyro['y'],
                    'z': gyro['z']
                },
                'temperature': temp,
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
            import math
            accel = self.sensor.get_accel_data()
            
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
