"""
Sensor Manager
Handles sensor data collection, processing, and management
"""

import time
import logging
from collections import defaultdict
from gpio_controller import GPIOController

logger = logging.getLogger(__name__)

class SensorManager:
    """Manages all sensor data and readings"""

    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode
        self.gpio = GPIOController(simulation_mode)

        # Initialize sensor data structure
        self.sensor_data = {
            'laser_sensors': {
                'left_front': 0,
                'left_back': 0,
                'right_front': 0,
                'right_back': 0,
                'back_left': 0,
                'back_right': 0
            },
            'ultrasonic_sensors': {
                'front_left': 0,
                'front_right': 0
            },
            'line_sensors': {
                'left': False,
                'center': False,
                'right': False
            },
            'tf_luna': {
                'distance': 0
            },
            'imu': {
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'temperature': 0.0
            }
        }

        # Sensor health tracking
        self.sensor_health = defaultdict(dict)

        # IMU data
        self.imu_data = {}
        self.imu_initialized = False

    def read_all_sensors(self):
        """Read all available sensors and return structured data"""
        if self.simulation_mode:
            return self._get_simulated_sensor_data()

        sensor_data = {}

        try:
            # Read Sharp IR sensors (GP2Y0A02YK0F)
            # Assuming channels 0-5 for IR sensors
            for i, sensor_name in enumerate(['left_front', 'left_back', 'right_front', 'right_back', 'back_left', 'back_right']):
                try:
                    distance = self.gpio.read_sharp_sensor(i, f'ir_{sensor_name}')
                    if distance is not None:
                        sensor_data[f'ir_{sensor_name}'] = distance
                        self._update_sensor_health(f'ir_{sensor_name}', 'healthy')
                    else:
                        sensor_data[f'ir_{sensor_name}'] = None
                        self._update_sensor_health(f'ir_{sensor_name}', 'no_data')
                except Exception as e:
                    logger.error(f'Error reading IR sensor {sensor_name}: {str(e)}')
                    sensor_data[f'ir_{sensor_name}'] = None
                    self._update_sensor_health(f'ir_{sensor_name}', 'error')

            # Ultrasonic sensors would be read here if available
            # For now, use simulated data
            sensor_data['ultrasonic_front_left'] = 500  # mm
            sensor_data['ultrasonic_front_right'] = 500  # mm

            # IMU data (would come from ROS2 topics)
            sensor_data['imu'] = self.imu_data if self.imu_data else self._get_simulated_imu_data()

        except Exception as e:
            logger.error(f'Error reading sensors: {str(e)}')

        return sensor_data

    def _get_simulated_sensor_data(self):
        """Return simulated sensor data for testing"""
        import math
        import random

        # Simulate some variation
        sim_time = time.time()

        return {
            'ir_left_front': 800 + random.randint(-100, 100),
            'ir_left_back': 750 + random.randint(-100, 100),
            'ir_right_front': 850 + random.randint(-100, 100),
            'ir_right_back': 700 + random.randint(-100, 100),
            'ir_back_left': 600 + random.randint(-100, 100),
            'ir_back_right': 650 + random.randint(-100, 100),
            'ultrasonic_front_left': 500 + random.randint(-50, 50),
            'ultrasonic_front_right': 550 + random.randint(-50, 50),
            'imu': {
                'orientation': {
                    'x': math.sin(sim_time * 0.1) * 0.1,  # Small roll
                    'y': math.cos(sim_time * 0.1) * 0.1,  # Small pitch
                    'z': 0.0
                },
                'angular_velocity': {
                    'x': math.sin(sim_time * 0.2) * 0.1,
                    'y': math.cos(sim_time * 0.2) * 0.1,
                    'z': 0.0
                },
                'linear_acceleration': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 9.81  # Gravity
                },
                'temperature': 25.0 + math.sin(sim_time * 0.01) * 2.0
            }
        }

    def _get_simulated_imu_data(self):
        """Return simulated IMU data"""
        import math
        sim_time = time.time()

        return {
            'orientation': {
                'x': math.sin(sim_time * 0.05) * 5.0,  # Roll ±5°
                'y': math.cos(sim_time * 0.05) * 3.0,  # Pitch ±3°
                'z': 0.0  # Yaw
            },
            'angular_velocity': {
                'x': math.sin(sim_time * 0.1) * 2.0,
                'y': math.cos(sim_time * 0.1) * 1.5,
                'z': 0.0
            },
            'linear_acceleration': {
                'x': math.sin(sim_time * 0.08) * 0.5,
                'y': math.cos(sim_time * 0.08) * 0.5,
                'z': 9.81 + math.sin(sim_time * 0.1) * 0.2  # ~9.8 m/s² with variation
            },
            'temperature': 25.0 + math.sin(sim_time * 0.02) * 5.0
        }

    def _update_sensor_health(self, sensor_name, status):
        """Update sensor health tracking"""
        if sensor_name not in self.sensor_health:
            self.sensor_health[sensor_name] = {}

        self.sensor_health[sensor_name]['status'] = status
        self.sensor_health[sensor_name]['last_update'] = time.time()

        if status == 'error':
            self.sensor_health[sensor_name]['error_count'] = self.sensor_health[sensor_name].get('error_count', 0) + 1
        elif status == 'healthy':
            self.sensor_health[sensor_name]['error_count'] = 0

    def get_sensor_health(self):
        """Get sensor health status"""
        return dict(self.sensor_health)

    def update_imu_data(self, imu_data):
        """Update IMU data from ROS2 topics"""
        self.imu_data = imu_data
        self.imu_initialized = True

    def read_imu_data(self):
        """Read current IMU data"""
        if self.imu_data:
            return self.imu_data
        else:
            return self._get_simulated_imu_data()

    def calibrate_imu(self):
        """Calibrate IMU (simplified implementation)"""
        if self.simulation_mode:
            logger.info("[SIM] IMU calibration completed")
            return True

        # In a real implementation, this would perform actual calibration
        logger.info("IMU calibration completed (simplified)")
        return True

    def cleanup(self):
        """Clean up sensor resources"""
        if self.gpio:
            self.gpio.cleanup()
