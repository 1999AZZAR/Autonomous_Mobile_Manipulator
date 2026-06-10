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

    def __init__(self, simulation_mode=False, mega_interface=None):
        self.simulation_mode = simulation_mode
        self.mega_interface = mega_interface
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

        # Try to read from Mega first
        if self.mega_interface and self.mega_interface.mega_connected:
            mega_data = self._read_sensors_from_mega()
            if mega_data:
                return mega_data

        # Fallback to simulation if Mega not available
        logger.warning("Mega not available, using simulated sensor data")
        return self._get_simulated_sensor_data()

    def _read_sensors_from_mega(self):
        """Read sensor data from Arduino Mega"""
        try:
            mega_sensor_data = self.mega_interface.read_sensor_data()
            if mega_sensor_data:
                logger.info("Successfully read sensor data from Mega")
                # Add IMU data (still simulated for now)
                mega_sensor_data['imu'] = self._get_simulated_imu_data()
                # Normalize to flat keys for frontend
                return self._normalize_sensor_data(mega_sensor_data)
            else:
                logger.warning("Failed to read sensor data from Mega, using simulated data")
                return self._get_simulated_sensor_data()

        except Exception as e:
            logger.error(f"Error reading sensors from Mega: {str(e)}")
            return self._get_simulated_sensor_data()

    def _normalize_sensor_data(self, data):
        """Convert nested mega sensor data to flat keys for frontend dashboard."""
        laser = data.get('laser_sensors', {})
        ultra = data.get('ultrasonic_sensors', {})
        imu = data.get('imu', {})
        line = data.get('line_sensors', {})
        tf = data.get('tf_luna', {})

        # IR sensors from mega are already in mm (parsed by mega_interface)
        # Ultrasonic sensors from mega are in cm, convert to mm
        return {
            # Flat keys for frontend dashboard
            'laser_left_front': laser.get('left_front', 0),
            'laser_left_back': laser.get('left_back', 0),
            'laser_right_front': laser.get('right_front', 0),
            'laser_right_back': laser.get('right_back', 0),
            'laser_back_left': laser.get('back_left', 0),
            'laser_back_right': laser.get('back_right', 0),
            'ultra_front_left': ultra.get('front_left', 0),
            'ultra_front_right': ultra.get('front_right', 0),
            'line_left': line.get('left', 0),
            'line_center': line.get('center', 0),
            'line_right': line.get('right', 0),
            'imu_heading': imu.get('orientation', {}).get('z', 0),
            'imu_pitch': imu.get('orientation', {}).get('y', 0),
            'imu_roll': imu.get('orientation', {}).get('x', 0),
            'tf_luna_distance': tf.get('distance', 0),
            # Nested keys for internal control systems
            'laser_sensors': laser,
            'ultrasonic_sensors': ultra,
            'line_sensors': line,
            'imu': imu,
            'tf_luna': tf,
        }

    def read_all_sensors_old(self):
        """Read all available sensors and return structured data"""
        sensor_data = {}

        try:
            # Read Sharp IR sensors (GP2Y0A02YK0F)
            # Assuming channels 0-5 for IR sensors
            laser_sensors = {}
            for i, sensor_name in enumerate(['left_front', 'left_back', 'right_front', 'right_back', 'back_left', 'back_right']):
                try:
                    distance = self.gpio.read_sharp_sensor(i, f'ir_{sensor_name}')
                    if distance is not None:
                        laser_sensors[sensor_name] = distance
                        sensor_data[f'ir_{sensor_name}'] = distance  # Keep flat structure for display
                        self._update_sensor_health(f'ir_{sensor_name}', 'healthy')
                    else:
                        laser_sensors[sensor_name] = None
                        sensor_data[f'ir_{sensor_name}'] = None
                        self._update_sensor_health(f'ir_{sensor_name}', 'no_data')
                except Exception as e:
                    logger.error(f'Error reading IR sensor {sensor_name}: {str(e)}')
                    laser_sensors[sensor_name] = None
                    sensor_data[f'ir_{sensor_name}'] = None
                    self._update_sensor_health(f'ir_{sensor_name}', 'error')

            # Ultrasonic sensors would be read here if available
            # For now, use simulated data
            ultrasonic_sensors = {
                'front_left': 500,  # mm
                'front_right': 500   # mm
            }
            sensor_data['ultrasonic_front_left'] = 500  # Keep flat for display
            sensor_data['ultrasonic_front_right'] = 500  # Keep flat for display

            # IMU data (would come from ROS2 topics)
            imu_data = self.imu_data if self.imu_data else self._get_simulated_imu_data()
            sensor_data['imu'] = imu_data

        except Exception as e:
            logger.error(f'Error reading sensors: {str(e)}')
            # Set default structured data even on error
            laser_sensors = {name: None for name in ['left_front', 'left_back', 'right_front', 'right_back', 'back_left', 'back_right']}
            ultrasonic_sensors = {'front_left': None, 'front_right': None}

        # Always add structured data for path planning and control systems
        sensor_data['laser_sensors'] = laser_sensors
        sensor_data['ultrasonic_sensors'] = ultrasonic_sensors

        return sensor_data

    def _get_simulated_sensor_data(self):
        """Return simulated sensor data for testing"""
        import math
        import random

        # Simulate some variation
        sim_time = time.time()

        # Generate sensor values in mm
        laser_left_front = 800 + random.randint(-100, 100)
        laser_left_back = 750 + random.randint(-100, 100)
        laser_right_front = 850 + random.randint(-100, 100)
        laser_right_back = 700 + random.randint(-100, 100)
        laser_back_left = 600 + random.randint(-100, 100)
        laser_back_right = 650 + random.randint(-100, 100)
        ultra_front_left = 500 + random.randint(-50, 50)
        ultra_front_right = 550 + random.randint(-50, 50)

        return {
            # Flat keys for frontend dashboard
            'laser_left_front': laser_left_front,
            'laser_left_back': laser_left_back,
            'laser_right_front': laser_right_front,
            'laser_right_back': laser_right_back,
            'laser_back_left': laser_back_left,
            'laser_back_right': laser_back_right,
            'ultra_front_left': ultra_front_left,
            'ultra_front_right': ultra_front_right,
            'line_left': 0,
            'line_center': 1,
            'line_right': 0,
            'imu_heading': 0.0,
            'imu_pitch': math.sin(sim_time * 0.1) * 3.0,
            'imu_roll': math.cos(sim_time * 0.1) * 5.0,
            'tf_luna_distance': 0,
            'mega_connected': False,
            # Nested keys for internal control systems
            'laser_sensors': {
                'left_front': laser_left_front,
                'left_back': laser_left_back,
                'right_front': laser_right_front,
                'right_back': laser_right_back,
                'back_left': laser_back_left,
                'back_right': laser_back_right,
            },
            'ultrasonic_sensors': {
                'front_left': ultra_front_left,
                'front_right': ultra_front_right,
            },
            'line_sensors': {
                'left': False,
                'center': True,
                'right': False,
            },
            'imu': {
                'orientation': {
                    'x': math.sin(sim_time * 0.05) * 5.0,
                    'y': math.cos(sim_time * 0.05) * 3.0,
                    'z': 0.0,
                },
                'angular_velocity': {
                    'x': math.sin(sim_time * 0.1) * 2.0,
                    'y': math.cos(sim_time * 0.1) * 1.5,
                    'z': 0.0,
                },
                'linear_acceleration': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 9.81,
                },
                'temperature': 25.0 + math.sin(sim_time * 0.01) * 2.0,
            },
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
