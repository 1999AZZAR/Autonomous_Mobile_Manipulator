"""
Configuration settings for the Autonomous Mobile Manipulator
"""

import os

# Flask configuration
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 8000
FLASK_DEBUG = False

# Serial communication settings
MEGA_BAUDRATE = 115200
MEGA_TIMEOUT = 1
MEGA_WRITE_TIMEOUT = 1
MEGA_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']

# ROS2 settings
ROS2_NODE_NAME = 'web_robot_interface'
ROS2_ACTUATOR_SERVICE_TIMEOUT = 5.0

# Sensor settings
ADC_VREF = 3.3
ADC_RESOLUTION = 1024
SPI_SPEED = 1350000

# Motor control settings
MAX_WHEEL_SPEED = 100
DEFAULT_SPEED_PERCENT = 50

# Simulation mode
DEFAULT_SIMULATION_MODE = False

# IMU settings
IMU_CALIBRATION_SAMPLES = 100

# Logging
LOG_LEVEL = 'INFO'
