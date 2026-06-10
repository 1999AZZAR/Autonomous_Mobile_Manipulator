"""
Configuration settings for the Autonomous Mobile Manipulator
"""

import os

# Flask configuration
FLASK_HOST = '0.0.0.0'
FLASK_PORT = int(os.environ.get('FLASK_PORT', '8000'))
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

# Database settings
DATABASE_URL = os.environ.get('DATABASE_URL', 'postgresql://robot:robot_secret@localhost:5432/robot_automation')

# Automation settings
AUTOMATION_CACHE_INTERVAL = 0.1  # Sensor cache update interval (seconds)
AUTOMATION_RATE_LIMIT_WINDOW = 5.0  # Rate limit window (seconds)
AUTOMATION_MAX_EXECUTIONS_PER_WINDOW = 10  # Max executions per window
AUTOMATION_MAX_TRIGGER_DEPTH = 5  # Max circular trigger depth

# AI Decision Engine settings
AI_BACKEND = os.environ.get('AI_BACKEND', 'hybrid')  # local, api, hybrid
AI_MODEL = os.environ.get('AI_MODEL', 'gpt-4o')
AI_LOOP_INTERVAL = float(os.environ.get('AI_LOOP_INTERVAL', '3.0'))  # seconds
AI_DECISION_HISTORY_SIZE = 5
AI_OPENAI_API_KEY = os.environ.get('OPENAI_API_KEY', '')

# Camera settings
CAMERA_ID = int(os.environ.get('CAMERA_ID', '0'))
CAMERA_WIDTH = int(os.environ.get('CAMERA_WIDTH', '640'))
CAMERA_HEIGHT = int(os.environ.get('CAMERA_HEIGHT', '480'))

# Waypoint settings
WAYPOINT_RECORD_INTERVAL = 0.5  # seconds between auto-recorded waypoints
WAYPOINT_POSITION_TOLERANCE = 0.15  # meters
WAYPOINT_HEADING_TOLERANCE = 10.0  # degrees
