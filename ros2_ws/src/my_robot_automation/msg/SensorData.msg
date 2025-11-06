# Sensor data message
std_msgs/Header header

# Distance sensors (meters)
float32 ultrasonic_front
float32 ultrasonic_back_left
float32 ultrasonic_back_right

# IR proximity sensors (meters)
float32 ir_front
float32 ir_left
float32 ir_right

# Line sensor data (bit pattern: 0b11111111 = all sensors see line)
int32 line_sensor_raw
bool line_detected
float32 line_position  # -1.0 (left) to 1.0 (right), 0.0 = center

# IMU data
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Quaternion orientation

# Battery status
float32 battery_voltage
float32 battery_percentage
bool battery_low_warning

# Sensor health status
bool ultrasonic_healthy
bool ir_healthy
bool line_sensor_healthy
bool imu_healthy
bool battery_healthy
