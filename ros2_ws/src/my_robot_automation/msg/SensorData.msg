# Sensor data message
std_msgs/Header header

# IR Distance sensors (6x Sharp GP2Y0A02YK0F - wall alignment)
float32 distance_left_front
float32 distance_left_back
float32 distance_right_front
float32 distance_right_back
float32 distance_back_left
float32 distance_back_right

# HC-SR04 Ultrasonic sensors (2x front)
float32 ultrasonic_front_left
float32 ultrasonic_front_right

# Line sensors (3x individual)
int32 line_sensor_left
int32 line_sensor_center
int32 line_sensor_right

# TF-Luna LIDAR sensor (single-point)
float32 tf_luna_distance
int32 tf_luna_strength
float32 tf_luna_temperature

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
bool tf_luna_healthy
