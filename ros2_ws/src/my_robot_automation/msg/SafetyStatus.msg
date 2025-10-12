# Safety status message
std_msgs/Header header
bool emergency_stop_active
bool collision_detected
bool obstacle_detected
bool battery_low
bool communication_lost
bool hardware_error
float32 min_distance_to_obstacle
string[] active_safety_warnings
string[] active_safety_errors
geometry_msgs/PoseStamped last_safe_pose
