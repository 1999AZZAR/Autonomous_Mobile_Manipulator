# Robot status message
std_msgs/Header header
string robot_name
string mode                    # "AUTONOMOUS", "MANUAL", "EMERGENCY", "MAINTENANCE"
string state                   # "IDLE", "MOVING", "MANIPULATING", "NAVIGATING", "ERROR"
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist current_velocity
bool emergency_stop_active
bool safety_systems_ok
float32 battery_level
float32 cpu_usage
float32 memory_usage
string[] active_tasks
