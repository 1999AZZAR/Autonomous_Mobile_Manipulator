# Task status message
std_msgs/Header header
string task_id
string task_type               # "PICK_PLACE", "PATROL", "OBSTACLE_AVOIDANCE", "NAVIGATE", "CALIBRATION"
string status                  # "PENDING", "RUNNING", "COMPLETED", "FAILED", "CANCELLED"
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped[] waypoints
float32 timeout_seconds
string[] parameters
string error_message
float32 progress_percentage    # 0.0 to 100.0
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
builtin_interfaces/Duration elapsed_time
