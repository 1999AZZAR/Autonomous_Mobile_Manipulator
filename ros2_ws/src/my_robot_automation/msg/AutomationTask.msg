# Automation task message
std_msgs/Header header
string task_id
string task_type               # "PICK_PLACE", "PATROL", "OBSTACLE_AVOIDANCE", "NAVIGATE"
string status                  # "PENDING", "RUNNING", "COMPLETED", "FAILED", "CANCELLED"
geometry_msgs/PoseStamped target_pose
geometry_msgs/PoseStamped[] waypoints
float32 timeout_seconds
string[] parameters
string error_message
float32 progress_percentage
