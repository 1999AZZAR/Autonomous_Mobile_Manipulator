# Navigation status message
std_msgs/Header header

# Localization status
bool localization_active
float32 localization_confidence  # 0.0 to 1.0
geometry_msgs/PoseWithCovarianceStamped current_pose

# Navigation status
bool navigation_active
string navigation_state          # "IDLE", "PLANNING", "MOVING", "RECOVERING", "FAILED"
geometry_msgs/PoseStamped target_pose
nav_msgs/Path current_path

# Map status
bool map_available
string map_name
builtin_interfaces/Time map_last_updated
geometry_msgs/Pose map_origin

# Safety zones and obstacles
geometry_msgs/Polygon[] safety_zones
geometry_msgs/PoseStamped[] detected_obstacles

# Performance metrics
float32 path_length
float32 distance_to_goal
builtin_interfaces/Duration estimated_time_remaining
