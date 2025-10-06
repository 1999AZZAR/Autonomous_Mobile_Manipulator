# Software Configuration Guide

This guide provides comprehensive instructions for configuring the software components of the Autonomous Mobile Manipulator robot system.

## System Architecture Overview

### Software Stack Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   n8n       │  │   ROS 2     │  │   LabVIEW Client    │  │
│  │ Automation  │  │ Framework   │  │   (Optional)        │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│              Container Runtime (Docker)                     │
├─────────────────────────────────────────────────────────────┤
│                    Operating System                         │
├─────────────────────────────────────────────────────────────┤
│                    Hardware Layer                           │
└─────────────────────────────────────────────────────────────┘
```

## ROS 2 Configuration

### Workspace Setup

#### ROS 2 Workspace Structure

```
ros2_ws/
├── src/                          # Source code packages
│   ├── my_robot_bringup/        # Main bringup package
│   ├── my_robot_description/    # Robot model package
│   └── custom_packages/         # Additional packages
├── install/                      # Built packages (auto-generated)
├── build/                        # Build artifacts (auto-generated)
└── log/                         # Log files (auto-generated)
```

#### Package Dependencies

**Core Dependencies** (already configured in package.xml):
```xml
<!-- Navigation Stack -->
<nav2_bringup>
<nav2_map_server>
<nav2_amcl>
<nav2_bt_navigator>

<!-- Manipulation -->
<moveit_ros_planning_interface>
<moveit_ros_move_group>
<moveit_planners_ompl>

<!-- Sensors -->
<rplidar_ros>
<imu_filter_madgwick>
<realsense2_camera>

<!-- Hardware Interface -->
<ros2_control>
<controller_manager>
<diff_drive_controller>
```

### Controller Configuration

#### ROS 2 Control Setup

**Controller Configuration File**: `ros2_ws/src/my_robot_description/config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: position_controllers/GripperActionController
```

**Velocity Limits**:
```yaml
# Maximum velocities for safety
linear:
  x:
    max_velocity: 1.0    # m/s
    min_velocity: -1.0   # m/s
    max_acceleration: 2.0  # m/s²

angular:
  z:
    max_velocity: 2.0    # rad/s
    min_velocity: -2.0   # rad/s
    max_acceleration: 4.0  # rad/s²
```

### Navigation Configuration

#### Nav2 Parameters

**Navigation Configuration**: `ros2_ws/src/my_robot_bringup/config/nav2_params.yaml`

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 30
    max_particles: 2000
    min_particles: 500
    odom_alpha1: 0.2
    odom_alpha2: 0.2
    odom_alpha3: 0.2
    odom_alpha4: 0.2
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
      transform_tolerance: 0.1
      global_frame: map
      robot_base_frame: base_link

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      use_approach_linear_velocity_scaling: true
      max_allowed_time_to_collision: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_max_radius: 0.9
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      cost_scaling_dist_is_linear: true
```

### Sensor Configuration

#### LiDAR Configuration

**RPLidar Setup**:
```yaml
# rplidar_ros configuration
rplidar_ros:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    serial_baudrate: 115200
    frame_id: "laser_frame"
    inverted: false
    angle_compensate: true
    scan_mode: "Standard"  # Standard, Boost, Sensitivity
```

**Laser Scan Processing**:
```yaml
# Laser scan filtering and processing
laser_filters:
  ros__parameters:
    scan_filter_chain:
    - name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        lower_threshold: 0.1
        upper_threshold: 10.0
    - name: angular_filter
      type: laser_filters/LaserScanAngularFilter
      params:
        lower_angle: -2.0
        upper_angle: 2.0
```

#### IMU Configuration

**IMU Filter Setup**:
```yaml
# IMU filter configuration for sensor fusion
imu_filter_madgwick:
  ros__parameters:
    use_mag: false
    publish_tf: false
    world_frame: "enu"
    frequency: 50.0
    gain: 0.1
    zeta: 0.0
    mag_bias_x: 0.0
    mag_bias_y: 0.0
    mag_bias_z: 0.0
    orientation_stddev: 0.1
```

#### Camera Configuration

**Camera Parameters**:
```yaml
# Camera calibration and configuration
camera_info_manager:
  ros__parameters:
    camera_name: "camera"
    camera_info_url: "file://${ROS_HOME}/camera_info/camera.yaml"
    frame_id: "camera_link"

image_proc:
  ros__parameters:
    brightness: 0
    contrast: 32
    saturation: 32
    hue: 0
    gamma: 100
    sharpness: 0
    backlight_compensation: 0
    exposure_auto: 3
    exposure_absolute: 100
    exposure_auto_priority: 0
```

## Docker Configuration

### Container Optimization

#### Production Container Settings

```yaml
# docker-compose.prod.yml
services:
  ros2-sim:
    environment:
      - ROS_DOMAIN_ID=42                    # Unique domain ID
      - ROS_LOG_LEVEL=info                  # Reduce log verbosity
      - DISPLAY=${DISPLAY}                  # GUI support
      - QT_X11_NO_MITSHM=1                  # X11 optimization
    deploy:
      resources:
        limits:
          memory: 2G                        # Memory limit
          cpus: '2.0'                       # CPU limit
        reservations:
          memory: 1G                        # Memory reservation
          cpus: '1.0'                       # CPU reservation
    restart: unless-stopped                # Auto-restart policy

  n8n:
    environment:
      - GENERIC_TIMEZONE=Asia/Jakarta       # Timezone setting
      - N8N_LOG_LEVEL=info                  # Log level
      - N8N_LOG_OUTPUT=console              # Log output destination
    restart: unless-stopped
```

#### Development Container Settings

```yaml
# docker-compose.dev.yml
services:
  ros2-sim:
    environment:
      - ROS_LOG_LEVEL=debug                 # Verbose logging for debugging
      - GAZEBO_VERBOSE=1                    # Gazebo debug output
      - DISPLAY=${DISPLAY}
    volumes:
      - ./ros2_ws/src:/root/ros2_ws/src     # Live code mounting
      - /tmp/.X11-unix:/tmp/.X11-unix:rw    # X11 socket access

  n8n:
    ports:
      - "5678:5678"                         # Development port access
    environment:
      - N8N_LOG_LEVEL=debug                 # Debug logging
```

### Network Configuration

#### ROS 2 Domain Setup

```bash
# Set ROS domain ID for multi-robot scenarios
export ROS_DOMAIN_ID=42

# Verify domain isolation
ros2 topic list  # Should only show topics from same domain
```

#### Network Interface Configuration

```yaml
# Network interface priority for ROS 2
# Edit /etc/ros/setup.bash or container environment

# Prioritize Ethernet over WiFi for better performance
export ROS_IP=192.168.1.100  # Static IP assignment
export ROS_MASTER_URI=http://192.168.1.100:11311
```

## n8n Workflow Configuration

### Workflow Environment Variables

**n8n Configuration** (`docker-compose.yml`):
```yaml
environment:
  - GENERIC_TIMEZONE=Asia/Jakarta
  - N8N_HOST=0.0.0.0
  - N8N_PORT=5678
  - N8N_PROTOCOL=http
  - N8N_LOG_LEVEL=info
  - N8N_LOG_OUTPUT=console
```

### Custom Node Installation

**ROS 2 Publisher Node**:
```javascript
// n8n_data/nodes/ros2_publisher.js
const rosnodejs = require('rosnodejs');

class Ros2Publisher {
    async execute() {
        const nh = await rosnodejs.initNode('n8n_bridge');
        const pub = nh.advertise('/cmd_vel', 'geometry_msgs/Twist');

        const twist = {
            linear: { x: this.linear_x || 0, y: this.linear_y || 0, z: 0 },
            angular: { x: 0, y: 0, z: this.angular_z || 0 }
        };

        pub.publish(twist);
        return [{ json: { success: true, command: twist } }];
    }
}
```

**ROS 2 Subscriber Node**:
```javascript
// n8n_data/nodes/ros2_subscriber.js
const rosnodejs = require('rosnodejs');

class Ros2Subscriber {
    async execute() {
        const nh = await rosnodejs.initNode('n8n_subscriber');
        const sub = nh.subscribe('/scan', 'sensor_msgs/LaserScan');

        return new Promise((resolve) => {
            sub.on('message', (msg) => {
                resolve([{ json: { laser_data: msg } }]);
            });
        });
    }
}
```

### Workflow Import and Export

**Export Workflows**:
```bash
# From n8n web interface:
# Settings → Import/Export → Export
# Save workflow JSON files to n8n_data/workflows/
```

**Import Workflows**:
```bash
# Copy workflow files to container
docker cp robot_basic_control.json ros2_sim_container:/root/n8n_data/workflows/

# Restart n8n to load new workflows
docker restart n8n_container
```

## System Integration Configuration

### Coordinate Frame Setup

#### TF Tree Configuration

```yaml
# Static transform publishers
base_to_laser_tf:
  parent_frame: "base_link"
  child_frame: "laser_frame"
  translation: [0.0, 0.0, 0.2]
  rotation: [0.0, 0.0, 0.0]

base_to_imu_tf:
  parent_frame: "base_link"
  child_frame: "imu_link"
  translation: [0.1, 0.0, 0.1]
  rotation: [0.0, 0.0, 0.0]

base_to_camera_tf:
  parent_frame: "base_link"
  child_frame: "camera_link"
  translation: [0.2, 0.0, 0.3]
  rotation: [0.0, 0.0, 0.0]
```

### Sensor Calibration

#### LiDAR Calibration

```bash
# Calibrate LiDAR mounting angle
ros2 run rplidar_ros rplidar_calibration

# Adjust frame transformations if needed
# Edit static transform publishers in launch files
```

#### IMU Calibration

```bash
# Calibrate IMU orientation
ros2 run imu_filter_madgwick imu_calibration

# Set magnetic declination for location
ros2 param set /imu_filter_madgwick mag_declination 0.0
```

#### Camera Calibration

```bash
# Camera intrinsic calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw \
  camera:=/camera

# Save calibration data
ros2 run camera_calibration camera_calibration_parsers \
  save /camera/camera_info/$(ros-args)
```

## Performance Tuning

### Memory Optimization

#### ROS 2 Memory Settings

```yaml
# Node memory configuration
memory:
  enable_monitoring: true
  warning_threshold: 0.8
  error_threshold: 0.9

# Message queue optimization
queues:
  cmd_vel:
    depth: 10
    reliability: reliable
    durability: volatile
```

#### Docker Memory Limits

```yaml
# Container memory constraints
deploy:
  resources:
    limits:
      memory: 2G
    reservations:
      memory: 1G
```

### CPU Optimization

#### Real-time Scheduling

```yaml
# Set CPU affinity for critical nodes
cpu_affinity:
  controller_manager: "0-1"
  sensor_nodes: "2-3"

# Set scheduling policies
scheduling_policy:
  controller_manager: "SCHED_FIFO"
  sensor_nodes: "SCHED_RR"
```

#### Thread Configuration

```yaml
# Optimize thread counts
threading:
  num_threads: 4
  use_simd: true
  thread_pool_size: 8
```

## Security Configuration

### Network Security

#### Firewall Configuration

```bash
# Allow necessary ports
sudo ufw allow 5678/tcp    # n8n web interface
sudo ufw allow 8765/tcp    # WebSocket (Foxglove)
sudo ufw allow 11311/tcp   # ROS Master (if used)

# Restrict access to specific IPs
sudo ufw allow from 192.168.1.0/24 to any port 5678

# Enable firewall
sudo ufw enable
```

#### API Authentication

**n8n Authentication**:
```yaml
# Enable basic authentication
environment:
  - N8N_BASIC_AUTH_ACTIVE=true
  - N8N_BASIC_AUTH_USER=username
  - N8N_BASIC_AUTH_PASSWORD=password
```

**Webhook Authentication**:
```javascript
// Add authentication to workflows
const authHeader = $request.headers.authorization;
if (!authHeader || !authHeader.startsWith('Bearer ')) {
  throw new Error('Authentication required');
}
```

### Data Security

#### Encrypted Communication

```yaml
# Enable HTTPS for n8n
environment:
  - N8N_PROTOCOL=https
  - N8N_SSL_CERT=/ssl/cert.pem
  - N8N_SSL_KEY=/ssl/key.pem
```

#### Secure Configuration

```bash
# Secure configuration files
chmod 600 config/*.yaml
chmod 700 scripts/*.sh

# Encrypt sensitive data
gpg --encrypt --recipient username config/secrets.yaml
```

## Monitoring Configuration

### System Monitoring

#### Prometheus Metrics

```yaml
# Prometheus configuration for ROS 2 monitoring
global:
  scrape_interval: 15s

scrape_configs:
  - job_name: 'ros2_nodes'
    static_configs:
      - targets: ['ros2_sim_container:9090']

  - job_name: 'n8n_workflows'
    static_configs:
      - targets: ['n8n_container:9100']
```

#### Grafana Dashboards

**Key Metrics to Monitor**:
- CPU and memory usage per container
- ROS 2 topic publication rates
- Network latency and throughput
- Error rates and response times
- Battery level and power consumption

### Log Configuration

#### Structured Logging

```yaml
# Log format configuration
logging:
  format: "json"
  level: "info"
  output: "console"
  rotation:
    max_size: "100M"
    max_age: "30d"
    max_backups: 3
```

#### Log Aggregation

```bash
# Centralized logging setup
# Forward container logs to central log server
# Configure log rotation policies
# Set up log analysis and alerting
```

## Environment-Specific Configuration

### Development Environment

#### Debug Configuration

```yaml
# Enable debug features for development
environment:
  - ROS_LOG_LEVEL=debug
  - GAZEBO_VERBOSE=1
  - N8N_LOG_LEVEL=debug

# Mount source code for live editing
volumes:
  - ./ros2_ws/src:/root/ros2_ws/src
```

### Production Environment

#### Performance Configuration

```yaml
# Optimize for production performance
environment:
  - ROS_LOG_LEVEL=warn
  - N8N_LOG_LEVEL=info

# Resource constraints
deploy:
  resources:
    limits:
      memory: 2G
      cpus: '2.0'
```

### Testing Environment

#### Test Configuration

```yaml
# Enable test-specific features
environment:
  - TEST_MODE=true
  - MOCK_SENSORS=true
  - SIMULATION_SPEED=2.0

# Test data generation
# Automated test execution
# Performance benchmarking
```

## Configuration Validation

### Configuration Testing

#### Syntax Validation

```bash
# Validate YAML configuration files
python3 -c "import yaml; yaml.safe_load(open('config/controllers.yaml'))"

# Validate ROS 2 launch files
ros2 launch my_robot_bringup robot.launch.py --dry-run

# Check Docker Compose configuration
docker compose config
```

#### Functional Testing

```bash
# Test configuration with simulation
ros2 launch my_robot_bringup gazebo_world.launch.py

# Verify all topics are publishing
ros2 topic list

# Test sensor data quality
ros2 topic echo /scan --once
ros2 topic echo /imu/data --once

# Validate control responsiveness
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
```

### Performance Benchmarking

#### Benchmarking Script

```bash
#!/bin/bash
# performance-benchmark.sh

echo "Starting performance benchmarks..."

# Measure startup time
START_TIME=$(date +%s.%N)
docker compose up -d
END_TIME=$(date +%s.%N)
STARTUP_TIME=$(echo "$END_TIME - $START_TIME" | bc)
echo "Startup time: $STARTUP_TIME seconds"

# Measure API response time
curl -w "@curl-format.txt" -s -o /dev/null http://localhost:5678

# Measure topic throughput
ros2 topic bw /cmd_vel
ros2 topic bw /scan

# Monitor resource usage
docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"
```

## Backup and Recovery

### Configuration Backup

#### Automated Backup

```bash
#!/bin/bash
# config-backup.sh

# Backup configuration files
cp -r Autonomous_Mobile_Manipulator/ ~/backups/config/$(date +%Y%m%d)/

# Backup Docker volumes
docker run --rm -v autonomous_mobile_manipulator_n8n_data:/data -v ~/backups:/backup alpine tar czf /backup/n8n_data_$(date +%Y%m%d).tar.gz -C /data ./

# Backup ROS 2 workspace
docker run --rm -v ros2_ws:/workspace alpine tar czf ~/backups/ros2_ws_$(date +%Y%m%d).tar.gz -C /workspace ./
```

### Configuration Recovery

```bash
# Restore from backup
cp -r ~/backups/config/$(ls -t ~/backups/config/ | head -1)/* Autonomous_Mobile_Manipulator/

# Restart services with restored configuration
docker compose down
docker compose up --build -d
```

## Troubleshooting Configuration Issues

### Common Configuration Problems

#### YAML Syntax Errors

**Issue**: Invalid YAML formatting
```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config/file.yaml'))"

# Check for common issues:
# - Incorrect indentation
# - Missing quotes around strings
# - Invalid boolean values (true/false vs True/False)
```

#### ROS 2 Parameter Issues

**Issue**: Parameters not loading correctly
```bash
# Check parameter loading
ros2 param list /node_name

# Verify parameter types
ros2 param get /node_name parameter_name

# Check parameter file paths
ros2 launch my_robot_bringup robot.launch.py --show-args
```

#### Docker Configuration Issues

**Issue**: Volume mounting problems
```bash
# Check volume permissions
ls -la Autonomous_Mobile_Manipulator/

# Verify Docker user permissions
id $USER | grep docker

# Test volume mounting
docker run --rm -v $(pwd):/test alpine ls /test
```

## Advanced Configuration

### Custom Controller Implementation

#### PID Controller Tuning

```yaml
# Custom PID gains for differential drive
pid_gains:
  linear:
    kp: 1.0
    ki: 0.1
    kd: 0.05
    i_clamp: 1.0

  angular:
    kp: 2.0
    ki: 0.2
    kd: 0.1
    i_clamp: 2.0
```

#### Trajectory Planning

```yaml
# Motion planning parameters
trajectory:
  max_velocity: 1.0
  max_acceleration: 2.0
  lookahead_distance: 0.6
  goal_tolerance:
    position: 0.1
    orientation: 0.1
```

### Multi-Robot Configuration

#### Fleet Configuration

```yaml
# Multi-robot setup
robots:
  robot_1:
    id: 1
    ros_domain_id: 1
    ip_address: 192.168.1.101

  robot_2:
    id: 2
    ros_domain_id: 2
    ip_address: 192.168.1.102

# Fleet management settings
fleet:
  coordinator_ip: 192.168.1.100
  communication_protocol: "websocket"
  heartbeat_interval: 1.0
```

## Support and Maintenance

### Configuration Versioning

#### Version Control Strategy

```bash
# Track configuration changes
git add config/
git commit -m "config: Update navigation parameters for improved performance

- Increased lookahead distance from 0.5m to 0.6m
- Adjusted PID gains for smoother motion
- Updated sensor fusion weights"

# Tag configuration versions
git tag -a config-v1.2.0 -m "Configuration version 1.2.0"
```

### Configuration Documentation

#### Parameter Documentation

```markdown
<!-- config/README.md -->
# Configuration Parameter Reference

## Navigation Parameters (`nav2_params.yaml`)

### AMCL Parameters
- `alpha1-5`: Sensor model parameters for laser scan matching
- `max_particles`: Maximum number of particles for localization
- `min_particles`: Minimum number of particles for localization

### Controller Parameters
- `desired_linear_vel`: Target linear velocity (m/s)
- `lookahead_dist`: Lookahead distance for path following (m)
- `max_angular_accel`: Maximum angular acceleration (rad/s²)
```

## Resources and References

### Configuration References
- [ROS 2 Parameter Tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [MoveIt2 Configuration](https://moveit.ros.org/moveit!/doc/tutorials/tutorials.html)

### Best Practices
- [ROS 2 Configuration Management](https://docs.ros.org/en/iron/How-To-Guides/Configuration-Management.html)
- [Docker Compose Best Practices](https://docs.docker.com/compose/production/)
- [n8n Configuration Guide](https://docs.n8n.io/hosting/configuration/)

---

*This software configuration guide ensures optimal setup and performance of the Autonomous Mobile Manipulator robot system through proper configuration management and tuning.*
