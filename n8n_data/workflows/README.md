# n8n Robot Control Workflows

This directory contains ready-to-import n8n workflow files for controlling your LKS 2025 autonomous mobile manipulator robot. These workflows demonstrate various automation patterns for 3-wheeled omnidirectional robot control.

## Quick Start

1. **Access n8n Web Interface**: `http://localhost:5678`
2. **Import Workflows**: Go to **Settings** → **Import/Export** → **Import**
3. **Select Workflow Files**: Choose the `.json` files from this directory
4. **Activate Workflows**: Click the **Active** toggle for each workflow

## Available Workflows

### 1. Robot Basic Movement Control (`robot_basic_control.json`)

**Purpose**: Manual robot control interface for basic movement commands

**Trigger**: Webhook (`POST /robot-control`)
**Features**:

- Forward, backward, left, right movement
- Clockwise/counter-clockwise rotation
- Emergency stop functionality

**Usage**:

```bash
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "forward"}'
```

**Commands Available**:

- `forward` - Move robot forward (0.5 m/s)
- `backward` - Move robot backward (-0.5 m/s)
- `left` - Strafe left (0.5 m/s lateral)
- `right` - Strafe right (-0.5 m/s lateral)
- `rotate_cw` - Rotate clockwise (0.5 rad/s)
- `rotate_ccw` - Rotate counter-clockwise (-0.5 rad/s)
- `stop` - Emergency stop (0 velocity)

### 2. Autonomous Square Patrol (`robot_patrol.json`)

**Purpose**: Automated square patrol pattern for surveillance tasks

**Trigger**: Schedule (every 30 seconds)
**Pattern**: Move forward 2m → Rotate 90° → Repeat (4 times)

**Features**:

- Scheduled autonomous patrol
- Square pattern movement
- Configurable timing and distances

**Customization**:

- Edit schedule interval in workflow settings
- Modify movement distances and rotation angles
- Add pause delays between movements

### 3. Reactive Obstacle Avoidance (`robot_obstacle_avoidance.json`)

**Purpose**: Real-time obstacle avoidance using laser scan data

**Trigger**: ROS 2 topic subscription (`/scan`)
**Behavior**:

- **Normal Speed** (> 1.0m): 0.5 m/s forward
- **Slow Down** (0.5-1.0m): 0.2 m/s forward
- **Strafe Right** (< 0.5m): 0.3 m/s lateral movement

**Features**:

- Real-time laser scan processing
- Reactive movement decisions
- Omnidirectional obstacle avoidance

### 4. Pick and Place Automation (`robot_pick_place.json`)

**Purpose**: Complete pick and place task automation

**Trigger**: Webhook (`POST /robot/pick_place`)
**Sequence**:

1. Navigate to pickup location
2. Lower arm and open gripper
3. Grip object
4. Navigate to place location
5. Release object

**Features**:

- Coordinated navigation and manipulation
- Error handling and status reporting
- Configurable pickup and place locations

### 5. Emergency Stop Monitor (`robot_emergency_stop.json`)

**Purpose**: Multi-trigger emergency stop system for safety

**Triggers**:

- Emergency button (webhook)
- IMU movement thresholds
- External emergency signals

**Actions**:

- Immediate robot stop (0 velocity)
- Arm retraction to safe position
- Email notification to team
- Emergency event logging

**Safety Features**:

- Multiple trigger sources (OR logic)
- Immediate response (< 100ms)
- Persistent logging for analysis
- Team notification system

### 6. Robot Test & Verification (`robot_test_verification.json`)

**Purpose**: Test and verify robot control systems

**Trigger**: Manual trigger for testing
**Tests**:

- Basic movement verification
- Status reporting
- System health checks

## Technical Requirements

### Custom n8n Nodes Required

You'll need to create these custom n8n nodes:

#### 1. ROS 2 Publisher Node

```javascript
// File: custom_nodes/ros2_publisher.js
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

#### 2. ROS 2 Subscriber Node

```javascript
// File: custom_nodes/ros2_subscriber.js
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

### Installation Steps

1. **Install Dependencies**:

```bash
npm install rosnodejs
```

2. **Create Custom Nodes Directory**:

```bash
mkdir -p ~/.n8n/custom_nodes
```

3. **Copy Node Files**:

```bash
cp custom_nodes/ros2_*.js ~/.n8n/custom_nodes/
```

4. **Restart n8n**:

```bash
docker restart n8n_container
```

## Usage Examples

### Manual Robot Control

```bash
# Move robot forward
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "forward"}'

# Rotate robot clockwise
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "rotate_cw"}'

# Emergency stop
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "stop"}'
```

### Pick and Place Task

```bash
curl -X POST http://localhost:5678/webhook/robot/pick_place \
  -H "Content-Type: application/json" \
  -d '{
    "pickup_location": "table_1",
    "place_location": "bin_a",
    "object_type": "cube"
  }'
```

### Emergency Stop

```bash
# Trigger emergency stop
curl -X POST http://localhost:5678/webhook/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"emergency": true, "reason": "manual_stop"}'
```

## Monitoring and Debugging

### View Workflow Executions

1. Go to **Executions** tab in n8n
2. Filter by workflow name
3. Check execution logs for errors

### ROS 2 Topic Monitoring

```bash
# Monitor robot velocity commands
ros2 topic echo /cmd_vel

# Monitor laser scan data
ros2 topic echo /scan

# Check system status
ros2 topic echo /test_status
```

### Workflow Logs

- Check n8n execution logs for workflow errors
- Monitor `/emergency_log_YYYY-MM-DD.txt` files for safety events
- Review ROS 2 logs in container: `docker logs ros2_sim_container`

## Performance Tips

1. **Schedule Optimization**: Adjust patrol intervals based on battery life
2. **Movement Smoothing**: Add gradual acceleration/deceleration
3. **Error Handling**: Implement retry logic for failed operations
4. **Resource Monitoring**: Track CPU/memory usage of workflows

## Security Considerations

1. **Webhook Authentication**: Add API keys to webhook triggers
2. **Input Validation**: Validate all incoming parameters
3. **Rate Limiting**: Implement rate limiting for frequent commands
4. **Access Control**: Restrict webhook access to trusted sources

## Customization Guide

### Modifying Movement Parameters

Edit the velocity values in each workflow:

```json
{
  "linear_x": 0.3,    // Forward/backward speed (m/s)
  "linear_y": 0.2,    // Lateral speed (m/s)
  "angular_z": 0.5    // Rotation speed (rad/s)
}
```

### Adding New Movement Commands

1. Edit the workflow's Set node
2. Add new command mappings
3. Update webhook trigger documentation

### Custom Behaviors

- **Path Planning**: Integrate with Nav2 for waypoint navigation
- **Object Detection**: Add computer vision workflows
- **Voice Control**: Implement speech recognition triggers
- **IoT Integration**: Connect with smart home systems

## Troubleshooting

### Common Issues

**Workflow not triggering:**

- Check webhook URL is correct
- Verify n8n container is running
- Check network connectivity

**ROS 2 communication errors:**

- Ensure ROS 2 container is running
- Verify topic names match robot configuration
- Check custom nodes are properly installed

**Movement not working:**

- Verify robot is powered on and connected
- Check motor drivers and wheel encoders
- Calibrate robot kinematics parameters

**Emergency stop not responding:**

- Check IMU sensor data quality
- Verify email notification settings
- Test emergency button webhook

## Additional Resources

- [n8n Documentation](https://docs.n8n.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Custom n8n Nodes Guide](https://docs.n8n.io/integrations/creating-nodes/)
- [ROS 2 Topic Communication](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

**Happy Robot Automation! 🤖**

*These workflows are specifically designed for your 3-wheeled omnidirectional robot and leverage its unique movement capabilities for optimal performance in the LKS 2025 competition.*
