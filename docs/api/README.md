# API Documentation

This documentation provides comprehensive information about the available APIs for controlling and monitoring the Autonomous Mobile Manipulator system.

## Table of Contents

- [API Overview](#api-overview)
  - [Available Interfaces](#available-interfaces)
- [Robot REST API](#robot-rest-api)
  - [Basic Movement Control](#basic-movement-control)
  - [Actuator Control](#actuator-control)
  - [Status Monitoring](#status-monitoring)
- [n8n Interface](#n8n-interface)
- [WebSocket API](#websocket-api)
- [ROS 2 Services](#ros-2-services)
  - [Available Services](#available-services)
  - [Python Service Client Examples](#python-service-client-examples)
- [ROS 2 Topics](#ros-2-topics)
  - [Command Topics](#command-topics)
  - [Sensor Topics](#sensor-topics)
- [Error Handling](#error-handling)
  - [HTTP Status Codes](#http-status-codes)
  - [Response Format](#response-format)
- [Integration Examples](#integration-examples)
  - [Python Integration](#python-integration)
- [Support](#support)

## API Overview

The robot exposes multiple API interfaces for different use cases:

### Available Interfaces

| Interface | Protocol | Port | Purpose | Documentation |
|-----------|----------|------|---------|---------------|
| **Robot REST API** | HTTP/HTTPS | 5000 | Direct robot control | [Robot API](#robot-rest-api) |
| **n8n Web Interface** | HTTP/HTTPS | 5678 | Workflow automation | [n8n Interface](#n8n-interface) |
| **WebSocket API** | WebSocket | 8765 | Real-time communication | [WebSocket API](#websocket-api) |
| **ROS 2 Topics** | ROS 2 DDS | N/A | Low-level robot control | [ROS 2 Topics](#ros-2-topics) |
| **ROS 2 Services** | ROS 2 DDS | N/A | Advanced robotics services | [ROS 2 Services](#ros-2-services) |

## Robot REST API

The robot provides a direct REST API on port 5000 for control and monitoring.

### Basic Movement Control

```bash
# Move robot forward/backward
curl -X POST http://localhost:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn robot left/right
curl -X POST http://localhost:5000/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Stop robot
curl -X POST http://localhost:5000/api/robot/stop

# Emergency stop
curl -X POST http://localhost:5000/api/robot/emergency
```

### Actuator Control

```bash
# Control lifter
curl -X POST http://localhost:5000/api/robot/lifter \
  -H "Content-Type: application/json" \
  -d '{"action": "up", "speed": 0.5}'

# Control individual servo
curl -X POST http://localhost:5000/api/robot/servo \
  -H "Content-Type: application/json" \
  -d '{"servo": 1, "angle": 90}'

# Control all servos
curl -X POST http://localhost:5000/api/robot/servos \
  -H "Content-Type: application/json" \
  -d '{"action": "home"}'
```

### Status Monitoring

```bash
# Get comprehensive robot status (includes sensor data)
curl http://localhost:5000/api/robot/status

# Get detailed sensor data
curl http://localhost:5000/api/robot/sensors

# Get active tasks
curl http://localhost:5000/api/robot/tasks

# Cancel a running task
curl -X POST http://localhost:5000/api/robot/tasks/task_123/cancel \
  -H "Content-Type: application/json" \
  -d '{"reason": "User requested cancellation"}'

# Get navigation status
curl http://localhost:5000/api/robot/navigation/status

# Get navigation status with map data
curl http://localhost:5000/api/robot/navigation/status?include_map=true&include_path=true
```

## n8n Interface

The n8n workflow automation interface is available at http://localhost:5678 and provides:

- Visual workflow designer
- Pre-configured robot control workflows
- HTTP webhook triggers for external integration
- Scheduled automation tasks

## WebSocket API

Real-time communication interface available on port 8765.

### JavaScript Client Example

```javascript
// WebSocket connection for real-time control
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = function() {
  console.log('Connected to robot');
  
  // Send velocity command
  ws.send(JSON.stringify({
    type: 'velocity_command',
    linear: { x: 0.5, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 }
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Sensor data:', data);
};
```

## ROS 2 Services

Advanced robotics services for comprehensive robot control and monitoring.

### Available Services

| Service | Service Type | Description |
|---------|--------------|-------------|
| `/get_sensor_data` | `my_robot_automation/srv/GetSensorData` | Retrieve comprehensive sensor data |
| `/get_task_status` | `my_robot_automation/srv/GetTaskStatus` | Get status of active tasks |
| `/cancel_task` | `my_robot_automation/srv/CancelTask` | Cancel running automation tasks |
| `/get_navigation_status` | `my_robot_automation/srv/GetNavigationStatus` | Get navigation and localization status |
| `/get_robot_status` | `my_robot_automation/srv/GetRobotStatus` | Get comprehensive robot status |
| `/set_robot_mode` | `my_robot_automation/srv/SetRobotMode` | Set robot operating mode |
| `/emergency_stop` | `my_robot_automation/srv/EmergencyStop` | Emergency stop control |
| `/execute_pick_place` | `my_robot_automation/srv/ExecutePickPlace` | Execute pick and place operations |
| `/execute_patrol` | `my_robot_automation/srv/ExecutePatrol` | Execute autonomous patrol |
| `/execute_obstacle_avoidance` | `my_robot_automation/srv/ExecuteObstacleAvoidance` | Navigate with obstacle avoidance |

### Python Service Client Examples

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_automation.srv import GetSensorData, GetTaskStatus, CancelTask

class RobotServiceClient(Node):
    def __init__(self):
        super().__init__('robot_service_client')

    def get_sensor_data(self):
        """Get comprehensive sensor data"""
        client = self.create_client(GetSensorData, 'get_sensor_data')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sensor data service not available, waiting...')

        request = GetSensorData.Request()
        request.include_distance_sensors = True
        request.include_line_sensor = True
        request.include_imu_data = True
        request.include_battery_status = True

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Sensor data: ultrasonic_front={response.sensor_data.ultrasonic_front}')
                return response.sensor_data
        return None

    def get_task_status(self, task_id=''):
        """Get status of active tasks"""
        client = self.create_client(GetTaskStatus, 'get_task_status')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Task status service not available, waiting...')

        request = GetTaskStatus.Request()
        request.task_id = task_id

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Found {len(response.active_tasks)} active tasks')
            return response.active_tasks
        return []

    def cancel_task(self, task_id, reason='User requested'):
        """Cancel a running task"""
        client = self.create_client(CancelTask, 'cancel_task')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Cancel task service not available, waiting...')

        request = CancelTask.Request()
        request.task_id = task_id
        request.reason = reason

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Task {response.task_id} cancelled: {response.message}')
            return response.success
        return False

def main():
    rclpy.init()
    client = RobotServiceClient()

    # Get sensor data
    sensor_data = client.get_sensor_data()
    if sensor_data:
        print(f"Battery: {sensor_data.battery_percentage}%")

    # Get active tasks
    tasks = client.get_task_status()
    for task in tasks:
        print(f"Task {task.task_id}: {task.status}")

    # Cancel a task (if any exist)
    if tasks:
        client.cancel_task(tasks[0].task_id, 'API test cancellation')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Topics

Low-level robot control through ROS 2 topics.

### Command Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/picker/gripper` | `std_msgs/Bool` | Gripper open/close commands |
| `/picker/gripper_tilt` | `std_msgs/Float32` | Gripper tilt angle commands |
| `/picker/gripper_neck` | `std_msgs/Float32` | Gripper neck position commands |
| `/picker/gripper_base` | `std_msgs/Float32` | Gripper base height commands |
| `/containers/left_front` | `std_msgs/String` | Left front container commands |
| `/containers/left_back` | `std_msgs/String` | Left back container commands |
| `/containers/right_front` | `std_msgs/String` | Right front container commands |
| `/containers/right_back` | `std_msgs/String` | Right back container commands |

### Sensor Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR scan data |
| `/distance/front` | `std_msgs/Float32` | Front ultrasonic sensor |
| `/distance/back_left` | `std_msgs/Float32` | Back left ultrasonic sensor |
| `/distance/back_right` | `std_msgs/Float32` | Back right ultrasonic sensor |
| `/line_sensor/raw` | `std_msgs/Int32` | Line sensor raw data |
| `/imu/data` | `sensor_msgs/Imu` | IMU sensor data |
| `/battery/status` | `sensor_msgs/BatteryState` | Battery status data |

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, speed=0.5):
        twist = Twist()
        twist.linear.x = speed
        self.publisher.publish(twist)

def main():
    rclpy.init()
    controller = RobotController()
    controller.move_forward(0.5)
    rclpy.shutdown()
```

## Error Handling

### HTTP Status Codes

| Code | Description |
|------|-------------|
| `200` | Success |
| `400` | Bad Request |
| `404` | Not Found |
| `500` | Internal Server Error |

### Response Format

```json
{
  "success": true,
  "message": "Robot moved forward at 0.5 m/s"
}
```

## Integration Examples

### Python Integration

```python
import requests
import json

class RobotAPI:
    def __init__(self, base_url='http://localhost:5000'):
        self.base_url = base_url

    def move_forward(self, speed=0.5):
        """Move robot forward at specified speed"""
        response = requests.post(
            f'{self.base_url}/api/robot/move',
            json={'direction': 'forward', 'speed': speed}
        )
        return response.json()

    def get_status(self):
        """Get comprehensive robot status including sensor data"""
        response = requests.get(f'{self.base_url}/api/robot/status')
        return response.json()

    def get_sensor_data(self):
        """Get detailed sensor readings"""
        response = requests.get(f'{self.base_url}/api/robot/sensors')
        return response.json()

    def get_active_tasks(self):
        """Get list of currently active tasks"""
        response = requests.get(f'{self.base_url}/api/robot/tasks')
        return response.json()

    def cancel_task(self, task_id, reason='API request'):
        """Cancel a running task"""
        response = requests.post(
            f'{self.base_url}/api/robot/tasks/{task_id}/cancel',
            json={'reason': reason}
        )
        return response.json()

    def get_navigation_status(self, include_map=False, include_path=False):
        """Get navigation and localization status"""
        params = {}
        if include_map:
            params['include_map'] = 'true'
        if include_path:
            params['include_path'] = 'true'

        response = requests.get(
            f'{self.base_url}/api/robot/navigation/status',
            params=params
        )
        return response.json()

    def execute_pick_place(self, pickup_location, place_location, object_type='box'):
        """Execute pick and place operation"""
        response = requests.post(
            f'{self.base_url}/api/robot/pick-place',
            json={
                'pickup_location': pickup_location,
                'place_location': place_location,
                'object_type': object_type
            }
        )
        return response.json()

# Usage example
def main():
    robot = RobotAPI()

    # Get current status with sensor data
    status = robot.get_status()
    print(f"Robot mode: {status['data']['mode']}")
    print(f"Battery: {status['data']['battery_percentage']}%")
    print(f"Ultrasonic front: {status['data']['ultrasonic_front']}m")

    # Get detailed sensor data
    sensors = robot.get_sensor_data()
    print(f"Line sensor: {sensors['data']['line_sensor_raw']}")
    print(f"IMU orientation: {sensors['data']['orientation']['w']:.2f}")

    # Check for active tasks
    tasks = robot.get_active_tasks()
    print(f"Active tasks: {tasks['count']}")

    # Execute a pick and place operation
    pickup = {'position': {'x': 1.0, 'y': 0.0, 'z': 0.0}}
    place = {'position': {'x': -1.0, 'y': 0.0, 'z': 0.0}}

    result = robot.execute_pick_place(pickup, place, 'container')
    if result['success']:
        print(f"Pick and place completed: {result['message']}")
        # Could cancel the task if needed
        # robot.cancel_task(result['task_id'], 'Demo cancellation')
    else:
        print(f"Operation failed: {result['error']}")

if __name__ == '__main__':
    main()
```

## Support

For API integration questions:
- Review the specific API documentation sections
- Check the [Troubleshooting Guide](../troubleshooting/)
- Test with the provided examples first
