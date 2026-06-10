# ROS2 Packages

ROS2 Jazzy on Ubuntu 24.04. The ROS2 layer handles robot description (URDF) and system bringup. The control logic lives in the Python Flask backend, not in ROS2 nodes.

## Packages

| Package | Purpose |
|---------|---------|
| `my_robot_description` | URDF model, robot description |
| `my_robot_bringup` | Launch files for simulation and real robot |
| `my_robot_automation` | Flask API, automation engine, AI decision engine, sensor management |

## Build

```bash
# Inside backend container
cd /root/ros2_ws
colcon build --packages-select my_robot_description my_robot_bringup my_robot_automation
source install/setup.bash
```

The backend container runs `colcon build` automatically on startup.

## Communication

The Python backend (Flask) handles all REST API and WebSocket communication. It communicates with the Arduino Mega over serial. ROS2 packages provide the robot description and launch infrastructure.

```
Flask Backend ──serial──► Arduino Mega (sensors, motors, servos)
Flask Backend ◄──WebSocket──► Frontend (real-time sensor data)
Flask Backend ──Prisma──► PostgreSQL (automation rules, paths, decisions)
Flask Backend ──Redis──► Redis (sensor cache, rate limiting)
```

## CycloneDDS Configuration

ROS_DOMAIN_ID=42 by default. Configuration file at `ros2_ws/cyclonedds_config.xml`.

```bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///root/ros2_ws/cyclonedds_config.xml
```

## Monitoring

- Watchdog script: `ros2_ws/ros2_watchdog.sh` — monitors critical processes, auto-restarts on failure
- Health check: Docker healthcheck on backend container
- Logs: `docker compose logs backend`
