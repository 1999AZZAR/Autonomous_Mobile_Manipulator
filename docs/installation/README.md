# Installation Guide

## Prerequisites

- **OS**: Ubuntu 22.04+ or Debian 12+ (64-bit)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free for Docker images
- **Docker**: 24.x+ with Docker Compose v2
- **Git**: any recent version

## Install Docker

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo usermod -aG docker $USER
newgrp docker
```

Verify:
```bash
docker --version
docker compose version
```

## Clone and Run

```bash
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git
cd Autonomous_Mobile_Manipulator
docker compose up --build -d
```

First build takes 3-5 minutes. Subsequent starts take 30-60 seconds.

## Verify

```bash
# Check containers
docker compose ps

# Backend health
curl http://localhost:8001/health

# Frontend
curl -s http://localhost:3000 | head -5
```

Expected output:
```
NAME                STATUS          PORTS
robot_backend       Up (healthy)    0.0.0.0:8001->8001/tcp
robot_frontend      Up              0.0.0.0:3000->80/tcp
robot_postgres      Up (healthy)    5432/tcp
robot_redis         Up (healthy)    6379/tcp
```

## Access

- **Frontend UI**: http://localhost:3000
- **REST API**: http://localhost:8001
- **WebSocket**: ws://localhost:8001/ws/sensors

## Serial Port (Physical Robot)

For the Arduino Mega connection, the backend container needs access to the USB serial device:

```bash
# Find the port
ls /dev/ttyUSB* /dev/ttyACM*

# Add device mapping to docker-compose.yml under backend service:
# devices:
#   - "/dev/ttyUSB0:/dev/ttyUSB0"
```

## X11 Display (Optional, for ROS2 GUI tools)

```bash
xhost +local:docker
```

## Uninstall

```bash
docker compose down -v
cd ..
rm -rf Autonomous_Mobile_Manipulator
```
