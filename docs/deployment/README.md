# Deployment Guide

Four Docker containers: PostgreSQL, Redis, Python backend, TypeScript frontend.

## Quick Start

```bash
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git
cd Autonomous_Mobile_Manipulator
docker compose up --build -d
```

First build takes 3-5 minutes (ROS2 colcon build + npm install + Prisma generate).

## Services

| Service | Container | Port | Purpose |
|---------|-----------|------|---------|
| postgres | robot_postgres | 5432 (internal) | PostgreSQL 16 |
| redis | robot_redis | 6379 (internal) | Sensor cache, rate limiting |
| backend | robot_backend | 8001 | Flask API + ROS2 |
| frontend | robot_frontend | 3000 | nginx serving TypeScript UI |

## Ports

| Port | Service | External Access |
|------|---------|-----------------|
| 3000 | Frontend (nginx) | Yes — primary UI |
| 8001 | Backend (Flask) | Yes — REST API + WebSocket |
| 5432 | PostgreSQL | No — internal only |
| 6379 | Redis | No — internal only |

## Environment Variables

Set in `docker-compose.yml`:

```yaml
backend:
  environment:
    - FLASK_PORT=8001
    - DATABASE_URL=postgresql://robot:robot_secret@postgres:5432/robot_automation
    - REDIS_URL=redis://redis:6379/0
```

## Backend Startup Sequence

1. Install Python dependencies (`pip3 install -r requirements.txt`)
2. Generate Prisma client (`prisma generate`)
3. Push schema to database (`prisma db push --accept-data-loss`)
4. Build ROS2 packages (`colcon build`)
5. Start Flask API on port 8001

The backend container logs `Backend API: http://localhost:8001` when ready.

## Production Deployment (Raspberry Pi 5)

### Prerequisites

- Raspberry Pi 5 (8GB RAM recommended)
- Ubuntu Server 24.04 LTS (64-bit)
- Docker installed (`curl -fsSL https://get.docker.com | sh`)
- USB connection to Arduino Mega

### Steps

```bash
# Clone on Pi
git clone https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator.git
cd Autonomous_Mobile_Manipulator

# Build and start
docker compose up --build -d

# Verify
docker compose ps
curl http://localhost:8001/health
```

### Auto-start on Boot

```bash
sudo tee /etc/systemd/system/robot.service > /dev/null <<EOF
[Unit]
Description=Robot Control System
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/ubuntu/Autonomous_Mobile_Manipulator
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl start robot
```

### Serial Port Access

The backend container needs access to the Arduino Mega USB serial port:

```bash
# Find the port
ls /dev/ttyUSB* /dev/ttyACM*

# The docker-compose.yml maps /dev devices — update if your port differs
# Or run with --device flag
```

## Health Check

```bash
# Backend health
curl http://localhost:8001/health

# Frontend
curl -s http://localhost:3000 | head -5

# Container status
docker compose ps

# Logs
docker compose logs backend --tail 20
```

## Rebuild

```bash
# Full rebuild (no cache)
docker compose build --no-cache
docker compose up -d

# Backend only
docker compose up --build backend -d
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Backend won't start | Check `docker compose logs backend` — usually Prisma or colcon build failure |
| Port 8001 in use | `lsof -i :8001` — kill conflicting process |
| Port 3000 in use | `lsof -i :3000` — kill conflicting process |
| Serial port not found | Check USB cable, `ls /dev/ttyUSB*`, ensure Mega is powered |
| Database connection failed | Check postgres container: `docker compose logs postgres` |
| Redis connection failed | Check redis container: `docker compose logs redis` |
| Frontend can't reach backend | Check nginx config proxies to `backend:8001` (Docker DNS) |
