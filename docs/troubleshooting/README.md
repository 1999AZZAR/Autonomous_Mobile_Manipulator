# Troubleshooting

## Quick Diagnostics

```bash
# Container status
docker compose ps

# Backend logs
docker compose logs backend --tail 30

# Redis connectivity
docker exec robot_redis redis-cli ping

# PostgreSQL connectivity
docker exec robot_postgres pg_isready -U robot

# Serial port
ls /dev/ttyUSB* /dev/ttyACM*
```

## Common Issues

### Backend Won't Start

**Symptom**: `robot_backend` exits immediately or stays in restart loop.

```bash
docker compose logs backend --tail 50
```

Common causes:
- **Prisma migration failure**: Database schema mismatch. Fix: `docker compose down -v && docker compose up --build`
- **colcon build failure**: ROS2 package compilation error. Fix: check `docker compose logs backend` for the specific build error.
- **pip install failure**: Network issue or dependency conflict. Fix: rebuild with `--no-cache`.
- **Port 8001 in use**: Another process occupies the port. Fix: `lsof -i :8001` and kill the process.

### Frontend Can't Reach Backend

**Symptom**: UI loads but shows connection errors or empty data.

- Check backend is running: `docker compose ps`
- Check nginx config proxies correctly: `docker exec robot_frontend cat /etc/nginx/conf.d/default.conf`
- Backend must be accessible as `backend:8001` from inside Docker network

### Port Conflicts

```bash
# Find what's using the port
lsof -i :3000
lsof -i :8001

# Kill the process
kill <PID>
```

### Serial Port Not Found

**Symptom**: `mega_interface.py` can't open `/dev/ttyUSB0`.

```bash
# Check if Mega is connected
ls /dev/ttyUSB* /dev/ttyACM*

# Check permissions
ls -la /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

### Database Connection Failed

```bash
# Check postgres container
docker compose logs postgres

# Reset database
docker compose down -v
docker compose up --build -d
```

### Redis Connection Failed

```bash
# Check redis container
docker compose logs redis

# Test connectivity
docker exec robot_redis redis-cli ping
# Should return: PONG
```

### Sensor Data Shows Zeros

**Symptom**: `/api/robot/sensors` returns all-zero values.

- In simulation mode (`--simulation` flag), sensor data comes from mock generators
- In real mode, check serial connection to Mega
- Check `docker compose logs backend` for serial port errors
- Verify Mega firmware is running (open serial monitor at 115200 baud)

### Frontend Build Errors

```bash
# TypeScript errors
docker exec robot_frontend npx tsc --noEmit

# Full rebuild
docker compose up --build frontend -d
```

### Docker Permission Denied

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Prisma Client Not Generated

**Symptom**: `prisma-client-py` import errors in backend.

```bash
docker exec robot_backend bash -c "cd /root/ros2_ws/src/my_robot_automation && prisma generate"
```

## Emergency Stop

If the robot is moving uncontrollably:

1. **Software**: POST to `/api/robot/emergency-stop`
2. **Physical**: Disconnect power supply immediately
3. **Mega command**: Send `v` over serial for force stop

## Logs

| Service | Log Location |
|---------|-------------|
| Backend | `docker compose logs backend` |
| Frontend | `docker compose logs frontend` |
| PostgreSQL | `docker compose logs postgres` |
| Redis | `docker compose logs redis` |
| Mega serial | Backend stdout (check `docker compose logs backend`) |

## Reset Everything

```bash
docker compose down -v
docker system prune -f
docker compose up --build -d
```

This removes all containers, volumes, and rebuilds from scratch. Database data is lost.
