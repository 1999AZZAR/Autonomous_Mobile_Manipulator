# Development Guide

## Development Environment

Edit code on your host machine. Run and test inside Docker containers. Volume mounts sync `ros2_ws/src/` and `frontend/src/` to the containers.

### IDE Setup

VS Code with these extensions:
- Python (`ms-python.python`)
- TypeScript (`ms-vscode.typescript`)
- Docker (`ms-azuretools.vscode-docker`)
- ROS (`ms-iot.ros`)
- GitLens (`eamodio.gitlens`)

## Workflow

### Backend (Python)

1. Edit files in `ros2_ws/src/my_robot_automation/scripts/`
2. Rebuild and restart:
   ```bash
   docker compose up --build backend -d
   ```
3. Check logs:
   ```bash
   docker compose logs backend -f
   ```

### Frontend (TypeScript)

1. Edit files in `frontend/src/`
2. Vite dev server auto-reloads. For production build:
   ```bash
   docker compose up --build frontend -d
   ```
3. TypeScript check:
   ```bash
   docker exec robot_frontend npx tsc --noEmit
   ```

### Arduino Mega Firmware

1. Edit files in `for_the_mega/`
2. Upload via Arduino IDE or PlatformIO
3. Reference: [COMMANDS.md](../for_the_mega/COMMANDS.md), [PINOUT.md](../for_the_mega/PINOUT.md)

## Project Conventions

### Python

- Async Prisma calls go through `_run_async()` helper (persistent background event loop)
- JSON Prisma fields require `prisma.Json()` wrapper
- Nullable JSON fields must be omitted from create data (not `None`)
- No `ultralytics` in requirements.txt — handled gracefully in code

### TypeScript

- Strict mode enabled
- Carbon Design System components
- Phosphor Icons for iconography
- Each tab is a separate component file in `src/components/`
- Three.js digital twin in `src/engine/`

### Serial Protocol

- Single-character commands for movement
- Two-character commands for sensors/actuators
- Three-character parameterized commands (`ta45`, `ga90`)
- Reference: `for_the_mega/COMMANDS.md`

## Testing

### Manual API Test

```bash
# Health
curl http://localhost:8001/health

# Sensors
curl http://localhost:8001/api/robot/sensors

# Movement
curl -X POST http://localhost:8001/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Emergency stop
curl -X POST http://localhost:8001/api/robot/emergency-stop
```

### Frontend Build Check

```bash
docker exec robot_frontend npx tsc --noEmit
```

### Prisma Schema Check

```bash
docker exec robot_backend bash -c "cd /root/ros2_ws/src/my_robot_automation && prisma validate"
```

## Git Workflow

```bash
# Feature branch
git checkout -b feature/my-feature

# Commit convention
git commit -m "feat: Add new sensor visualization"
git commit -m "fix: Correct IR ADC to mm conversion"
git commit -m "docs: Update pinout reference"

# Push and create PR
git push origin feature/my-feature
```

## Common Tasks

### Add a New API Endpoint

1. Add route in `app.py` or `automation_api.py`
2. Add TypeScript client function in `frontend/src/api.ts`
3. Add TypeScript interface in `frontend/src/types.ts`
4. Add UI component in `frontend/src/components/`
5. Register route in `main.py` if new blueprint

### Add a New Sensor

1. Connect sensor to Arduino Mega (see `for_the_mega/PINOUT.md`)
2. Add parsing in `mega_interface.py`
3. Add to `sensor_manager.py` collection
4. Add flat key mapping in `_normalize_sensor_data()`
5. Add TypeScript interface in `types.ts`

### Add a New Database Model

1. Add model to `ros2_ws/src/my_robot_automation/prisma/schema.prisma`
2. Run `prisma generate && prisma db push` inside container
3. Add API routes using `_run_async()` for all DB calls
