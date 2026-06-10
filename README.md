# Autonomous Mobile Manipulator

Hexagonal 3-wheel omnidirectional mobile manipulator with Arduino Mega real-time control, Raspberry Pi high-level planning, and containerized deployment.

## Architecture

```
┌─────────────────────────────────────────────┐
│              Raspberry Pi                    │
│  ┌─────────┐  ┌──────────┐  ┌───────────┐  │
│  │  ROS2   │  │  Flask   │  │  nginx    │  │
│  │  (nav)  │  │  :8001   │  │  :3000    │  │
│  └─────────┘  └──────────┘  └───────────┘  │
│       │            │              │          │
│   mega_interface  Redis + PG    frontend    │
│  ┌─────────┐  ┌──────────┐  ┌───────────┐  │
│  │  serial │  │ :5432    │  │  TS+Vite  │  │
│  │  :9090  │  │          │  │           │  │
│  └─────────┘  └──────────┘  └───────────┘  │
└─────────────────────────────────────────────┘
                    │
                    ▼
         User Browser (http://pi:3000)
```

## Hardware

- **MCU**: Arduino Mega (serial 115200 baud)
- **Drive**: 3x omni wheels (PG23 motors, L298N drivers, YFROBOT v2 shield)
- **Sensors**: 6x Sharp IR (GP2Y0A02YK0F), 2x HC-SR04 ultrasonic, 3x line sensors
- **IMU**: MPU6050 (on Raspberry Pi via I2C)
- **Manipulator**: Gripper servo (open/close/half), tilt servo (0-180°), lifter motor (up/down with limit switches)
- **Camera**: USB OpenCV (640x480, power-aware, off by default)

## Mega Serial Commands

| Category | Commands |
|----------|----------|
| Movement | `f` `b` `l` `r` `q` `e` `z` `x` (8-directional) |
| Rotation | `c` `w` (spin), `t` `y` (turn), `a` `j` (arc) |
| Speed | `5` `6` `7` `8` `9` `0` (50%-100%) |
| Turbo | `o` (toggle) |
| Gripper | `no` `nc` `nh` (open/close/half), `ga<angle>` |
| Tilt | `mu` `md` `mc` (up/down/center), `ta<angle>` |
| Lifter | `u` `d` (up/down) |
| Safety | `se` `sd` (enable/disable perimeter), `ls` (limit switches) |
| Sensors | `sr` (read all), `spe` `spd` (enable/disable publishing) |
| Control | `s` (stop), `p` (status), `v` (emergency stop) |
| Test | `1`-`4` (motor test), `g` (figure-8), `h` (rotation) |

## Quick Start

```bash
# Clone and start
git clone <repo-url> && cd Autonomous_Mobile_Manipulator
docker compose up -d

# Access
# Frontend: http://localhost:3000
# API:      http://localhost:8001/api/*
# WebSocket: ws://localhost:8001/ws/sensors
```

## Docker Services

| Service | Port | Purpose |
|---------|------|---------|
| frontend | 3000 | TypeScript UI (Vite build, nginx) |
| backend | 8001 | Flask REST API + automation engine |
| postgres | 5432 | Prisma database |
| redis | 6379 | Sensor cache + rate limiting |

## API Endpoints

### Robot Control

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/command` | Send raw mega command |
| POST | `/api/robot/move` | Move in direction |
| POST | `/api/robot/turn` | Turn left/right |
| POST | `/api/robot/stop` | Stop all movement |
| POST | `/api/robot/speed` | Set speed (0-100%) |
| POST | `/api/robot/turbo` | Toggle turbo |
| POST | `/api/robot/emergency-stop` | Emergency stop |
| GET | `/api/robot/position` | Get robot position |

### Manipulator

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/robot/picker/gripper` | Open/close gripper |
| POST | `/api/robot/picker/gripper_tilt` | Set tilt angle |
| POST | `/api/robot/lifter/up` | Lifter up |
| POST | `/api/robot/lifter/down` | Lifter down |
| POST | `/api/robot/camera/tilt` | Camera tilt (up/down/center) |

### Sensors

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/sensors` | All sensor readings |
| GET | `/api/status` | System status |
| GET | `/api/feeds` | Sensor feed values |
| GET | `/ws/sensors` | SSE sensor stream |

### Safety

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/robot/safety/perimeter` | Enable/disable perimeter safety |
| GET | `/api/robot/safety/limit-switches` | Test limit switches |
| POST | `/api/robot/sensor-publishing` | Enable/disable sensor publishing |

### Sequences

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/robot/sequences/execute` | Execute sequence |
| POST | `/api/robot/sequences/save` | Save sequence |
| GET | `/api/robot/sequences/load/<name>` | Load sequence |
| GET | `/api/robot/sequences/list` | List sequences |

### Automations

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/automations` | List automations |
| POST | `/api/automations` | Create automation |
| PUT | `/api/automations/<id>` | Update automation |
| DELETE | `/api/automations/<id>` | Delete automation |
| POST | `/api/automations/<id>/toggle` | Enable/disable |
| POST | `/api/automations/<id>/run` | Run automation |

### AI Decision Engine

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/ai/status` | AI engine status |
| POST | `/api/ai/start` | Start AI loop |
| POST | `/api/ai/stop` | Stop AI loop |
| POST | `/api/ai/analyze` | Analyze once |
| GET | `/api/ai/decisions` | Decision history |
| POST | `/api/ai/guidance` | Inject human guidance |

### Waypoint Memory

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/waypoints/paths` | List saved paths |
| POST | `/api/waypoints/paths` | Create path |
| POST | `/api/waypoints/record` | Start/stop recording |
| POST | `/api/waypoints/replay` | Start/stop replay |
| GET | `/api/waypoints/status` | Recording/replay status |

## Project Structure

```
Autonomous_Mobile_Manipulator/
├── docker-compose.yml
├── README.md
├── TODO.md                    # Digital twin roadmap
├── for_the_mega/              # Arduino Mega firmware docs
│   ├── COMMANDS.md            # Complete command reference
│   ├── PINOUT.md              # Hardware pin assignments
│   ├── LOGGING_CODES.md       # Compact logging codes
│   └── PID_CONTROL.md         # PID tuning guide
├── frontend/                  # TypeScript + Vite
│   └── src/
│       ├── api.ts             # API client
│       ├── types.ts           # TypeScript interfaces
│       ├── main.ts            # Tab routing + init
│       └── components/
│           ├── dashboard.ts   # Sensor values + KPI
│           ├── movement.ts    # 8-directional controls
│           ├── manipulator.ts # Arm + gripper
│           ├── map-view.ts    # Canvas navigation map
│           ├── automations.ts # Automation CRUD
│           ├── ai-control.ts  # AI + camera + waypoints
│           └── system.ts      # DB/Redis/engine status
├── ros2_ws/src/my_robot_automation/
│   ├── prisma/schema.prisma   # Database schema (7 models)
│   ├── scripts/
│   │   ├── main.py            # Entry point
│   │   ├── mega_interface.py  # Serial communication
│   │   ├── sensor_manager.py  # Sensor data collection
│   │   ├── automation_engine.py  # IFTTT rules + rate limiting
│   │   ├── automation_api.py  # Flask blueprints (AI + waypoints)
│   │   ├── app.py             # Flask REST API (robot control)
│   │   ├── camera_service.py  # USB camera (power-aware)
│   │   ├── ai_decision.py     # AI brain (YOLO + API)
│   │   ├── waypoint_memory.py # Waypoint record/replay
│   │   ├── path_planning.py   # A* pathfinding
│   │   ├── gpio_controller.py # GPIO (simulation mode)
│   │   └── config.py          # Configuration
│   └── requirements.txt
└── docs/                      # Documentation
```

## Database Models (Prisma)

| Model | Purpose |
|-------|---------|
| Automation | IFTTT-style automation rules |
| AutomationCondition | Trigger conditions |
| AutomationAction | Actions to execute |
| AutomationLog | Execution history |
| SavedPath | Waypoint paths |
| Waypoint | Individual waypoints |
| AiDecision | AI decision history |

## AI Decision Engine

Three operating modes:

1. **REPLAY** — Saved waypoints via IMU dead reckoning (camera OFF)
2. **IFTTT** — Sensor-triggered automations (camera OFF)
3. **AI** — Camera + YOLOv8 + OpenAI API reasoning (camera ON)

The engine auto-selects the best mode based on the task goal.

## Development

```bash
# Backend (Python)
cd ros2_ws/src/my_robot_automation
pip install -r requirements.txt
python scripts/main.py --simulation

# Frontend (TypeScript)
cd frontend
npm install
npm run dev    # Vite dev server
npm run build  # Production build

# Database
npx prisma generate
npx prisma db push
```

## Deployment

The system runs on Raspberry Pi 5 with Docker. Dashboard accessible over local network, intranet, or internet (with port forwarding or tunnel).

```bash
# On Raspberry Pi
docker compose up -d

# Check status
docker compose ps
curl http://localhost:8001/api/status
```
