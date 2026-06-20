# Autonomous Mobile Manipulator

Hexagonal 3-wheel omnidirectional mobile manipulator with Arduino Mega real-time control, host-based path planning (RRT*), digital twin simulation (Three.js), and containerized deployment.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│              Host (RPi / Laptop / Mini PC)           │
│  ┌──────────────────┐  ┌──────────────────────────┐ │
│  │  Flask API :8001 │  │  Frontend (Vite) :3000   │ │
│  │  - robot control │  │  - Dashboard             │ │
│  │  - sensor stream │  │  - Digital Twin (3D)     │ │
│  │  - AI decision   │  │  - Map View (canvas)     │ │
│  │  - path planning │  │  - Controls              │ │
│  │  - RRT* replan   │  │                          │ │
│  └────────┬─────────┘  └──────────────────────────┘ │
│           │                       ▲                  │
│           │  HTTP/WS              │ HTTP proxy       │
│           ▼                       │                  │
│  ┌─────────────────────────────────┴──────────┐    │
│  │        nginx (frontend → backend proxy)    │    │
│  └────────────────────────────────────────────┘    │
│           │                                        │
│           ▼ serial 115200                          │
│  ┌────────────────────────────────────────────┐    │
│  │      Arduino Mega (I/O device)             │    │
│  │  Sensors: 6× IR, 2× ultrasonic, 3× line   │    │
│  │  Actuators: 3× motors, gripper, tilt,     │    │
│  │             lifter                         │    │
│  │  Safety: virtual bumper + APF last-resort  │    │
│  └────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

## Hardware

- **MCU**: Arduino Mega (serial 115200 baud)
- **Drive**: 3× omni wheels (PG23 motors, L298N drivers, YFROBOT v2 shield)
- **Sensors**: 6× Sharp IR (GP2Y0A02YK0F), 2× HC-SR04 ultrasonic, 3× line sensors
- **IMU**: MPU6050 (on host via I2C — no Mega involvement)
- **Camera**: USB OpenCV (640×480, power-aware, off by default)
- **Lidar**: TF-Luna (on host via UART — no Mega involvement)
- **Manipulator**: Gripper servo (open/close/half), tilt servo (0-180°), lifter motor (up/down with limit switches)

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

## Quick Start (Simulation)

```bash
# Clone and start simulation (no hardware needed)
docker compose up -d

# Access
# Frontend + Digital Twin: http://localhost:3000
# API:                     http://localhost:8001
# WebSocket:               ws://localhost:8001/ws/sensors
```

The simulation mode auto-starts the AI decision loop in the browser. No hardware, ROS2, or Gazebo required.

## Docker Services

| Service | Port | Purpose |
|---------|------|---------|
| frontend | 3000 | TypeScript UI (Vite build, nginx) + Digital Twin (Three.js) |
| backend | 8001 | Flask REST API + RRT* path planning + AI engine |

Optional (with `--profile full`): Postgres (5432), Redis (6379) — for automation persistence and sensor caching.

## Digital Twin

The frontend includes a Three.js-based 3D simulation environment:

- **Robot model**: Hexagonal chassis, 3 omni wheels, gripper arm, sensor positions
- **Sensor visualization**: IR cones, ultrasonic arcs, line sensor spots
- **Environment**: Grid floor, obstacles, walls, targets
- **Path overlay**: RRT* tree expansion, planned path, waypoints
- **Physics**: Realistic sensor mock (IR raycasting, ultrasonic specular reflection, IMU gyro drift)

Switch to the 3D view via the Digital Twin tab in the dashboard.

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

### Path Planning

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/path/status` | RRT* planner status |
| POST | `/api/path/plan` | Plan path to target (x, y) |
| POST | `/api/path/start` | Start path execution |
| POST | `/api/path/stop` | Stop path execution |

### Sequences

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/robot/sequences/execute` | Execute sequence |
| POST | `/api/robot/sequences/save` | Save sequence |
| GET | `/api/robot/sequences/load/<name>` | Load sequence |
| GET | `/api/robot/sequences/list` | List sequences |

### Automations (requires Postgres)

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

### Navigation Server

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/navigate/status` | Navigation status |
| POST | `/api/navigate/to` | Navigate to coordinate |
| POST | `/api/navigate/cancel` | Cancel navigation |
| POST | `/api/navigate/reset` | Reset position |

## Project Structure

```
Autonomous_Mobile_Manipulator/
├── docker-compose.yml
├── README.md
├── TODO.md                    # Development roadmap
├── for_the_mega/              # Arduino Mega firmware
│   ├── omni_motor_control/    # Main firmware (2289 lines)
│   ├── COMMANDS.md            # Serial command reference
│   ├── PINOUT.md              # Hardware pin assignments
│   └── PID_CONTROL.md         # PID tuning guide
├── frontend/                  # TypeScript + Vite + Three.js
│   ├── src/
│   │   ├── api.ts             # API client
│   │   ├── types.ts           # TypeScript interfaces
│   │   ├── main.ts            # Tab routing + init
│   │   └── components/
│   │       ├── dashboard.ts   # Sensor values + KPI
│   │       ├── movement.ts    # 8-directional controls
│   │       ├── manipulator.ts # Arm + gripper
│   │       ├── map-view.ts    # Canvas navigation map
│   │       ├── automations.ts # Automation CRUD
│   │       ├── ai-control.ts  # AI + camera + waypoints
│   │       └── system.ts      # System status
│   └── src/engine/            # Digital Twin engine
│       ├── scenario.ts        # Simulation scenarios
│       ├── mock-sensors.ts    # Sensor physics emulation
│       └── three-scene.ts     # Three.js 3D world
├── ros2_ws/src/
│   └── my_robot_automation/
│       ├── scripts/
│       │   ├── main.py         # Entry point + FSM control loop
│       │   ├── app.py          # Flask API (1680 lines)
│       │   ├── config.py       # Configuration
│       │   ├── fsm.py          # Priority-based FSM (9 states)
│       │   ├── line_follower.py  # PID line tracking
│       │   ├── task_sequencer.py # Multi-step mission chaining
│       │   ├── mega_interface.py  # Serial comm
│       │   ├── sensor_manager.py  # Sensor data
│       │   ├── path_planning.py   # RRT* + KD-tree
│       │   ├── automation_engine.py  # IFTTT rules
│       │   ├── automation_api.py # Automation API
│       │   ├── ai_decision.py  # AI decision engine
│       │   ├── camera_service.py  # USB camera
│       │   ├── gpio_controller.py # GPIO
│       │   ├── waypoint_memory.py # Waypoints
│       │   └── ml/              # MLP models
│       └── requirements.txt
```

## Finite State Machine (FSM)

The robot behavior is governed by a priority-based state machine (`fsm.py`):

```
Priority  State
─────────────────────
 100      ESTOP          (highest)
  80      LINE_FOLLOW
  70      TASK_SEQ
  60      IFTTT
  50      AI_VISION
  40      WAYPOINT
  30      MANUAL
  20      CALIBRATE
  10      IDLE            (lowest)
```

A state can only preempt a lower-priority state. The FSM auto-detects line presence via the 3 line sensors and transitions to `LINE_FOLLOW` when confidence ≥ 2/3. Transition hooks activate/deactivate subsystems (e.g., ESTOP stops all motors, disengages line follower, halts task sequencer).

## Line Follower (`line_follower.py`)

PID-based line tracking using the 3 bottom IR sensors:

- **Error input**: `(line_left, line_center, line_right)` → float error value
- **PID output**: Maps to movement commands — `f` (straight), `q`/`e` (gentle curve), `t`/`y` (hard turn)
- **Auto-engage**: FSM switches to `LINE_FOLLOW` when line confidence ≥ threshold
- **Auto-disengage**: Falls back to previous mode when line is lost

## Task Sequencer (`task_sequencer.py`)

Chains multi-step missions programmed via the API:

- **Data model**: `TaskSequence` → `TaskStep[]` with `ActionType` + `StepCondition`
- **Conditions**: `wait-for-sensor` (threshold comparison), `wait-for-time`, `wait-for-position`, `always`
- **Actions**: move, turn, gripper (open/close/half), tilt, lifter, wait, goto-waypoint, follow-line, rotate, call-sub
- **Retry**: Configurable retry count per step; executor continues on failure
- **Queue**: Multiple sequences enqueued and executed sequentially

## Path Planning (RRT*)

The robot uses RRT* (Rapidly-exploring Random Tree star) for navigation:

- **KD-tree spatial index** — O(log n) nearest neighbor queries, rebuilt every 50 iterations
- **Dynamic obstacles** — Sensor obstacles age out after 5 cycles
- **Replan loop** — Obstacle detected → stop → replan → execute (1s rate limiter)
- **GridMap** — 10m × 10m occupancy grid (100×100 cells, 10cm resolution)
- **WaypointNavigator** — RRT* path → turn/move command sequence
- **APF virtual bumper** — Last-resort safety on Mega (direct repulsion at +PI)

All path planning runs on the host — Mega only executes movement commands.

## AI Decision Engine

Three operating modes (managed by FSM):

1. **REPLAY** — Saved waypoints via dead reckoning (camera OFF)
2. **IFTTT** — Sensor-triggered automations (camera OFF)
3. **AI_VISION** — Camera + MLP / YOLO reasoning (camera ON)

The engine auto-selects the best mode based on the task goal. All ML runs offline (no cloud API dependency).

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
```

## Deployment

```bash
# Build and run — simulation mode (no hardware)
docker compose up -d --build

# With database + redis for full features
docker compose --profile full up -d

# Check backend health
curl http://localhost:8001/health

# View logs
docker compose logs -f backend

# Stop
docker compose down
```
