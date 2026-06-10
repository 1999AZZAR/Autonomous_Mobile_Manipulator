# Software Architecture

The system separates concerns across three layers: a TypeScript frontend, a Python REST API backend, and ROS2 packages for robot description and bringup. The backend also hosts an automation engine (IFTTT-style rules) and an AI decision engine (camera + YOLOv8 + API reasoning).

## Stack

| Layer | Technology | Port |
|-------|-----------|------|
| Frontend | TypeScript + Vite, Carbon dark theme | 3000 (nginx) |
| Backend | Python Flask + Flask-SocketIO | 8001 |
| Database | PostgreSQL 16 + Prisma Python client | 5432 (internal) |
| Cache | Redis 7 | 6379 (internal) |
| Robot control | Arduino Mega (serial, 115200 baud) | USB |
| Robot description | ROS2 Jazzy (URDF, bringup) | — |

## Project Structure

```
ros2_ws/src/my_robot_automation/
├── scripts/
│   ├── main.py                 # Entry point: inits all services, registers blueprints
│   ├── app.py                  # Flask REST API + WebSocket
│   ├── automation_engine.py    # Redis cache, rate limiting, async helper
│   ├── automation_api.py       # Blueprint routes: automation, AI, waypoints
│   ├── sensor_manager.py       # Sensor data collection from Mega
│   ├── mega_interface.py       # Serial communication with Arduino Mega
│   ├── camera_service.py       # Power-aware USB camera (OFF by default)
│   ├── waypoint_memory.py      # Save/load/replay waypoint paths
│   ├── ai_decision.py          # AI decision engine (REPLAY/IFTTT/AI modes)
│   └── config.py               # All configuration constants
├── prisma/
│   └── schema.prisma           # Database schema (7 models)
└── requirements.txt            # Python dependencies

frontend/
├── src/
│   ├── main.ts                 # Tab routing, emergency stop
│   ├── api.ts                  # REST + WebSocket client
│   ├── types.ts                # TypeScript interfaces
│   ├── components/
│   │   ├── dashboard.ts        # Overview panel
│   │   ├── movement.ts         # 8-direction movement controls
│   │   ├── manipulator.ts      # Gripper, tilt, lifter
│   │   ├── map-view.ts         # 2D navigation map (canvas)
│   │   ├── digital-twin.ts     # 3D scene (Three.js)
│   │   ├── ai-control.ts       # AI decision engine panel
│   │   ├── automations.ts      # Automation rule builder
│   │   └── system.ts           # System status, logs
│   ├── engine/                 # Three.js digital twin
│   │   ├── scene.ts            # Renderer, camera, lighting
│   │   ├── robot-model.ts      # Hexagonal chassis, sensors, gripper
│   │   ├── physics.ts          # Kinematic simulation
│   │   ├── mock-sensors.ts     # Raycast-based mock sensor data
│   │   ├── sensor-viz.ts       # Sensor arc visualization
│   │   ├── waypoint-viz.ts     # Path rendering
│   │   ├── scenario.ts         # Simulation presets
│   │   ├── recording.ts        # Session recording
│   │   └── playback.ts         # Playback engine
│   └── state/
│       └── twin-state.ts       # Shared 3D state store
└── index.html                  # Sidebar navigation, tab panels
```

## Backend Routes

### Robot Control (`app.py`)

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/robot/sensors` | All sensor data (flat keys) |
| GET | `/api/robot/sensors/diagnostics` | Sensor diagnostics |
| POST | `/api/robot/move` | Directional movement |
| POST | `/api/robot/turn` | Rotation |
| POST | `/api/robot/stop` | Stop all motors |
| POST | `/api/robot/emergency-stop` | Emergency stop |
| POST | `/api/robot/speed` | Set speed multiplier |
| POST | `/api/robot/turbo` | Toggle turbo mode |
| POST | `/api/robot/wheels/<id>` | Individual wheel control |
| POST | `/api/robot/picker/gripper` | Open/close gripper |
| POST | `/api/robot/picker/gripper_tilt` | Set tilt angle |
| POST | `/api/robot/lifter/up` | Lift up |
| POST | `/api/robot/lifter/down` | Lift down |
| POST | `/api/robot/camera/tilt` | Camera tilt |
| POST | `/api/robot/safety/perimeter` | Enable/disable virtual bumper |
| GET | `/api/robot/safety/limit-switches` | Limit switch status |
| POST | `/api/robot/sensor-publishing` | Enable/disable sensor data stream |
| POST | `/api/robot/sequences/execute` | Execute saved sequence |
| POST | `/api/robot/sequences/save` | Save movement sequence |
| GET | `/api/robot/sequences/list` | List saved sequences |
| GET | `/api/robot/position` | Robot position + sensor readings |
| GET | `/api/feeds` | Sensor feed data |
| WS | `/ws/sensors` | Real-time sensor WebSocket |

### Automation Rules (`automation_api.py` — automation_bp)

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/automations` | List all rules |
| POST | `/api/automations` | Create rule |
| GET | `/api/automations/<id>` | Get rule |
| PUT | `/api/automations/<id>` | Update rule |
| DELETE | `/api/automations/<id>` | Delete rule |
| POST | `/api/automations/<id>/toggle` | Enable/disable |
| POST | `/api/automations/<id>/run` | Execute rule now |
| GET | `/api/automations/<id>/logs` | Rule execution logs |

### AI Decision Engine (`automation_api.py` — ai_bp)

| Method | Endpoint | Purpose |
|--------|----------|---------|
| POST | `/api/ai/start` | Start AI mode |
| POST | `/api/ai/stop` | Stop AI mode |
| GET | `/api/ai/status` | Current AI status |
| POST | `/api/ai/analyze` | Trigger analysis |
| GET | `/api/ai/decisions` | Decision history |
| POST | `/api/ai/guidance` | Inject human guidance |
| GET | `/api/ai/camera/snapshot` | Camera JPEG snapshot |

### Waypoint Memory (`automation_api.py` — waypoint_bp)

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/waypoints/paths` | List saved paths |
| POST | `/api/waypoints/paths` | Save path |
| GET | `/api/waypoints/paths/<id>` | Get path details |
| DELETE | `/api/waypoints/paths/<id>` | Delete path |
| POST | `/api/waypoints/record` | Start recording |
| POST | `/api/waypoints/stop` | Stop recording |
| POST | `/api/waypoints/replay/<id>` | Replay path |
| POST | `/api/waypoints/replay/stop` | Stop replay |
| GET | `/api/waypoints/status` | Recording/replay status |

## Data Flow

```
┌──────────┐  HTTP/WS  ┌──────────┐  Serial  ┌──────────┐
│ Frontend │──────────►│ Backend  │─────────►│ Arduino  │
│ (TS)     │◄──────────│ (Flask)  │◄─────────│ Mega     │
└──────────┘           └────┬─────┘          └──────────┘
                            │
                     ┌──────┴──────┐
                     │             │
                ┌────┴───┐  ┌─────┴────┐
                │ Postgres│  │  Redis   │
                │ (Prisma)│  │ (cache)  │
                └────────┘  └──────────┘
```

Sensor data flows: Mega serial → `mega_interface.py` → `sensor_manager.py` → Redis cache + WebSocket push to frontend. Backend also serves sensor data via REST (`/api/robot/sensors`).

## Three Navigation Modes

| Mode | Camera | Trigger | Fallback |
|------|--------|---------|----------|
| REPLAY | OFF | Saved waypoint match | — |
| IFTTT | OFF | Sensor rules | — |
| AI | ON | Camera + YOLOv8/API | Falls back to IFTTT |

The AI decision engine (`ai_decision.py`) selects the best mode. If the camera fails or AI crashes, it falls back to IFTTT sensor rules.

## Database Models (Prisma)

| Model | Purpose |
|-------|---------|
| Automation | IFTTT-style sensor rules |
| AutomationLog | Rule execution history |
| SensorReading | Cached sensor snapshots |
| RobotSession | Movement session tracking |
| SavedPath | Waypoint paths for replay |
| Waypoint | Individual waypoints within paths |
| AiDecision | AI decision history with reasoning |

Schema: `ros2_ws/src/my_robot_automation/prisma/schema.prisma`
