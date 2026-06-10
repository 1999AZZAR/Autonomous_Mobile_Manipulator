# TODO: IFTTT Automation Engine

## Goal
Add a local IFTTT-style automation engine to the robot. Users define IF/THEN rules with sensor conditions and robot actions, evaluated in real-time. PostgreSQL + Prisma ORM. API + web UI.

## Architecture
```
Flask App (:8001) ←→ PostgreSQL (:5432) via Prisma
       ↕                    ↕
  Arduino Mega          Automation Engine
  (serial)              (sensor cache, evaluator, executor, scheduler)
```

## Tasks

### Phase 1: Infrastructure
- [x] Add PostgreSQL container to `docker-compose.yml`
- [x] Create `ros2_ws/prisma/schema.prisma` (Automation, AutomationCondition, AutomationAction, AutomationLog)
- [x] Update `Dockerfile` — install prisma CLI, `prisma generate`, `prisma db push`
- [x] Update `requirements.txt` — add `prisma`, `psycopg2-binary`
- [x] Update `config.py` — add DATABASE_URL, automation settings

### Phase 2: Core Engine
- [x] Create `scripts/automation_engine.py`
  - Prisma client initialization
  - Sensor value cache (read at 10Hz from sensor_manager)
  - Condition evaluator (>, <, ==, !=, >=, <= with ALL/ANY matching)
  - Action executor (move, turn, gripper, stop, delay, webhook, trigger chain)
  - Time-based scheduler (cron-like for scheduled triggers)
  - Rate limiting (max 10 executions per rule per 5s)
  - Circular trigger protection (max depth 5)
  - Else-branch support

### Phase 3: API
- [x] Create `scripts/automation_api.py` — Flask blueprint
  - `GET    /api/automations` — list all
  - `POST   /api/automations` — create
  - `GET    /api/automations/<id>` — get one
  - `PUT    /api/automations/<id>` — update
  - `DELETE /api/automations/<id>` — delete
  - `POST   /api/automations/<id>/toggle` — enable/disable
  - `POST   /api/automations/<id>/run` — manual trigger
  - `GET    /api/automations/<id>/logs` — execution history
  - `GET    /api/feeds` — available sensor feeds
  - `GET    /api/feeds/<key>/value` — current sensor value

### Phase 4: Web UI
- [x] Add automation page to Flask web interface
  - Automation list with enable/disable toggles
  - Rule builder form (conditions + actions)
  - Manual trigger buttons
  - Execution log viewer

### Phase 5: Integration
- [x] Update `main.py` — init AutomationEngine, start scheduler, run `prisma db push`
- [x] Update `app.py` — mount automation API blueprint, add nav link

### Phase 6: Test
- [ ] Build Docker image, verify PostgreSQL connection
- [ ] Test CRUD API endpoints
- [ ] Test automation rule evaluation with simulated sensor data
- [ ] Test action execution (move, stop, gripper commands via mega_interface)
- [ ] Test time-based triggers
- [ ] Test manual trigger via API

## Data Model

### Feed Keys (Sensor Inputs)
| Feed Key | Source | Unit |
|----------|--------|------|
| `laser_left_front` | sensor_manager | cm |
| `laser_left_back` | sensor_manager | cm |
| `laser_right_front` | sensor_manager | cm |
| `laser_right_back` | sensor_manager | cm |
| `laser_back_left` | sensor_manager | cm |
| `laser_back_right` | sensor_manager | cm |
| `ultra_front_left` | sensor_manager | cm |
| `ultra_front_right` | sensor_manager | cm |
| `line_left` | sensor_manager | bool |
| `line_center` | sensor_manager | bool |
| `line_right` | sensor_manager | bool |
| `imu_heading` | sensor_manager | deg |
| `imu_pitch` | sensor_manager | deg |
| `imu_roll` | sensor_manager | deg |
| `tf_luna_distance` | sensor_manager | cm |
| `mega_connected` | mega_interface | bool |

### Action Types (Robot Commands)
| Action | Payload | Description |
|--------|---------|-------------|
| `move` | forward/backward/left/right | Move robot |
| `stop` | (none) | Emergency stop |
| `turn` | left/right/degrees | Turn robot |
| `gripper` | open/close | Control gripper |
| `gripper_tilt` | 0-180 | Set gripper tilt |
| `speed` | 0-100 | Set motor speed |
| `patrol` | start/stop | Toggle patrol mode |
| `waypoint` | name | Navigate to waypoint |
| `delay` | ms | Delay before next action |
| `webhook` | URL | POST to external URL |
| `trigger` | automation_id | Chain another automation |

## Files
| File | Action |
|------|--------|
| `docker-compose.yml` | Modify — add postgres service |
| `ros2_ws/prisma/schema.prisma` | Create |
| `scripts/automation_engine.py` | Create |
| `scripts/automation_api.py` | Create |
| `scripts/app.py` | Modify — mount blueprint, nav link |
| `scripts/main.py` | Modify — init engine |
| `scripts/config.py` | Modify — DB config |
| `requirements.txt` | Modify — add deps |
| `Dockerfile` | Modify — prisma setup |
