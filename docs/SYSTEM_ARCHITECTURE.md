# System Architecture - Control & Monitoring

**Last Updated:** 2025-11-10  
**Status:** Production Ready

## Overview

The system provides two complementary control interfaces:

1. **Web UI** - Primary control and monitoring interface (Required)
2. **N8N** - Automation and workflow engine (Optional but Recommended)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        OPERATOR                             │
└───────────────┬────────────────────────┬────────────────────┘
                │                        │
                │                        │
        ┌───────▼────────┐      ┌───────▼────────┐
        │   Web UI       │      │   N8N          │
        │   (Required)   │      │   (Optional)   │
        │   Port 8080    │      │   Port 5678    │
        └───────┬────────┘      └───────┬────────┘
                │                        │
                └────────┬───────────────┘
                         │
                 ┌───────▼────────┐
                 │  REST API      │
                 │  Port 5000     │
                 └───────┬────────┘
                         │
                 ┌───────▼────────┐
                 │  ROS2 System   │
                 │  (Robot Core)  │
                 └────────────────┘
```

## Web UI - Primary Interface

### Purpose
**Complete robot control and monitoring** from a single, user-friendly interface.

### Core Functions (Required for Operation)

#### 1. Direct Robot Control
- **Movement:** Forward, backward, strafe, rotate
- **Speed Control:** Adjustable velocity (0.1-2.0 m/s)
- **Gripper Control:** Open, close, lift, tilt
- **Container Control:** Individual container management
- **Emergency Stop:** Immediate halt button

#### 2. Path Planning
- **Waypoint Management:** Add, remove, visualize waypoints
- **Pattern Templates:** Pre-built paths (square, triangle, hexagon, line)
- **Path Execution:** Start, stop, pause patrol routes
- **Navigation Shortcuts:** Go-to-point, return home
- **Path Storage:** Save/load frequently used routes

#### 3. Real-time Monitoring
- **Sensor Data:**
  - 6x Laser distance sensors
  - 2x Ultrasonic sensors
  - TF-Luna LIDAR
  - 3x Line sensors
  - IMU (orientation, velocity, acceleration)
  - 4x Container load sensors
  
- **System Status:**
  - Robot mode and position
  - Velocity and heading
  - Safety status
  - Battery level
  - System health

- **Logs & History:**
  - Last 3 commands executed
  - System log entries
  - Real-time activity feed

#### 4. Sensor Calibration
- **IMU Calibration:** Set zero reference for orientation
- **Status Feedback:** Visual confirmation of calibration

#### 5. Hardware Information
- **GPIO Pinout:** Complete Raspberry Pi 5 pin mapping
- **Power Distribution:** Voltage rails and current ratings
- **Safety Notes:** Hardware precautions

### When to Use Web UI

**✅ Always Use For:**
- Manual robot operation
- Testing and debugging
- Real-time monitoring
- Quick navigation tasks
- Emergency interventions
- System diagnostics
- Sensor calibration
- Learning and training

**✅ Best For:**
- Direct human control
- Immediate feedback required
- One-off operations
- Troubleshooting
- Live demonstrations
- Development and testing

### Web UI Advantages

1. **Visual Feedback:** See everything in real-time
2. **Immediate Control:** No workflow setup needed
3. **User Friendly:** Intuitive interface
4. **Complete Access:** All robot functions in one place
5. **No Dependencies:** Works independently
6. **Low Latency:** Direct API communication
7. **Mobile Friendly:** Responsive design

---

## N8N - Automation Engine

### Purpose
**Automated workflows and complex decision-making** for repetitive or scheduled tasks.

### Core Functions (Optional Enhancement)

#### 1. Automated Workflows
- **Scheduled Patrols:** Time-based route execution
- **Sensor-Triggered Actions:** Respond to sensor thresholds
- **Multi-Step Operations:** Complex task sequences
- **Conditional Logic:** If-then decision trees

#### 2. Integration & Orchestration
- **External Systems:** Connect to databases, APIs, services
- **Data Processing:** Transform and analyze sensor data
- **Notifications:** Send alerts via email, webhook, etc.
- **Logging:** Store execution history

#### 3. Advanced Automation
- **Event-Driven Responses:** React to specific conditions
- **Parallel Operations:** Multiple simultaneous tasks
- **Error Handling:** Retry logic and fallbacks
- **State Machines:** Complex behavior management

### When to Use N8N

**✅ Use When:**
- Running repetitive tasks
- Scheduling operations
- Implementing complex logic
- Integrating with external systems
- Automating routine procedures
- Monitoring sensor patterns
- Building autonomous behaviors

**✅ Best For:**
- Unattended operation
- Long-running tasks
- Pattern detection
- Data collection
- System integration
- Advanced automation

### N8N Advantages

1. **Automation:** Hands-free operation
2. **Scheduling:** Time-based execution
3. **Logic:** Complex decision trees
4. **Integration:** Connect to external services
5. **Scalability:** Handle multiple workflows
6. **Persistence:** Workflow history and logs

---

## Decision Matrix

### Use Web UI When:
| Scenario | Web UI | N8N |
|----------|--------|-----|
| Manual control needed | ✅ Required | ❌ Not needed |
| Quick navigation task | ✅ Ideal | ⚠️ Overkill |
| Testing new feature | ✅ Ideal | ❌ Not needed |
| Live monitoring | ✅ Required | ❌ Not needed |
| Emergency response | ✅ Required | ❌ Not needed |
| Training operators | ✅ Ideal | ❌ Not needed |

### Use N8N When:
| Scenario | Web UI | N8N |
|----------|--------|-----|
| Scheduled patrol | ⚠️ Manual | ✅ Automated |
| Sensor threshold alerts | ⚠️ Manual watch | ✅ Automated |
| Multi-step sequence | ⚠️ Repetitive | ✅ Automated |
| Data logging | ❌ Limited | ✅ Comprehensive |
| External integration | ❌ Not available | ✅ Built-in |
| Unattended operation | ❌ Not safe | ✅ Designed for it |

### Use Both When:
| Scenario | Web UI Role | N8N Role |
|----------|-------------|----------|
| Warehouse automation | Monitor & intervene | Execute patrol routes |
| Research experiment | Data review | Data collection |
| Production line | Oversight | Repetitive tasks |
| Security patrol | Live monitoring | Scheduled rounds |

---

## Recommended Configurations

### Configuration 1: Development/Testing
**Components:** Web UI Only

**Use Case:**
- Testing new features
- Development work
- Learning the system
- Manual operation

**Setup:**
```bash
docker compose up ros2-sim
# Access: http://localhost:8080
```

**Advantages:**
- Simplest setup
- Fast iteration
- Direct control
- No workflow overhead

---

### Configuration 2: Production Manual
**Components:** Web UI + ROS2 + Hardware

**Use Case:**
- Manual warehouse operation
- Demonstration robots
- Educational robots
- Research platforms

**Setup:**
```bash
# On Raspberry Pi 5
./start_robot.sh
# Access: http://[robot-ip]:8080
```

**Advantages:**
- Full manual control
- Real-time monitoring
- Simple maintenance
- Easy troubleshooting

---

### Configuration 3: Production Automated (Recommended)
**Components:** Web UI + N8N + ROS2 + Hardware

**Use Case:**
- Autonomous warehouse robots
- Production lines
- Security patrols
- Data collection

**Setup:**
```bash
docker compose up -d
# Web UI: http://localhost:8080
# N8N: http://localhost:5678
```

**Advantages:**
- Full automation capabilities
- Manual override available
- Comprehensive monitoring
- Maximum flexibility

---

## System Dependencies

### Required Components
1. **ROS2 System** - Robot operating system
2. **REST API Server** - HTTP interface (Port 5000)
3. **Web UI** - Control interface (Port 8080)

### Optional Components
1. **N8N** - Workflow automation (Port 5678)
2. **Database** - Historical data storage
3. **External Services** - Integration endpoints

---

## API Architecture

### REST API (Port 5000)
**Role:** Central communication hub

**Endpoints:**
- Movement control
- Sensor data retrieval
- System status
- Patrol execution
- Emergency stop
- IMU calibration
- Logs and history

**Clients:**
- Web UI (primary)
- N8N workflows
- Custom applications
- Command-line tools

---

## Communication Flow

### Manual Control Flow
```
Operator → Web UI → REST API → ROS2 → Hardware → Sensors
                                        ↓
                                    Actuators
```

### Automated Control Flow
```
N8N Workflow → REST API → ROS2 → Hardware → Sensors
                            ↓
Web UI (monitoring) ← REST API ← ROS2
```

### Monitoring Flow
```
Hardware → Sensors → ROS2 → REST API → Web UI → Operator
                              ↓
                          N8N (optional)
```

---

## Operational Modes

### Mode 1: Direct Control
**Active:** Web UI  
**Inactive:** N8N

**Use:** Manual operation, testing, training

**Operator Action:** Direct button clicks and input

---

### Mode 2: Supervised Automation
**Active:** Web UI + N8N  
**Role:** N8N executes, Web UI monitors

**Use:** Automated tasks with human oversight

**Operator Action:** Monitor and intervene if needed

---

### Mode 3: Autonomous Operation
**Active:** N8N  
**Monitoring:** Web UI (optional)

**Use:** Fully automated operation

**Operator Action:** Periodic checks, emergency response

---

## Failsafe Design

### Emergency Stop Priority
1. **Hardware Button** - Highest priority (physical)
2. **Web UI Button** - Direct API call
3. **N8N Workflow** - Can trigger via API
4. **ROS2 Watchdog** - Automatic safety

### Intervention Hierarchy
```
Emergency Stop → Web UI Control → N8N Automation → Autonomous Mode
(Highest Priority)                                  (Lowest Priority)
```

---

## Deployment Recommendations

### Small Scale / Development
**Setup:** Web UI only  
**N8N:** Not needed  
**Reasoning:** Direct control sufficient

### Medium Scale / Production
**Setup:** Web UI + N8N  
**N8N:** Recommended  
**Reasoning:** Automation benefits outweigh setup cost

### Large Scale / Fleet
**Setup:** Web UI + N8N + External Services  
**N8N:** Required  
**Reasoning:** Cannot manually control multiple robots

---

## Migration Path

### Phase 1: Learn with Web UI
Start with manual control to understand robot behavior.

### Phase 2: Create N8N Workflows
Build automation for repetitive tasks.

### Phase 3: Supervised Automation
Run N8N workflows while monitoring via Web UI.

### Phase 4: Full Automation
Trust workflows for unattended operation.

### Phase 5: Scale Up
Deploy to multiple robots with centralized N8N.

---

## Summary

### Web UI (Required)
✅ **Primary interface** for control and monitoring  
✅ **Complete functionality** without N8N  
✅ **Always accessible** for human operators  
✅ **Emergency control** and safety overrides

### N8N (Optional but Recommended)
⚙️ **Enhances capabilities** with automation  
⚙️ **Not required** for basic operation  
⚙️ **Recommended** for production environments  
⚙️ **Essential** for autonomous operation

---

## Conclusion

The system is designed with **flexibility** as a core principle:

- **Beginners:** Use Web UI exclusively
- **Intermediate:** Add N8N for common tasks
- **Advanced:** Full automation with manual override
- **Production:** Both systems working together

**The Web UI provides complete control. N8N adds automation. Together, they create a powerful, flexible robotics platform.**

