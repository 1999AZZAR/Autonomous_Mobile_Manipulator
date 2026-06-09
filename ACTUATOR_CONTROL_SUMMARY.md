# Actuator Control System Summary

## Architecture

All actuator control routes through the Arduino Mega via serial communication (115200 baud). The Raspberry Pi sends single-character commands over UART, and the Mega executes real-time motor/servo control via the YFROBOT shield.

**Control flow:**

```
Web UI / REST API  -->  mega_interface.py  -->  Serial (115200 baud)  -->  Arduino Mega  -->  Motors/Servos
```

## Gripper System (4 Actuators)

| Actuator | Command Interface | Range |
|----------|-------------------|-------|
| Gripper Open/Close | `mega_interface.control_gripper(command)` | "open" / "close" |
| Gripper Tilt | `mega_interface.set_gripper_tilt(angle)` | 0-180 degrees |
| Gripper Neck (360 deg) | `mega_interface.set_gripper_neck(position)` | -1.0 to 1.0 |
| Gripper Base Height | `gpio_controller.set_gripper_base(height)` | -1.0 to 1.0 |

**API Endpoints:**

```bash
# Open/close gripper
POST /api/robot/picker/gripper
{"command": "open"}

# Set tilt angle (0-180)
POST /api/robot/picker/gripper_tilt
{"angle": 90}

# Set neck position (-1 to 1)
POST /api/robot/picker/gripper_neck
{"position": 0.5}

# Set base height (-1 to 1)
POST /api/robot/picker/gripper_base
{"height": 0.8}
```

## Omni Wheel Motor System (3 Drive Motors + 1 Lifter)

| Motor | Mega Command | Direction |
|-------|-------------|-----------|
| Forward | `f` | Front movement |
| Backward | `b` | Rear movement |
| Strafe Left | `l` | Lateral left |
| Strafe Right | `r` | Lateral right |
| Rotate CW | `c` | Clockwise spin |
| Rotate CCW | `w` | Counter-clockwise spin |
| Turn Left | `t` | Forward + left rotation |
| Turn Right | `y` | Forward + right rotation |
| Arc Left | `a` | Gentle left curve |
| Arc Right | `j` | Gentle right curve |

**Speed Control:**

| Command | Speed |
|---------|-------|
| `5` | 50% |
| `6` | 60% |
| `7` | 70% |
| `8` | 80% |
| `9` | 90% |
| `0` | 100% |

**Lifter Control:**

| Command | Action |
|---------|--------|
| `u` | Lift up |
| `d` | Lift down |
| `s` | Emergency stop (all motors) |

**API Endpoints:**

```bash
POST /api/robot/move
{"direction": "forward", "speed": 0.5}

POST /api/robot/turn
{"direction": "left", "speed": 0.5}

POST /api/robot/stop
{}
```

## Container System (4 Compartments)

| Container | API ID |
|-----------|--------|
| Left Front | `left_front` |
| Left Back | `left_back` |
| Right Front | `right_front` |
| Right Back | `right_back` |

**API Endpoints:**

```bash
POST /api/robot/containers/<container_id>
{"action": "load"}
```

## Testing

```bash
# Motor commands via serial
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "f"}'
curl -X POST http://localhost:8000/api/serial/send -d '{"command": "s"}'

# Gripper control
curl -X POST http://localhost:8000/api/robot/picker/gripper -d '{"command": "open"}'

# Status check
curl http://localhost:8000/api/mega/status
```

## System Status

```json
{
  "mega_connected": true,
  "simulation_mode": false,
  "system_status": "operational",
  "actuators": {
    "gripper": "routed via Mega serial",
    "motors": "3 omni + 1 lifter via Mega PID",
    "containers": "4 compartments via API"
  }
}
```

## Notes

- Motor control uses PID velocity control on the Arduino Mega with 10ms sample time.
- Gripper base control is currently not implemented on the Mega firmware — routed through `gpio_controller.py` which logs a warning.
- Servo homing is not yet implemented on the Mega side.
- All commands fall back to simulation mode if the Mega serial connection is unavailable.
