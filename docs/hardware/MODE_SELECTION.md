# Robot Operation Modes

## Overview

The robot system can run in different modes depending on your hardware and use case.

## Mode Comparison Table

| Mode | Hardware Required | Sensors | Use Case |
|------|------------------|---------|----------|
| **Hardware Mode** | Yes | Real sensors (MPU6050, IR, etc.) | Production, real robot operation |
| **Simulation Mode** | No | Simulated data | Development, testing, no hardware |
| **Auto-Detect Mode** | Optional | Use hardware if available, simulate if not | Default, flexible |

## Starting in Different Modes

### Hardware Mode (Real Sensors)

**When to use:**
- Running on actual Raspberry Pi with sensors connected
- Production environment
- Testing with real hardware
- IMU, IR sensors, servos connected

**Requirements:**
- MPU6050 IMU connected (I2C 0x68)
- I2C permissions (`i2c` group membership)
- Sharp IR sensors via MCP3008 (SPI)
- Physical hardware connected

**How to start:**

```bash
# Simple way
./start_hardware.sh

# Manual way
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py --hardware
```

**What happens:**
- Forces hardware mode
- Will ERROR if sensors not available
- IMU data from real MPU6050
- IR distance readings from real sensors
- No fallback to simulation

**Expected output:**
```
Starting in HARDWARE mode (forced)
MPU6050 initialized successfully at address 0x68
SPI interface initialized successfully
```

### Simulation Mode (No Hardware)

**When to use:**
- Development without physical robot
- Testing interface on laptop/desktop
- CI/CD pipelines
- No sensors available

**Requirements:**
- None - works anywhere
- No GPIO/I2C/SPI needed

**How to start:**

```bash
# Simple way
./start_simulation.sh

# Manual way
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py --simulation
```

**What happens:**
- Forces simulation mode
- All sensor data is simulated
- Smooth sine wave patterns for IMU
- Realistic distance sensor simulation
- No hardware access attempted

**Expected output:**
```
Starting in SIMULATION mode (forced)
Running in SIMULATION mode (no hardware access)
```

### Auto-Detect Mode (Default)

**When to use:**
- Unsure of hardware availability
- Want automatic fallback
- Testing on different machines

**How to start:**

```bash
# Just run without flags
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

**What happens:**
- Checks if SPI hardware available
- Tries to initialize MPU6050
- If hardware found → Uses it
- If hardware not found → Simulates it

**Expected output (with hardware):**
```
Starting in AUTO-DETECT mode
MPU6050 initialized successfully at address 0x68
```

**Expected output (without hardware):**
```
Starting in AUTO-DETECT mode
WARNING: spidev not available. Running in SIMULATION mode.
```

## Checking Current Mode

### Via Web UI
1. Open http://localhost:8000
2. Click "Status" tab
3. Look for "simulation_mode" field

### Via API
```bash
curl http://localhost:8000/api/robot/status | python3 -m json.tool
```

Look for:
```json
{
  "success": true,
  "data": {
    "simulation_mode": false,  // false = hardware, true = simulation
    "imu_initialized": true,
    "spi_initialized": true
  }
}
```

### Via Startup Logs
Look for these messages when starting:
- `Starting in HARDWARE mode (forced)` - Hardware mode
- `Starting in SIMULATION mode (forced)` - Simulation mode  
- `Starting in AUTO-DETECT mode` - Will auto-detect

## Troubleshooting Mode Issues

### Stuck in Simulation Mode (Want Hardware)

**Problem:** Web interface shows simulated data even with hardware connected

**Solutions:**

1. **Use explicit hardware mode:**
   ```bash
   ./start_hardware.sh
   ```

2. **Check I2C permissions:**
   ```bash
   python3 diagnose_imu.py
   ```

3. **Verify sensor connection:**
   ```bash
   i2cdetect -y 1  # Should show 68
   ```

4. **Check startup logs:**
   Look for "MPU6050 initialized successfully"

### Fails in Hardware Mode (No Sensors)

**Problem:** Error starting in hardware mode without sensors

**Solution:**

Use simulation mode instead:
```bash
./start_simulation.sh
```

Or let it auto-detect:
```bash
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

## Docker / Container Modes

### With Docker Compose

**Production Mode (Hardware):**
```bash
./run.sh start          # Uses docker-compose.yml
```
- Sets `ROBOT_MODE=hardware` environment variable
- Accesses real GPIO/I2C/SPI
- Requires privileged mode

**Development Mode (Simulation):**
```bash
./run.sh start --dev    # Uses docker-compose.dev.yml
```
- Sets `ROBOT_MODE=simulation` environment variable
- No hardware access needed
- Faster startup

### Without Docker (Direct on Pi)

Run the scripts directly:
```bash
./start_hardware.sh     # Hardware mode
./start_simulation.sh   # Simulation mode
```

## Mode Selection Flowchart

```
┌─────────────────────────┐
│ Do you have physical    │
│ robot with sensors?     │
└────────┬────────────────┘
         │
    ┌────┴────┐
    │   YES   │   NO
    │         │
    ▼         ▼
┌───────┐ ┌──────────┐
│ Use   │ │ Use      │
│ HARD- │ │ SIMULA-  │
│ WARE  │ │ TION     │
│ mode  │ │ mode     │
└───────┘ └──────────┘
    │         │
    ▼         ▼
./start_   ./start_
hardware   simulation
.sh        .sh
```

## Best Practices

1. **Development:** Use simulation mode for UI/logic development
2. **Testing:** Use auto-detect for flexible testing
3. **Production:** Use hardware mode with error checking
4. **CI/CD:** Always use simulation mode
5. **Demo:** Use simulation if no hardware available

## Summary

| Scenario | Command | Mode |
|----------|---------|------|
| Real robot with sensors | `./start_hardware.sh` | Hardware (forced) |
| No hardware available | `./start_simulation.sh` | Simulation (forced) |
| Flexible (has hardware) | `python3 web_robot_interface.py` | Auto-detect → Hardware |
| Flexible (no hardware) | `python3 web_robot_interface.py` | Auto-detect → Simulation |
| Docker production | `./run.sh start` | Hardware |
| Docker development | `./run.sh start --dev` | Simulation |

## Quick Decision

**Have real robot with connected sensors?**
- YES → `./start_hardware.sh`
- NO → `./start_simulation.sh`
- MAYBE → Let it auto-detect (default)

