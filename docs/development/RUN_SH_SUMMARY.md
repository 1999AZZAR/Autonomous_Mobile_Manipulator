# run.sh - Complete Management Script

## Overview

The `run.sh` script is a comprehensive management tool for the Autonomous Mobile Manipulator system. It handles all aspects of Docker container management, system monitoring, and troubleshooting.

## Features

### ✅ Container Management
- Start/stop/restart Docker containers
- Support for development and production modes
- Automatic service health checking
- Graceful shutdown handling

### ✅ System Monitoring
- Real-time container status
- Resource usage (CPU, memory)
- Service health checks (Web interface, n8n)
- Detailed logging with filtering

### ✅ Development Tools
- Interactive shell access to containers
- Live log streaming
- Docker resource management
- System cleanup utilities

### ✅ Prerequisites Checking
- Docker installation verification
- Docker Compose availability
- Docker daemon status
- User permissions (docker group)

### ✅ User-Friendly Interface
- Color-coded output
- Clear success/error messages
- Comprehensive help documentation
- Intuitive command structure

## Commands Reference

### Primary Commands

```bash
# Start the system
./run.sh start              # Production mode (real hardware)
./run.sh start --dev        # Development mode (simulated sensors)

# Stop the system
./run.sh stop

# Restart the system
./run.sh restart            # Production mode
./run.sh restart --dev      # Development mode
```

### Monitoring Commands

```bash
# Check system status
./run.sh status

# View logs
./run.sh logs                    # All containers
./run.sh logs ros2_sim_container # Specific container

# Enter container shell
./run.sh shell
```

### Maintenance Commands

```bash
# Check prerequisites
./run.sh check

# Update system
./run.sh update

# Clean system (removes containers and volumes)
./run.sh clean

# Show help
./run.sh help
```

## Command Details

### `./run.sh start [--dev]`

Starts the robot system with full initialization:

**Production Mode (default):**
- Launches ROS2 with real hardware support
- Initializes MPU6050 IMU sensor
- Connects to Sharp IR distance sensors via SPI
- Full robot control capabilities

**Development Mode (`--dev`):**
- Fast startup without hardware dependencies
- Simulated sensor data
- Perfect for testing and development
- No physical sensors required

**Process:**
1. Checks prerequisites
2. Selects appropriate Docker Compose file
3. Starts containers in detached mode
4. Waits for services to initialize
5. Verifies service health
6. Displays access information

**Output Example:**
```
=========================================================================
  Starting Autonomous Mobile Manipulator
=========================================================================
ℹ Mode: Production (with real hardware/Gazebo)
ℹ Starting Docker containers...
✓ Containers started
ℹ Waiting for services to initialize...
✓ ROS2 container is running
✓ Web interface is accessible

=========================================================================
  Access Information
=========================================================================
Robot System is Ready!

Web Interfaces:
  Primary Dashboard: http://localhost:8000
  Remote Access:     http://100.74.72.71:8000

n8n Automation:
  Local:  http://localhost:5678
  Remote: http://100.74.72.71:5678
```

### `./run.sh stop`

Gracefully stops all containers:
- Stops running containers
- Preserves data volumes
- Cleans up resources
- Quick restart capability

### `./run.sh restart [--dev]`

Restarts the system:
- Stops all containers
- Waits 2 seconds
- Starts containers with specified mode
- Useful after configuration changes

### `./run.sh status`

Displays comprehensive system status:

**Container Status:**
- Running/Stopped/Not Found status
- Uptime information
- Health indicators

**Resource Usage:**
- CPU percentage per container
- Memory usage and limits
- Real-time statistics

**Service Health:**
- Web Interface accessibility (port 8000)
- n8n Automation accessibility (port 5678)
- API endpoint status

**Output Example:**
```
Container Status:
✓ ROS2 Container: Running

All Containers:
NAMES                STATUS              PORTS
ros2_sim_container   Up 5 minutes        
n8n_container        Up 5 minutes        
node-red-updated     Up 5 minutes

Resource Usage:
NAME                 CPU %     MEM USAGE / LIMIT
ros2_sim_container   10.84%    883.2MiB / 7.751GiB
n8n_container        0.00%     201.6MiB / 7.751GiB

Services:
✓ Web Interface (port 8000): Accessible
✓ n8n Automation (port 5678): Accessible
```

### `./run.sh logs [service]`

Streams container logs:

**All Logs (default):**
```bash
./run.sh logs
```
Shows logs from all containers with timestamps

**Specific Service:**
```bash
./run.sh logs ros2_sim_container
```
Shows logs from specific container only

**Features:**
- Live streaming (updates in real-time)
- Last 50 lines shown initially
- Color-coded output
- Ctrl+C to exit

### `./run.sh shell`

Enters interactive shell in ROS2 container:
- Full bash access
- ROS2 environment sourced
- Access to all robot tools
- Useful for debugging

**Example Usage:**
```bash
./run.sh shell

# Inside container:
ros2 topic list
ros2 node list
python3 -c "from mpu6050_reader import MPU6050Reader; print('OK')"
exit
```

### `./run.sh check`

Checks all prerequisites:
- Docker installation and version
- Docker Compose installation
- Docker daemon status
- User permissions (docker group membership)
- Exit code 0 if all OK, 1 if issues found

**Output Example:**
```
=========================================================================
  Checking Prerequisites
=========================================================================
✓ Docker is installed (27.3.1)
✓ Docker Compose is installed
✓ User is in docker group
✓ Docker daemon is running
```

### `./run.sh clean`

Removes all containers and volumes:
- **WARNING:** Destructive operation
- Requires confirmation (type "yes")
- Stops all containers
- Removes volumes
- Prunes unused Docker resources

**Use Cases:**
- Fresh start after errors
- Free up disk space
- Reset to clean state

### `./run.sh update`

Updates the system:
1. Pulls latest Git changes (if in Git repo)
2. Rebuilds Docker images from scratch
3. Does NOT restart automatically (use `./run.sh restart` after)

**Use Cases:**
- After pulling code updates
- After modifying Dockerfile
- After dependency changes

## Color Coding

The script uses colors for clarity:
- 🟢 **Green (✓):** Success messages
- 🔴 **Red (✗):** Error messages
- 🟡 **Yellow (⚠):** Warning messages
- 🔵 **Blue (ℹ):** Informational messages

## Environment Variables

The script automatically sets:
- `COMPOSE_FILE`: Selected Docker Compose file
- `PROJECT_NAME`: autonomous_mobile_manipulator
- `CONTAINER_NAME`: ros2_sim_container

## File Structure

```
run.sh
├── Helper Functions
│   ├── print_header()
│   ├── print_success()
│   ├── print_error()
│   ├── print_warning()
│   └── print_info()
├── Prerequisites Check
│   └── check_prerequisites()
├── Container Management
│   ├── check_container_status()
│   ├── start_containers()
│   ├── stop_containers()
│   └── restart_containers()
├── Monitoring
│   ├── show_status()
│   ├── check_service_health()
│   └── show_logs()
├── Utilities
│   ├── exec_shell()
│   ├── display_access_info()
│   ├── clean_system()
│   └── update_system()
└── Main Script
    ├── Command parsing
    └── Error handling
```

## Error Handling

The script includes comprehensive error handling:
- Exit on error (`set -e`)
- Prerequisite validation before operations
- Service health verification
- Clear error messages with solutions
- Graceful failure modes

## Integration Points

### Docker Compose Files

- **Production:** `docker-compose.yml`
  - Full ROS2 stack
  - Hardware access (I2C, SPI)
  - Real sensors

- **Development:** `docker-compose.dev.yml`
  - Simulated sensors
  - Fast startup
  - No hardware dependencies

### Services Monitored

1. **Web Interface (Port 8000)**
   - Primary robot control dashboard
   - Real-time sensor data
   - Path planning UI
   - Status monitoring

2. **n8n Automation (Port 5678)**
   - Workflow automation
   - Integration with external services
   - Event-driven automation

3. **ROS2 Container**
   - Core robot services
   - Sensor processing
   - Control algorithms

## Troubleshooting

### "Docker daemon is not running"
```bash
sudo systemctl start docker
```

### "Permission denied" errors
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### Container won't start
```bash
./run.sh logs              # Check logs for errors
./run.sh clean             # Clean system
./run.sh start             # Try again
```

### Web interface not accessible
```bash
./run.sh status            # Check if services are running
./run.sh restart           # Restart everything
# Wait 30 seconds for full initialization
```

## Best Practices

1. **Always check status first:**
   ```bash
   ./run.sh status
   ```

2. **Use logs for debugging:**
   ```bash
   ./run.sh logs | grep -i error
   ```

3. **Clean restart for major issues:**
   ```bash
   ./run.sh stop
   ./run.sh clean  # Type "yes" when prompted
   ./run.sh start
   ```

4. **Development mode for testing:**
   ```bash
   ./run.sh start --dev  # Fast, no hardware needed
   ```

5. **Monitor resource usage:**
   ```bash
   watch -n 2 './run.sh status'
   ```

## Performance

- **Start time:** ~15-30 seconds for production mode
- **Start time:** ~10-15 seconds for development mode
- **Memory usage:** ~1-1.5 GB total for all containers
- **CPU usage:** ~5-15% idle, up to 50% under load

## Future Enhancements

Potential additions:
- Automatic backups
- Health monitoring with alerts
- Performance profiling
- Remote deployment support
- Multi-node orchestration

## Related Documentation

- **QUICK_START.md** - Quick reference guide
- **SETUP_GUIDE.md** - Detailed setup instructions
- **API_DOCUMENTATION.md** - API reference
- **MPU6050_SETUP.md** - IMU sensor setup
- **SENSOR_WIRING.md** - Hardware wiring guide

## Version History

- **v1.0** (2025-11-11)
  - Initial comprehensive management script
  - Full Docker Compose integration
  - Status monitoring and health checks
  - Development and production modes
  - Interactive help and documentation

---

**Quick Reference:** Run `./run.sh help` anytime for command reference!

