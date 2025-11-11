# LKS Robot - Quick Reference Card

## Two Main Commands

All operations use just 2 scripts:

### 1. Setup (One-Time Configuration)

```bash
./setup --rpi         # Raspberry Pi setup
./setup --pc          # PC/development setup
```

### 2. Start (Runtime Operations)

```bash
# Starting the robot
./start --hw          # Hardware mode (real sensors)
./start --sim         # Simulation mode (virtual sensors)
./start --test        # Test mode (interactive testing)

# System management
./start --status      # Show system status
./start --logs        # View container logs
./start --stop        # Stop all containers
./start --restart     # Restart containers
./start --shell       # Enter container shell
./start --clean       # Clean Docker system
```

## Common Workflows

### First-Time Setup on Raspberry Pi

```bash
./setup --rpi
# Follow prompts
sudo reboot
./start --hw
./start --test
```

### First-Time Setup on PC

```bash
./setup --pc
./start --sim
./start --test
```

### Daily Development

```bash
./start --sim         # Start simulation
# Do your work
./start --stop        # Stop when done
```

### Production on Raspberry Pi

```bash
./start --hw          # Start hardware
./start --status      # Check health
./start --logs        # Monitor logs
```

### System Troubleshooting

```bash
./start --status      # Check what's running
./start --test        # Run system tests
./start --logs        # View error logs
./start --restart     # Try restarting
./start --clean       # Clean and rebuild
```

## Access URLs

Once started:

- **Web Dashboard**: http://localhost:8000
- **Robot API**: http://localhost:5000
- **n8n Automation**: http://localhost:5678
- **WebSocket**: ws://localhost:8765

## Quick Tips

### Get Help
```bash
./setup --help
./start --help
```

### Force Rebuild
```bash
./start --sim --build
```

### Run in Foreground (See All Output)
```bash
./start --sim --foreground
```

### Enter Container for Debugging
```bash
./start --shell
# Inside container:
ros2 topic list
ros2 node list
exit
```

### Check System Status
```bash
./start --status      # Container status
docker ps             # Running containers
docker logs <name>    # Specific container logs
```

### Stop Everything
```bash
./start --stop        # Graceful stop
docker compose down   # Force stop
```

## Troubleshooting Quick Fixes

### Container Won't Start
```bash
./start --stop
./start --clean
./start --sim --build
```

### Permission Denied (Docker)
```bash
sudo usermod -aG docker $USER
# Logout and login again
```

### Port Already in Use
```bash
./start --stop
# Wait a few seconds
./start --sim
```

### ROS2 Workspace Not Built
```bash
./setup --pc          # Builds workspace
# Or manually:
cd ros2_ws
colcon build
```

### GPIO Permission Denied (Raspberry Pi)
```bash
sudo usermod -aG gpio,i2c $USER
# Logout and login again
```

## Advanced Operations

### Direct ROS2 (No Docker)
```bash
./start --direct
```

### Custom Compose File
```bash
docker compose -f docker-compose.dev.yml up -d
```

### View Specific Service Logs
```bash
docker logs -f ros2_sim_container
docker logs -f n8n_container
```

### Execute Command in Container
```bash
docker exec ros2_sim_container ros2 topic list
```

## Workflow Management

```bash
# Import workflows
./workflow_management_tools.sh import-enhanced

# List available workflows
./workflow_management_tools.sh list

# Check n8n status
./workflow_management_tools.sh status

# Export workflows
./workflow_management_tools.sh export
```

## File Locations

### Scripts
- Main scripts: `./setup`, `./start`
- Old scripts: `deprecated_scripts/`
- Utilities: `workflow_management_tools.sh`, `ros2_log_maintenance.sh`

### Configuration
- Docker: `docker-compose.yml`, `docker-compose.dev.yml`, `docker-compose.prod.yml`
- ROS2: `ros2_ws/`
- n8n data: `n8n_data/`
- Logs: `ros2_logs/`

### Documentation
- Main: `README.md`
- API: `docs/api/`
- Setup: `docs/installation/`
- Hardware: `docs/hardware/`
- Troubleshooting: `docs/troubleshooting/`

## Test Mode Commands

When running `./start --test`, interactive menu options:

1. Test movement (forward)
2. Test movement (turn left)
3. Test movement (turn right)
4. Test movement (stop)
5. Read all sensors
6. Test gripper
7. View ROS2 topics
8. View system status
9. Exit test mode

## Environment Modes

### Hardware Mode (`--hw`)
- Real sensors required
- Raspberry Pi with GPIO
- I2C/SPI interfaces active
- Production operation

### Simulation Mode (`--sim`)
- No hardware needed
- Any PC with Docker
- Simulated sensor data
- Development/testing

### Test Mode (`--test`)
- System verification
- API endpoint testing
- Interactive control
- Sensor verification

## Remember

- Run `./setup` once per system
- Use `./start` for daily operations
- Check `--help` for all options
- Use `--test` to verify system
- Keep logs clean: `ros2_log_maintenance.sh`
- Update workflows: `workflow_management_tools.sh`

## Quick Health Check

```bash
./start --status
curl http://localhost:8000
curl http://localhost:5000/health
docker ps
```

All 4 should respond/work if system is healthy.

