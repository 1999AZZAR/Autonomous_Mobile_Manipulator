# Deprecated Scripts

This folder contains the old scripts that have been consolidated into the new unified `setup` and `start` scripts.

## Migration Guide

### Old Scripts → New Scripts

**Setup Scripts:**
- `setup_raspberry_pi.sh` → `./setup --rpi`
- `build_workspace.sh` → `./setup --pc` (includes workspace building)

**Start Scripts:**
- `start_hardware.sh` → `./start --hw`
- `start_simulation.sh` → `./start --sim`
- `start_robot.sh` → `./start --hw` or `./start --sim`
- `run.sh` → `./start` (with various flags)

**Utility Scripts:**
- `interact_with_robot.sh` → `./start --test` (interactive menu included)
- `launch_gazebo_gui.sh` → `./start --sim` (Gazebo included)
- `run_ros2_script.sh` → `./start --direct` (direct ROS2 execution)

## Why Consolidated?

The old script structure had significant redundancy:
- Multiple scripts doing similar tasks
- Inconsistent flags and options
- Confusing for new users
- Harder to maintain

The new structure provides:
- Two clear entry points: `setup` and `start`
- Consistent command-line interface
- Easier to understand and use
- Better maintainability

## New Usage

```bash
# Setup (one-time)
./setup --rpi          # Raspberry Pi setup
./setup --pc           # PC/development setup

# Start the system
./start --hw           # Hardware mode (real sensors)
./start --sim          # Simulation mode
./start --test         # Test mode with interactive menu

# System operations
./start --status       # Check status
./start --logs         # View logs
./start --stop         # Stop system
./start --restart      # Restart system
./start --shell        # Enter container shell
```

## Preservation

These old scripts are kept for reference only. They should not be used for new deployments.
Use the new `setup` and `start` scripts instead.

