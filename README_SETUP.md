# Robot Setup and Usage

## Important: Project Root Directory

All commands in this documentation assume you are in the **project root directory**.

To find your project root:
```bash
# Your project root is where these files exist:
# - diagnose_imu.py
# - test_imu_endpoint.py
# - run_ros2_script.sh
# - ros2_ws/ (directory)
# - notes.txt

# Example:
cd ~/Autonomous_Mobile_Manipulator  # or wherever you cloned the project
pwd  # This shows your current directory - this is your "project root"
```

## Quick Start (3 Steps)

### 1. Navigate to Project Root
```bash
cd /path/to/your/Autonomous_Mobile_Manipulator
```

### 2. Run Diagnostics
```bash
python3 diagnose_imu.py
```
This will check everything and tell you what needs to be fixed.

### 3. Start Web Interface
```bash
cd ros2_ws
python3 src/my_robot_automation/scripts/web_robot_interface.py
```

## All Commands Are Relative

All scripts and documentation use **relative paths**, so they work on any machine.

Just make sure you're in the project root directory when you run commands.

## Common Mistakes

### ❌ Wrong (Hardcoded Path)
```bash
cd /home/azzar/project/robotic/lks_robot_project
python3 diagnose_imu.py
```

### ✅ Correct (Relative Path)
```bash
cd ~/Autonomous_Mobile_Manipulator  # or your actual project location
python3 diagnose_imu.py
```

## Finding Your Project Root

If you're lost, use these commands:

```bash
# Find where the script is
find ~ -name "diagnose_imu.py" 2>/dev/null

# Go to that directory
cd /path/shown/above

# Verify you're in the right place
ls -la
# You should see: diagnose_imu.py, ros2_ws/, notes.txt, etc.
```

## Documentation Files

All these files use relative paths:

- `STARTUP_GUIDE.md` - Complete startup guide
- `QUICK_COMMANDS.md` - Quick command reference
- `IMU_TROUBLESHOOTING.md` - IMU troubleshooting
- `QUICK_IMU_TEST.md` - Quick IMU testing

## Helper Scripts (Portable)

These scripts automatically find their location:

- `diagnose_imu.py` - Finds ros2_ws directory automatically
- `run_ros2_script.sh` - Uses relative path from its location
- `test_imu_endpoint.py` - Works from any directory

## For Different Machines

When you clone this project on a different machine:

1. Clone to any location: `git clone <repo> ~/my_robot`
2. Navigate there: `cd ~/my_robot`
3. Run commands as documented: `python3 diagnose_imu.py`

No path changes needed! Everything is relative.

## Project Structure

```
project-root/                    # Your "project root" - can be anywhere
├── diagnose_imu.py             # Run this first
├── test_imu_endpoint.py        # Test API
├── run_ros2_script.sh          # ROS2 helper
├── notes.txt                   # Project notes
├── STARTUP_GUIDE.md            # Full guide
├── QUICK_COMMANDS.md           # Quick reference
├── ros2_ws/                    # ROS2 workspace
│   ├── src/
│   │   └── my_robot_automation/
│   │       └── scripts/
│   │           ├── web_robot_interface.py
│   │           ├── mpu6050_reader.py
│   │           └── ...
│   └── install/
└── ...
```

## Environment Setup

The scripts will work from any location, but for consistency:

```bash
# Set an environment variable for your project (optional)
echo "export ROBOT_PROJECT=~/Autonomous_Mobile_Manipulator" >> ~/.bashrc
source ~/.bashrc

# Now you can:
cd $ROBOT_PROJECT
python3 diagnose_imu.py
```

## Summary

- **All paths are relative** - works on any machine
- **Start from project root** - where diagnose_imu.py is located
- **No hardcoded paths** - everything finds itself
- **Portable** - clone and run anywhere

If you get "file not found" errors, you're probably not in the project root directory.

