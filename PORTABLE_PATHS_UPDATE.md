# Portable Paths Update

## What Changed

All hardcoded paths have been removed and replaced with relative paths to make the project portable across different machines and users.

## Before (Hardcoded)

```bash
# ❌ Didn't work on other machines
cd /home/azzar/project/robotic/lks_robot_project
python3 diagnose_imu.py
```

## After (Portable)

```bash
# ✅ Works anywhere
cd ~/Autonomous_Mobile_Manipulator  # or wherever you put it
python3 diagnose_imu.py
```

## Files Updated

### 1. diagnose_imu.py
- Now finds `ros2_ws/src/my_robot_automation/scripts` dynamically
- Checks both script directory and current working directory
- Shows warning if not run from project root
- All recommendations use relative paths

### 2. run_ros2_script.sh
- Uses `${BASH_SOURCE[0]}` to find its own location
- Calculates ros2_ws path relative to script location
- Shows clear error if ros2_ws not found
- Works from any directory structure

### 3. STARTUP_GUIDE.md
- All commands updated to use relative paths
- Added "from project root" comments
- Removed all hardcoded /home/azzar paths

### 4. QUICK_COMMANDS.md
- Updated all command examples
- File locations now show relative paths
- Added project root context to all commands

### 5. README_SETUP.md (NEW)
- Explains project root concept
- Shows how to find project root
- Common mistakes and solutions
- Works-anywhere approach

## How It Works Now

### Dynamic Path Resolution

**diagnose_imu.py:**
```python
# Finds scripts directory automatically
script_dir = os.path.dirname(os.path.abspath(__file__))
scripts_path = os.path.join(script_dir, 'ros2_ws', 'src', 'my_robot_automation', 'scripts')

# Falls back to current directory if not found
if not os.path.exists(scripts_path):
    scripts_path = os.path.join(os.getcwd(), 'ros2_ws', 'src', 'my_robot_automation', 'scripts')
```

**run_ros2_script.sh:**
```bash
# Finds its own location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS2_WS="$SCRIPT_DIR/ros2_ws"
```

### Project Root Concept

All documentation now refers to "project root" - the directory where these files exist:
- `diagnose_imu.py`
- `test_imu_endpoint.py`
- `run_ros2_script.sh`
- `ros2_ws/` directory
- `notes.txt`

## Usage on Any Machine

### Machine 1 (Original)
```bash
cd ~/Autonomous_Mobile_Manipulator
python3 diagnose_imu.py  # Works!
```

### Machine 2 (Clone)
```bash
git clone <repo> /opt/robot
cd /opt/robot
python3 diagnose_imu.py  # Works!
```

### Machine 3 (Different User)
```bash
cd /home/different_user/my_robot_project
python3 diagnose_imu.py  # Works!
```

## Smart Features

### 1. Auto-Detection
Scripts automatically find their dependencies:
```python
# Before: hardcoded
sys.path.insert(0, '/home/azzar/project/robotic/...')

# After: dynamic
script_dir = os.path.dirname(os.path.abspath(__file__))
scripts_path = os.path.join(script_dir, 'ros2_ws', '...')
```

### 2. Helpful Warnings
If you run from wrong directory:
```
⚠ WARNING: You are not in the project root directory!
⚠ Please run this from: /actual/project/path

To fix:
  cd /actual/project/path
  python3 diagnose_imu.py
```

### 3. Error Messages Show Context
```bash
ERROR: ros2_ws directory not found at: /current/path/ros2_ws
Please run this script from the project root directory
```

## Benefits

1. **Portable** - Clone and run anywhere
2. **Multi-user** - Works for any username
3. **Flexible** - Any directory structure
4. **Clear** - Shows what directory to use
5. **Safe** - Validates paths before using

## Testing Portability

To verify it works on your system:

```bash
# 1. Clone/copy project to new location
cp -r ~/Autonomous_Mobile_Manipulator /tmp/test_robot

# 2. Go there
cd /tmp/test_robot

# 3. Run diagnostics
python3 diagnose_imu.py

# 4. It works! No path changes needed.
```

## Documentation Updated

All these files now use relative paths:
- ✅ `STARTUP_GUIDE.md`
- ✅ `QUICK_COMMANDS.md`
- ✅ `IMU_TROUBLESHOOTING.md` (already was relative)
- ✅ `QUICK_IMU_TEST.md` (already was relative)
- ✅ `README_SETUP.md` (new)
- ✅ `diagnose_imu.py`
- ✅ `run_ros2_script.sh`

## Key Principle

**"Project Root"** = Directory containing:
- These helper scripts
- The `ros2_ws/` directory
- Project documentation

All commands assume you start from there.

## Quick Reference

### Find Project Root
```bash
# Where is diagnose_imu.py?
find ~ -name "diagnose_imu.py" 2>/dev/null

# Go there
cd /path/shown/above

# This is your project root
pwd
```

### Run From Project Root
```bash
# All commands work from here:
python3 diagnose_imu.py           # Diagnostics
python3 test_imu_endpoint.py      # API test
./run_ros2_script.sh <script>     # ROS2 helper
cd ros2_ws && ...                 # Web interface
```

## Summary

**Before:** Only worked on one machine with one user in one directory

**After:** Works on any machine, any user, any directory structure

Just make sure you're in the project root (where `diagnose_imu.py` lives) and everything works!

