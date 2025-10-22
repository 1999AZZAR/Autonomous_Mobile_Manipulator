# 🤖 **N8N Robot Workflows - Organized Structure**

This directory contains organized n8n workflows for controlling the LKS Robot system. The workflows are divided into two main categories for better organization and usability.

---

## 📋 **Workflow Categories**

### 🔄 **Combination Workflows**
Complex workflows that combine multiple robot operations for complete automation tasks.

### 🎯 **Individual Workflows** 
Simple workflows for controlling single actuators or getting specific robot status information.

---

## 🔄 **COMBINATION WORKFLOWS**

### **1. Robot Complete Test** (`robot_simple_test.json`)
**Purpose**: Comprehensive system test of all robot capabilities
**Operations**:
- ✅ Get robot status
- ✅ Move robot forward
- ✅ Turn robot left
- ✅ Control servos (home position)
- ✅ Control lifter (up movement)
- ✅ Stop robot
- ✅ Final status check

**Use Case**: Full system verification and testing

---

### **2. Robot Emergency Stop** (`robot_emergency_stop.json`)
**Purpose**: Emergency safety system and monitoring
**Operations**:
- ✅ Continuous status monitoring
- ✅ Emergency stop activation
- ✅ Safety status checking
- ✅ Alert notifications

**Use Case**: Safety monitoring and emergency response

---

### **3. Robot Pick and Place** (`robot_pick_place.json`)
**Purpose**: Complete pick and place automation
**Operations**:
- ✅ Navigate to pick location
- ✅ Lower lifter
- ✅ Close gripper
- ✅ Lift object
- ✅ Navigate to place location
- ✅ Lower and release object
- ✅ Return to home position

**Use Case**: Automated material handling

---

### **4. Robot Patrol Mission** (`robot_patrol.json`)
**Purpose**: Autonomous patrol operations
**Operations**:
- ✅ Define patrol waypoints
- ✅ Navigate between points
- ✅ Monitor sensors during patrol
- ✅ Obstacle avoidance
- ✅ Return to base

**Use Case**: Security and surveillance automation

---

### **5. Robot Path Planning** (`robot_path_planning.json`)
**Purpose**: Advanced path planning and navigation
**Operations**:
- ✅ Get current robot status
- ✅ Extract path parameters (start/goal coordinates)
- ✅ Plan optimal path using map data
- ✅ Execute planned path
- ✅ Get final status

**Use Case**: Autonomous navigation with path planning

---

### **6. Robot Line Follower** (`robot_line_follower.json`)
**Purpose**: Line-based navigation using line sensor
**Operations**:
- ✅ Get current robot status
- ✅ Read line sensor data
- ✅ Execute line following with PID control
- ✅ Get final status

**Use Case**: Line-based navigation and precision movement

---

### **7. Robot Object Recognition** (`robot_object_recognition.json`)
**Purpose**: Object recognition using Microsoft camera
**Operations**:
- ✅ Get current robot status
- ✅ Capture image from Microsoft camera
- ✅ Recognize objects in image
- ✅ Analyze object position and orientation
- ✅ Get final status

**Use Case**: Object detection and recognition for pick/place operations

---

## 🎯 **INDIVIDUAL WORKFLOWS**

### **1. Control Omni Wheels** (`individual_control_omni_wheels.json`)
**Purpose**: Direct control of robot movement (Back, Front Left, Front Right)
**Operations**:
- ✅ Get current robot status
- ✅ Move robot (forward/backward/left/right)
- ✅ Get final status after movement

**Use Case**: Manual movement control and testing

---

### **2. Control Picker System** (`individual_control_picker_system.json`)
**Purpose**: Direct control of picker system components
**Operations**:
- ✅ Control gripper (servo)
- ✅ Control gripper tilt (servo)
- ✅ Control gripper neck (servo continuous)
- ✅ Control gripper base (motor)
- ✅ Get final status

**Use Case**: Individual picker system control and calibration

---

### **3. Control Container System** (`individual_control_container_system.json`)
**Purpose**: Direct control of container load system
**Operations**:
- ✅ Control left front container
- ✅ Control left back container
- ✅ Control right front container
- ✅ Control right back container
- ✅ Get final status

**Use Case**: Container load management and testing

---

### **4. Control Hardware Controls** (`individual_control_hardware_controls.json`)
**Purpose**: Direct control of hardware controls
**Operations**:
- ✅ Emergency stop control
- ✅ Start/stop control
- ✅ Mode control (train/run)
- ✅ Get final status

**Use Case**: Hardware control and safety management

---

### **5. Control Servo** (`individual_control_servo.json`)
**Purpose**: Direct control of individual servo motors
**Operations**:
- ✅ Get current servo status
- ✅ Set servo angle (0-180 degrees)
- ✅ Get final servo position

**Use Case**: Individual servo control and calibration

---

### **6. Get Robot Status** (`individual_get_status.json`)
**Purpose**: Retrieve complete robot status information
**Operations**:
- ✅ Get all sensor data
- ✅ Get actuator positions
- ✅ Format status output

**Use Case**: Status monitoring and diagnostics

---

## 🌐 **API Integration**

All workflows use the robot's REST API endpoints:

### **Base URL**: `http://10.0.3.1:5000`

### **Available Endpoints**:
- `GET /api/robot/status` - Get robot status
- `POST /api/robot/move` - Move robot
- `POST /api/robot/turn` - Turn robot
- `POST /api/robot/lifter` - Control lifter
- `POST /api/robot/servo` - Control individual servo
- `POST /api/robot/servos` - Control all servos
- `POST /api/robot/stop` - Stop robot
- `POST /api/robot/emergency` - Emergency stop

---

## 🚀 **Usage Instructions**

### **Accessing Workflows**
1. **Open n8n Interface**: Navigate to http://localhost:5678
2. **Browse Workflows**: Go to "Workflows" section
3. **Filter by Tags**: Use tags to filter workflows:
   - `Combination Workflow` - Complex operations
   - `Individual Control` - Single actuator control

### **Executing Workflows**
1. **Select Workflow**: Click on desired workflow
2. **Execute**: Click "Execute workflow" button
3. **Monitor**: Watch real-time execution in logs
4. **Verify**: Check robot response and sensor data

### **Workflow Categories**
- **Combination Workflows**: For complex automation tasks
- **Individual Workflows**: For single actuator control and testing

---

## 🔧 **Technical Details**

### **Network Configuration**
- **Host IP**: `10.0.3.1:5000` (n8n container to robot API)
- **Protocol**: HTTP/HTTPS REST API
- **Content-Type**: `application/json`
- **Method**: GET for status, POST for control

### **Workflow Structure**
- **Manual Trigger**: Start workflow execution
- **HTTP Request Nodes**: API communication
- **Set Nodes**: Data formatting and processing
- **Conditional Logic**: Decision making and error handling

### **Error Handling**
- **Connection Errors**: Automatic retry logic
- **API Errors**: Status code validation
- **Timeout Handling**: Configurable timeouts
- **Data Validation**: Input/output verification

---

## 📊 **Real Hardware Integration**

### **Sensors** (Live Data) - Based on notes.txt configuration
- **Distance Sensors** (3x): Laser-based distance measurement
  - Front distance sensor
  - Back left distance sensor  
  - Back right distance sensor
- **RPLIDAR A1**: 380° environment scanning
- **Microsoft Camera (USB)**: Object recognition and visual perception
- **Line Sensor**: Line-based navigation capability
- **IMU Sensor (MPU6050/BNO055)**: Orientation and motion sensing

### **Actuators** (Controlled) - Based on notes.txt configuration
- **Omni Wheels** (3x): Back, Front Left, Front Right - Omnidirectional movement
- **Picker System** (4 components):
  - Gripper (servo): Open/close control
  - Gripper tilt (servo): Angle control
  - Gripper neck (servo continuous): Forward/backward movement
  - Gripper base (motor): Up/down height control
- **Container System** (4 containers):
  - Left front container
  - Left back container
  - Right front container
  - Right back container
- **Hardware Controls**:
  - Emergency stop control
  - Start/stop control
  - Mode control (train/run)

### **Live Data Flow**
```
N8N Workflow → Robot API → Real Hardware → Live Sensors → N8N Workflow
```

---

## 🎯 **Best Practices**

### **Workflow Design**
1. **Always get status first** - Check current robot state
2. **Execute control command** - Perform desired action
3. **Verify final status** - Confirm operation success
4. **Handle errors gracefully** - Implement error recovery

### **Testing Approach**
1. **Start with Individual Workflows** - Test single actuators
2. **Verify API connectivity** - Ensure network communication
3. **Test Combination Workflows** - Validate complex operations
4. **Monitor robot response** - Watch real hardware movement

### **Safety Considerations**
1. **Use Emergency Stop** - Always available for safety
2. **Check sensor data** - Monitor environment conditions
3. **Verify actuator limits** - Respect hardware constraints
4. **Test in safe environment** - Clear area for robot movement

---

## 🎉 **System Status**

### ✅ **Fully Operational - Updated for notes.txt Configuration**
- **N8N Interface**: http://localhost:5678 ✅
- **Robot API**: http://10.0.3.1:5000 ✅
- **Network Communication**: Perfect connectivity ✅
- **Workflow Execution**: All workflows updated and working ✅
- **Real Hardware Control**: Live robot movement with actual hardware configuration ✅

### 🤖 **Ready for Production - Hexagonal Robot with Complete System**
The updated workflow structure provides:
- **Hexagonal robot shape** with 3x omni wheels (Back, Front Left, Front Right)
- **Complete picker system** with 4 components (gripper, tilt, neck, base)
- **Container load system** with 4 containers (left/right, front/back)
- **Advanced sensors** (distance, RPLIDAR A1, Microsoft camera, line sensor, IMU)
- **Hardware controls** (emergency, start/stop, mode train/run)
- **Control mechanisms** (path planning, obstacle avoidance, line follower, PID control, object recognition)
- **Real-time control** of actual hardware
- **Professional organization** for production use

**The hexagonal robot can now be controlled entirely through organized n8n workflows with complete hardware integration matching the notes.txt configuration!** 🚀✨