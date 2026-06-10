#include <PID_v1.h>
#include <MotorDriver.h>
#include "config.h"

MotorDriver motorDriver = MotorDriver(YF_IIC_RZ);

struct EncoderPins {
  int encA;
  int encB;
};

EncoderPins encoders[4] = {
  {3, 5},  // Motor 1 - Lifter
  {2, 4},  // Motor 2 - Front Right
  {7, 6},  // Motor 3 - Front Left
  {9, 8}   // Motor 4 - Back
};

volatile long encoderCount[4] = {0, 0, 0, 0};
long lastEncoderCount[4] = {0, 0, 0, 0};
unsigned long lastEncoderTime[4] = {0, 0, 0, 0};

double rpmHistory[4][RPM_FILTER_SIZE] = {0};
int rpmHistoryIndex[4] = {0};
double filteredRPM[4] = {0, 0, 0, 0};
double smoothedRPM[4] = {0, 0, 0, 0};
double rpmAlpha = RPM_ALPHA;
double rpmAlphaFast = RPM_ALPHA_FAST;

double lastRPM[4] = {0, 0, 0, 0};
double maxRPMChange = MAX_RPM_CHANGE;
double maxRPMChangeRotation = MAX_RPM_CHANGE_ROTATION;

long totalEncoderCount[4] = {0, 0, 0, 0};
double motorPosition[4] = {0, 0, 0, 0};

double syncError[4] = {0, 0, 0, 0};
double syncKp = SYNC_KP;
double targetSyncRPM = 0;
bool synchronizationActive = false;
int activeMotorCount = 0;
bool motorIntendedActive[4] = {false, false, false, false};

unsigned long lastRotationCommand = 0;
bool fastRotationMode = false;

double setpoint[4] = {0, 0, 0, 0};
double input[4] = {0, 0, 0, 0};
double output[4] = {0, 0, 0, 0};
bool motorsStopped = true;
bool lifterActive = false;

// Lifter limit switch states
bool lifterAtTop = false;
bool lifterAtBottom = false;
unsigned long lifterMovementStartTime = 0;
bool lifterSafetyTimeout = false;

double prev_setpoint[4] = {0, 0, 0, 0};
float speedMultiplier = 1.0;

// int16_t rawGyroX, rawGyroY, rawGyroZ;
// int16_t rawAccelX, rawAccelY, rawAccelZ;
// float gyroX, gyroY, gyroZ;
// float accelX, accelY, accelZ;
float temperature;
double robotHeading = 0.0;
double targetHeading = 0.0;
double headingError = 0.0;
// double headingKp = HEADING_KP;
// double headingKi = HEADING_KI;
// double headingKd = HEADING_KD;
double headingCorrection = 0.0;
double headingIntegral = 0.0;
double headingPreviousError = 0.0;
bool headingCorrectionEnabled = true;
double gyroDriftX = 0.0;
double gyroDriftY = 0.0;
double gyroDriftZ = 0.0;
// int gyroCalibrationSamples = GYRO_CALIBRATION_SAMPLES;
bool gyroCalibrated = false;

PID pid[4] = {
  PID(&input[0], &output[0], &setpoint[0], Lifter_Kp, Lifter_Ki, Lifter_Kd, DIRECT),
  PID(&input[1], &output[1], &setpoint[1], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT),
  PID(&input[2], &output[2], &setpoint[2], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT),
  PID(&input[3], &output[3], &setpoint[3], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT)
};

struct IRDistanceData {
  float voltage;
  float distance;
  bool valid;
};

IRDistanceData irLeft1, irLeft2;
IRDistanceData irRight1, irRight2;
IRDistanceData irBack1, irBack2;

struct UltrasonicData {
  long duration;
  float distance;
  bool valid;
};

UltrasonicData ultrasonicFrontLeft;
UltrasonicData ultrasonicFrontRight;

struct LineSensorData {
  int rawValue;
  bool onLine;
};

LineSensorData lineLeft, lineCenter, lineRight;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 50;  // Reduced from 100ms to 50ms for 2x faster sensor updates

// === SERVO CONTROL VARIABLES ===
int currentTiltAngle = TILT_SERVO_DEFAULT;
int currentGripperAngle = GRIPPER_SERVO_DEFAULT;

// === PERIMETER SAFETY SYSTEM VARIABLES ===
unsigned long lastPerimeterCheck = 0;
bool perimeterSafetyEnabled = true;
bool sensorPublishingEnabled = true;  // Enable sensor data publishing by default
bool individualWheelControlEnabled = false;  // Individual wheel control mode  // Master enable/disable for perimeter safety

enum SafetyLevel {
  SAFE,
  WARNING,
  CRITICAL
};

enum MovementDirection {
  STOPPED,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD_LEFT,
  BACKWARD_RIGHT,
  ROTATE_CW,
  ROTATE_CCW,
  COMPLEX  // Multiple directions or rotation
};

struct PerimeterStatus {
  SafetyLevel frontLeftIR;
  SafetyLevel frontRightIR;
  SafetyLevel leftIR;
  SafetyLevel rightIR;
  SafetyLevel backIR;
  SafetyLevel frontUltrasonic;
  SafetyLevel overall;
  bool emergencyBrake;
  unsigned long lastObstacleTime;
};

// Virtual Force Field Structure for Artificial Potential Field method
struct VirtualForce {
  float x;        // Force component in X direction (normalized -1 to 1)
  float y;        // Force component in Y direction (normalized -1 to 1)
  float magnitude; // Force magnitude (normalized 0 to 1)
};

struct SensorForce {
  VirtualForce repulsiveForce;  // Repulsive force from this sensor
  float distance;              // Current distance reading (mm)
  bool valid;                  // Whether sensor reading is valid
  float influenceDistance;     // Maximum influence distance (mm)
  float scalingFactor;         // Force scaling factor
};

// Virtual Bumper Status with force field calculations
struct VirtualBumperStatus {
  SensorForce irLeft1;
  SensorForce irLeft2;
  SensorForce irRight1;
  SensorForce irRight2;
  SensorForce irBack1;
  SensorForce irBack2;
  SensorForce ultrasonicFrontLeft;
  SensorForce ultrasonicFrontRight;

  VirtualForce totalForce;     // Sum of all repulsive forces
  float maxForceMagnitude;     // Maximum force magnitude detected
  bool forceFieldActive;       // Whether virtual forces are being applied
};

PerimeterStatus perimeterStatus = {SAFE, SAFE, SAFE, SAFE, SAFE, SAFE, SAFE, false, 0};
VirtualBumperStatus virtualBumperStatus = {
  // Initialize all sensor forces with default values
  {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0},
  {0, 0, 0}, 0.0, false
};
unsigned long lastAutoBrake = 0;
MovementDirection currentMovementDirection = STOPPED;
MovementDirection previousMovementDirection = STOPPED;
unsigned long lastDirectionChange = 0;
const unsigned long DIRECTION_CHANGE_TIMEOUT = 800; // 800ms of fast response after direction change
const unsigned long DIRECTION_CHANGE_BUFFER = 150;  // 150ms buffer to prevent command spam

void setup() {
  Serial.begin(115200);
  Serial.println("Omni Wheel Robot Control Starting...");

  // Step 1: Initialize motor driver first (as per original design)
  Serial.println("Step 1: Initializing motor driver...");
  motorDriver.begin();
  motorDriver.motorConfig(1, 1, 1, 1);

  // Set PWM frequency to 50Hz for servo control (required!)
  motorDriver.setPWMFreq(50);
  delay(500);

  // Step 2: Test I2C bus health after motor driver init
  Serial.println("Step 2: Testing I2C bus health...");
  testI2CBus();
  delay(200);

  for (int i = 0; i < 4; i++) {
    pinMode(encoders[i].encA, INPUT_PULLUP);
    pinMode(encoders[i].encB, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encoders[0].encA), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].encA), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].encA), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].encA), encoderISR3, CHANGE);

  for (int i = 0; i < 4; i++) {
    pid[i].SetMode(AUTOMATIC);
    pid[i].SetSampleTime(PID_SAMPLE_TIME);
    pid[i].SetOutputLimits(PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX);
  }

  Serial.println("Initializing sensors...");
  pinMode(ULTRASONIC_FRONT_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_LEFT_ECHO, INPUT);
  pinMode(ULTRASONIC_FRONT_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_RIGHT_ECHO, INPUT);

  // Initialize lifter limit switches
  pinMode(LIFTER_TOP_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LIFTER_BOTTOM_LIMIT_PIN, INPUT_PULLUP);

  // Initialize servos through YFROBOT shield channels
  Serial.println("Initializing servos through YFROBOT shield channels...");

  // Test servo channels with default positions
  Serial.print("Setting tilt servo (channel ");
  Serial.print(TILT_SERVO_CHANNEL);
  Serial.print(") to ");
  Serial.print(TILT_SERVO_DEFAULT);
  Serial.println("°");
  motorDriver.servoWrite(TILT_SERVO_CHANNEL, TILT_SERVO_DEFAULT);

  Serial.print("Setting gripper servo (channel ");
  Serial.print(GRIPPER_SERVO_CHANNEL);
  Serial.print(") to ");
  Serial.print(GRIPPER_SERVO_DEFAULT);
  Serial.println("°");
  motorDriver.servoWrite(GRIPPER_SERVO_CHANNEL, GRIPPER_SERVO_DEFAULT);

  currentTiltAngle = TILT_SERVO_DEFAULT;
  currentGripperAngle = GRIPPER_SERVO_DEFAULT;

  Serial.println("Servos initialized.");

  Serial.println("Sensors initialized.");

  Serial.println("🚀 HIGH-POWER SMOOTH DIRECTION MODE ACTIVATED!");
  Serial.println("⚡ Turbo PID (10ms), adaptive acceleration, maximum torque");
  Serial.println("💪 8x motor power, 3x speed limits, intelligent direction smoothing");
  Serial.println("🎯 Omni motors: Smooth direction changes, Lifter: Ultimate power");
  Serial.println("Cmd: f,b,l,r,t,y,c,w,q,e,z,x,a,j,s,p,o,1-4,g,h,u,d,5-9,0,m,n,sr,se,sd,ls,v");
  Serial.println("Servo: m[u/d/c](tilt), n[o/c/h](gripper), ta<0-180>, ga<0-180>");
  Serial.println("Sensors: sp[e/d] (sensor publish enable/disable)");
  Serial.println("Wheels: w[0-3][-100-100] (individual wheel control), wstop (stop all)");
}

void loop() {
  // Update motor control every PID sample time (now 10ms for ultra-fast response)
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= PID_SAMPLE_TIME) {
    updateMotorControl();
    lastUpdate = millis();
  }

  // Update all sensors periodically (now every 50ms instead of 100ms)
  updateAllSensors();

  // Update perimeter safety monitoring - ALWAYS ACTIVE virtual bumper (now every 25ms)
  updatePerimeterSafety();

  // Update virtual force field for potential field method
  updateVirtualForceField();

  // Publish sensor data for ROS2 integration
  publishSensorData();

  // Optimized serial command processing - ultra-responsive
  static String commandBuffer = "";
  static unsigned long lastCommandTime = 0;

  // Process serial commands with higher priority
  while (Serial.available()) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n' || incomingChar == '\r') {
      // End of command - process immediately
      if (commandBuffer.length() > 0) {
        commandBuffer.trim();
        if (commandBuffer.length() > 0) {
          executePiCommand(commandBuffer);
          lastCommandTime = millis();
        }
        commandBuffer = "";
      }
    } else if (incomingChar != ' ' && incomingChar != '\t') {
      // Add character to buffer (no length limit for responsiveness)
      if (commandBuffer.length() < 20) {  // Prevent buffer overflow
      commandBuffer += incomingChar;
    }
    }
  }

  // Timeout protection - clear stale buffers after 1 second
  if (commandBuffer.length() > 0 && millis() - lastCommandTime > 1000) {
    Serial.println("TIMEOUT: Clearing stale command buffer");
    commandBuffer = "";
  }
}

// Encoder interrupt service routines
void encoderISR0() {
  encoderCount[0] += readEncoder(0);
}

void encoderISR1() {
  encoderCount[1] += readEncoder(1);
}

void encoderISR2() {
  encoderCount[2] += readEncoder(2);
}

void encoderISR3() {
  encoderCount[3] += readEncoder(3);
}

// Read encoder state
int readEncoder(int motorIndex) {
  int state = digitalRead(encoders[motorIndex].encA) * 2 + digitalRead(encoders[motorIndex].encB);
  static int lastState[4] = {0, 0, 0, 0};

  int direction = 0;
  switch (lastState[motorIndex]) {
    case 0:
      if (state == 1) direction = 1;
      else if (state == 2) direction = -1;
      break;
    case 1:
      if (state == 3) direction = 1;
      else if (state == 0) direction = -1;
      break;
    case 2:
      if (state == 0) direction = 1;
      else if (state == 3) direction = -1;
      break;
    case 3:
      if (state == 2) direction = 1;
      else if (state == 1) direction = -1;
      break;
  }
  lastState[motorIndex] = state;
  return direction;
}

// Enhanced encoder processing functions
double applyMovingAverageFilter(int motorIndex, double newRPM) {
  // Add new RPM to history buffer
  rpmHistory[motorIndex][rpmHistoryIndex[motorIndex]] = newRPM;
  rpmHistoryIndex[motorIndex] = (rpmHistoryIndex[motorIndex] + 1) % RPM_FILTER_SIZE;

  // Calculate moving average
  double sum = 0;
  for (int i = 0; i < RPM_FILTER_SIZE; i++) {
    sum += rpmHistory[motorIndex][i];
  }

  filteredRPM[motorIndex] = sum / RPM_FILTER_SIZE;
  return filteredRPM[motorIndex];
}

double applyExponentialSmoothing(int motorIndex, double newRPM) {
  // Adaptive exponential smoothing - ultra-responsive during fast rotation
  double alpha = fastRotationMode ? rpmAlphaFast : rpmAlpha;
  smoothedRPM[motorIndex] = alpha * newRPM + (1.0 - alpha) * smoothedRPM[motorIndex];
  return smoothedRPM[motorIndex];
}

double applyAccelerationLimiting(int motorIndex, double targetRPM) {
  // Check if we're in fast rotation mode (recent rotation command) - EXTENDED to 5 seconds
  bool isFastRotation = false;
  if (fastRotationMode && (millis() - lastRotationCommand) < 5000) {  // 5 seconds of MAXIMUM fast response
    isFastRotation = true;
  } else if ((millis() - lastRotationCommand) > 6000) {  // Disable fast mode after 6 seconds
    fastRotationMode = false;
  }

  // Determine if this is a rotation movement (all motors have same speed)
  bool isRotation = false;
  if (motorIndex >= 1 && motorIndex <= 3) {  // Only for omni motors
    double absSetpoint = abs(setpoint[motorIndex]);
    // Check if all omni motors have the same absolute setpoint (indicates rotation)
    if (absSetpoint > 10 && abs(setpoint[1]) == absSetpoint && abs(setpoint[2]) == absSetpoint && abs(setpoint[3]) == absSetpoint) {
      isRotation = true;
    }
  }

  // Skip acceleration limiting for fast rotation mode - INSTANT RESPONSE
  if (isFastRotation) {
    lastRPM[motorIndex] = targetRPM;
    return targetRPM;
  }

  // Adaptive acceleration based on movement context
  double maxChange = maxRPMChange;  // Start with base acceleration
  bool isDirectionChange = (millis() - lastDirectionChange) < DIRECTION_CHANGE_TIMEOUT;

  if (motorIndex == 0) {
    // Lifter motor: allow faster changes for responsiveness
    maxChange *= 2.0;  // Doubled for lifter
  } else if (isDirectionChange) {
    // Direction change: very fast response for smooth transitions
    maxChange *= 3.0;  // Triple acceleration during direction changes
  } else if (isRotation) {
    // Omni motors in rotation: moderate speed for smoothness
    maxChange = maxRPMChangeRotation;  // Full rotation speed
  } else {
    // Omni motors in translation: responsive but stable
    maxChange *= 1.5;  // 50% faster for omni motors
  }

  // Limit rate of RPM change for smooth acceleration/deceleration
  double rpmChange = targetRPM - lastRPM[motorIndex];

  if (abs(rpmChange) > maxChange) {
    rpmChange = (rpmChange > 0) ? maxChange : -maxChange;
    targetRPM = lastRPM[motorIndex] + rpmChange;
  }

  lastRPM[motorIndex] = targetRPM;
  return targetRPM;
}

void updateMotorPosition(int motorIndex, long countDiff) {
  // Track total encoder counts and position
  totalEncoderCount[motorIndex] += countDiff;
  motorPosition[motorIndex] = (double)totalEncoderCount[motorIndex] / (ENCODER_CPR * GEAR_RATIO);
}

// Motor synchronization functions
void updateMotorSynchronization() {
  // Calculate average RPM of active motors for synchronization
  double totalRPM = 0;
  activeMotorCount = 0;

  // Count active motors based on intended active flags
  for (int i = 1; i < 4; i++) {  // Skip lifter motor
    if (motorIntendedActive[i] && !motorsStopped) {
      totalRPM += smoothedRPM[i];
      activeMotorCount++;
    }
  }

  // Calculate target synchronized RPM
  if (activeMotorCount > 0) {
    targetSyncRPM = totalRPM / activeMotorCount;
    synchronizationActive = true;
  } else {
    targetSyncRPM = 0;
    synchronizationActive = false;
  }

  // Calculate synchronization errors only for intended active motors
  for (int i = 1; i < 4; i++) {
    if (motorIntendedActive[i] && !motorsStopped && synchronizationActive) {
      syncError[i] = targetSyncRPM - smoothedRPM[i];
    } else {
      syncError[i] = 0;
    }
  }
}

double applySynchronizationCorrection(int motorIndex, double targetRPM) {
  // Apply synchronization correction only to intended active motors
  if (synchronizationActive && motorIntendedActive[motorIndex] && !motorsStopped) {
    double correction = syncError[motorIndex] * syncKp;
    targetRPM += correction;

    // Constrain correction to reasonable limits
    targetRPM = constrain(targetRPM, -MAX_RPM * 1.2, MAX_RPM * 1.2);
  }

  return targetRPM;
}

void updateMotorControl() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 4; i++) {
    long countDiff = encoderCount[i] - lastEncoderCount[i];
    unsigned long timeDiff = currentTime - lastEncoderTime[i];

    if (timeDiff > 0) {
      double revolutions = (double)countDiff / (ENCODER_CPR * GEAR_RATIO);
      double rawRPM = (revolutions * 60000.0) / timeDiff;

      double filteredRPM = applyMovingAverageFilter(i, rawRPM);
      double smoothedRPM = applyExponentialSmoothing(i, filteredRPM);
      input[i] = smoothedRPM;

      updateMotorPosition(i, countDiff);
    }

    lastEncoderCount[i] = encoderCount[i];
    lastEncoderTime[i] = currentTime;
  }

  // DISABLED: Motor synchronization causing sluggishness
  // if (!fastRotationMode) {
  //   updateMotorSynchronization();
  // }
  synchronizationActive = false;  // Ensure sync is disabled

  if (lifterActive) {
    // Continuous limit switch safety check
    bool shouldStopLifter = false;

    if (lifterSafetyTimeout) {
      Serial.println("LFT:3");
      shouldStopLifter = true;
    } else if ((setpoint[0] < 0 && lifterAtTop) || (setpoint[0] > 0 && lifterAtBottom)) {
      if (setpoint[0] < 0 && lifterAtTop) {
        Serial.println("LFT:4");
      } else if (setpoint[0] > 0 && lifterAtBottom) {
        Serial.println("LFT:5");
      }
      shouldStopLifter = true;
    }

    if (shouldStopLifter) {
      lifterActive = false;
      setpoint[0] = 0;
      motorDriver.stopMotor(M1);
    } else {
      pid[0].Compute();
      int lifterSpeed = map(output[0], -255, 255, -4096, 4096);
      motorDriver.setSingleMotor(1, lifterSpeed);
    }
  } else {
    motorDriver.stopMotor(M1);
  }

  if (!motorsStopped) {
    for (int i = 1; i < 4; i++) {
      if (!motorIntendedActive[i]) {
        motorDriver.stopMotor(i + 1);
        setpoint[i] = 0;
        continue;
      }
      pid[i].Compute();
      int motorSpeed = map(output[i], -255, 255, -4096, 4096);
      motorDriver.setSingleMotor(i+1, motorSpeed);
    }
  } else {
    motorDriver.stopMotor(M2);
    motorDriver.stopMotor(M3);
    motorDriver.stopMotor(M4);
  }
}

// Clear all previous motor commands to ensure correct motor rotation
void clearMotorCommands() {
  // Stop all motors immediately to clear any previous PWM commands
  motorDriver.stopMotor(MAll);

  // Small delay to ensure motor driver processes the stop command
  delayMicroseconds(100);

  // Reset motor driver PWM channels to ensure clean state
  // This prevents residual PWM signals from previous commands
  for (int i = 1; i <= 4; i++) {
    motorDriver.setSingleMotor(i, 0);
  }

  // Additional delay to ensure PWM channels are cleared
  delayMicroseconds(50);
}

void setOmniSpeeds(double vx, double vy, double omega) {
  // Clear any previous motor commands before setting new speeds
  clearMotorCommands();

  motorsStopped = false;
  lifterActive = false;

  // Determine movement direction for intelligent sensor selection
  const double threshold = 0.1; // Minimum speed to consider as movement
  bool hasForward = vx > threshold;
  bool hasBackward = vx < -threshold;
  bool hasLeft = vy > threshold;
  bool hasRight = vy < -threshold;
  bool hasRotation = abs(omega) > threshold;

  if (hasRotation) {
  if (omega > 0) currentMovementDirection = ROTATE_CCW;
  else if (omega < 0) currentMovementDirection = ROTATE_CW;
  } else if (hasForward && hasLeft) {
    currentMovementDirection = FORWARD_LEFT;
  } else if (hasForward && hasRight) {
    currentMovementDirection = FORWARD_RIGHT;
  } else if (hasBackward && hasLeft) {
    currentMovementDirection = BACKWARD_LEFT;
  } else if (hasBackward && hasRight) {
    currentMovementDirection = BACKWARD_RIGHT;
  } else if (hasForward) {
    currentMovementDirection = FORWARD;
  } else if (hasBackward) {
    currentMovementDirection = BACKWARD;
  } else if (hasLeft) {
    currentMovementDirection = LEFT;
  } else if (hasRight) {
    currentMovementDirection = RIGHT;
  } else {
    currentMovementDirection = STOPPED;
  }

  // Detect direction changes for smoother transitions
  if (currentMovementDirection != previousMovementDirection && currentMovementDirection != STOPPED) {
    // Only allow direction change if enough time has passed since last change
    if (millis() - lastDirectionChange > DIRECTION_CHANGE_BUFFER) {
      lastDirectionChange = millis();
      previousMovementDirection = currentMovementDirection;
    } else {
      // Revert to previous direction to prevent jerky transitions
      currentMovementDirection = previousMovementDirection;
    }
  }

  // Apply virtual force field to modify desired velocities for obstacle avoidance
  double vx_modified = vx;
  double vy_modified = vy;
  double omega_modified = omega;
  applyVirtualForceField(vx_modified, vy_modified, omega_modified);

  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  double motor_speeds[4] = {0, 0, 0, 0};

  double forward_component = abs(vx_modified);
  double sideways_component = abs(vy_modified);
  double rotation_component = abs(omega_modified);
  bool use_back_wheel = (sideways_component > 0.1) || (rotation_component > 0.1);

  for (int i = 1; i < 4; i++) {
    double angle_rad = MOTOR_ANGLES[i] * PI / 180.0;

    if (!use_back_wheel && i == 3) {
      motor_speeds[i] = 0;
      motorIntendedActive[i] = false;
    } else {
      // Correct inverse kinematics for omni wheels with virtual force field avoidance:
      // ω_i = [-vx_modified * cos(θ) + vy_modified * sin(θ) + R * ω_modified]
      motor_speeds[i] = (-vx_modified * cos(angle_rad) + vy_modified * sin(angle_rad) + (omega_modified * ROBOT_RADIUS)) * speedMultiplier;
    }
  }

  double max_speed = 0.0;
  for (int i = 1; i < 4; i++) {
    max_speed = max(max_speed, abs(motor_speeds[i]));
  }

  double scale_factor = (max_speed > 1.0) ? (1.0 / max_speed) : 1.0;

  for (int i = 1; i < 4; i++) {
    double normalized_speed = motor_speeds[i] * scale_factor;
    double target_rpm = normalized_speed * BASE_SPEED;
    target_rpm = constrain(target_rpm, -MAX_RPM, MAX_RPM);

    motorIntendedActive[i] = (abs(normalized_speed) > 0.01);
    // DISABLED: Synchronization correction causing sluggishness
    // target_rpm = applySynchronizationCorrection(i, target_rpm);
    target_rpm = applyAccelerationLimiting(i, target_rpm);

    setpoint[i] = target_rpm;
    prev_setpoint[i] = target_rpm;
  }
}

// Movement functions
void moveForward() {
  Serial.println("Moving Forward (Motor 2 backward + Motor 3 forward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Forward: Motor 2 (FR) backward + Motor 3 (FL) forward
  setpoint[1] = -0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2) - backward
  setpoint[2] = 0.8 * BASE_SPEED * speedMultiplier;   // FL (Motor 3) - forward
  setpoint[3] = 0;                                    // Back (Motor 4) - idle
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL

  currentMovementDirection = FORWARD;
}

void moveBackward() {
  Serial.println("Moving Backward (Motor 2 forward + Motor 3 backward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Backward: Motor 2 (FR) forward + Motor 3 (FL) backward
  setpoint[1] = 0.8 * BASE_SPEED * speedMultiplier;   // FR (Motor 2) - forward
  setpoint[2] = -0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3) - backward
  setpoint[3] = 0;                                    // Back (Motor 4) - idle
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL

  currentMovementDirection = BACKWARD;
}

void moveLeft() {
  Serial.println("Moving Left (FR backward + Back forward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Left strafe: FR backward + Back forward
  setpoint[1] = -0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2) - backward
  setpoint[2] = 0;                                    // FL (Motor 3) - idle
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;   // Back (Motor 4) - forward
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = LEFT;
}

void moveRight() {
  Serial.println("Moving Right (FR forward + Back backward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Right strafe: FR forward + Back backward
  setpoint[1] = 0.8 * BASE_SPEED * speedMultiplier;   // FR (Motor 2) - forward
  setpoint[2] = 0;                                    // FL (Motor 3) - idle
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4) - backward
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = RIGHT;
}

// Diagonal movement functions - REFINED: specific wheel combinations
void moveForwardLeft() {
  Serial.println("Moving Forward-Left (FL forward + Back forward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Forward-left diagonal: FL forward + Back forward
  setpoint[1] = 0;                                    // FR (Motor 2) - idle
  setpoint[2] = 0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3) - forward
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4) - forward
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = FORWARD_LEFT;
}

void moveForwardRight() {
  Serial.println("Moving Forward-Right (FR forward + Back backward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Forward-right diagonal: FR forward + Back backward
  setpoint[1] = 0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2) - forward
  setpoint[2] = 0;                                    // FL (Motor 3) - idle
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4) - backward
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = FORWARD_RIGHT;
}

void moveBackwardLeft() {
  Serial.println("Moving Backward-Left (FL backward + Back backward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Backward-left diagonal: FL backward + Back backward
  setpoint[1] = 0;                                     // FR (Motor 2) - idle
  setpoint[2] = -0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3) - backward
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4) - backward
  setpoint[0] = 0;                                     // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = BACKWARD_LEFT;
}

void moveBackwardRight() {
  Serial.println("Moving Backward-Right (FR backward + Back forward)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Backward-right diagonal: FR backward + Back forward
  setpoint[1] = -0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2) - backward
  setpoint[2] = 0;                                     // FL (Motor 3) - idle
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;   // Back (Motor 4) - forward
  setpoint[0] = 0;                                     // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  currentMovementDirection = BACKWARD_RIGHT;
}

// Arc movement functions (forward + rotation)
void arcLeft() {
  Serial.println("Arc Left");
  setOmniSpeeds(0.8, 0.0, -0.5);  // Forward + left rotation
  setpoint[0] = 0;
}

void arcRight() {
  Serial.println("Arc Right");
  setOmniSpeeds(0.8, 0.0, 0.5);   // Forward + right rotation
  setpoint[0] = 0;
}

// Turn functions (slight rotation while moving forward)
void turnLeft() {
  Serial.println("Turning Left");
  setOmniSpeeds(0.5, 0.0, -0.8);  // Slow forward + turn
  setpoint[0] = 0;
}

void turnRight() {
  Serial.println("Turning Right");
  setOmniSpeeds(0.5, 0.0, 0.8);   // Slow forward + turn
  setpoint[0] = 0;
}

void rotateCW() {
  Serial.println("Rotating Clockwise (TURBO MODE - MAXIMUM RESPONSE)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  lastRotationCommand = millis();
  fastRotationMode = true;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use maximum speed for rotation (negative omega = clockwise)
  setOmniSpeeds(0.0, 0.0, -1.0);
  setpoint[0] = 0;

  // Explicitly set all omni motors as intended active for rotation
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back
}

void rotateCCW() {
  Serial.println("Rotating Counter-Clockwise (TURBO MODE - MAXIMUM RESPONSE)");
  clearMotorCommands();  // Clear previous commands for correct motor rotation
  lastRotationCommand = millis();
  fastRotationMode = true;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use maximum speed for rotation (positive omega = counter-clockwise)
  setOmniSpeeds(0.0, 0.0, 1.0);
  setpoint[0] = 0;

  // Explicitly set all omni motors as intended active for rotation
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back
}

void stopMotors() {
  Serial.println("Stopping - All motors disabled");
  motorsStopped = true;   // Disable omni motor control
  lifterActive = false;   // Disable lifter control
  fastRotationMode = false; // Disable fast rotation mode

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use YFROBOT library stop function for immediate stop
  motorDriver.stopMotor(MAll);  // Stop all motors immediately

  // Reset PID setpoints and acceleration tracking
  for (int i = 0; i < 4; i++) {
    setpoint[i] = 0;
    prev_setpoint[i] = 0;  // Reset acceleration tracking
    input[i] = 0;
    output[i] = 0;

    // Reset encoder smoothing data for clean restart
    smoothedRPM[i] = 0;
    lastRPM[i] = 0;
    syncError[i] = 0;
    for (int j = 0; j < RPM_FILTER_SIZE; j++) {
      rpmHistory[i][j] = 0;
    }
    rpmHistoryIndex[i] = 0;
  }
}

// Control lifter motor with limit switch safety
void liftUp() {
  if (lifterAtTop) {
    Serial.println("LFT:1");
    return;
  }

  Serial.println("LFT:U");
  clearMotorCommands();  // Clear previous commands for correct lifter operation
  lifterActive = true;
  motorsStopped = true;
  setpoint[0] = -LIFT_SPEED;
  lifterMovementStartTime = millis();
}

void liftDown() {
  if (lifterAtBottom) {
    Serial.println("LFT:2");
    return;
  }

  Serial.println("LFT:D");
  clearMotorCommands();  // Clear previous commands for correct lifter operation
  lifterActive = true;
  motorsStopped = true;
  setpoint[0] = LIFT_SPEED;
  lifterMovementStartTime = millis();
}

// Execute Pi commands from Raspberry Pi - ULTRA RESPONSIVE
void executePiCommand(String command) {
  // PRIORITY: Handle immediate stop commands first (highest priority)
  if (command == "s" || command == "v") {
    if (command == "s") stopMotors();
    else if (command == "v") emergencyStop();
    return; // Immediate return for critical commands
  }

  // Handle single-character commands from Pi with optimized processing
  if (command.length() == 1) {
    char cmd = command.charAt(0);

    // Movement commands - high priority
    // Check for direction change spam prevention
    static MovementDirection lastCommandDirection = STOPPED;
    MovementDirection newDirection = STOPPED;

    // Map command to direction for spam prevention
    switch (cmd) {
      case 'f': newDirection = FORWARD; break;
      case 'b': newDirection = BACKWARD; break;
      case 'l': newDirection = LEFT; break;
      case 'r': newDirection = RIGHT; break;
      case 'q': newDirection = FORWARD_LEFT; break;
      case 'e': newDirection = FORWARD_RIGHT; break;
      case 'z': newDirection = BACKWARD_LEFT; break;
      case 'x': newDirection = BACKWARD_RIGHT; break;
      case 'c': newDirection = ROTATE_CW; break;
      case 'w': newDirection = ROTATE_CCW; break;
      case 't': case 'y': case 'a': case 'j': newDirection = COMPLEX; break;
    }

    // Prevent direction change spam
    if (newDirection != STOPPED && newDirection != lastCommandDirection) {
      if (millis() - lastDirectionChange < DIRECTION_CHANGE_BUFFER) {
        Serial.println("DIR_CHANGE_TOO_FAST"); // Debug message
        return; // Ignore rapid direction changes
      }
      lastCommandDirection = newDirection;
    }

    switch (cmd) {
      case 'f': moveForward(); return;      // Immediate return for responsiveness
      case 'b': moveBackward(); return;
      case 'l': moveLeft(); return;
      case 'r': moveRight(); return;
      case 'q': moveForwardLeft(); return;
      case 'e': moveForwardRight(); return;
      case 'z': moveBackwardLeft(); return;
      case 'x': moveBackwardRight(); return;
      case 'c': rotateCW(); return;
      case 'w': rotateCCW(); return;
      case 't': turnLeft(); return;
      case 'y': turnRight(); return;
      case 'a': arcLeft(); return;
      case 'j': arcRight(); return;
    }

    // Control commands
    switch (cmd) {
      case 'u': liftUp(); return;
      case 'd': liftDown(); return;
      case 'p': printStatus(); return;
      case 'g': calibrateEncoders(); return;
      case 'h': figureEight(); return;
      case 'o': toggleFastRotation(); return;
      case '1': case '2': case '3': case '4':
        testMotor(cmd - '0'); return;
      case '5': case '6': case '7': case '8': case '9': case '0':
        setSpeedMultiplier(cmd); return;
      default:
        Serial.println("UNKNOWN_CMD");
        return;
    }
  } else if (command.startsWith("sr")) {
    // Sensor readings request from Pi
    transmitSensorData();
  } else if (command.startsWith("ta")) {
    // Tilt angle command: ta90
    String angleStr = command.substring(2);
    int angle = angleStr.toInt();
    if (angle >= 0 && angle <= 180) {
      setTiltAngle(angle);
      Serial.print("TILT:");
      Serial.println(angle);
    } else {
      Serial.println("INVALID_TILT_ANGLE");
    }
  } else if (command.startsWith("ga")) {
    // Gripper angle command: ga45
    String angleStr = command.substring(2);
    int angle = angleStr.toInt();
    if (angle >= 0 && angle <= 180) {
      setGripperAngle(angle);
      Serial.print("GRIP:");
      Serial.println(angle);
    } else {
      Serial.println("INVALID_GRIPPER_ANGLE");
    }
  } else if (command.startsWith("no")) {
    // Gripper open
    openGripper();
    Serial.println("GRIPPER_OPEN");
  } else if (command.startsWith("nc")) {
    // Gripper close
    closeGripper();
    Serial.println("GRIPPER_CLOSE");
  } else if (command.startsWith("nh")) {
    // Gripper half open
    halfOpenGripper();
    Serial.println("GRIPPER_HALF");
  } else if (command.startsWith("mu")) {
    // Tilt up
    tiltUp();
    Serial.println("TILT_UP");
  } else if (command.startsWith("md")) {
    // Tilt down
    tiltDown();
    Serial.println("TILT_DOWN");
  } else if (command.startsWith("mc")) {
    // Center tilt
    centerTilt();
    Serial.println("TILT_CENTER");
  } else {
    Serial.println("UNKNOWN_CMD");
  }
}

// Execute commands from serial (terminal input)
void executeCommand(String command) {

  // Handle servo angle commands (ta<angle>, ga<angle>)
  if (command.startsWith("ta") || command.startsWith("TA")) {
    // Tilt angle command: ta90
    String angleStr = command.substring(2);
    int angle = angleStr.toInt();
    if (angle >= 0 && angle <= 180) {
      setTiltAngle(angle);
    } else {
      Serial.println("Invalid tilt angle (0-180)");
    }
    return;
  } else if (command.startsWith("ga") || command.startsWith("GA")) {
    // Gripper angle command: ga45
    String angleStr = command.substring(2);
    int angle = angleStr.toInt();
    if (angle >= 0 && angle <= 180) {
      setGripperAngle(angle);
    } else {
      Serial.println("Invalid gripper angle (0-180)");
    }
    return;
  }

  if (command == "sr") {
    // Enhanced detailed sensor readings with comprehensive display
    printEnhancedSensorReadings();
    return;
  } else if (command == "sd") {
    // Disable perimeter safety system
    perimeterSafetyEnabled = false;
    Serial.println("VBD:0"); // Virtual Bumper Disabled
    Serial.println("WARNING: Robot will not automatically stop for obstacles!");
    return;
  } else if (command == "se") {
    // Enable perimeter safety system
    perimeterSafetyEnabled = true;
    Serial.println("VBE:1"); // Virtual Bumper Enabled
    return;
  } else if (command == "ls") {
    int topRaw = digitalRead(LIFTER_TOP_LIMIT_PIN);
    int bottomRaw = digitalRead(LIFTER_BOTTOM_LIMIT_PIN);
    Serial.print("LS:");
    Serial.print(topRaw);
    Serial.print(",");
    Serial.println(bottomRaw);
    return;
  } else if (command == "spe") {
    // Enable sensor publishing
    sensorPublishingEnabled = true;
    Serial.println("SPE:1"); // Sensor Publishing Enabled
    return;
  } else if (command == "spd") {
    // Disable sensor publishing
    sensorPublishingEnabled = false;
    Serial.println("SPD:0"); // Sensor Publishing Disabled
    return;
  } else if (command.startsWith("w") && command.length() >= 3) {
    // Individual wheel control: w[wheel][speed]
    // wheel: 0=Lifter, 1=FR, 2=FL, 3=Back
    // speed: -100 to 100 (maps to -1.0 to 1.0)
    char wheelChar = command.charAt(1);
    int wheelIndex = wheelChar - '0';

    if (wheelIndex >= 0 && wheelIndex <= 3) {
      String speedStr = command.substring(2);
      int speedInt = speedStr.toInt();

      // Constrain speed to -100 to 100
      speedInt = constrain(speedInt, -100, 100);

      // Convert to -1.0 to 1.0 range
      double speed = speedInt / 100.0;

      setWheelSpeed(wheelIndex, speed);
      Serial.print("WSET:");
      Serial.print(wheelIndex);
      Serial.print(",");
      Serial.println(speed);
    } else {
      Serial.println("ERROR: Invalid wheel index (0-3)");
    }
    return;
  } else if (command == "wstop") {
    // Stop all individual wheel control
    stopAllWheels();
    Serial.println("WSTOP: All wheels stopped");
    return;
  }

  // Handle single-character motor commands
  char singleCommand = command.charAt(0);
  switch (singleCommand) {
    case 'f':
    case 'F':
      moveForward();
      break;
    case 'b':
    case 'B':
      moveBackward();
      break;
    case 'l':
    case 'L':
      moveLeft();
      break;
    case 'r':
    case 'R':
      moveRight();
      break;
    // Diagonal movements
    case 'q':
    case 'Q':
      moveForwardLeft();
      break;
    case 'e':
    case 'E':
      moveForwardRight();
      break;
    case 'z':
    case 'Z':
      moveBackwardLeft();
      break;
    case 'x':
    case 'X':
      moveBackwardRight();
      break;
    // Arc movements
    case 'a':
    case 'A':
      arcLeft();
      break;
    case 'j':
    case 'J':
      arcRight();
      break;
    // Turns
    case 't':
      turnLeft();
      break;
    case 'y':
      turnRight();
      break;
    case 'c':
    case 'C':
      rotateCW();
      break;
    case 'w':
    case 'W':
      rotateCCW();
      break;
    // Speed control
    case '0':
      setSpeed(1.0);
      Serial.println("SPD:100");
      break;
    case '5':
      setSpeed(0.5);
      Serial.println("SPD:50");
      break;
    case '6':
      setSpeed(0.6);
      Serial.println("SPD:60");
      break;
    case '7':
      setSpeed(0.7);
      Serial.println("SPD:70");
      break;
    case '8':
      setSpeed(0.8);
      Serial.println("SPD:80");
      break;
    case '9':
      setSpeed(0.9);
      Serial.println("SPD:90");
      break;
    // Special commands
    case 'g':
    case 'G':
      calibrateEncoders();
      break;
    case 'h':
    case 'H':
      figureEight();
      break;
    case 'o':
    case 'O':
      // Toggle TURBO mode (permanent fast response)
      if (fastRotationMode) {
        fastRotationMode = false;
        Serial.println("TURBO:0"); // Turbo Mode Disabled
      } else {
        fastRotationMode = true;
        lastRotationCommand = millis(); // Prevent auto-disable
        Serial.println("TURBO:1"); // Turbo Mode Enabled
      }
      break;
    case 'u':
    case 'U':
      liftUp();
      break;
    case 'd':
    case 'D':
      liftDown();
      break;
    case 's':
    case 'S':
      stopMotors();
      break;
    case 'p':
    case 'P':
      printStatus();
      break;
    case '1':
    case '2':
    case '3':
    case '4':
      testMotor(singleCommand - '0');
      break;
    case 'v':
    case 'V':
      // Force stop all motors (emergency stop)
      Serial.println("EMERGENCY STOP");
      motorDriver.stopMotor(MAll);
      motorsStopped = true;
      lifterActive = false;
      fastRotationMode = false;
      // Reset all setpoints and flags
      for (int i = 0; i < 4; i++) {
        setpoint[i] = 0;
        prev_setpoint[i] = 0;
        motorIntendedActive[i] = false;
      }
      break;
    // Servo commands
    case 'm':
    case 'M':
      // Tilt servo commands (followed by second character)
      if (command.length() > 1) {
        char subCommand = command.charAt(1);
        switch (subCommand) {
          case 'u':
          case 'U':
            tiltUp();
            break;
          case 'd':
          case 'D':
            tiltDown();
            break;
          case 'c':
          case 'C':
            centerTilt();
            break;
          default:
            Serial.println("Unknown tilt cmd: mu,md,mc");
            break;
        }
      } else {
        Serial.println("Tilt cmds: mu(up),md(down),mc(center)");
      }
      break;
    case 'n':
    case 'N':
      // Gripper servo commands (followed by second character)
      if (command.length() > 1) {
        char subCommand = command.charAt(1);
        switch (subCommand) {
          case 'o':
          case 'O':
            openGripper();
            break;
          case 'c':
          case 'C':
            closeGripper();
            break;
          case 'h':
          case 'H':
            halfOpenGripper();
            break;
          default:
            Serial.println("Unknown gripper cmd: no,nc,nh");
            break;
        }
      } else {
        Serial.println("Gripper cmds: no(open),nc(close),nh(half)");
      }
      break;
    default:
      Serial.println("Unknown cmd: f,b,l,r,t,y,c,w,u,d,s,p,1-4,m,g,ir,ic,im,ih,sr,se,sd,ls,v");
      break;
  }
}

// =============== SERVO CONTROL FUNCTIONS ===============

// Set tilt servo angle (0-180 degrees)
void setTiltAngle(int angle) {
  angle = constrain(angle, TILT_SERVO_MIN_ANGLE, TILT_SERVO_MAX_ANGLE);
  Serial.print("Setting tilt servo (channel ");
  Serial.print(TILT_SERVO_CHANNEL);
  Serial.print(") to ");
  Serial.print(angle);
  Serial.println("°");
  motorDriver.servoWrite(TILT_SERVO_CHANNEL, angle);
  currentTiltAngle = angle;
  Serial.print("TILT:");
  Serial.println(angle);
}

// Set gripper servo angle (0=open, 180=closed)
void setGripperAngle(int angle) {
  angle = constrain(angle, GRIPPER_SERVO_MIN_ANGLE, GRIPPER_SERVO_MAX_ANGLE);
  Serial.print("Setting gripper servo (channel ");
  Serial.print(GRIPPER_SERVO_CHANNEL);
  Serial.print(") to ");
  Serial.print(angle);
  Serial.println("°");
  motorDriver.servoWrite(GRIPPER_SERVO_CHANNEL, angle);
  currentGripperAngle = angle;
  Serial.print("GRIP:");
  Serial.println(angle);
}

// Open gripper (set to minimum angle)
void openGripper() {
  setGripperAngle(GRIPPER_SERVO_MIN_ANGLE);
}

// Close gripper (set to maximum angle)
void closeGripper() {
  setGripperAngle(GRIPPER_SERVO_MAX_ANGLE);
}

// Tilt up (set to minimum angle)
void tiltUp() {
  setTiltAngle(TILT_SERVO_MIN_ANGLE);
}

// Tilt down (set to maximum angle)
void tiltDown() {
  setTiltAngle(TILT_SERVO_MAX_ANGLE);
}

// Center tilt (set to default position)
void centerTilt() {
  setTiltAngle(TILT_SERVO_DEFAULT);
}

// Half-open gripper (set to default position)
void halfOpenGripper() {
  setGripperAngle(GRIPPER_SERVO_DEFAULT);
}

// Emergency stop function
void emergencyStop() {
  Serial.println("EMERGENCY STOP ACTIVATED");
  motorDriver.stopMotor(MAll);
  motorsStopped = true;
  lifterActive = false;
  fastRotationMode = false;
  individualWheelControlEnabled = false;
  // Reset all setpoints
  for (int i = 0; i < 4; i++) {
    setpoint[i] = 0;
    motorIntendedActive[i] = false;
  }
}

// Individual Wheel Control Functions
void setWheelSpeed(int wheelIndex, double speed) {
  // wheelIndex: 0=Lifter, 1=FR, 2=FL, 3=Back
  // speed: -1.0 to +1.0 (negative = reverse, positive = forward)

  if (wheelIndex < 0 || wheelIndex > 3) {
    Serial.println("ERROR: Invalid wheel index");
    return;
  }

  // Clear previous commands before setting new wheel speed for correct rotation
  clearMotorCommands();

  // Enable individual wheel control mode
  individualWheelControlEnabled = true;
  motorsStopped = false;

  // Convert speed (-1.0 to 1.0) to RPM setpoint
  double rpm = speed * MAX_RPM * speedMultiplier;

  // Set the setpoint for the specific wheel
  setpoint[wheelIndex] = rpm;
  prev_setpoint[wheelIndex] = rpm;

  // Set intended active flag for this wheel
  motorIntendedActive[wheelIndex] = (abs(speed) > 0.01);

  Serial.print("Wheel ");
  Serial.print(wheelIndex);
  Serial.print(" set to ");
  Serial.println(speed);
}

void stopAllWheels() {
  Serial.println("Stopping all wheels individually");
  clearMotorCommands();  // Clear all commands for clean stop
  motorsStopped = true;
  lifterActive = false;
  individualWheelControlEnabled = false;

  for (int i = 0; i < 4; i++) {
    setpoint[i] = 0;
    motorIntendedActive[i] = false;
  }

  motorDriver.stopMotor(MAll);
}

// Toggle fast rotation mode - ULTRA RESPONSIVE
void toggleFastRotation() {
  if (fastRotationMode) {
    fastRotationMode = false;
    Serial.println("TURBO:0 - Normal mode activated");
  } else {
    fastRotationMode = true;
    lastRotationCommand = millis();
    Serial.println("TURBO:1 - ULTRA FAST MODE activated (5x responsiveness)");
    // Immediately apply fast filtering to all motors
    rpmAlpha = rpmAlphaFast;
  }
}

// Set speed multiplier
void setSpeedMultiplier(char speedCmd) {
  switch (speedCmd) {
    case '0': speedMultiplier = 1.0; Serial.println("SPD:100"); break;
    case '5': speedMultiplier = 0.75; Serial.println("SPD:75"); break;
    case '6': speedMultiplier = 0.8; Serial.println("SPD:80"); break;
    case '7': speedMultiplier = 0.85; Serial.println("SPD:85"); break;
    case '8': speedMultiplier = 0.9; Serial.println("SPD:90"); break;
    case '9': speedMultiplier = 0.95; Serial.println("SPD:95"); break;
  }
}

// Transmit sensor data to Pi
void transmitSensorData() {
  Serial.print("SENSORS:");
  // IR sensors
  Serial.print("IR_LF:"); Serial.print(irLeft1.valid ? String(irLeft1.distance, 1) : "INV"); Serial.print(",");
  Serial.print("IR_LB:"); Serial.print(irLeft2.valid ? String(irLeft2.distance, 1) : "INV"); Serial.print(",");
  Serial.print("IR_RF:"); Serial.print(irRight1.valid ? String(irRight1.distance, 1) : "INV"); Serial.print(",");
  Serial.print("IR_RB:"); Serial.print(irRight2.valid ? String(irRight2.distance, 1) : "INV"); Serial.print(",");
  Serial.print("IR_BL:"); Serial.print(irBack1.valid ? String(irBack1.distance, 1) : "INV"); Serial.print(",");
  Serial.print("IR_BR:"); Serial.print(irBack2.valid ? String(irBack2.distance, 1) : "INV"); Serial.print(",");
  // Ultrasonic sensors
  Serial.print("US_FL:"); Serial.print(ultrasonicFrontLeft.valid ? String(ultrasonicFrontLeft.distance, 1) : "INV"); Serial.print(",");
  Serial.print("US_FR:"); Serial.print(ultrasonicFrontRight.valid ? String(ultrasonicFrontRight.distance, 1) : "INV"); Serial.print(",");
  // Motor data
  Serial.print("MTR1_RPM:"); Serial.print(smoothedRPM[0], 1); Serial.print(",");
  Serial.print("MTR2_RPM:"); Serial.print(smoothedRPM[1], 1); Serial.print(",");
  Serial.print("MTR3_RPM:"); Serial.print(smoothedRPM[2], 1); Serial.print(",");
  Serial.print("MTR4_RPM:"); Serial.print(smoothedRPM[3], 1); Serial.print(",");
  // Status
  Serial.print("SYNC:"); Serial.print(synchronizationActive ? "1" : "0"); Serial.print(",");
  Serial.print("VFF:"); Serial.print(perimeterSafetyEnabled ? "1" : "0"); Serial.print(",");
  Serial.print("MOVING:"); Serial.print((!motorsStopped || lifterActive) ? "1" : "0");
  Serial.println();
}

// =============== SENSOR FUNCTIONS ===============

IRDistanceData readIRDistanceSensor(int pin) {
  IRDistanceData data;
  int rawValue = analogRead(pin);
  data.voltage = (float)rawValue * 5.0 / 1023.0;

  if (data.voltage >= IR_VOLTAGE_MIN && data.voltage <= IR_VOLTAGE_MAX) {
    // Updated formula for Sharp GP2Y0A02YK0F: distance = 1.0 / (voltage * 0.0064 + 0.00005)
    data.distance = 1.0 / (data.voltage * 0.0064 + 0.00005);
    data.valid = (data.distance >= IR_DISTANCE_MIN && data.distance <= IR_DISTANCE_MAX);
  } else {
    data.distance = 0;
    data.valid = false;
  }
  return data;
}

void updateIRDistanceSensors() {
  irLeft1 = readIRDistanceSensor(IR_LEFT_1_PIN);
  irLeft2 = readIRDistanceSensor(IR_LEFT_2_PIN);
  irRight1 = readIRDistanceSensor(IR_RIGHT_1_PIN);
  irRight2 = readIRDistanceSensor(IR_RIGHT_2_PIN);
  irBack1 = readIRDistanceSensor(IR_BACK_1_PIN);
  irBack2 = readIRDistanceSensor(IR_BACK_2_PIN);
}

UltrasonicData readUltrasonicSensor(int trigPin, int echoPin) {
  UltrasonicData data;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT);
  if (duration > 0) {
    data.duration = duration;
    data.distance = (duration * SOUND_SPEED) / 2.0;
    data.valid = true;
  } else {
    data.duration = 0;
    data.distance = 0;
    data.valid = false;
  }
  return data;
}

void updateUltrasonicSensors() {
  ultrasonicFrontLeft = readUltrasonicSensor(ULTRASONIC_FRONT_LEFT_TRIG, ULTRASONIC_FRONT_LEFT_ECHO);
  ultrasonicFrontRight = readUltrasonicSensor(ULTRASONIC_FRONT_RIGHT_TRIG, ULTRASONIC_FRONT_RIGHT_ECHO);
}

LineSensorData readLineSensor(int pin) {
  LineSensorData data;
  data.rawValue = analogRead(pin);
  data.onLine = (data.rawValue > LINE_SENSOR_THRESHOLD);
  return data;
}

void updateLineSensors() {
  lineLeft = readLineSensor(LINE_SENSOR_LEFT);
  lineCenter = readLineSensor(LINE_SENSOR_CENTER);
  lineRight = readLineSensor(LINE_SENSOR_RIGHT);
}

void updateAllSensors() {
  if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    updateIRDistanceSensors();
    updateUltrasonicSensors();
    updateLineSensors();
    updateLifterLimitSwitches();  // Update lifter limit switches
    lastSensorUpdate = millis();
  }
}

// // IMU Functions


// ROS2-style Publishing Functions
void publishSensorData() {
  static unsigned long lastPublishTime = 0;

  if (sensorPublishingEnabled && millis() - lastPublishTime >= 100) {  // Publish every 100ms if enabled
    // Publish IR distance sensors
    publishIRDistanceData();

    // Publish ultrasonic sensors
    publishUltrasonicData();

    lastPublishTime = millis();
  }
}

void publishIRDistanceData() {
  // Format: DIST:left_front:value,left_back:value,right_front:value,right_back:value,back_left:value,back_right:value
  Serial.print("DIST:");
  Serial.print(irLeft1.distance); Serial.print(",");
  Serial.print(irLeft2.distance); Serial.print(",");
  Serial.print(irRight1.distance); Serial.print(",");
  Serial.print(irRight2.distance); Serial.print(",");
  Serial.print(irBack1.distance); Serial.print(",");
  Serial.println(irBack2.distance);
}

void publishUltrasonicData() {
  // Format: US:front_left:value,front_right:value
  Serial.print("US:");
  Serial.print(ultrasonicFrontLeft.distance); Serial.print(",");
  Serial.println(ultrasonicFrontRight.distance);
}


// Update lifter limit switch states
void updateLifterLimitSwitches() {
  // Read limit switches (active LOW when pressed - normally open switches)
  int topRaw = digitalRead(LIFTER_TOP_LIMIT_PIN);
  int bottomRaw = digitalRead(LIFTER_BOTTOM_LIMIT_PIN);

  lifterAtTop = (topRaw == LOW);
  lifterAtBottom = (bottomRaw == LOW);

  // Debug output when switches change state (uncomment for troubleshooting)
  static bool lastTop = false;
  static bool lastBottom = false;
  if (lifterAtTop != lastTop || lifterAtBottom != lastBottom) {
    Serial.print("LSD:");
    Serial.print(topRaw);
    Serial.print(",");
    Serial.println(bottomRaw);
    lastTop = lifterAtTop;
    lastBottom = lifterAtBottom;
  }

  // Update safety timeout
  if (lifterActive) {
    if (lifterMovementStartTime == 0) {
      lifterMovementStartTime = millis();
      lifterSafetyTimeout = false;
    } else if (millis() - lifterMovementStartTime > LIFTER_SAFETY_TIMEOUT_MS) {
      lifterSafetyTimeout = true;
      Serial.println("LIFTER SAFETY: Movement timeout exceeded!");
    }
  } else {
    lifterMovementStartTime = 0;
    lifterSafetyTimeout = false;
  }
}

// =============== I2C BUS MANAGEMENT ===============

// Reset I2C bus to clear any conflicts from previous device initialization
void resetI2CBus() {
  Serial.println("Resetting I2C bus...");

  // Force SDA and SCL low to reset any stuck devices
  pinMode(20, OUTPUT); // SDA
  pinMode(21, OUTPUT); // SCL
  digitalWrite(20, LOW);
  digitalWrite(21, LOW);
  delay(10);

  // Release pins and reinitialize Wire
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  delay(10);

  // Reinitialize Wire library
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(100000); // Start with standard 100kHz
  delay(100);

  Serial.println("I2C bus reset complete.");
}

// Test I2C bus health
bool testI2CBus() {
  // Try to communicate with a known device
  Wire.beginTransmission(0x40); // Motor driver address
  byte motorError = Wire.endTransmission();

  Serial.print("I2C:");
  Serial.println(motorError == 0 ? "1" : "0");

  return (motorError == 0);
}

// Enhanced sensor readings with comprehensive display and visual indicators
void printEnhancedSensorReadings() {
  Serial.println("Sensors:");

  // IR sensors compact
  Serial.print("IR:L=");
  Serial.print(irLeft1.valid ? String(irLeft1.distance, 0) : "INV");
  Serial.print(",");
  Serial.print(irLeft2.valid ? String(irLeft2.distance, 0) : "INV");
  Serial.print(" R=");
  Serial.print(irRight1.valid ? String(irRight1.distance, 0) : "INV");
  Serial.print(",");
  Serial.print(irRight2.valid ? String(irRight2.distance, 0) : "INV");
  Serial.print(" B=");
  Serial.print(irBack1.valid ? String(irBack1.distance, 0) : "INV");
  Serial.print(",");
  Serial.println(irBack2.valid ? String(irBack2.distance, 0) : "INV");

  // Ultrasonic sensors compact
  Serial.print("Ultra:L=");
  Serial.print(ultrasonicFrontLeft.valid ? String(ultrasonicFrontLeft.distance, 1) : "INV");
  Serial.print(" R=");
  Serial.println(ultrasonicFrontRight.valid ? String(ultrasonicFrontRight.distance, 1) : "INV");

  // Line sensors compact
  Serial.print("Line:L=");
  Serial.print(lineLeft.onLine ? "ON" : "OFF");
  Serial.print(" C=");
  Serial.print(lineCenter.onLine ? "ON" : "OFF");
  Serial.print(" R=");
  Serial.println(lineRight.onLine ? "ON" : "OFF");

  // Safety status
  Serial.print("Safety:");
  Serial.print(getSafetyLevelString(perimeterStatus.overall));
  Serial.println(getSafetyLevelSymbol(perimeterStatus.overall));
}

// Determine which sensors to check based on movement direction
void getRelevantSensorsForDirection(MovementDirection direction, bool& checkFrontIR, bool& checkBackIR, bool& checkLeftIR, bool& checkRightIR, bool& checkUltrasonic) {
  // Default: check all sensors for safety
  checkFrontIR = true;
  checkBackIR = true;
  checkLeftIR = true;
  checkRightIR = true;
  checkUltrasonic = true;

  // For specific directions, only check relevant sensors to reduce false positives
  switch (direction) {
    case FORWARD:
      checkFrontIR = true;      // Primary: front sensors for forward movement
      checkBackIR = false;      // Don't check back sensors when moving forward
      checkLeftIR = true;       // Keep side sensors for comprehensive safety
      checkRightIR = true;
      checkUltrasonic = true;   // Front ultrasonic for forward movement
      break;

    case BACKWARD:
      checkFrontIR = false;     // Don't check front sensors when moving backward
      checkBackIR = true;       // Primary: back sensors for backward movement
      checkLeftIR = true;       // Keep side sensors
      checkRightIR = true;
      checkUltrasonic = false;  // No ultrasonic for backward (we only have front ultrasonic)
      break;

    case LEFT:
      checkFrontIR = true;      // Keep front for comprehensive safety
      checkBackIR = true;
      checkLeftIR = true;       // Primary: left sensors for left movement
      checkRightIR = false;     // Don't check right sensors when moving left
      checkUltrasonic = true;
      break;

    case RIGHT:
      checkFrontIR = true;      // Keep front for comprehensive safety
      checkBackIR = true;
      checkLeftIR = false;      // Don't check left sensors when moving right
      checkRightIR = true;      // Primary: right sensors for right movement
      checkUltrasonic = true;
      break;

    case FORWARD_LEFT:
      checkFrontIR = true;      // Primary: front sensors
      checkBackIR = false;
      checkLeftIR = true;       // Primary: left sensors
      checkRightIR = false;
      checkUltrasonic = true;
      break;

    case FORWARD_RIGHT:
      checkFrontIR = true;      // Primary: front sensors
      checkBackIR = false;
      checkLeftIR = false;
      checkRightIR = true;      // Primary: right sensors
      checkUltrasonic = true;
      break;

    case BACKWARD_LEFT:
      checkFrontIR = false;
      checkBackIR = true;       // Primary: back sensors
      checkLeftIR = true;       // Primary: left sensors
      checkRightIR = false;
      checkUltrasonic = false;
      break;

    case BACKWARD_RIGHT:
      checkFrontIR = false;
      checkBackIR = true;       // Primary: back sensors
      checkLeftIR = false;
      checkRightIR = true;      // Primary: right sensors
      checkUltrasonic = false;
      break;

    case ROTATE_CW:
    case ROTATE_CCW:
    case COMPLEX:
    case STOPPED:
      // Check all sensors for rotation or complex movements, or when stopped (for safety)
      checkFrontIR = true;
      checkBackIR = true;
      checkLeftIR = true;
      checkRightIR = true;
      checkUltrasonic = true;
      break;
  }
}

void updatePerimeterSafety() {
  if (!perimeterSafetyEnabled) return;  // Safety system disabled

  // EXCLUDE LIFTER FROM VIRTUAL SENSOR BUMPER/SAFETY LOGIC
  // Lifter operates with independent limit switch safety system only
  if (lifterActive) return;

  unsigned long currentTime = millis();
  if (currentTime - lastPerimeterCheck < PERIMETER_CHECK_INTERVAL) return;

  lastPerimeterCheck = currentTime;

  // PRIORITIZE SENSORS BASED ON MOVEMENT DIRECTION FOR SAFETY CONTROL
  perimeterStatus.emergencyBrake = false;
  SafetyLevel highestLevel = SAFE;

  // Reset all status levels
  perimeterStatus.leftIR = SAFE;
  perimeterStatus.rightIR = SAFE;
  perimeterStatus.backIR = SAFE;
  perimeterStatus.frontUltrasonic = SAFE;

  // MAIN SAFETY LOGIC: Different sensors control safety based on movement direction
  switch (currentMovementDirection) {
    case FORWARD:
      // FORWARD: Ultrasonic sensors are PRIMARY controllers, back sensors completely ignored
      {
        SafetyLevel ultrasonic_Level = SAFE;
        if (ultrasonicFrontLeft.valid) {
          float dist_mm = ultrasonicFrontLeft.distance * 10.0;
          ultrasonic_Level = max(ultrasonic_Level, (dist_mm <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                               (dist_mm <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }
        if (ultrasonicFrontRight.valid) {
          float dist_mm = ultrasonicFrontRight.distance * 10.0;
          ultrasonic_Level = max(ultrasonic_Level, (dist_mm <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                               (dist_mm <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }

        // Ultrasonic sensors are PRIMARY for forward movement
        highestLevel = max(highestLevel, ultrasonic_Level);
        perimeterStatus.frontUltrasonic = ultrasonic_Level;

        // Side sensors for additional safety (but not primary)
        if (irLeft1.valid || irLeft2.valid) {
          SafetyLevel leftLevel = SAFE;
          if (irLeft1.valid) leftLevel = max(leftLevel, (irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irLeft2.valid) leftLevel = max(leftLevel, (irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.leftIR = leftLevel;
          highestLevel = max(highestLevel, leftLevel);
        }

        if (irRight1.valid || irRight2.valid) {
          SafetyLevel rightLevel = SAFE;
          if (irRight1.valid) rightLevel = max(rightLevel, (irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irRight2.valid) rightLevel = max(rightLevel, (irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.rightIR = rightLevel;
          highestLevel = max(highestLevel, rightLevel);
        }
      }
      break;

    case BACKWARD:
      // BACKWARD: Back sensors are MAIN controllers, front sensors completely ignored
      {
        SafetyLevel backIR_Level = SAFE;
        if (irBack1.valid) {
          backIR_Level = max(backIR_Level, (irBack1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                          (irBack1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }
        if (irBack2.valid) {
          backIR_Level = max(backIR_Level, (irBack2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                          (irBack2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }

        // Back sensors are PRIMARY for backward movement
        highestLevel = max(highestLevel, backIR_Level);
        perimeterStatus.backIR = backIR_Level;

        // Side sensors for additional safety
        if (irLeft1.valid || irLeft2.valid) {
          SafetyLevel leftLevel = SAFE;
          if (irLeft1.valid) leftLevel = max(leftLevel, (irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irLeft2.valid) leftLevel = max(leftLevel, (irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.leftIR = leftLevel;
          highestLevel = max(highestLevel, leftLevel);
        }

        if (irRight1.valid || irRight2.valid) {
          SafetyLevel rightLevel = SAFE;
          if (irRight1.valid) rightLevel = max(rightLevel, (irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irRight2.valid) rightLevel = max(rightLevel, (irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.rightIR = rightLevel;
          highestLevel = max(highestLevel, rightLevel);
        }
      }
      break;

    case LEFT:
      // LEFT: Left sensors are PRIMARY, right sensors secondary
      {
        SafetyLevel leftIR_Level = SAFE;
        if (irLeft1.valid) {
          leftIR_Level = max(leftIR_Level, (irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                         (irLeft1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }
        if (irLeft2.valid) {
          leftIR_Level = max(leftIR_Level, (irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                         (irLeft2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }

        // Left sensors are PRIMARY
        highestLevel = max(highestLevel, leftIR_Level);
        perimeterStatus.leftIR = leftIR_Level;

        // Right sensors for additional safety
        if (irRight1.valid || irRight2.valid) {
          SafetyLevel rightLevel = SAFE;
          if (irRight1.valid) rightLevel = max(rightLevel, (irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irRight2.valid) rightLevel = max(rightLevel, (irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.rightIR = rightLevel;
          highestLevel = max(highestLevel, rightLevel);
        }

        // Ultrasonic sensors for comprehensive safety
        SafetyLevel ultrasonic_Level = SAFE;
        if (ultrasonicFrontLeft.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                           (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        if (ultrasonicFrontRight.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                            (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);

        highestLevel = max(highestLevel, ultrasonic_Level);
        perimeterStatus.frontUltrasonic = ultrasonic_Level;
      }
      break;

    case RIGHT:
      // RIGHT: Right sensors are PRIMARY, left sensors secondary
      {
        SafetyLevel rightIR_Level = SAFE;
        if (irRight1.valid) {
          rightIR_Level = max(rightIR_Level, (irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                           (irRight1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }
        if (irRight2.valid) {
          rightIR_Level = max(rightIR_Level, (irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                           (irRight2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        }

        // Right sensors are PRIMARY
        highestLevel = max(highestLevel, rightIR_Level);
        perimeterStatus.rightIR = rightIR_Level;

        // Left sensors for additional safety
        if (irLeft1.valid || irLeft2.valid) {
          SafetyLevel leftLevel = SAFE;
          if (irLeft1.valid) leftLevel = max(leftLevel, (irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irLeft2.valid) leftLevel = max(leftLevel, (irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.leftIR = leftLevel;
          highestLevel = max(highestLevel, leftLevel);
        }

        // Ultrasonic sensors for comprehensive safety
        SafetyLevel ultrasonic_Level = SAFE;
        if (ultrasonicFrontLeft.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                           (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
        if (ultrasonicFrontRight.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                            (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);

        highestLevel = max(highestLevel, ultrasonic_Level);
        perimeterStatus.frontUltrasonic = ultrasonic_Level;
      }
      break;

    default:
      // For all other directions (diagonal, rotation, complex, stopped): Check all sensors
      {
        // Check all IR sensors
        if (irLeft1.valid || irLeft2.valid) {
          SafetyLevel leftLevel = SAFE;
          if (irLeft1.valid) leftLevel = max(leftLevel, (irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irLeft2.valid) leftLevel = max(leftLevel, (irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                   (irLeft2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.leftIR = leftLevel;
          highestLevel = max(highestLevel, leftLevel);
        }

        if (irRight1.valid || irRight2.valid) {
          SafetyLevel rightLevel = SAFE;
          if (irRight1.valid) rightLevel = max(rightLevel, (irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irRight2.valid) rightLevel = max(rightLevel, (irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                    (irRight2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.rightIR = rightLevel;
          highestLevel = max(highestLevel, rightLevel);
        }


        if (irBack1.valid || irBack2.valid) {
          SafetyLevel backIR_Level = SAFE;
          if (irBack1.valid) backIR_Level = max(backIR_Level, (irBack1.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                          (irBack1.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (irBack2.valid) backIR_Level = max(backIR_Level, (irBack2.distance <= IR_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                          (irBack2.distance <= IR_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.backIR = backIR_Level;
          highestLevel = max(highestLevel, backIR_Level);
        }

        if (ultrasonicFrontLeft.valid || ultrasonicFrontRight.valid) {
          SafetyLevel ultrasonic_Level = SAFE;
          if (ultrasonicFrontLeft.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                             (ultrasonicFrontLeft.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          if (ultrasonicFrontRight.valid) ultrasonic_Level = max(ultrasonic_Level, (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL) ? CRITICAL :
                                                              (ultrasonicFrontRight.distance * 10.0 <= ULTRASONIC_SAFETY_DISTANCE_WARNING) ? WARNING : SAFE);
          perimeterStatus.frontUltrasonic = ultrasonic_Level;
          highestLevel = max(highestLevel, ultrasonic_Level);
        }
      }
      break;
  }

  perimeterStatus.overall = highestLevel;

  // Handle automatic emergency braking - modified for potential field method
  if (highestLevel == CRITICAL) {
    if (CURRENT_BUMPER_MODE == POTENTIAL_FIELD) {
      // In potential field mode, only trigger emergency brake for extreme situations
      // Allow the force field to handle most obstacle avoidance
      bool extremeSituation = false;

      // Check for very close obstacles (within 50% of critical distance)
      if (perimeterStatus.frontUltrasonic == CRITICAL) {
        extremeSituation = true;
      }
      if (perimeterStatus.leftIR == CRITICAL || perimeterStatus.rightIR == CRITICAL) {
        extremeSituation = true;
      }
      if (perimeterStatus.backIR == CRITICAL) {
        extremeSituation = true;
      }

      if (extremeSituation && (currentTime - lastAutoBrake > BRAKE_COOLDOWN_MS)) {
        Serial.println("POTENTIAL FIELD: Emergency brake triggered for extreme proximity!");
        triggerEmergencyBrake();
        lastAutoBrake = currentTime;
        perimeterStatus.emergencyBrake = true;
        perimeterStatus.lastObstacleTime = currentTime;
      }
    } else {
      // Legacy binary mode - trigger brake on any CRITICAL reading
      if (currentTime - lastAutoBrake > BRAKE_COOLDOWN_MS) {
        triggerEmergencyBrake();
        lastAutoBrake = currentTime;
        perimeterStatus.emergencyBrake = true;
        perimeterStatus.lastObstacleTime = currentTime;
      }
    }
  }
}

// Trigger emergency brake - automatic safety system
void triggerEmergencyBrake() {
  Serial.println("\nEMERGENCY BRAKE ACTIVATED - OBSTACLE DETECTED! ***");

  // Show which sensors triggered the brake (any sensor at CRITICAL level)
  Serial.println("Triggering sensors (CRITICAL zone):");

  // Check IR sensors
  if (perimeterStatus.leftIR == CRITICAL) {
    Serial.println("  - LEFT IR sensors (critical distance)");
    if (irLeft1.valid && irLeft1.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Left 1: " + String(irLeft1.distance) + "mm");
    if (irLeft2.valid && irLeft2.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Left 2: " + String(irLeft2.distance) + "mm");
  }

  if (perimeterStatus.rightIR == CRITICAL) {
    Serial.println("  - RIGHT IR sensors (critical distance)");
    if (irRight1.valid && irRight1.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Right 1: " + String(irRight1.distance) + "mm");
    if (irRight2.valid && irRight2.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Right 2: " + String(irRight2.distance) + "mm");
  }

  if (perimeterStatus.backIR == CRITICAL) {
    Serial.println("  - BACK IR sensors (critical distance)");
    if (irBack1.valid && irBack1.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Back 1: " + String(irBack1.distance) + "mm");
    if (irBack2.valid && irBack2.distance <= IR_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * IR Back 2: " + String(irBack2.distance) + "mm");
  }

  // Check ultrasonic sensors
  if (perimeterStatus.frontUltrasonic == CRITICAL) {
    Serial.println("  - FRONT Ultrasonic sensors (critical distance)");
    if (ultrasonicFrontLeft.valid && ultrasonicFrontLeft.distance * 10 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * Ultrasonic Left: " + String(ultrasonicFrontLeft.distance) + "cm");
    if (ultrasonicFrontRight.valid && ultrasonicFrontRight.distance * 10 <= ULTRASONIC_SAFETY_DISTANCE_CRITICAL)
      Serial.println("    * Ultrasonic Right: " + String(ultrasonicFrontRight.distance) + "cm");
  }

  // Immediate emergency stop
  motorsStopped = true;
  lifterActive = false;
  fastRotationMode = false;

  // Stop all motors immediately
  motorDriver.stopMotor(MAll);

  // Reset all setpoints and motor states
  for (int i = 0; i < 4; i++) {
    setpoint[i] = 0;
    prev_setpoint[i] = 0;
    motorIntendedActive[i] = false;
    smoothedRPM[i] = 0;
    lastRPM[i] = 0;
    syncError[i] = 0;
  }

  // Apply emergency deceleration to any active movement
  speedMultiplier *= EMERGENCY_DECELERATION;

  Serial.print("Emergency deceleration applied: ");
  Serial.print(speedMultiplier * 100, 1);
  Serial.println("%");
  Serial.println("Use 's' to resume normal operation");
  Serial.println("Virtual bumper system prevented collision!");
}

// ============================================================================
// VIRTUAL FORCE FIELD FUNCTIONS (Artificial Potential Field Method)
// NOTE: Virtual force field only affects omni motors (1-3), NOT the lifter (0)
// Lifter operates independently with its own limit switch safety system
// LIFTER IS COMPLETELY EXCLUDED FROM VIRTUAL SENSOR BUMPER/SAFETY LOGIC
// ============================================================================

// Calculate repulsive force for a single sensor using Artificial Potential Field
VirtualForce calculateRepulsiveForce(float distance, float influenceDistance, float scalingFactor, float sensorAngle) {
  VirtualForce force = {0, 0, 0};

  if (distance >= influenceDistance) {
    // No force if beyond influence distance
    return force;
  }

  // Calculate repulsive force magnitude using improved potential field formula
  // F_rep = η * (1/ρ - 1/ρ0)^2 * (1/ρ^2) where ρ is distance, ρ0 is influence distance
  float rho0 = influenceDistance;
  float rho = max(distance, 1.0); // Prevent division by zero

  // More aggressive force calculation for closer obstacles
  float distanceRatio = (rho0 - rho) / rho0; // 0 when far, 1 when very close
  float repulsiveMagnitude = scalingFactor * pow(distanceRatio, 1.5) * (1.0 / (rho * rho / 10000.0)); // Normalized

  // Exponential increase when very close to obstacles (within 25% of influence distance)
  if (distanceRatio > 0.75) {
    repulsiveMagnitude *= 3.0; // Triple force for very close obstacles
  }

  // Limit maximum force
  repulsiveMagnitude = min(repulsiveMagnitude, APF_MAX_FORCE);

  // Calculate force direction (perpendicular to sensor direction, pushing away from obstacle)
  // For a sensor at angle θ, the repulsive force should push perpendicular to the obstacle surface
  float forceAngle = sensorAngle + PI/2; // Perpendicular to sensor direction

  force.x = repulsiveMagnitude * cos(forceAngle);
  force.y = repulsiveMagnitude * sin(forceAngle);
  force.magnitude = repulsiveMagnitude;

  return force;
}

// Update virtual force field status for all sensors
void updateVirtualForceField() {
  if (!perimeterSafetyEnabled) {
    virtualBumperStatus.forceFieldActive = false;
    return;
  }

  // Reset total force
  virtualBumperStatus.totalForce = {0, 0, 0};
  virtualBumperStatus.maxForceMagnitude = 0.0;
  virtualBumperStatus.forceFieldActive = false;

  // Update IR sensors
  // Left sensors (oriented at 90 degrees - pushing right when obstacle detected)
  if (irLeft1.valid) {
    virtualBumperStatus.irLeft1.distance = irLeft1.distance;
    virtualBumperStatus.irLeft1.valid = true;
    virtualBumperStatus.irLeft1.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irLeft1.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irLeft1.repulsiveForce = calculateRepulsiveForce(
      irLeft1.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, PI/2); // 90°
  } else {
    virtualBumperStatus.irLeft1 = {0};
  }

  if (irLeft2.valid) {
    virtualBumperStatus.irLeft2.distance = irLeft2.distance;
    virtualBumperStatus.irLeft2.valid = true;
    virtualBumperStatus.irLeft2.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irLeft2.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irLeft2.repulsiveForce = calculateRepulsiveForce(
      irLeft2.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, PI/2); // 90°
  } else {
    virtualBumperStatus.irLeft2 = {0};
  }

  // Right sensors (oriented at -90 degrees - pushing left when obstacle detected)
  if (irRight1.valid) {
    virtualBumperStatus.irRight1.distance = irRight1.distance;
    virtualBumperStatus.irRight1.valid = true;
    virtualBumperStatus.irRight1.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irRight1.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irRight1.repulsiveForce = calculateRepulsiveForce(
      irRight1.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, -PI/2); // -90°
  } else {
    virtualBumperStatus.irRight1 = {0};
  }

  if (irRight2.valid) {
    virtualBumperStatus.irRight2.distance = irRight2.distance;
    virtualBumperStatus.irRight2.valid = true;
    virtualBumperStatus.irRight2.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irRight2.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irRight2.repulsiveForce = calculateRepulsiveForce(
      irRight2.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, -PI/2); // -90°
  } else {
    virtualBumperStatus.irRight2 = {0};
  }

  // Back sensors (oriented at 180 degrees - pushing forward when obstacle detected)
  if (irBack1.valid) {
    virtualBumperStatus.irBack1.distance = irBack1.distance;
    virtualBumperStatus.irBack1.valid = true;
    virtualBumperStatus.irBack1.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irBack1.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irBack1.repulsiveForce = calculateRepulsiveForce(
      irBack1.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, PI); // 180°
  } else {
    virtualBumperStatus.irBack1 = {0};
  }

  if (irBack2.valid) {
    virtualBumperStatus.irBack2.distance = irBack2.distance;
    virtualBumperStatus.irBack2.valid = true;
    virtualBumperStatus.irBack2.influenceDistance = APF_INFLUENCE_DISTANCE_IR;
    virtualBumperStatus.irBack2.scalingFactor = APF_SCALING_FACTOR_IR;
    virtualBumperStatus.irBack2.repulsiveForce = calculateRepulsiveForce(
      irBack2.distance, APF_INFLUENCE_DISTANCE_IR, APF_SCALING_FACTOR_IR, PI); // 180°
  } else {
    virtualBumperStatus.irBack2 = {0};
  }

  // Ultrasonic sensors (oriented at 0 degrees - pushing backward when obstacle detected)
  if (ultrasonicFrontLeft.valid) {
    virtualBumperStatus.ultrasonicFrontLeft.distance = ultrasonicFrontLeft.distance * 10.0; // Convert to mm
    virtualBumperStatus.ultrasonicFrontLeft.valid = true;
    virtualBumperStatus.ultrasonicFrontLeft.influenceDistance = APF_INFLUENCE_DISTANCE_ULTRASONIC;
    virtualBumperStatus.ultrasonicFrontLeft.scalingFactor = APF_SCALING_FACTOR_ULTRASONIC;
    virtualBumperStatus.ultrasonicFrontLeft.repulsiveForce = calculateRepulsiveForce(
      ultrasonicFrontLeft.distance * 10.0, APF_INFLUENCE_DISTANCE_ULTRASONIC, APF_SCALING_FACTOR_ULTRASONIC, 0); // 0°
  } else {
    virtualBumperStatus.ultrasonicFrontLeft = {0};
  }

  if (ultrasonicFrontRight.valid) {
    virtualBumperStatus.ultrasonicFrontRight.distance = ultrasonicFrontRight.distance * 10.0; // Convert to mm
    virtualBumperStatus.ultrasonicFrontRight.valid = true;
    virtualBumperStatus.ultrasonicFrontRight.influenceDistance = APF_INFLUENCE_DISTANCE_ULTRASONIC;
    virtualBumperStatus.ultrasonicFrontRight.scalingFactor = APF_SCALING_FACTOR_ULTRASONIC;
    virtualBumperStatus.ultrasonicFrontRight.repulsiveForce = calculateRepulsiveForce(
      ultrasonicFrontRight.distance * 10.0, APF_INFLUENCE_DISTANCE_ULTRASONIC, APF_SCALING_FACTOR_ULTRASONIC, 0); // 0°
  } else {
    virtualBumperStatus.ultrasonicFrontRight = {0};
  }

  // Sum all repulsive forces
  SensorForce* sensors[] = {
    &virtualBumperStatus.irLeft1, &virtualBumperStatus.irLeft2,
    &virtualBumperStatus.irRight1, &virtualBumperStatus.irRight2,
    &virtualBumperStatus.irBack1, &virtualBumperStatus.irBack2,
    &virtualBumperStatus.ultrasonicFrontLeft, &virtualBumperStatus.ultrasonicFrontRight
  };

  for (int i = 0; i < 8; i++) {
    if (sensors[i]->valid) {
      virtualBumperStatus.totalForce.x += sensors[i]->repulsiveForce.x;
      virtualBumperStatus.totalForce.y += sensors[i]->repulsiveForce.y;
      virtualBumperStatus.maxForceMagnitude = max(virtualBumperStatus.maxForceMagnitude,
                                                  sensors[i]->repulsiveForce.magnitude);
      virtualBumperStatus.forceFieldActive = true;
    }
  }

  // Calculate total force magnitude
  virtualBumperStatus.totalForce.magnitude = sqrt(
    virtualBumperStatus.totalForce.x * virtualBumperStatus.totalForce.x +
    virtualBumperStatus.totalForce.y * virtualBumperStatus.totalForce.y);
}

// Apply virtual force field to desired velocities using inverse kinematics
void applyVirtualForceField(double& vx_desired, double& vy_desired, double& omega_desired) {
  if (CURRENT_BUMPER_MODE != POTENTIAL_FIELD) {
    return;
  }

  // Always apply forces if any sensor detects obstacles (don't require forceFieldActive)
  if (virtualBumperStatus.totalForce.magnitude < 0.001) {
    return; // No significant forces to apply
  }

  // Use fixed scaling based on maximum speed for consistent avoidance behavior
  // This ensures strong avoidance even when robot is moving slowly or stopped
  float forceScale = 1.0; // Full force application

  // Apply repulsive forces to desired velocities
  // The force field pushes the robot away from obstacles
  vx_desired += virtualBumperStatus.totalForce.x * forceScale;
  vy_desired += virtualBumperStatus.totalForce.y * forceScale;

  // Allow higher velocity changes for effective obstacle avoidance
  float maxVelocityChange = 1.5; // Allow up to 150% of normal speed for avoidance
  vx_desired = constrain(vx_desired, -maxVelocityChange, maxVelocityChange);
  vy_desired = constrain(vy_desired, -maxVelocityChange, maxVelocityChange);
  omega_desired = constrain(omega_desired, -1.0, 1.0); // Rotation unchanged

  Serial.print("VFF:");
  Serial.print(virtualBumperStatus.totalForce.x, 2);
  Serial.print(",");
  Serial.print(virtualBumperStatus.totalForce.y, 2);
  Serial.print(",");
  Serial.print(virtualBumperStatus.totalForce.magnitude, 2);
  Serial.print("|");
  Serial.print(vx_desired, 2);
  Serial.print(",");
  Serial.println(vy_desired, 2);
}

// Get safety level as string for display
String getSafetyLevelString(SafetyLevel level) {
  switch (level) {
    case SAFE: return "SAFE";
    case WARNING: return "WARNING";
    case CRITICAL: return "CRITICAL";
    default: return "UNKNOWN";
  }
}

// Get safety level color symbol
char getSafetyLevelSymbol(SafetyLevel level) {
  switch (level) {
    case SAFE: return '+';
    case WARNING: return '!';
    case CRITICAL: return 'X';
    default: return '?';
  }
}

// Get movement direction as string for display
String getMovementDirectionString(MovementDirection direction) {
  switch (direction) {
    case STOPPED: return "STOPPED";
    case FORWARD: return "FORWARD";
    case BACKWARD: return "BACKWARD";
    case LEFT: return "LEFT";
    case RIGHT: return "RIGHT";
    case FORWARD_LEFT: return "FORWARD-LEFT";
    case FORWARD_RIGHT: return "FORWARD-RIGHT";
    case BACKWARD_LEFT: return "BACKWARD-LEFT";
    case BACKWARD_RIGHT: return "BACKWARD-RIGHT";
    case ROTATE_CW: return "ROTATE CW";
    case ROTATE_CCW: return "ROTATE CCW";
    case COMPLEX: return "COMPLEX";
    default: return "UNKNOWN";
  }
}

// Print enhanced status with encoder data and synchronization
void printStatus() {
  Serial.println("=== Enhanced Motor Status ===");
  Serial.print("Omni Motors Stopped: ");
  Serial.println(motorsStopped ? "YES" : "NO");
  Serial.print("Lifter Active: ");
  Serial.println(lifterActive ? "YES" : "NO");
  Serial.print("Lifter Status: Top=");
  Serial.print(lifterAtTop ? "HIT" : "OK");
  Serial.print(" Bottom=");
  Serial.print(lifterAtBottom ? "HIT" : "OK");
  Serial.print(" Timeout=");
  Serial.println(lifterSafetyTimeout ? "YES" : "NO");
  Serial.print("Servo Status: Tilt=");
  Serial.print(currentTiltAngle);
  Serial.print("° Gripper=");
  Serial.print(currentGripperAngle);
  Serial.println("°");
  Serial.print("Synchronization: ");
  Serial.print(synchronizationActive ? "1" : "0");
  if (synchronizationActive) {
    Serial.print(" (Motors: ");
    for (int i = 1; i < 4; i++) {
      if (motorIntendedActive[i]) {
        Serial.print(i);
        Serial.print(" ");
      }
    }
    Serial.print("active)");
  }
  Serial.print(" | FastMode: ");
  Serial.print(fastRotationMode ? "ON" : "OFF");
  Serial.print(" | ForceStop: ");
  bool anyForceStop = false;
  for (int i = 1; i < 4; i++) {
    if (!motorIntendedActive[i] && !motorsStopped) {
      Serial.print(i);
      Serial.print(" ");
      anyForceStop = true;
    }
  }
  if (!anyForceStop) Serial.print("none");
  Serial.println("");
  Serial.print("STS:");
  Serial.print(synchronizationActive ? "1" : "0");
  Serial.print(",");
  Serial.print(perimeterSafetyEnabled ? "1" : "0");
  Serial.print(",");
  Serial.println((!motorsStopped || lifterActive) ? "1" : "0");


  // Virtual Bumper Status
  Serial.print("VB:");
  Serial.print(perimeterSafetyEnabled ? "1" : "0");
  Serial.print(" | Direction: ");
  Serial.print(getMovementDirectionString(currentMovementDirection));
  Serial.print(" | Safety: ");
  Serial.print(getSafetyLevelString(perimeterStatus.overall));
  Serial.print(" ");
  Serial.print(getSafetyLevelSymbol(perimeterStatus.overall));
  if (perimeterStatus.emergencyBrake) {
    Serial.print(" [BRAKE ACTIVE]");
  } else if (perimeterStatus.lastObstacleTime > 0) {
    unsigned long timeAgo = millis() - perimeterStatus.lastObstacleTime;
    Serial.print(" [Last brake: ");
    Serial.print(timeAgo / 1000.0, 1);
    Serial.print("s ago]");
  }
  Serial.println("");

  // Sensor Status with PRIORITY levels based on movement direction
  Serial.println("=== Sensor Priority Status ===");

  // Show movement direction and priority explanation
  String directionMsg;
  switch (currentMovementDirection) {
    case FORWARD:
      directionMsg = "FORWARD: Ultrasonic PRIMARY, Back IGNORED";
      break;
    case BACKWARD:
      directionMsg = "BACKWARD: Back sensors PRIMARY, Front IGNORED";
      break;
    case LEFT:
      directionMsg = "LEFT: Left PRIMARY, Right SECONDARY";
      break;
    case RIGHT:
      directionMsg = "RIGHT: Right PRIMARY, Left SECONDARY";
      break;
    case FORWARD_LEFT:
      directionMsg = "FORWARD-LEFT: Front+Left PRIMARY";
      break;
    case FORWARD_RIGHT:
      directionMsg = "FORWARD-RIGHT: Front+Right PRIMARY";
      break;
    case BACKWARD_LEFT:
      directionMsg = "BACKWARD-LEFT: Back+Left PRIMARY";
      break;
    case BACKWARD_RIGHT:
      directionMsg = "BACKWARD-RIGHT: Back+Right PRIMARY";
      break;
    default:
      directionMsg = "COMPLEX/ROTATION: All sensors equal priority";
      break;
  }
  Serial.println(directionMsg);

  // IR Distance Sensors with PRIORITY display
  Serial.print("IR(mm): L=");
  Serial.print(irLeft1.valid ? String(irLeft1.distance, 0) : "INV");
  Serial.print(",");
  Serial.print(irLeft2.valid ? String(irLeft2.distance, 0) : "INV");

  // Determine priority for LEFT sensors
  if (currentMovementDirection == LEFT || currentMovementDirection == FORWARD_LEFT || currentMovementDirection == BACKWARD_LEFT) {
    Serial.println(" [PRIMARY]");
  } else if (currentMovementDirection == FORWARD || currentMovementDirection == BACKWARD || currentMovementDirection == RIGHT ||
             currentMovementDirection == FORWARD_RIGHT || currentMovementDirection == BACKWARD_RIGHT) {
    Serial.println(" [SECONDARY]");
  } else {
    Serial.println(" [MONITORED]");
  }

  Serial.print("R=");
  Serial.print(irRight1.valid ? String(irRight1.distance, 0) : "INV");
  Serial.print(",");
  Serial.print(irRight2.valid ? String(irRight2.distance, 0) : "INV");

  // Determine priority for RIGHT sensors
  if (currentMovementDirection == RIGHT || currentMovementDirection == FORWARD_RIGHT || currentMovementDirection == BACKWARD_RIGHT) {
    Serial.println(" [PRIMARY]");
  } else if (currentMovementDirection == FORWARD || currentMovementDirection == BACKWARD || currentMovementDirection == LEFT ||
             currentMovementDirection == FORWARD_LEFT || currentMovementDirection == BACKWARD_LEFT) {
    Serial.println(" [SECONDARY]");
  } else {
    Serial.println(" [MONITORED]");
  }

  Serial.print("B=");
  Serial.print(irBack1.valid ? String(irBack1.distance, 0) : "INV");
  Serial.print(",");
  Serial.print(irBack2.valid ? String(irBack2.distance, 0) : "INV");

  // Determine priority for BACK sensors
  if (currentMovementDirection == BACKWARD || currentMovementDirection == BACKWARD_LEFT || currentMovementDirection == BACKWARD_RIGHT) {
    Serial.println(" [PRIMARY]");
  } else if (currentMovementDirection == FORWARD || currentMovementDirection == FORWARD_LEFT || currentMovementDirection == FORWARD_RIGHT ||
             currentMovementDirection == LEFT || currentMovementDirection == RIGHT) {
    Serial.println(" [IGNORED]");
  } else {
    Serial.println(" [MONITORED]");
  }

  // Ultrasonic Sensors with PRIORITY display
  Serial.print("Ultra(cm): L=");
  Serial.print(ultrasonicFrontLeft.valid ? String(ultrasonicFrontLeft.distance, 1) : "INV");
  Serial.print(" R=");
  Serial.print(ultrasonicFrontRight.valid ? String(ultrasonicFrontRight.distance, 1) : "INV");

  // Determine priority for ULTRASONIC sensors
  if (currentMovementDirection == FORWARD || currentMovementDirection == FORWARD_LEFT || currentMovementDirection == FORWARD_RIGHT) {
    Serial.println(" [PRIMARY]");
  } else if (currentMovementDirection == LEFT || currentMovementDirection == RIGHT) {
    Serial.println(" [SECONDARY]");
  } else if (currentMovementDirection == BACKWARD || currentMovementDirection == BACKWARD_LEFT || currentMovementDirection == BACKWARD_RIGHT) {
    Serial.println(" [IGNORED]");
  } else {
    Serial.println(" [MONITORED]");
  }

  // Line Sensors
  Serial.println("Line Sensors:");
  Serial.print("  Left: ");
  Serial.print(lineLeft.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineLeft.rawValue);
  Serial.print(") | Center: ");
  Serial.print(lineCenter.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineCenter.rawValue);
  Serial.print(") | Right: ");
  Serial.print(lineRight.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineRight.rawValue);
  Serial.println(")");
  Serial.println("==================");

  if (synchronizationActive) {
    Serial.print("SYNC:");
    Serial.print(targetSyncRPM, 1);
    Serial.print(",");
    Serial.println(activeMotorCount);
  }

  for (int i = 0; i < 4; i++) {
    Serial.print("MTR:");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.print(setpoint[i], 1);
    Serial.print(",");
    Serial.print(smoothedRPM[i], 1);
    Serial.print(",");
    Serial.print(syncError[i], 1);
    Serial.print(",");
    Serial.print(motorPosition[i], 2);
    Serial.print(",");
    Serial.println(output[i], 1);
  }
  Serial.println("==================");
}

void setSpeed(float multiplier) {
  speedMultiplier = constrain(multiplier, 0.1, 1.0);
}

// Calibration sequence
void calibrateMotors() {
  Serial.println("Starting motor calibration...");

  // Test each motor individually
  for (int i = 1; i <= 4; i++) {
    Serial.print("Calibrating Motor ");
    Serial.println(i);
    testMotor(i);
    delay(1000);
  }

  // Test basic movements
  Serial.println("Testing forward movement...");
  moveForward();
  delay(2000);
  stopMotors();

  Serial.println("Testing rotation...");
  rotateCW();
  delay(3000);
  stopMotors();

  Serial.println("Calibration complete!");
}

// Encoder calibration function
void calibrateEncoders() {
  Serial.println("Calibrating encoders and resetting position tracking...");

  // Reset all encoder tracking data
  for (int i = 0; i < 4; i++) {
    encoderCount[i] = 0;
    lastEncoderCount[i] = 0;
    totalEncoderCount[i] = 0;
    motorPosition[i] = 0;
    smoothedRPM[i] = 0;
    filteredRPM[i] = 0;
    lastRPM[i] = 0;
    syncError[i] = 0;

    // Reset history buffers
    for (int j = 0; j < RPM_FILTER_SIZE; j++) {
      rpmHistory[i][j] = 0;
    }
    rpmHistoryIndex[i] = 0;
  }

  // Reset synchronization data
  targetSyncRPM = 0;
  synchronizationActive = false;
  activeMotorCount = 0;

  Serial.println("Encoder calibration complete - all positions and sync data reset to zero");
}

// Figure-8 pattern for testing
void figureEight() {
  Serial.println("Starting Figure-8 pattern with smooth encoder-controlled movements...");

  // First loop - right turn with smooth transitions
  Serial.println("Right loop...");
  setOmniSpeeds(0.6, 0.0, 0.8);  // Forward + right turn
  delay(3000);

  // Smooth transition
  stopMotors();
  delay(1000);  // Longer pause for encoder stabilization

  // Second loop - left turn
  Serial.println("Left loop...");
  setOmniSpeeds(0.6, 0.0, -0.8); // Forward + left turn
  delay(3000);

  stopMotors();
  Serial.println("Figure-8 complete - encoder data stabilized!");
}

// Test individual motor at FULL POWER
void testMotor(int motorIndex) {
  Serial.print("FULL POWER Test - Motor ");
  Serial.print(motorIndex);

  bool wasStopped = motorsStopped;    // Remember previous omni state
  bool wasLifterActive = lifterActive; // Remember previous lifter state

  if (motorIndex == 1) {
    // Special lifter test with limit switch safety
    Serial.println(" (Lifter) - WITH LIMIT SWITCH SAFETY");
    int topRaw = digitalRead(LIFTER_TOP_LIMIT_PIN);
    int bottomRaw = digitalRead(LIFTER_BOTTOM_LIMIT_PIN);
    Serial.print("LS:");
    Serial.print(topRaw);
    Serial.print(",");
    Serial.println(bottomRaw);
    Serial.println("LFT:T");

    if (!lifterAtTop) {
      Serial.println("LFT:UT");
      lifterActive = true;
      motorsStopped = true;
      setpoint[0] = -LIFT_SPEED;
      lifterMovementStartTime = millis();

      // Let the main PID loop handle control for 2 seconds
      unsigned long testStart = millis();
      while (millis() - testStart < 2000 && lifterActive) {
        updateLifterLimitSwitches();
        delay(20); // Give main loop time to run
      }

      Serial.println("LFT:UC");
    } else {
      Serial.println("LFT:1");
    }

    lifterActive = false;
    setpoint[0] = 0;
    delay(1000);

    if (!lifterAtBottom) {
      Serial.println("LFT:DT");
      lifterActive = true;
      motorsStopped = true;
      setpoint[0] = LIFT_SPEED;
      lifterMovementStartTime = millis();

      // Let the main PID loop handle control for 2 seconds
      unsigned long testStart = millis();
      while (millis() - testStart < 2000 && lifterActive) {
        updateLifterLimitSwitches();
        delay(20); // Give main loop time to run
      }

      Serial.println("LFT:DC");
    } else {
      Serial.println("LFT:2");
    }

    lifterActive = false;
    setpoint[0] = 0;

    Serial.println("LFT:TC");

  } else {
    // Regular motor test for omni wheels
    switch(motorIndex) {
      case 2: Serial.println(" (Front Right)"); break;
      case 3: Serial.println(" (Front Left)"); break;
      case 4: Serial.println(" (Back)"); break;
    }
    Serial.println("Power level: 4000/4096 (97% max) - 3 seconds each direction");

    // Temporarily disable all automatic control for testing
    motorsStopped = true;
    lifterActive = false;

    // Test forward - FULL POWER
    Serial.println("Forward (FULL POWER)...");
    motorDriver.setSingleMotor(motorIndex, 4000);  // Near maximum speed
    delay(3000);

    // Stop
    motorDriver.stopMotor(motorIndex);
    delay(1000);

    // Test backward - FULL POWER
    Serial.println("Backward (FULL POWER)...");
    motorDriver.setSingleMotor(motorIndex, -4000); // Near maximum speed reverse
    delay(3000);

    // Stop
    motorDriver.stopMotor(motorIndex);
  }

  // Restore previous states
  motorsStopped = wasStopped;
  lifterActive = wasLifterActive;

  Serial.println("Test complete");
}
