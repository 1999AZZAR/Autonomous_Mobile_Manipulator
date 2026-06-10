// Configuration file for Hexagonal 3-Wheel Omni Robot Control
// Contains all configurable constants and PID tunings

#ifndef CONFIG_H
#define CONFIG_H

// === ENCODER CONFIGURATION ===
#define RPM_FILTER_SIZE 10                   // Number of samples for RPM filtering (increased for stability)
const int ENCODER_CPR = 28;                  // Counts per revolution (for PG28 motors)
const double GEAR_RATIO = 1.0;              // Gear ratio (adjust if geared motors)

// === MOTOR SPEED LIMITS ===
const double MAX_RPM = 300.0;               // Maximum RPM for motors - increased 3x for more torque
const double BASE_SPEED = 200.0;            // Base speed for movements - increased 2.5x for more power
const double TURN_SPEED = 100.0;            // Speed for turning movements - increased 3.3x for better turning
const double LIFT_SPEED = 150.0;            // Speed for lifter motor - increased 3x for lifting power

// === PID CONFIGURATION ===
const double PID_SAMPLE_TIME = 10;          // PID sample time in ms (reduced from 20ms for 2x faster response)
const int PID_OUTPUT_LIMIT_MIN = -2048;     // PID output minimum (PWM) - increased 8x for maximum torque
const int PID_OUTPUT_LIMIT_MAX = 2048;      // PID output maximum (PWM) - increased 8x for maximum torque

// PID Tunings (Kp, Ki, Kd) for each motor - optimized for responsiveness
const double Lifter_Kp = 3.0;               // Lifter PID - more responsive (was 2.0)
const double Lifter_Ki = 1.2;               // Higher integral for steady state (was 0.5)
const double Lifter_Kd = 0.05;              // Reduced derivative for less oscillation (was 0.1)

const double Omni_Kp = 4.0;                  // Omni motors - tuned for high power stability
const double Omni_Ki = 1.5;                 // Reduced integral to prevent windup with high power
const double Omni_Kd = 0.1;                  // Increased damping for smooth multi-motor coordination

// === MOTOR SYNCHRONIZATION ===
const double SYNC_KP = 0.05;                // Synchronization PID gain (much lower for responsiveness)

// === ACCELERATION LIMITING ===
const double MAX_RPM_CHANGE = 80.0;         // Maximum RPM change per sample (reduced for smoother coordination)
const double MAX_RPM_CHANGE_ROTATION = 150.0; // Moderate changes for rotation (prevent oscillations)

// === FILTERING AND SMOOTHING ===
const double RPM_ALPHA = 0.4;               // Exponential smoothing factor (increased filtering for stability)
const double RPM_ALPHA_FAST = 0.6;          // Fast smoothing during rotation (more stable)

// === MOTOR ANGLES (degrees) ===
// Triangular 3-wheel omni robot configuration
const double MOTOR_ANGLES[4] = {0, 45, 135, 180};  // Lifter, FR, FL, Back

// === ROBOT GEOMETRY ===
// Distance from robot center to wheel contact point (normalized to 1.0 for simplicity)
// In a real robot, this would be the actual distance in meters or consistent units
const double ROBOT_RADIUS = 1.0;

// === SENSOR PIN CONFIGURATION ===

// IR Distance Sensors (Sharp GP2Y0A02YK0F) - Analog pins
// Range: 20-150cm (200-1500mm), Output: 0.4V-2.7V
// Using available analog pins (A0-A2 shield headers, A9-A11 direct connection)
const int IR_LEFT_1_PIN = A0;      // Left side IR sensor 1 (shield header A0)
const int IR_LEFT_2_PIN = A1;      // Left side IR sensor 2 (shield header A1)
const int IR_RIGHT_1_PIN = A2;     // Right side IR sensor 1 (shield header A2)
const int IR_RIGHT_2_PIN = A9;     // Right side IR sensor 2 (direct A9 - jumper needed)
const int IR_BACK_1_PIN = A10;     // Back side IR sensor 1 (direct A10 - jumper needed)
const int IR_BACK_2_PIN = A11;     // Back side IR sensor 2 (direct A11 - jumper needed)

// HC-SR04 Ultrasonic Sensors - Digital pins
// Front distance sensors
const int ULTRASONIC_FRONT_LEFT_TRIG = 22;   // Front left ultrasonic trigger
const int ULTRASONIC_FRONT_LEFT_ECHO = 23;   // Front left ultrasonic echo
const int ULTRASONIC_FRONT_RIGHT_TRIG = 24;  // Front right ultrasonic trigger
const int ULTRASONIC_FRONT_RIGHT_ECHO = 25;  // Front right ultrasonic echo

// Line Sensors (Digital/Analog pins) - for line following
// Assembled side by side for robot alignment and navigation
const int LINE_SENSOR_LEFT = A6;    // Left line sensor
const int LINE_SENSOR_CENTER = A7;  // Center line sensor
const int LINE_SENSOR_RIGHT = A8;   // Right line sensor

// === SENSOR CONSTANTS ===

// IR Distance Sensor Constants (Sharp GP2Y0A02YK0F)
const float IR_VOLTAGE_MIN = 0.25;  // Minimum voltage output (1500mm) - expanded range
const float IR_VOLTAGE_MAX = 2.8;   // Maximum voltage output (20mm) - expanded range
const float IR_DISTANCE_MIN = 20;  // Minimum distance in mm (closer range for safety)
const float IR_DISTANCE_MAX = 1500; // Maximum distance in mm (0.25V)

// Ultrasonic Sensor Constants
const float ULTRASONIC_TIMEOUT = 30000; // Timeout in microseconds (50ms max distance)
const float SOUND_SPEED = 0.0343;       // Speed of sound in cm/us at 20°C

// Line Sensor Constants
const int LINE_SENSOR_THRESHOLD = 512;  // Threshold for line detection (0-1023)

// === LIFTER LIMIT SWITCH CONFIGURATION ===
const int LIFTER_TOP_LIMIT_PIN = 26;     // Top limit switch (normally open)
const int LIFTER_BOTTOM_LIMIT_PIN = 27;  // Bottom limit switch (normally open)

// Lifter Safety Constants
const unsigned long LIFTER_SAFETY_TIMEOUT_MS = 5000; // Maximum time for lifter movement (5 seconds)
const float LIFTER_SAFETY_CURRENT_THRESHOLD = 2.0;   // Current threshold for stall detection (if available)

// === SERVO CONFIGURATION ===
// YFROBOT shield servo channels (check your shield labeling)
// SERVO 01 = channel 8, SERVO 02 = channel 9, etc.
// If servos don't work, try channels 0, 1, 10, 11, etc.
const int TILT_SERVO_CHANNEL = 8;      // YFROBOT shield servo channel for tilt servo
const int GRIPPER_SERVO_CHANNEL = 9;   // YFROBOT shield servo channel for gripper servo

// Servo angle limits (degrees)
const int TILT_SERVO_MIN_ANGLE = 0;     // Minimum tilt angle
const int TILT_SERVO_MAX_ANGLE = 145;   // Maximum tilt angle
const int GRIPPER_SERVO_MIN_ANGLE = 0;  // Minimum gripper angle (open)
const int GRIPPER_SERVO_MAX_ANGLE = 180; // Maximum gripper angle (closed)

// Default servo positions
const int TILT_SERVO_DEFAULT = 90;      // Default tilt position (centered)
const int GRIPPER_SERVO_DEFAULT = 90;   // Default gripper position (half-open)

// === PERIMETER SAFETY CONSTANTS ===

// Virtual Force Field Parameters (Artificial Potential Field Method)
const float APF_INFLUENCE_DISTANCE_IR = 400.0;         // Influence distance for IR sensors (mm) - increased
const float APF_INFLUENCE_DISTANCE_ULTRASONIC = 500.0; // Influence distance for ultrasonic sensors (mm) - increased
const float APF_SCALING_FACTOR_IR = 2.0;               // Scaling factor for IR repulsive forces - increased
const float APF_SCALING_FACTOR_ULTRASONIC = 3.0;       // Scaling factor for ultrasonic repulsive forces - increased
const float APF_MAX_FORCE = 1.0;                       // Maximum repulsive force (normalized 0-1) - increased

// Legacy Safety Distances (kept for backward compatibility)
const float IR_SAFETY_DISTANCE_CRITICAL = 80.0;        // Critical zone: immediate stop (100mm = 10cm)
const float IR_SAFETY_DISTANCE_WARNING = 250.0;        // Warning zone: slow down (250mm = 25cm)
const float ULTRASONIC_SAFETY_DISTANCE_CRITICAL = 150.0; // Critical zone: immediate stop (50mm = 5cm)
const float ULTRASONIC_SAFETY_DISTANCE_WARNING = 250.0;  // Warning zone: slow down (150mm = 15cm)

// Emergency Brake Settings (fallback for extreme situations)
const unsigned long BRAKE_COOLDOWN_MS = 1500;         // Minimum time between emergency brakes (reduced from 2500ms for faster recovery)
const float EMERGENCY_DECELERATION = 0.3;             // Emergency deceleration factor (reduced from 0.5 for more aggressive braking)
const unsigned long PERIMETER_CHECK_INTERVAL = 25;     // Check perimeter every 25ms (doubled frequency from 50ms)

// Virtual Bumper Modes
enum VirtualBumperMode {
  LEGACY_BINARY,      // Original binary SAFE/WARNING/CRITICAL zones
  POTENTIAL_FIELD,    // New Artificial Potential Field method
  HYBRID             // Combined approach
};

const VirtualBumperMode CURRENT_BUMPER_MODE = POTENTIAL_FIELD; // Set desired mode


#endif // CONFIG_H
