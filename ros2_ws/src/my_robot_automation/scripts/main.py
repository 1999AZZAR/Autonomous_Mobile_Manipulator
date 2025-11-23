#!/usr/bin/env python3
"""
Main entry point for Autonomous Mobile Manipulator
Coordinates ROS2, Flask web interface, Mega serial communication, and sensor management
"""

import sys
import time
import threading
import argparse
import logging
import math
from config import DEFAULT_SIMULATION_MODE
from app import FlaskApp
from mega_interface import MegaInterface
from sensor_manager import SensorManager
from path_planning import GridMap, PathPlanner, MovementSequence, WaypointNavigator

# ROS2 import (optional - for systems with ROS2 installed)
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    ROS2_AVAILABLE = False

# Import ROS2 interface only if ROS2 is available
if ROS2_AVAILABLE:
    try:
        from ros2_interface import ROS2Interface
    except ImportError:
        ROS2Interface = None
        ROS2_AVAILABLE = False
else:
    ROS2Interface = None

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [%(name)s]: %(message)s'
)
logger = logging.getLogger(__name__)

class AutonomousMobileManipulator:
    """Main application coordinator"""

    def __init__(self, simulation_mode=None):
        self.simulation_mode = simulation_mode if simulation_mode is not None else DEFAULT_SIMULATION_MODE

        logger.info(f"Starting Autonomous Mobile Manipulator (simulation_mode={self.simulation_mode})")

        # Initialize components
        self.mega_interface = MegaInterface()
        self.sensor_manager = SensorManager(simulation_mode=self.simulation_mode, mega_interface=self.mega_interface)

        # Initialize navigation state for IMU-based waypoint navigation
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, z] in ENU coordinates (meters)
        self.current_orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] from IMU (degrees)
        self.last_imu_time = time.time()
        self.position_initialized = False
        self.velocity_estimate = [0.0, 0.0, 0.0]  # Estimated velocity for dead reckoning

        # Initialize path planning
        self.grid_map = GridMap(width=100, height=100, resolution=0.1)  # 10m x 10m grid
        self.path_planner = PathPlanner(self.grid_map)
        self.movement_sequence = MovementSequence()
        self.waypoint_navigator = WaypointNavigator(self.path_planner)

        # Initialize ROS2 interface if available (non-blocking)
        if ROS2_AVAILABLE:
            try:
                self.ros2_interface = ROS2Interface(sensor_manager=self.sensor_manager)
                # Initialize ROS2 services in background thread to avoid blocking
                ros2_init_thread = threading.Thread(target=self._initialize_ros2_services, daemon=True)
                ros2_init_thread.start()
                logger.info("ROS2 interface created - services initializing in background")
            except Exception as e:
                logger.warning(f"ROS2 interface failed to initialize: {str(e)}")
                self.ros2_interface = None
        else:
            self.ros2_interface = None
            logger.info("ROS2 interface disabled (ROS2 not available)")

        self.flask_app = FlaskApp(
            mega_interface=self.mega_interface,
            sensor_manager=self.sensor_manager,
            ros2_interface=self.ros2_interface,
            simulation_mode=self.simulation_mode,
            main_app=self  # Pass reference to self for IMU access
        )

        logger.info("Path planning components initialized")

        # Start IMU position tracking for coordinate-based navigation
        # Start IMU position tracking for waypoint navigation (works with simulated IMU data)
        threading.Thread(target=self._imu_position_tracking, daemon=True).start()
        logger.info("IMU position tracking started for waypoint navigation")

        logger.info("All components initialized successfully")

    def _imu_position_tracking(self):
        """Track robot position using IMU data for coordinate-based navigation"""
        logger.info("Starting IMU-based position tracking")

        # Initialize position tracking variables
        position_initialized = False
        last_valid_time = time.time()
        consecutive_errors = 0

        while True:
            try:
                current_time = time.time()
                imu_data = None

                # Try to get IMU data from ROS2
                if self.ros2_interface and hasattr(self.ros2_interface, 'imu_data'):
                    imu_data = self.ros2_interface.imu_data

                # Fallback to simulated IMU data if ROS2 not available or no data
                if not imu_data:
                    # Simulate IMU data for testing waypoint navigation
                    # Only rotate slowly when not initialized to avoid confusion
                    if not position_initialized:
                        yaw_value = 0.0
                    else:
                        # Very slow rotation for simulation (1° per second)
                        yaw_value = (current_time - last_valid_time) * 0.01745  # 1°/s in radians

                    imu_data = {
                        'orientation': {
                            'x': 0.0,  # roll
                            'y': 0.0,  # pitch
                            'z': yaw_value
                        },
                        'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # No rotation in sim
                        'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81}
                    }

                if imu_data:
                    dt = current_time - self.last_imu_time

                    # Skip if dt is too large (system was paused or error occurred)
                    if dt > 1.0:
                        logger.debug(f"Large time gap ({dt:.1f}s), resetting IMU tracking")
                        dt = 0.1  # Use reasonable default
                        self.last_imu_time = current_time - dt

                    # Extract orientation (yaw for heading)
                    if 'orientation' in imu_data and 'z' in imu_data['orientation']:
                        # Handle both quaternion (z component) and direct yaw values
                        yaw_rad = imu_data['orientation']['z']

                        # If value seems to be in degrees, convert to radians
                        if abs(yaw_rad) > 2 * 3.14159:  # Likely degrees
                            yaw_rad = yaw_rad * 3.14159 / 180.0

                        self.current_orientation[2] = yaw_rad * 180.0 / 3.14159  # Convert to degrees

                        # Keep heading in -180 to 180 range
                        while self.current_orientation[2] > 180:
                            self.current_orientation[2] -= 360
                        while self.current_orientation[2] < -180:
                            self.current_orientation[2] += 360

                    # Extract angular velocity for heading correction (more accurate than just orientation)
                    if 'angular_velocity' in imu_data and dt > 0:
                        omega_z_deg = imu_data['angular_velocity'].get('z', 0.0) * 180.0 / 3.14159  # Convert to deg/s
                        # Only apply angular velocity correction if it's reasonable (< 90°/s)
                        if abs(omega_z_deg) < 90:
                            self.current_orientation[2] += omega_z_deg * dt

                    # Position estimation using wheel odometry (much more accurate than IMU acceleration)
                    # For now, we'll use a simple model assuming constant velocity during movement
                    # In a real system, you'd integrate wheel encoder data
                    if dt > 0 and position_initialized:
                        # Estimate movement based on commanded actions (simplified)
                        # This should be replaced with actual wheel encoder feedback
                        pass  # For now, position stays the same until waypoint navigation provides feedback

                    # Mark as initialized after first successful reading
                    if not position_initialized:
                        position_initialized = True
                        self.position_initialized = True
                        logger.info("IMU position tracking initialized")

                    self.last_imu_time = current_time
                    last_valid_time = current_time
                    consecutive_errors = 0

                    # Debug logging (reduced frequency)
                    if int(current_time) % 10 == 0:  # Every 10 seconds
                        logger.debug(".2f"
                                   ".1f")

                else:
                    consecutive_errors += 1
                    if consecutive_errors > 10:
                        logger.warning("No IMU data available for 10+ iterations")
                        consecutive_errors = 0

                time.sleep(0.1)  # 10Hz update rate

            except Exception as e:
                consecutive_errors += 1
                logger.error(f"IMU position tracking error: {str(e)}")
                if consecutive_errors > 5:
                    logger.warning("Multiple IMU tracking errors, resetting position tracking")
                    position_initialized = False
                    consecutive_errors = 0
                time.sleep(0.5)  # Back off on errors
                time.sleep(1.0)

    def get_current_position(self):
        """Get current robot position and orientation"""
        return {
            'position': self.current_position.copy(),
            'orientation': self.current_orientation.copy(),
            'initialized': self.position_initialized
        }

    def _initialize_ros2_services(self):
        """Initialize ROS2 services in background thread"""
        try:
            logger.info("Initializing ROS2 actuator services...")
            self.ros2_interface.initialize_actuator_services()
            logger.info("ROS2 actuator services initialized")

            logger.info("Initializing ROS2 actuator clients...")
            self.ros2_interface.initialize_actuator_clients()
            logger.info("ROS2 actuator clients initialized")

        except Exception as e:
            logger.error(f"ROS2 services initialization failed: {str(e)}")
            logger.info("Continuing without ROS2 features")

    def run(self):
        """Run the complete system"""
        try:
            # Start ROS2 in a separate thread (if available)
            if ROS2_AVAILABLE and self.ros2_interface:
                ros2_thread = threading.Thread(target=self._run_ros2, daemon=True)
                ros2_thread.start()
                logger.info("ROS2 thread started")
            else:
                logger.info("ROS2 thread not started (ROS2 not available)")

            # Start Flask web interface (blocking)
            logger.info("Starting Flask web interface...")
            self.flask_app.run()

        except KeyboardInterrupt:
            logger.info("Received interrupt signal, shutting down...")
        except Exception as e:
            logger.error(f"Error running application: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            logger.info("Application shutdown initiated")
            self.cleanup()

    def _run_ros2(self):
        """Run ROS2 node in separate thread"""
        try:
            logger.info("Starting ROS2 node...")
            rclpy.spin(self.ros2_interface)
        except Exception as e:
            logger.error(f"ROS2 node error: {str(e)}")
        finally:
            rclpy.shutdown()

    def cleanup(self):
        """Clean up all resources"""
        logger.info("Cleaning up resources...")

        try:
            if self.mega_interface:
                self.mega_interface.cleanup()
        except Exception as e:
            logger.error(f"Error cleaning up Mega interface: {str(e)}")

        try:
            if self.sensor_manager:
                self.sensor_manager.cleanup()
        except Exception as e:
            logger.error(f"Error cleaning up sensor manager: {str(e)}")

        logger.info("Cleanup completed")

    def reset_position(self):
        """Reset robot position to origin for waypoint navigation"""
        try:
            self.current_position = [0.0, 0.0, 0.0]
            self.current_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
            self.velocity_estimate = [0.0, 0.0, 0.0]
            self.last_imu_time = time.time()
            self.position_initialized = True
            logger.info("Robot position reset to origin (0,0,0)")
        except Exception as e:
            logger.error(f"Error resetting position: {str(e)}")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Autonomous Mobile Manipulator Control System')
    parser.add_argument('--simulation', action='store_true',
                       help='Run in simulation mode (no hardware required)')
    parser.add_argument('--hardware', action='store_true',
                       help='Force hardware mode (require real hardware)')
    parser.add_argument('--host', default='0.0.0.0',
                       help='Flask server host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8000,
                       help='Flask server port (default: 8000)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable Flask debug mode')

    args = parser.parse_args()

    # Determine simulation mode
    if args.simulation:
        simulation_mode = True
        logger.info("Running in SIMULATION mode (forced)")
    elif args.hardware:
        simulation_mode = False
        logger.info("Running in HARDWARE mode (forced)")
    else:
        simulation_mode = DEFAULT_SIMULATION_MODE
        logger.info(f"Running in {'SIMULATION' if simulation_mode else 'AUTO-DETECT'} mode")

    try:
        # Initialize ROS2 if available
        if ROS2_AVAILABLE:
            rclpy.init()
            logger.info("ROS2 initialized successfully")
        else:
            logger.warning("ROS2 not available - running without ROS2 features")

        # Create and run the application
        app = AutonomousMobileManipulator(simulation_mode=simulation_mode)
        app.run()

    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
    except Exception as e:
        logger.error(f"Application error: {str(e)}")
        sys.exit(1)
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
