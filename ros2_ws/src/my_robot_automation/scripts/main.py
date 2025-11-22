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
from ros2_interface import ROS2Interface

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
        self.sensor_manager = SensorManager(simulation_mode=self.simulation_mode)
        self.mega_interface = MegaInterface()

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

        # Initialize ROS2 interface if available
        if ROS2_AVAILABLE:
            self.ros2_interface = ROS2Interface(sensor_manager=self.sensor_manager)
            # Initialize ROS2 services
            self.ros2_interface.initialize_actuator_services()
            self.ros2_interface.initialize_actuator_clients()
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
        if ROS2_AVAILABLE and not self.simulation_mode:
            threading.Thread(target=self._imu_position_tracking, daemon=True).start()
            logger.info("IMU position tracking started for waypoint navigation")

        logger.info("All components initialized successfully")

    def _imu_position_tracking(self):
        """Track robot position using IMU data for coordinate-based navigation"""
        logger.info("Starting IMU-based position tracking")

        while True:
            try:
                if self.ros2_interface and hasattr(self.ros2_interface, 'imu_data'):
                    imu_data = self.ros2_interface.imu_data

                    if imu_data:
                        current_time = time.time()
                        dt = current_time - self.last_imu_time

                        # Extract orientation (yaw for heading)
                        if 'orientation' in imu_data:
                            # Convert quaternion to euler angles (simplified - using yaw)
                            # In practice, you'd use proper quaternion to euler conversion
                            yaw = imu_data['orientation'].get('z', 0.0) * 180.0 / 3.14159  # Convert to degrees
                            self.current_orientation[2] = yaw  # yaw/heading

                        # Extract angular velocity for heading correction
                        if 'angular_velocity' in imu_data:
                            # Simple integration for heading (dead reckoning)
                            omega_z = imu_data['angular_velocity'].get('z', 0.0)
                            self.current_orientation[2] += omega_z * dt * 180.0 / 3.14159

                            # Keep heading in -180 to 180 range
                            while self.current_orientation[2] > 180:
                                self.current_orientation[2] -= 360
                            while self.current_orientation[2] < -180:
                                self.current_orientation[2] += 360

                        # Estimate velocity from IMU acceleration (very simplified)
                        if 'linear_acceleration' in imu_data and dt > 0:
                            # Remove gravity (assuming Z is up)
                            accel_x = imu_data['linear_acceleration'].get('x', 0.0)
                            accel_y = imu_data['linear_acceleration'].get('y', 0.0)

                            # Update velocity estimate
                            self.velocity_estimate[0] += accel_x * dt
                            self.velocity_estimate[1] += accel_y * dt

                            # Apply damping to prevent drift
                            self.velocity_estimate[0] *= 0.95
                            self.velocity_estimate[1] *= 0.95

                            # Update position using dead reckoning
                            heading_rad = self.current_orientation[2] * 3.14159 / 180.0
                            self.current_position[0] += (self.velocity_estimate[0] * math.cos(heading_rad) -
                                                        self.velocity_estimate[1] * math.sin(heading_rad)) * dt
                            self.current_position[1] += (self.velocity_estimate[0] * math.sin(heading_rad) +
                                                        self.velocity_estimate[1] * math.cos(heading_rad)) * dt

                        self.last_imu_time = current_time
                        self.position_initialized = True

                        # Debug logging (reduced frequency)
                        if int(current_time) % 5 == 0:  # Every 5 seconds
                            logger.debug(f"IMU Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) "
                                       f"Heading: {self.current_orientation[2]:.1f}°")

                time.sleep(0.1)  # 10Hz update rate

            except Exception as e:
                logger.error(f"IMU position tracking error: {str(e)}")
                time.sleep(1.0)

    def get_current_position(self):
        """Get current robot position and orientation"""
        return {
            'position': self.current_position.copy(),
            'orientation': self.current_orientation.copy(),
            'initialized': self.position_initialized
        }

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
            self.flask_app.run()

        except KeyboardInterrupt:
            logger.info("Shutting down...")
        except Exception as e:
            logger.error(f"Error running application: {str(e)}")
        finally:
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
