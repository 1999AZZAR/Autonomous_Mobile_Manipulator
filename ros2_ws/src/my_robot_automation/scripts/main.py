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
        self.ros2_interface = ROS2Interface(sensor_manager=self.sensor_manager)
        self.flask_app = FlaskApp(
            mega_interface=self.mega_interface,
            sensor_manager=self.sensor_manager,
            ros2_interface=self.ros2_interface,
            simulation_mode=self.simulation_mode
        )

        # Initialize ROS2 services
        self.ros2_interface.initialize_actuator_services()
        self.ros2_interface.initialize_actuator_clients()

        logger.info("All components initialized successfully")

    def run(self):
        """Run the complete system"""
        try:
            # Start ROS2 in a separate thread
            ros2_thread = threading.Thread(target=self._run_ros2, daemon=True)
            ros2_thread.start()

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
        # Initialize ROS2
        rclpy.init()

        # Create and run the application
        app = AutonomousMobileManipulator(simulation_mode=simulation_mode)
        app.run()

    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
    except Exception as e:
        logger.error(f"Application error: {str(e)}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
