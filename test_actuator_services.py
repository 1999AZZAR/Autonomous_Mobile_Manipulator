#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_automation.srv import ControlGripper, SetGripperTilt, MoveRobot, ControlContainer

class ActuatorServiceTester(Node):
    def __init__(self):
        super().__init__('actuator_service_tester')

        # Create service clients
        self.control_gripper_client = self.create_client(ControlGripper, 'actuator/control_gripper')
        self.set_gripper_tilt_client = self.create_client(SetGripperTilt, 'actuator/set_gripper_tilt')
        self.move_robot_client = self.create_client(MoveRobot, 'actuator/move_robot')
        self.control_container_client = self.create_client(ControlContainer, 'actuator/control_container')

    def test_services(self):
        """Test all actuator services"""
        print("Testing actuator services...")

        # Test gripper control
        if self.control_gripper_client.wait_for_service(timeout_sec=5.0):
            request = ControlGripper.Request()
            request.command = "open"

            future = self.control_gripper_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                print(f"Gripper control: {response.success} - {response.message}")
            else:
                print("Gripper control service call failed")
        else:
            print("Gripper control service not available")

        # Test gripper tilt
        if self.set_gripper_tilt_client.wait_for_service(timeout_sec=5.0):
            request = SetGripperTilt.Request()
            request.angle = 45.0

            future = self.set_gripper_tilt_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                print(f"Gripper tilt: {response.success} - {response.message}")
            else:
                print("Gripper tilt service call failed")
        else:
            print("Gripper tilt service not available")

        print("Service testing complete!")

def main():
    rclpy.init()

    # First start the web interface in background
    import subprocess
    import time

    print("Starting web interface...")
    web_process = subprocess.Popen([
        'ssh', 'raspi@100.74.72.71',
        'cd /home/raspi/Autonomous_Mobile_Manipulator/ros2_ws && '
        'source /opt/ros/jazzy/setup.bash && '
        'source install/setup.bash && '
        'python3 src/my_robot_automation/scripts/web_robot_interface.py --hardware'
    ])

    # Wait for it to start
    time.sleep(10)

    # Test the services
    tester = ActuatorServiceTester()
    tester.test_services()

    rclpy.shutdown()
    web_process.terminate()

if __name__ == '__main__':
    main()
