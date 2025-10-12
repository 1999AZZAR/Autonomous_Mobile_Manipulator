#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.get_logger().info('Gripper Controller started')

def main(args=None):
    rclpy.init(args=args)
    controller = GripperController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
