#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class PickPlaceExecutor(Node):
    def __init__(self):
        super().__init__('pick_place_executor')
        self.get_logger().info('Pick Place Executor started')

def main(args=None):
    rclpy.init(args=args)
    executor = PickPlaceExecutor()
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
