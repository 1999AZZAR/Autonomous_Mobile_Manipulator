#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class RobotStatusServer(Node):
    def __init__(self):
        super().__init__('robot_status_server')
        self.get_logger().info('Robot Status Server started')

def main(args=None):
    rclpy.init(args=args)
    server = RobotStatusServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
