#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self.get_logger().info('Navigation Server started')

def main(args=None):
    rclpy.init(args=args)
    server = NavigationServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
