#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class EmergencyStopServer(Node):
    def __init__(self):
        super().__init__('emergency_stop_server')
        self.get_logger().info('Emergency Stop Server started')

def main(args=None):
    rclpy.init(args=args)
    server = EmergencyStopServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
