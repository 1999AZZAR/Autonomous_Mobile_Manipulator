#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ObstacleAvoidanceServer(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_server')
        self.get_logger().info('Obstacle Avoidance Server started')

def main(args=None):
    rclpy.init(args=args)
    server = ObstacleAvoidanceServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
