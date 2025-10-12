#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ObstacleAvoidanceExecutor(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_executor')
        self.get_logger().info('Obstacle Avoidance Executor started')

def main(args=None):
    rclpy.init(args=args)
    executor = ObstacleAvoidanceExecutor()
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
