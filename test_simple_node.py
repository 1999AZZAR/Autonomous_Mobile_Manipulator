#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_automation.srv import ControlGripper

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.srv = self.create_service(
            ControlGripper, 'test_service', self.callback)
        self.get_logger().info('Test node initialized')

    def callback(self, request, response):
        response.success = True
        response.message = "Test successful"
        response.status = "OK"
        return response

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
