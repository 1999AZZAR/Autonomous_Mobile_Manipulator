#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MQTTBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        self.get_logger().info('MQTT Bridge started')

def main(args=None):
    rclpy.init(args=args)
    bridge = MQTTBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
