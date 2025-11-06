#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from datetime import datetime

# ROS2 imports
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Polygon, Pose
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from tf2_msgs.msg import TFMessage
import tf2_ros

# Custom imports
from my_robot_automation.srv import GetNavigationStatus
from my_robot_automation.msg import NavigationStatus


class NavigationStatusServer(Node):
    """ROS2 service server for retrieving navigation status"""

    def __init__(self):
        super().__init__('navigation_status_server')

        # Service for getting navigation status
        self.navigation_status_service = self.create_service(
            GetNavigationStatus, 'get_navigation_status', self.get_navigation_status_callback
        )

        # Subscribers for navigation topics
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10
        )
        self.global_plan_sub = self.create_subscription(
            Path, '/plan', self.global_plan_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Current navigation status
        self.navigation_status = NavigationStatus()
        self.last_amcl_pose = None
        self.last_global_plan = None
        self.last_map = None

        # Initialize navigation status
        self.reset_navigation_status()

        self.get_logger().info('Navigation Status Server started')

    def reset_navigation_status(self):
        """Reset navigation status to default values"""
        now = self.get_clock().now().to_msg()
        self.navigation_status.header.stamp = now
        self.navigation_status.header.frame_id = 'map'

        # Localization status
        self.navigation_status.localization_active = False
        self.navigation_status.localization_confidence = 0.0

        # Navigation status
        self.navigation_status.navigation_active = False
        self.navigation_status.navigation_state = 'IDLE'

        # Map status
        self.navigation_status.map_available = False
        self.navigation_status.map_name = 'unknown'

        # Performance metrics
        self.navigation_status.path_length = 0.0
        self.navigation_status.distance_to_goal = 0.0

    def amcl_pose_callback(self, msg):
        """Callback for AMCL pose estimates"""
        self.last_amcl_pose = msg
        self.navigation_status.current_pose = msg
        self.navigation_status.localization_active = True

        # Calculate localization confidence from covariance
        # Use the average of diagonal elements as confidence measure
        covariance = msg.pose.covariance
        diagonal_sum = covariance[0] + covariance[7] + covariance[14] + covariance[21] + covariance[28] + covariance[35]
        avg_covariance = diagonal_sum / 6.0

        # Convert covariance to confidence (lower covariance = higher confidence)
        # Assuming typical covariance range of 0.01 to 1.0
        confidence = max(0.0, min(1.0, 1.0 - (avg_covariance / 0.5)))
        self.navigation_status.localization_confidence = confidence

    def global_plan_callback(self, msg):
        """Callback for global navigation plan"""
        self.last_global_plan = msg
        self.navigation_status.current_path = msg
        self.navigation_status.navigation_active = True

        # Calculate path length
        if len(msg.poses) > 1:
            path_length = 0.0
            for i in range(1, len(msg.poses)):
                dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
                dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
                path_length += (dx**2 + dy**2)**0.5
            self.navigation_status.path_length = path_length

            # Calculate distance to goal (from current pose to last pose in path)
            if self.last_amcl_pose:
                goal = msg.poses[-1]
                current = self.last_amcl_pose.pose.pose
                dx = goal.pose.position.x - current.position.x
                dy = goal.pose.position.y - current.position.y
                self.navigation_status.distance_to_goal = (dx**2 + dy**2)**0.5

                # Set target pose
                self.navigation_status.target_pose = goal
            else:
                self.navigation_status.distance_to_goal = path_length

    def map_callback(self, msg):
        """Callback for map updates"""
        self.last_map = msg
        self.navigation_status.map_available = True
        self.navigation_status.map_name = 'current_map'
        self.navigation_status.map_last_updated = self.get_clock().now().to_msg()
        self.navigation_status.map_origin = msg.info.origin

    def get_navigation_status_callback(self, request, response):
        """Service callback for getting navigation status"""
        try:
            self.get_logger().debug('Navigation status request received')

            # Update header timestamp
            self.navigation_status.header.stamp = self.get_clock().now().to_msg()

            # If no map data requested, clear map fields to reduce response size
            if not request.include_map_data:
                self.navigation_status.map_available = False
                self.navigation_status.map_name = ''
                # Keep other map fields as defaults

            # If no path data requested, clear path to reduce response size
            if not request.include_path_data:
                self.navigation_status.current_path = Path()
                self.navigation_status.path_length = 0.0

            response.navigation_status = self.navigation_status
            response.success = True
            response.message = 'Navigation status retrieved successfully'

            self.get_logger().debug('Navigation status response sent')

        except Exception as e:
            self.get_logger().error(f'Error getting navigation status: {str(e)}')
            response.success = False
            response.message = f'Error retrieving navigation status: {str(e)}'
            # Return default navigation status on error
            response.navigation_status = self.navigation_status

        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        nav_server = NavigationStatusServer()

        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(nav_server)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            nav_server.destroy_node()

    except Exception as e:
        print(f"Error starting navigation status server: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
