#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
import threading
import time
import uuid
from datetime import datetime

# ROS2 imports
from geometry_msgs.msg import PoseStamped

# Custom imports
from my_robot_automation.srv import GetTaskStatus, CancelTask
from my_robot_automation.msg import TaskStatus, AutomationTask


class TaskManagementServer(Node):
    """ROS2 service server for task management operations"""

    def __init__(self):
        super().__init__('task_management_server')

        # Services for task management
        self.get_task_status_service = self.create_service(
            GetTaskStatus, 'get_task_status', self.get_task_status_callback
        )
        self.cancel_task_service = self.create_service(
            CancelTask, 'cancel_task', self.cancel_task_callback
        )

        # Subscriber for automation task updates
        self.task_update_sub = self.create_subscription(
            AutomationTask, '/automation/task_updates', self.task_update_callback, 10
        )

        # Task storage - in a real implementation, this would be more sophisticated
        self.active_tasks = {}  # task_id -> TaskStatus
        self.completed_tasks = {}  # task_id -> TaskStatus

        # Clean up completed tasks periodically
        self.create_timer(60.0, self.cleanup_old_tasks)  # Clean up every minute

        self.get_logger().info('Task Management Server started')

    def task_update_callback(self, msg):
        """Callback for automation task updates"""
        try:
            task_status = TaskStatus()
            task_status.header = msg.header
            task_status.task_id = msg.task_id
            task_status.task_type = msg.task_type
            task_status.status = msg.status
            task_status.target_pose = msg.target_pose
            task_status.waypoints = msg.waypoints
            task_status.timeout_seconds = msg.timeout_seconds
            task_status.parameters = msg.parameters
            task_status.error_message = msg.error_message
            task_status.progress_percentage = msg.progress_percentage

            # Set timestamps
            current_time = self.get_clock().now()
            if msg.task_id not in self.active_tasks and msg.task_id not in self.completed_tasks:
                task_status.start_time = current_time.to_msg()
            else:
                # Preserve start time if task already exists
                existing_task = self.active_tasks.get(msg.task_id, self.completed_tasks.get(msg.task_id))
                if existing_task:
                    task_status.start_time = existing_task.start_time

            # Update end time for completed/failed/cancelled tasks
            if msg.status in ['COMPLETED', 'FAILED', 'CANCELLED']:
                task_status.end_time = current_time.to_msg()
                start_time = Time.from_msg(task_status.start_time)
                end_time = Time.from_msg(task_status.end_time)
                task_status.elapsed_time = (end_time - start_time).to_msg()
                self.completed_tasks[msg.task_id] = task_status
                if msg.task_id in self.active_tasks:
                    del self.active_tasks[msg.task_id]
            else:
                self.active_tasks[msg.task_id] = task_status

            self.get_logger().debug(f'Task update: {msg.task_id} -> {msg.status}')

        except Exception as e:
            self.get_logger().error(f'Error processing task update: {str(e)}')

    def get_task_status_callback(self, request, response):
        """Service callback for getting task status"""
        try:
            self.get_logger().debug(f'Task status request for: {request.task_id}')

            if request.task_id:
                # Get specific task
                task = self.active_tasks.get(request.task_id, self.completed_tasks.get(request.task_id))
                if task:
                    response.active_tasks = [task]
                    response.success = True
                    response.message = f'Task {request.task_id} found'
                else:
                    response.success = False
                    response.message = f'Task {request.task_id} not found'
                    response.active_tasks = []
            else:
                # Get all active tasks
                response.active_tasks = list(self.active_tasks.values())
                response.success = True
                response.message = f'Found {len(response.active_tasks)} active tasks'

            self.get_logger().debug(f'Task status response: {response.message}')

        except Exception as e:
            self.get_logger().error(f'Error getting task status: {str(e)}')
            response.success = False
            response.message = f'Error retrieving task status: {str(e)}'
            response.active_tasks = []

        return response

    def cancel_task_callback(self, request, response):
        """Service callback for canceling tasks"""
        try:
            self.get_logger().info(f'Task cancellation request: {request.task_id} - {request.reason}')

            task = self.active_tasks.get(request.task_id)
            if not task:
                task = self.completed_tasks.get(request.task_id)
                if task:
                    response.success = False
                    response.message = f'Task {request.task_id} is already completed'
                    response.task_id = request.task_id
                    response.previous_status = task.status
                    return response
                else:
                    response.success = False
                    response.message = f'Task {request.task_id} not found'
                    response.task_id = request.task_id
                    response.previous_status = 'UNKNOWN'
                    return response

            # Update task status to cancelled
            previous_status = task.status
            task.status = 'CANCELLED'
            task.error_message = f'Cancelled: {request.reason}'
            task.end_time = self.get_clock().now().to_msg()
            start_time = Time.from_msg(task.start_time)
            end_time = Time.from_msg(task.end_time)
            task.elapsed_time = (end_time - start_time).to_msg()

            # Move to completed tasks
            self.completed_tasks[request.task_id] = task
            del self.active_tasks[request.task_id]

            # Publish task update
            automation_task = AutomationTask()
            automation_task.header.stamp = self.get_clock().now().to_msg()
            automation_task.header.frame_id = 'task_manager'
            automation_task.task_id = task.task_id
            automation_task.task_type = task.task_type
            automation_task.status = task.status
            automation_task.target_pose = task.target_pose
            automation_task.waypoints = task.waypoints
            automation_task.timeout_seconds = task.timeout_seconds
            automation_task.parameters = task.parameters
            automation_task.error_message = task.error_message
            automation_task.progress_percentage = task.progress_percentage

            # Note: We would need a publisher for this, but for now we'll just log
            self.get_logger().info(f'Task {request.task_id} cancelled: {request.reason}')

            response.success = True
            response.message = f'Task {request.task_id} cancelled successfully'
            response.task_id = request.task_id
            response.previous_status = previous_status

        except Exception as e:
            self.get_logger().error(f'Error cancelling task: {str(e)}')
            response.success = False
            response.message = f'Error cancelling task: {str(e)}'
            response.task_id = request.task_id
            response.previous_status = 'UNKNOWN'

        return response

    def cleanup_old_tasks(self):
        """Clean up old completed tasks to prevent memory buildup"""
        try:
            current_time = self.get_clock().now()
            cutoff_time = current_time - rclpy.duration.Duration(seconds=3600)  # 1 hour ago

            tasks_to_remove = []
            for task_id, task in self.completed_tasks.items():
                if rclpy.time.Time.from_msg(task.end_time) < cutoff_time:
                    tasks_to_remove.append(task_id)

            for task_id in tasks_to_remove:
                del self.completed_tasks[task_id]

            if tasks_to_remove:
                self.get_logger().info(f'Cleaned up {len(tasks_to_remove)} old completed tasks')

        except Exception as e:
            self.get_logger().error(f'Error cleaning up old tasks: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        task_server = TaskManagementServer()

        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(task_server)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            task_server.destroy_node()

    except Exception as e:
        print(f"Error starting task management server: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
