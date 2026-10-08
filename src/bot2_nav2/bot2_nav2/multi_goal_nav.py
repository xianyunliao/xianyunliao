#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class MultiGoalNavigator(Node):
    """Navigate through multiple goals sequentially."""

    def __init__(self):
        super().__init__('multi_goal_navigator')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # 根据你的地图修改。
        # 每个目标点：(x, y, yaw)
        self.goals = [
            (-1.590, -4.400, 0.000),
            (1.112, -2.325, 1.512),
            (1.930, 1.560, 0.000),
        ]

        self.current_goal_index = 0

    def create_goal(self, x, y, yaw):
        """Create a NavigateToPose goal."""

        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal.pose = pose

        return goal

    def start(self):
        """Wait for Nav2 and send the first goal."""

        self.get_logger().info(
            'Waiting for /navigate_to_pose action server...'
        )

        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                '/navigate_to_pose action server is unavailable.'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            '/navigate_to_pose is ready.'
        )

        self.send_next_goal()

    def send_next_goal(self):
        """Send the next goal."""

        if self.current_goal_index >= len(self.goals):
            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                'All goals reached successfully!'
            )
            self.get_logger().info(
                '========================================'
            )
            rclpy.shutdown()
            return

        x, y, yaw = self.goals[self.current_goal_index]
        goal_number = self.current_goal_index + 1

        self.get_logger().info(
            f'Going to goal {goal_number}/{len(self.goals)}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )

        goal_msg = self.create_goal(x, y, yaw)

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance."""

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Exception while sending goal: {exc}'
            )
            rclpy.shutdown()
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal {self.current_goal_index + 1} was rejected.'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Goal {self.current_goal_index + 1} accepted by Nav2.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.goal_result_callback
        )

    def feedback_callback(self, feedback_msg):
        """Print navigation feedback."""

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Goal {self.current_goal_index + 1}: '
            f'distance remaining = '
            f'{feedback.distance_remaining:.2f} m'
        )

    def goal_result_callback(self, future):
        """Handle navigation result."""

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Exception while receiving result: {exc}'
            )
            rclpy.shutdown()
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Goal {self.current_goal_index + 1} reached successfully.'
            )

            self.current_goal_index += 1

            # 只有当前目标成功后，才发送下一个目标。
            self.send_next_goal()

        else:
            self.get_logger().error(
                f'Goal {self.current_goal_index + 1} failed. '
                f'Nav2 status = {result.status}'
            )

            self.get_logger().error(
                'Navigation sequence stopped.'
            )

            rclpy.shutdown()


def main(args=None):
    """ROS 2 entry point."""

    rclpy.init(args=args)

    navigator = MultiGoalNavigator()

    try:
        navigator.start()
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        navigator.get_logger().info(
            'Navigation interrupted by user.'
        )

    finally:
        navigator.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

