#!/usr/bin/env python3
"""Bridge node forwarding /goal_pose PoseStamped messages to Nav2 /navigate_to_pose action."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self._subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info('GoalPoseBridge started: listening on /goal_pose -> forwarding to /navigate_to_pose')

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Received /goal_pose: target (x={x:.2f}, y={y:.2f}). Forwarding to /navigate_to_pose...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /navigate_to_pose is not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by /navigate_to_pose!')
            return

        self.get_logger().info('Goal accepted by /navigate_to_pose. Robot is navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation ABORTED!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation CANCELED!')
        else:
            self.get_logger().info(f'Navigation finished with status code {status}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
