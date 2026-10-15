"""Send one map-frame destination to Nav2 after AMCL is ready."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoToGoal(Node):
    """Navigate once to a configurable pose on the saved map."""

    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', -0.5)
        self.declare_parameter('goal_y', 0.6)
        self.declare_parameter('goal_yaw', 0.0)  # radians

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        if not all(math.isfinite(v) for v in
                   (self.goal_x, self.goal_y, self.goal_yaw)):
            raise ValueError('Goal coordinates and yaw must be finite numbers')

        self.localized = False
        self.sent = False
        self.done = False
        self.last_feedback_time = 0.0
        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.timer = self.create_timer(1.0, self.try_send_goal)
        self.get_logger().info(
            f'Waiting for /amcl_pose; goal: x={self.goal_x:.2f}, '
            f'y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f} rad')

    def on_amcl_pose(self, _msg):
        if not self.localized:
            self.localized = True
            self.get_logger().info('AMCL pose received')

    def try_send_goal(self):
        if not self.localized or self.sent:
            return
        if not self.action_client.server_is_ready():
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        self.sent = True
        self.timer.cancel()
        self.get_logger().info('Sending goal to navigate_to_pose')
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send goal: {exc}')
            self.done = True
            return

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.done = True
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_time >= 5.0:
            distance = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f'Distance remaining: {distance:.2f} m')
            self.last_feedback_time = now

    def on_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Could not read navigation result: {exc}')
        else:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal reached')
            else:
                self.get_logger().error(f'Navigation ended with status {status}')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GoToGoal()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
