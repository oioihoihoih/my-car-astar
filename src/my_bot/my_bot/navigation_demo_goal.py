"""Send one simulation goal once localization and camera inference are available."""

import math
import json

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationDemoGoal(Node):
    def __init__(self):
        super().__init__('navigation_demo_goal')
        for name, value in [('goal_x', 0.3), ('goal_y', -0.5), ('goal_yaw', 0.0)]:
            self.declare_parameter(name, value)
        self.goal = [self.get_parameter(name).value
                     for name in ['goal_x', 'goal_y', 'goal_yaw']]
        if not all(math.isfinite(value) for value in self.goal):
            raise ValueError('Example goal must be finite')
        self.ready = set()
        self.declare_parameter('require_safety', False)
        self.require_safety = self.get_parameter('require_safety').value
        self.safety_ready = not self.require_safety
        self.finished = False
        self.frame = 'map'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda _: self.ready.add('pose'), 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda _: self.ready.add('scan'), qos_profile_sensor_data)
        self.create_subscription(String, '/yolo/detections',
                                 lambda _: self.ready.add('yolo'), 10)
        # Respect a manual RViz goal chosen while the detector is starting.
        self.create_subscription(PoseStamped, '/goal_pose', self.on_manual_goal, 10)
        self.create_subscription(Bool, '/safety/stop', self.on_safety_stop, latched)
        self.create_subscription(String, '/safety/status', self.on_safety_status, latched)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.2, self.maybe_send)
        self.get_logger().info('Example goal waits for map, AMCL, scan, and YOLO')

    def on_map(self, msg):
        self.frame = msg.header.frame_id or 'map'
        self.ready.add('map')

    def on_manual_goal(self, _msg):
        if not self.finished:
            self.finished = True
            self.timer.cancel()
            self.get_logger().info('Manual goal received; automatic example cancelled')

    def on_safety_stop(self, msg):
        if self.require_safety:
            self.safety_ready = not msg.data

    def on_safety_status(self, msg):
        if self.require_safety:
            status = json.loads(msg.data)
            if status['latched'] and status['reason'] != 'Manual arming required':
                self.on_manual_goal(None)

    def maybe_send(self):
        if self.finished or not self.safety_ready or self.ready != {'map', 'pose', 'scan', 'yolo'}:
            return
        if not any(info.node_name == 'astar_pure_pursuit' for info in
                   self.get_subscriptions_info_by_topic('/goal_pose')):
            return
        try:
            self.buffer.lookup_transform(self.frame, 'base_footprint', Time())
        except TransformException:
            return
        goal = PoseStamped()
        goal.header.frame_id = self.frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = self.goal[:2]
        goal.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        goal.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        self.finished = True
        self.timer.cancel()
        self.publisher.publish(goal)
        self.get_logger().info(
            f'Example goal sent: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationDemoGoal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
