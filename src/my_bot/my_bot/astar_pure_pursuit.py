"""Subscribe to map, AMCL pose, and RViz goals; plan and follow an A* path."""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from my_bot.astar import (
    GridGeometry, inflate_obstacles, normalize_angle, pursuit_command,
    quaternion_yaw, run_astar, segment_is_free,
)


class AStarPurePursuit(Node):
    """Use a static occupancy grid for planning and live laser scans to stop."""

    def __init__(self):
        super().__init__('astar_pure_pursuit')
        defaults = {
            'linear_velocity': 0.12, 'max_angular_velocity': 0.6,
            'lookahead_distance': 0.35, 'goal_tolerance': 0.15,
            'goal_yaw_tolerance': 0.15, 'robot_radius': 0.22,
            'safety_margin': 0.05, 'occupied_threshold': 50,
            'obstacle_stop_distance': 0.30, 'scan_timeout': 1.0,
            'transform_timeout': 1.0, 'base_frame': 'base_footprint',
            'require_safety': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        for name in ('linear_velocity', 'max_angular_velocity',
                     'lookahead_distance', 'goal_tolerance',
                     'goal_yaw_tolerance', 'robot_radius',
                     'obstacle_stop_distance', 'scan_timeout', 'transform_timeout'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
        if not math.isfinite(self.safety_margin) or self.safety_margin < 0.0:
            raise ValueError('safety_margin must be nonnegative and finite')
        if not 1 <= self.occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be between 1 and 100')

        self.map_data = self.geometry = self.blocked = None
        self.map_frame = 'map'
        self.current_pose = [0.0, 0.0]
        self.current_yaw = 0.0
        self.localized = False
        self.goal = None
        self.waypoints = []
        self.path_index = 0
        self.plan_pending = False
        self.last_scan_time = None
        self.front_distance = 0.0
        self.status = ''
        self.safety_stopped = self.require_safety

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', latched)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/astar_costmap', latched)
        self.status_pub = self.create_publisher(String, '/astar_status', latched)
        self.safety_sub = self.create_subscription(
            Bool, '/safety/stop', self.safety_callback, latched)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.set_status('Waiting for /map, /amcl_pose, and /goal_pose')

    def set_status(self, message):
        """Publish and log each state change once."""
        if message != self.status:
            self.status = message
            self.status_pub.publish(String(data=message))
            self.get_logger().info(message)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def clear_path(self):
        self.waypoints = []
        self.path_index = 0
        self.publish_path()

    def safety_callback(self, msg):
        if not self.require_safety:
            return
        self.safety_stopped = msg.data
        if self.safety_stopped:
            self.stop_robot()
            self.goal = None
            self.plan_pending = False
            self.clear_path()
            self.set_status('Safety stop; goal cancelled; reset and send a new goal')

    def map_callback(self, msg):
        """Step 1: reshape the row-major array to map_data[row, column]."""
        if (msg.info.width == 0 or msg.info.height == 0
                or not math.isfinite(msg.info.resolution)
                or msg.info.resolution <= 0.0
                or len(msg.data) != msg.info.width * msg.info.height):
            self.stop_robot()
            self.map_data = self.geometry = self.blocked = None
            self.clear_path()
            self.set_status('Invalid map; stopped')
            return
        self.stop_robot()
        self.map_frame = msg.header.frame_id or 'map'
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))
        self.geometry = GridGeometry(
            msg.info.resolution, msg.info.width, msg.info.height,
            msg.info.origin.position.x, msg.info.origin.position.y,
            quaternion_yaw(msg.info.origin.orientation))
        self.blocked = inflate_obstacles(
            self.map_data, self.geometry.resolution,
            self.robot_radius + self.safety_margin, self.occupied_threshold)
        inflated = OccupancyGrid()
        inflated.header = msg.header
        inflated.info = msg.info
        inflated.data = (self.blocked.astype(np.int8) * 100).ravel().tolist()
        self.costmap_pub.publish(inflated)
        self.clear_path()
        self.plan_pending = self.goal is not None
        self.set_status(f'Map received: {msg.info.width} x {msg.info.height}; '
                        'set a goal with RViz 2D Goal Pose')

    def pose_callback(self, msg):
        """Step 2: accept AMCL initialization in the map frame."""
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            return
        pose = msg.pose.pose
        if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y)):
            return
        self.current_pose = [pose.position.x, pose.position.y]
        self.current_yaw = quaternion_yaw(pose.orientation)
        self.localized = True

    def goal_callback(self, msg):
        """Step 2: a new RViz goal replaces the previous path immediately."""
        self.stop_robot()
        self.clear_path()
        self.goal = None
        self.plan_pending = False
        if self.safety_stopped:
            self.set_status('Safety stop active; reset before sending a goal')
            return
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.set_status(f'Goal must be in {self.map_frame} frame; stopped')
            return
        pose = msg.pose
        goal = (pose.position.x, pose.position.y, quaternion_yaw(pose.orientation))
        if not all(math.isfinite(v) for v in goal):
            self.set_status('Invalid goal; stopped')
            return
        self.goal = goal
        self.plan_pending = True
        self.set_status(f'Goal received: ({goal[0]:.2f}, {goal[1]:.2f}); planning')

    def scan_callback(self, msg):
        """Check a +/-35 degree front sector using scan angles, not indices."""
        values = []
        for index, distance in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if abs(angle) > math.radians(35.0):
                continue
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                values.append(distance)
            elif distance == math.inf and math.isfinite(msg.range_max):
                values.append(msg.range_max)
        self.front_distance = min(values) if values else 0.0
        self.last_scan_time = self.get_clock().now().nanoseconds

    def update_robot_pose(self):
        """Use map-to-base TF for smooth odometry between AMCL updates."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            self.set_status('Waiting for map-to-base transform; stopped')
            return False
        now = self.get_clock().now().nanoseconds
        age = (now - Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        if age > self.transform_timeout or age < -self.transform_timeout:
            self.set_status('Position transform is stale; stopped')
            return False
        translation = transform.transform.translation
        self.current_pose = [translation.x, translation.y]
        self.current_yaw = quaternion_yaw(transform.transform.rotation)
        return all(math.isfinite(v) for v in (*self.current_pose, self.current_yaw))

    def plan_path(self):
        """Step 3: run A* on the footprint-inflated grid."""
        self.stop_robot()
        self.plan_pending = False
        self.clear_path()
        start = self.geometry.world_to_grid(*self.current_pose)
        end = self.geometry.world_to_grid(*self.goal[:2])
        if not all(self.geometry.contains(cell) for cell in (start, end)):
            self.set_status('Start or goal is outside the map; choose another goal')
            self.goal = None
            return
        if self.blocked[start] or self.blocked[end]:
            self.set_status('Start or goal is occupied/too close to a wall; stopped')
            self.goal = None
            return
        cells = run_astar(self.blocked, start, end)
        if cells is None:
            self.set_status('No reachable path; choose another goal')
            self.goal = None
            return
        self.waypoints = [self.geometry.grid_to_world(cell) for cell in cells]
        # Preserve the exact clicked goal rather than only its cell centre.
        self.waypoints.append(self.goal[:2])
        self.publish_path()
        self.set_status(f'Following A* path ({len(cells)} cells)')

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def tracking_target(self):
        """Choose a visible lookahead point without shortcuts through walls."""
        nearest = min(range(self.path_index, len(self.waypoints)),
                      key=lambda i: math.dist(self.current_pose, self.waypoints[i]))
        self.path_index = nearest
        target = None
        for point in self.waypoints[nearest:]:
            if not segment_is_free(self.geometry, self.blocked, self.current_pose, point):
                break
            target = point
            if math.dist(self.current_pose, point) >= self.lookahead_distance:
                break
        return target

    def control_loop(self):
        if self.safety_stopped:
            self.stop_robot()
            return
        if self.goal is None:
            self.stop_robot()
            return
        if self.map_data is None or not self.localized:
            self.stop_robot()
            self.set_status('Waiting for map and AMCL initial pose; stopped')
            return
        if not self.update_robot_pose():
            self.stop_robot()
            return
        if self.plan_pending:
            self.plan_path()
        if self.goal is None or not self.waypoints:
            self.stop_robot()
            return
        now = self.get_clock().now().nanoseconds
        if self.last_scan_time is None or not (
                0.0 <= (now - self.last_scan_time) / 1e9 <= self.scan_timeout):
            self.stop_robot()
            self.set_status('Waiting for a fresh /scan; stopped')
            return
        distance = math.dist(self.current_pose, self.goal[:2])
        cmd = Twist()
        if distance < self.goal_tolerance:
            error = normalize_angle(self.goal[2] - self.current_yaw)
            if abs(error) < self.goal_yaw_tolerance:
                self.stop_robot()
                self.goal = None
                self.set_status('Goal reached; stopped')
                return
            cmd.angular.z = max(-self.max_angular_velocity,
                                min(self.max_angular_velocity, 1.5 * error))
            self.set_status('Aligning final goal orientation')
        else:
            target = self.tracking_target()
            if target is None:
                self.stop_robot()
                self.plan_pending = True
                self.set_status('Robot left the traversable path; replanning')
                return
            speed = min(self.linear_velocity, max(0.03, 0.6 * distance))
            cmd.linear.x, cmd.angular.z = pursuit_command(
                self.current_pose, self.current_yaw, target, speed,
                self.max_angular_velocity)
            if cmd.linear.x > 0.0 and self.front_distance < self.obstacle_stop_distance:
                self.stop_robot()
                self.set_status('Obstacle ahead; stopped (static-map planner)')
                return
            self.set_status('Following A* path')
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = AStarPurePursuit()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
