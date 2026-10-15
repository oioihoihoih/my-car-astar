# pure_pursuit_obstacle.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt, sin, cos, pi

class PurePursuitLogic(Node):
    def __init__(self):
        super().__init__('pure_pursuit_obstacle')

        # 사용자가 정해야 하는 파라미터 삼형제
        self.lookahead_distance = 0.5 # 전방 주시 거리
        self.linear_velocity = 0.15 # 이동 속도
        self.obs_threshold = 0.45 # 장애물 감지 거리

        # 목표 위치의 맵 상의 좌표를 정확하게 써!
        self.path = [
            [3.4495,  2.0139]
        ]
        self.current_waypoint_index = 0

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_localized = False

        # initial value.... 99.9 means 'no obstacle'
        self.left_dist = 99.9
        self.front_dist = 99.9
        self.right_dist = 99.9

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Let's Go!")

    # 위치와 회전 상태를 받으면 그에 대응하여 각도를 계산하는!
    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.is_localized = True

    def scan_callback(self, msg):

		    # 장애물 바라보는 각도
        front_ranges = msg.ranges[0:40] + msg.ranges[-40:]

        # 좌우 비교를 위한 각도
        left_ranges = msg.ranges[20:70]
        right_ranges = msg.ranges[-70:-20]

        self.front_dist = self.get_min_dist(front_ranges)
        self.left_dist = self.get_min_dist(left_ranges)
        self.right_dist = self.get_min_dist(right_ranges)

    # 바라보는 각도 중에서 가장 가깝게 감지되는 것을 구한다
    def get_min_dist(self, range_list):

        valid_values = [x for x in range_list if x > 0.05 and x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    # 주기적으로 나의 주행을 계산하는 함수
    def control_loop(self):
        if not self.is_localized:
            return

        if self.front_dist < self.obs_threshold:
            self.get_logger().warn(f"Obstacle Ahead! ({self.front_dist:.2f}m)")
            self.avoid_obstacle()
            return

        self.follow_path()

    # 장애물 회피 로직
    def avoid_obstacle(self):

        cmd = Twist()
        cmd.linear.x = 0.0

        # 좌우로 트는 각도에 따라 잘 피할수도 못 피할수도
        if self.left_dist > self.right_dist:
            cmd.angular.z = 0.7
            self.get_logger().info("Turn Left!")
        else:
            cmd.angular.z = -0.7
            self.get_logger().info("Turn Right!")

        self.publisher_.publish(cmd)

    def follow_path(self):
        if self.current_waypoint_index >= len(self.path):
            cmd = Twist()
            self.publisher_.publish(cmd)
            return


        goal_x = self.path[self.current_waypoint_index][0]
        goal_y = self.path[self.current_waypoint_index][1]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = sqrt(pow(dx, 2) + pow(dy, 2))

        if distance < 0.2: # Goal tolerance
            self.current_waypoint_index += 1
            return

        target_angle = atan2(dy, dx)
        alpha = target_angle - self.current_yaw
        if alpha > pi: alpha -= 2 * pi
        elif alpha < -pi: alpha += 2 * pi

        cmd = Twist()
        cmd.linear.x = self.linear_velocity

        cmd.angular.z = self.linear_velocity * (2.0 * sin(alpha)) / distance

        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0


        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
