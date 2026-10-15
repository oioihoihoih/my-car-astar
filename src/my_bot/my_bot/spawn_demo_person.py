"""Insert an actual textured human model after the simulated robot starts moving."""

import math
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SpawnDemoPerson(Node):
    def __init__(self):
        super().__init__('spawn_demo_person')
        self.start = None
        self.sent = False
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        position = msg.pose.pose.position
        if self.start is None:
            self.start = position.x, position.y
        if self.sent or math.dist(self.start, (position.x, position.y)) < 0.35:
            return
        if not self.client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state.name = 'demo_person'
        request.state.reference_frame = 'world'
        request.state.pose.position.x = 1.5
        request.state.pose.position.y = -0.5
        request.state.pose.orientation.z = math.sin(math.pi / 4)
        request.state.pose.orientation.w = math.cos(math.pi / 4)
        self.sent = True
        future = self.client.call_async(request)
        future.add_done_callback(self.spawned)

    def spawned(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info('Preloaded 3D person entered view after 0.35m of motion')
        else:
            self.get_logger().error('Could not move the demo person into view')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnDemoPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
