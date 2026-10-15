"""Robot-local backup: request torque off if the safety gate's heartbeat stops."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import SetBool


class ActuatorWatchdog(Node):
    def __init__(self):
        super().__init__('actuator_watchdog')
        self.declare_parameter('cmd_topic', '/cmd_vel_safe')
        self.last = None
        self.last_attempt = 0.0
        self.disabled = False
        self.pending = None
        self.client = self.create_client(SetBool, '/motor_power')
        self.create_subscription(Twist, self.get_parameter('cmd_topic').value, self.received, 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.check, clock=self.steady_clock)

    def received(self, _msg):
        self.last = time.monotonic()
        self.disabled = False

    def check(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                self.disabled = self.pending.result().success
            except Exception as error:
                self.get_logger().error(str(error))
            self.pending = None
        if self.last is not None and time.monotonic() - self.last <= 0.5:
            self.disabled = False
            return
        if self.disabled or time.monotonic() - self.last_attempt < 1.0:
            return
        self.last_attempt = time.monotonic()
        if self.client.service_is_ready():
            self.pending = self.client.call_async(SetBool.Request(data=False))
            self.get_logger().error('Safety command heartbeat missing; requesting motor torque off')


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
