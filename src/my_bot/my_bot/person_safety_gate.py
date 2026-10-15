"""The only velocity publisher between navigation and the robot actuator."""

import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from my_bot.person_stop_policy import PersonStopPolicy


class PersonSafetyGate(Node):
    def __init__(self):
        super().__init__('person_safety_gate')
        defaults = {
            'input_cmd_topic': '/cmd_vel_nav', 'output_cmd_topic': '/cmd_vel',
            'detections_topic': '/yolo/detections',
            'auto_arm_startup': False, 'motor_power_on_trip': False,
            'motor_power_service': '/motor_power',
            'detection_timeout': 1.0, 'max_frame_age': 1.0,
            'command_timeout': 0.25, 'clear_time': 1.0,
            'person_confidence': 0.35, 'max_linear': 0.12, 'max_angular': 0.6,
            'state_file': str(Path.home() / '.ros/my_bot' /
                              ('person_stop_' + os.environ.get('ROS_DOMAIN_ID', '0') + '.json')),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if self.input_cmd_topic == self.output_cmd_topic:
            raise ValueError('Safety input and actuator output topics must differ')
        if not all(math.isfinite(v) and v > 0 for v in (self.max_linear, self.max_angular)):
            raise ValueError('Velocity limits must be positive and finite')
        self.state_path = Path(self.state_file).expanduser()
        restored = None
        if self.state_path.exists():
            try:
                restored = json.loads(self.state_path.read_text())['latched_reason']
                if restored is not None and not isinstance(restored, str):
                    raise ValueError('Invalid saved latch')
            except (OSError, ValueError, KeyError, TypeError):
                restored = 'Cannot read saved safety latch'
        self.policy = PersonStopPolicy(
            self.detection_timeout, self.max_frame_age, self.command_timeout,
            self.clear_time, self.person_confidence, self.auto_arm_startup, restored)
        self.saved_reason = object()  # Check state-file writability before arming.
        self.last_status = None
        self.last_graph_check = 0.0
        self.last_motor_attempt = 0.0
        self.motor_future = None
        self.motor_disabled = False
        self.motor_client = self.create_client(SetBool, self.motor_power_service)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output_pub = self.create_publisher(Twist, self.output_cmd_topic, 1)
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', latched)
        self.status_pub = self.create_publisher(String, '/safety/status', latched)
        self.create_subscription(Twist, self.input_cmd_topic, self.on_command, 1)
        self.create_subscription(String, self.detections_topic, self.on_detection, 1)
        self.create_service(Trigger, '/safety/reset', self.on_reset)
        self.create_service(Trigger, '/safety/stop_now', self.on_manual_stop)
        # Wall/steady time keeps the watchdog running even if /clock pauses.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)
        self.publish_zero()
        self.stop_pub.publish(Bool(data=True))
        self.tick()

    def publish_zero(self):
        self.output_pub.publish(Twist())

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.policy.set_command(values, time.monotonic())

    def on_detection(self, msg):
        try:
            report = json.loads(msg.data)
        except (ValueError, TypeError):
            self.policy.clear_since, self.policy.clear_count = None, 0
            self.policy.trip('Invalid perception JSON')
        else:
            self.policy.observe(report, time.monotonic(), self.get_clock().now().nanoseconds)
        # Stop from this callback as well as from the periodic watchdog.
        if self.policy.reason:
            self.publish_zero()
        self.tick()

    def persist(self):
        if self.saved_reason == self.policy.reason:
            return True
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.state_path.with_suffix('.tmp')
            with tmp.open('w') as stream:
                json.dump({'latched_reason': self.policy.reason}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            tmp.replace(self.state_path)
            self.saved_reason = self.policy.reason
            return True
        except OSError as error:
            self.policy.trip('Cannot persist safety latch')
            self.publish_zero()
            self.get_logger().error(f'Safety state write failed: {error}')
            return False

    def foreign_publishers(self):
        publishers = self.get_publishers_info_by_topic(self.output_cmd_topic)
        if len(publishers) > 1:
            return [p.node_name for p in publishers]
        return [p.node_name for p in publishers
                if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]

    def tick(self):
        now = time.monotonic()
        if now - self.last_graph_check >= 0.5:
            if self.foreign_publishers():
                self.policy.trip('Another node is publishing to the actuator topic')
            self.last_graph_check = now
        values, reason = self.policy.output(now)
        msg = Twist()
        msg.linear.x = max(-self.max_linear, min(self.max_linear, values[0]))
        msg.angular.z = max(-self.max_angular, min(self.max_angular, values[5]))
        self.output_pub.publish(msg)
        allowed, reason = self.policy.check(now)
        if not self.persist():
            allowed, reason = False, self.policy.reason
        self.stop_pub.publish(Bool(data=not allowed))
        state = (not allowed, self.policy.reason, reason)
        if state != self.last_status:
            self.status_pub.publish(String(data=json.dumps({
                'stopped': not allowed, 'latched': self.policy.reason is not None,
                'reason': reason, 'input': self.input_cmd_topic,
                'output': self.output_cmd_topic})))
            self.get_logger().info(f"{'STOPPED' if not allowed else 'ARMED'}: {reason}")
            self.last_status = state
        if self.motor_power_on_trip and self.policy.reason:
            self.disable_motors(now)

    def disable_motors(self, now):
        if self.motor_future is not None:
            if not self.motor_future.done():
                return
            try:
                self.motor_disabled = self.motor_future.result().success
            except Exception as error:
                self.get_logger().error(f'Motor disable request failed: {error}')
            self.motor_future = None
        if self.motor_disabled or now - self.last_motor_attempt < 1.0:
            return
        self.last_motor_attempt = now
        if self.motor_client.service_is_ready():
            self.motor_future = self.motor_client.call_async(SetBool.Request(data=False))

    def on_manual_stop(self, _request, response):
        self.policy.trip('Manual emergency stop')
        self.publish_zero()
        self.tick()
        response.success, response.message = True, 'Motion inhibited; manual reset required'
        return response

    def on_reset(self, _request, response):
        self.policy.check(time.monotonic())
        if self.foreign_publishers():
            response.success, response.message = False, 'Remove other actuator-topic publishers first'
            return response
        if self.motor_power_on_trip and not self.motor_client.service_is_ready():
            response.success, response.message = False, 'Motor power service is unavailable'
            return response
        if self.motor_power_on_trip:
            self.disable_motors(time.monotonic())
            if self.motor_future is not None or not self.motor_disabled:
                response.success, response.message = False, 'Wait for motor torque-off acknowledgement'
                return response
        old_reason = self.policy.reason
        response.success, response.message = self.policy.reset(time.monotonic())
        if response.success and not self.persist():
            self.policy.trip(old_reason or 'Cannot persist safety latch')
            response.success, response.message = False, 'Could not save reset; remaining stopped'
        if response.success:
            self.motor_disabled = False
            if self.motor_power_on_trip:
                response.message += '; explicitly enable /motor_power before sending the goal'
        self.tick()
        return response


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = PersonSafetyGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_zero()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
