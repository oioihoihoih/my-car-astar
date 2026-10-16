"""Send compact sensor state to a local/PC Laya server without blocking ROS."""

import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LayaBridge(Node):
    def __init__(self):
        super().__init__('laya_bridge')
        for name, value in {'server_url': 'http://127.0.0.1:8765/decide',
                            'request_period': 0.5, 'request_timeout': 2.0,
                            'sensor_timeout': 1.0, 'obstacle_distance': 0.30}.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        address = urlparse(self.server_url)
        if (address.scheme not in ('http', 'https') or not address.hostname
                or address.username or address.password):
            raise ValueError('Invalid Laya server URL')
        for value in (self.request_period, self.request_timeout,
                      self.sensor_timeout, self.obstacle_distance):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Laya bridge timeouts and distances must be positive')
        self.token = os.environ.get('LAYA_API_TOKEN', '')
        self.lock = threading.Lock()
        self.camera_at = self.lidar_at = None
        self.last_capture = None
        self.person_visible, self.front_clear = False, False
        self.front_distance = None
        self.latest_result = None
        self.stop_event = threading.Event()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.decisions = self.create_publisher(String, '/laya/decision', 1)
        self.status = self.create_publisher(String, '/laya/status', latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/yolo/detections', self.on_detection, 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.publish_result, clock=self.steady_clock)
        self.worker = threading.Thread(target=self.request_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'Laya bridge -> {address.scheme}://{address.netloc}{address.path}')

    def on_scan(self, msg):
        values = []
        for i, value in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > math.pi / 6:
                continue
            if value == math.inf and msg.range_max > 0:
                values.append(msg.range_max)
            elif math.isfinite(value) and msg.range_min <= value <= msg.range_max and value > 0:
                values.append(value)
        capture = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - capture) / 1e9
        with self.lock:
            self.lidar_at = (time.monotonic() - max(0.0, age)
                             if values and -0.2 <= age <= self.sensor_timeout else None)
            self.front_distance = min(values) if values else None
            self.front_clear = bool(values) and min(values) > self.obstacle_distance

    def on_detection(self, msg):
        try:
            report = json.loads(msg.data)
            stamp = report['header']['stamp']
            capture = stamp['sec'] * 10**9 + stamp['nanosec']
            age = (self.get_clock().now().nanoseconds - capture) / 1e9
            if (report['schema_version'] != 1 or 'person' not in report['monitored_classes']
                    or not 0 < report['confidence_threshold'] <= 0.35
                    or not -0.2 <= age <= self.sensor_timeout):
                raise ValueError('Invalid perception state')
            people = any((d['label'] == 'person' or d['class_id'] == 0)
                         and d['confidence'] >= 0.35 for d in report['detections'])
        except (KeyError, TypeError, ValueError):
            with self.lock:
                self.camera_at = None
            return
        with self.lock:
            if self.last_capture is not None and capture <= self.last_capture:
                return
            self.last_capture = capture
            self.camera_at = time.monotonic() - max(0.0, age)
            self.person_visible = people

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {'camera_fresh': self.camera_at is not None
                    and now - self.camera_at <= self.sensor_timeout,
                    'lidar_fresh': self.lidar_at is not None
                    and now - self.lidar_at <= self.sensor_timeout,
                    'person_visible': self.person_visible,
                    'front_clear': self.front_clear,
                    'front_distance_m': self.front_distance}

    def request_loop(self):
        # Never use an environment HTTP proxy for robot-to-local/PC traffic.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.stop_event.is_set():
            started = time.monotonic()
            request_id = str(uuid.uuid4())
            headers = {'Content-Type': 'application/json'}
            if self.token:
                headers['Authorization'] = 'Bearer ' + self.token
            request = urllib.request.Request(self.server_url, headers=headers, data=json.dumps({
                'schema_version': 1, 'request_id': request_id, 'state': self.snapshot()
            }, allow_nan=False).encode())
            try:
                with opener.open(request, timeout=self.request_timeout) as response:
                    data = response.read(16385)
                if len(data) > 16384:
                    raise ValueError('Laya reply is too large')
                result = json.loads(data)
                if result['request_id'] != request_id or time.monotonic() - started > self.request_timeout:
                    raise ValueError('Mismatched or delayed Laya reply')
                item = (result, started, None)
            except (OSError, urllib.error.URLError, ValueError, KeyError, TypeError) as error:
                item = (None, started, type(error).__name__)
            with self.lock:
                self.latest_result = item
            self.stop_event.wait(max(0.0, self.request_period - (time.monotonic() - started)))

    def publish_result(self):
        with self.lock:
            item, self.latest_result = self.latest_result, None
        if item is None:
            return
        result, started, error = item
        if error:
            self.status.publish(String(data=json.dumps({'connected': False, 'error': error})))
            # The safety gate independently expires the last real response.
            return
        result['request_age_s'] = round(time.monotonic() - started, 6)
        self.decisions.publish(String(data=json.dumps(result, allow_nan=False)))
        self.status.publish(String(data=json.dumps(dict(result, connected=True), allow_nan=False)))

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=0.2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LayaBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
