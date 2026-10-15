"""Run YOLO on the latest Gazebo camera frame and publish actual detections."""

import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloCamera(Node):
    def __init__(self):
        super().__init__('yolo_camera')
        defaults = {
            'model_path': '', 'input_topic': '/camera/image_raw',
            'image_topic': '/yolo/image', 'detections_topic': '/yolo/detections',
            'confidence': 0.35, 'image_size': 640, 'max_fps': 5.0,
            'device': 'cpu', 'cpu_threads': 2, 'target_class': 'stop sign',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if not math.isfinite(self.max_fps) or self.max_fps <= 0:
            raise ValueError('max_fps must be positive')
        if not 0 < self.confidence <= 1:
            raise ValueError('confidence must be in (0, 1]')
        if self.image_size < 32 or self.cpu_threads < 1:
            raise ValueError('image_size or cpu_threads is invalid')
        model_path = Path(self.model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO weights are missing: {model_path}')
        # The launch file selects the isolated interpreter before this import.
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(self.cpu_threads)
        self.model = YOLO(str(model_path))
        # CPU device setup inside Ultralytics resets the thread count. Apply the
        # limit again after setup so inference leaves CPU time for localization.
        self.model.add_callback(
            'on_predict_start',
            lambda _predictor: torch.set_num_threads(self.cpu_threads))
        self.class_ids = None
        if self.target_class:
            self.class_ids = [
                int(i) for i, name in self.model.names.items()
                if name == self.target_class
            ]
            if not self.class_ids:
                raise ValueError(f'Model has no class named {self.target_class}')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.last_processed_stamp = None
        self.last_report = 0.0
        # Keep only the latest frame; do not accumulate a camera-frame backlog.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, self.image_topic, qos)
        self.detection_pub = self.create_publisher(String, self.detections_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.on_image, qos)
        self.timer = self.create_timer(1.0 / self.max_fps, self.detect)
        self.get_logger().info(
            f'YOLO ready: {model_path.name}, device={self.device}; '
            f'{self.input_topic} -> {self.image_topic}')

    def on_image(self, msg):
        if msg.width and msg.height:
            self.latest_frame = msg

    def detect(self):
        msg = self.latest_frame
        if msg is None:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_processed_stamp:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return
        started = time.perf_counter()
        result = self.model.predict(
            source=frame, conf=self.confidence, imgsz=self.image_size,
            device=self.device, classes=self.class_ids,
            verbose=False, save=False)[0]
        self.last_processed_stamp = stamp
        objects = []
        if result.boxes is not None:
            for box in result.boxes.cpu():
                class_id = int(box.cls.item())
                objects.append({
                    'class_id': class_id,
                    'label': result.names[class_id],
                    'confidence': round(float(box.conf.item()), 4),
                    'bbox_xyxy': [round(float(v), 2) for v in box.xyxy[0].tolist()],
                })
        report = {
            'schema_version': 1,
            'confidence_threshold': self.confidence,
            'monitored_classes': [name for index, name in self.model.names.items()
                                  if self.class_ids is None or index in self.class_ids],
            'header': {'frame_id': msg.header.frame_id,
                       'stamp': {'sec': stamp[0], 'nanosec': stamp[1]}},
            'image_width': msg.width, 'image_height': msg.height,
            'inference_ms': round((time.perf_counter() - started) * 1000, 1),
            'detections': objects,
        }
        self.detection_pub.publish(String(data=json.dumps(report)))
        annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
        annotated.header = msg.header
        self.image_pub.publish(annotated)
        now = time.monotonic()
        if now - self.last_report >= 3.0:
            labels = ', '.join(f"{o['label']} {o['confidence']:.2f}" for o in objects)
            self.get_logger().info(
                f"Detected: {labels or 'none'}; {report['inference_ms']:.0f} ms")
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
