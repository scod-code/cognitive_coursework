import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.srv import StoreLandmark


class Topic2YoloLandmarkBridge(Node):
    """Store YOLO detections as map-frame landmark observations."""

    LANDMARK_LABELS = {
        'fastsign',
        'slowsign',
        'stopsign',
        'orange',
        'tree',
        'vehicle',
    }

    def __init__(self):
        super().__init__('topic2_yolo_landmark_bridge')

        self.declare_parameter('detections_topic', '/yolo/detections_json')
        self.declare_parameter('store_service', '/store_landmark')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'atlas/base_link')
        self.declare_parameter('min_confidence', 0.40)
        self.declare_parameter('cooldown_sec', 2.0)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.store_service = self.get_parameter('store_service').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.store_client = self.create_client(StoreLandmark, self.store_service)
        self.last_stored_by_label = {}
        self.last_service_warn_time = 0.0
        self.last_tf_warn_time = 0.0

        self.create_subscription(
            String,
            self.detections_topic,
            self.detections_cb,
            10,
        )

        self.get_logger().info(
            'Topic 2 YOLO landmark bridge started: '
            f'{self.detections_topic} -> {self.store_service}, '
            f'TF {self.target_frame} <- {self.source_frame}, '
            f'cooldown={self.cooldown_sec:.1f}s'
        )

    def detections_cb(self, msg):
        try:
            detections = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Ignoring invalid YOLO JSON: {exc}')
            return

        if not isinstance(detections, list):
            self.get_logger().warn('Ignoring YOLO JSON because it is not a list')
            return

        transform = self.lookup_robot_pose()
        if transform is None:
            return

        robot_x = float(transform.transform.translation.x)
        robot_y = float(transform.transform.translation.y)
        now = time.monotonic()

        for detection in detections:
            label = str(detection.get('label', '')).strip().lower()
            if label not in self.LANDMARK_LABELS:
                continue

            confidence = float(detection.get('conf', 0.0))
            if confidence < self.min_confidence:
                continue

            last_stored = self.last_stored_by_label.get(label, 0.0)
            if now - last_stored < self.cooldown_sec:
                continue

            self.store_landmark(label, robot_x, robot_y, confidence)
            self.last_stored_by_label[label] = now

    def lookup_robot_pose(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 2.0:
                self.get_logger().warn(
                    f'Cannot store landmark yet; TF lookup failed '
                    f'{self.target_frame} <- {self.source_frame}: {exc}'
                )
                self.last_tf_warn_time = now
            return None

    def store_landmark(self, label, x, y, confidence):
        if not self.store_client.service_is_ready():
            now = time.monotonic()
            if now - self.last_service_warn_time > 2.0:
                self.get_logger().warn(
                    f'Cannot store {label}; service {self.store_service} is not ready'
                )
                self.last_service_warn_time = now
            return

        request = StoreLandmark.Request()
        request.label = label
        request.x = float(x)
        request.y = float(y)
        request.z = 0.0
        request.confidence = float(confidence)

        future = self.store_client.call_async(request)
        future.add_done_callback(
            lambda done_future, req=request: self.store_done_cb(done_future, req)
        )

    def store_done_cb(self, future, request):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to store landmark {request.label}: {exc}'
            )
            return

        if response.success:
            self.get_logger().info(
                f'Stored landmark {request.label} at '
                f'({request.x:.2f}, {request.y:.2f}, {request.z:.2f}), '
                f'confidence={request.confidence:.2f}'
            )
        else:
            self.get_logger().warn(
                f'Landmark service rejected {request.label}: {response.message}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = Topic2YoloLandmarkBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
