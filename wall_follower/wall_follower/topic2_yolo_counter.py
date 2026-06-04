import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Topic2YoloCounter(Node):
    """Count object labels from YOLO detections and publish coursework status."""

    COUNT_LABELS = ('orange', 'tree', 'vehicle')

    def __init__(self):
        super().__init__('topic2_yolo_counter')

        self.declare_parameter('input_topic', '/yolo/detections_json')
        self.declare_parameter('output_topic', '/counting/status')
        self.declare_parameter('min_confidence', 0.45)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)

        self.status_pub = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(
            String,
            self.input_topic,
            self.detections_cb,
            10,
        )

        self.get_logger().info(
            'Topic 2 YOLO counter started: '
            f'{self.input_topic} -> {self.output_topic}'
        )

    def detections_cb(self, msg):
        try:
            detections = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Ignoring invalid YOLO JSON: {exc}')
            detections = []

        counts = {label: 0 for label in self.COUNT_LABELS}

        for detection in detections:
            label = str(detection.get('label', '')).strip().lower()
            confidence = float(detection.get('conf', 0.0))
            if confidence < self.min_confidence:
                continue
            if label in counts:
                counts[label] += 1

        status = String()
        status.data = json.dumps(counts)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = Topic2YoloCounter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
