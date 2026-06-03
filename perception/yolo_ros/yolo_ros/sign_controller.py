import json

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import Twist
try:
    from yolo_msgs.msg import DetectionArray
    HAS_TYPED_DETECTIONS = True
except Exception:
    DetectionArray = None
    HAS_TYPED_DETECTIONS = False

# Detection-to-behaviour mapping (updated for all 6 classes)
BEHAVIOUR_MAP = {
    "stopsign":  "STOP",       # robot halts completely
    "slowsign":  "SLOW",       # robot reduces speed
    "fastsign":  "FAST",       # robot increases speed
    "orange":    "COUNT_OBJ",  # robot slows and counts oranges
    "tree":      "COUNT_OBJ",  # robot slows and counts trees
    "vehicle":   "COUNT_OBJ",  # robot slows and counts vehicles
}

class SignController(Node):
    def __init__(self):
        super().__init__('sign_controller')
        self.declare_parameter('slow_speed', 0.05)
        self.declare_parameter('fast_speed', 0.2)
        self.declare_parameter('count_speed', 0.03)
        self.declare_parameter('stop_timeout', 3.0)
        self.slow_speed = float(self.get_parameter('slow_speed').get_parameter_value().double_value)
        self.fast_speed = float(self.get_parameter('fast_speed').get_parameter_value().double_value)
        self.count_speed = float(self.get_parameter('count_speed').get_parameter_value().double_value)
        self.stop_timeout = float(self.get_parameter('stop_timeout').get_parameter_value().double_value)

        self.sub_json = self.create_subscription(String, '/yolo/detections_json', self.detections_json_cb, 10)
        if HAS_TYPED_DETECTIONS:
            self.sub_typed = self.create_subscription(DetectionArray, '/yolo/detections', self.detections_typed_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_count = self.create_publisher(String, '/counting/status', 10)
        self.stop_until = None
        self.stop_timer = self.create_timer(0.1, self.enforce_stop)

        # Sector counting state
        self.sector_counts = {"orange": 0, "tree": 0, "vehicle": 0}
        self.current_sector = None  # which counting class is currently active

    def detections_json_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except Exception:
            detections = []
        self.handle_detections(detections)

    def detections_typed_cb(self, msg):
        detections = []
        for det in msg.detections:
            detections.append({
                'label': det.label,
                'conf': det.confidence,
                'xyxy': [det.x1, det.y1, det.x2, det.y2],
            })
        self.handle_detections(detections)

    def handle_detections(self, detections):
        # pick highest confidence detection
        best = None
        for d in detections:
            if best is None or d.get('conf', 0) > best.get('conf', 0):
                best = d
        cmd = Twist()
        if best is None:
            # no detections: do nothing
            return
        label = best.get('label', '')
        behaviour = BEHAVIOUR_MAP.get(label)

        if behaviour is None:
            return

        if behaviour == "STOP":
            self.stop_until = self.get_clock().now() + Duration(seconds=self.stop_timeout)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f'Stop sign detected: holding zero velocity for {self.stop_timeout:.1f}s')
        elif behaviour == "SLOW":
            cmd.linear.x = float(self.slow_speed)
            cmd.angular.z = 0.0
            self.get_logger().info('Slow sign detected: reducing speed')
        elif behaviour == "FAST":
            cmd.linear.x = float(self.fast_speed)
            cmd.angular.z = 0.0
            self.get_logger().info('Fast sign detected: increasing speed')
        elif behaviour == "COUNT_OBJ":
            # Slow down for accurate counting
            cmd.linear.x = float(self.count_speed)
            cmd.angular.z = 0.0

            # Count all detections of counting classes in this frame
            count_in_frame = {}
            for d in detections:
                det_label = d.get('label', '')
                if BEHAVIOUR_MAP.get(det_label) == "COUNT_OBJ":
                    count_in_frame[det_label] = count_in_frame.get(det_label, 0) + 1

            # Update sector counts (take the max seen in any single frame)
            for cls, cnt in count_in_frame.items():
                if cnt > self.sector_counts.get(cls, 0):
                    self.sector_counts[cls] = cnt

            self.current_sector = label
            self.get_logger().info(
                f'{label} detected (x{count_in_frame.get(label, 1)} in frame). '
                f'Current counts: {self.sector_counts}'
            )

            # Publish counting status
            status_msg = String()
            status_msg.data = json.dumps(self.sector_counts)
            self.pub_count.publish(status_msg)

        self.pub_cmd.publish(cmd)

    def enforce_stop(self):
        if self.stop_until is None:
            return
        if self.get_clock().now() >= self.stop_until:
            self.stop_until = None
            return
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SignController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
