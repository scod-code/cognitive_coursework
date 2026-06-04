"""
Sign Controller Node — Behaviour modifier for wall-following navigation.

Architecture:
    wall_follower → /wall_follower/cmd_vel → sign_controller → /cmd_vel → robot

The sign_controller ALWAYS forwards the wall_follower's Twist to /cmd_vel.
When YOLO detects a sign, the controller overrides the linear speed for a
short window, while preserving the wall_follower's angular.z so the robot
keeps following walls.

Behaviours:
    STOP     — zero linear + angular for stop_timeout seconds
    SLOW     — reduce linear speed for override_duration seconds
    FAST     — increase linear speed for override_duration seconds
    COUNT_OBJ — crawl speed, tally detections, publish to /counting/status
"""

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

# Minimum confidence required to act on a detection (filters false positives)
MIN_CONFIDENCE = 0.45


class SignController(Node):
    def __init__(self):
        super().__init__('sign_controller')
        self.declare_parameter('slow_speed', 0.05)
        self.declare_parameter('fast_speed', 0.3)
        self.declare_parameter('count_speed', 0.03)
        self.declare_parameter('stop_timeout', 3.0)
        self.declare_parameter('override_duration', 2.0)  # seconds to hold speed override
        self.slow_speed = float(self.get_parameter('slow_speed').get_parameter_value().double_value)
        self.fast_speed = float(self.get_parameter('fast_speed').get_parameter_value().double_value)
        self.count_speed = float(self.get_parameter('count_speed').get_parameter_value().double_value)
        self.stop_timeout = float(self.get_parameter('stop_timeout').get_parameter_value().double_value)
        self.override_duration = float(self.get_parameter('override_duration').get_parameter_value().double_value)

        # --- Subscriptions ---
        self.sub_json = self.create_subscription(
            String, '/yolo/detections_json', self.detections_json_cb, 10)
        if HAS_TYPED_DETECTIONS:
            self.sub_typed = self.create_subscription(
                DetectionArray, '/yolo/detections', self.detections_typed_cb, 10)

        # Subscribe to wall_follower output — we ALWAYS forward this to /cmd_vel
        self.sub_wall_cmd = self.create_subscription(
            Twist, '/wall_follower/cmd_vel', self._wall_cmd_cb, 10)

        # --- Publishers ---
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_count = self.create_publisher(String, '/counting/status', 10)

        # --- Speed override state ---
        # When a sign is detected, we override the wall_follower's linear speed
        # for a short window.  Outside that window, wall_follower drives unmodified.
        self.speed_override = None        # float or None
        self.override_expires = None      # rclpy.time.Time or None

        # Full-stop state (separate from speed override — blocks ALL motion)
        self.stop_until = None            # rclpy.time.Time or None

        # Sector counting state
        self.sector_counts = {"orange": 0, "tree": 0, "vehicle": 0}
        self.current_sector = None

        self.get_logger().info('SignController initialized — forwarding wall_follower commands')

    # ------------------------------------------------------------------ #
    #  Core forwarding: wall_follower → /cmd_vel (always runs)
    # ------------------------------------------------------------------ #
    def _wall_cmd_cb(self, msg: Twist):
        """Receive wall_follower command and forward to /cmd_vel with any active override."""
        now = self.get_clock().now()

        # 1. If in STOP hold, publish zero and return
        if self.stop_until is not None:
            if now < self.stop_until:
                self.pub_cmd.publish(Twist())  # zero velocity
                return
            else:
                self.stop_until = None  # stop period expired

        # 2. If a speed override is active and not expired, apply it
        cmd = Twist()
        cmd.angular.z = msg.angular.z  # ALWAYS preserve wall-follower turning

        if self.speed_override is not None and self.override_expires is not None:
            if now < self.override_expires:
                cmd.linear.x = self.speed_override
            else:
                # Override expired — clear it, pass through wall_follower speed
                self.speed_override = None
                self.override_expires = None
                cmd.linear.x = msg.linear.x
        else:
            # No override — pass through wall_follower speed unmodified
            cmd.linear.x = msg.linear.x

        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------ #
    #  Detection callbacks
    # ------------------------------------------------------------------ #
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
        # Filter to detections above the confidence threshold
        detections = [d for d in detections if d.get('conf', 0) >= MIN_CONFIDENCE]

        # Pick highest confidence detection
        best = None
        for d in detections:
            if best is None or d.get('conf', 0) > best.get('conf', 0):
                best = d

        if best is None:
            # No confident detections — wall_follower drives unmodified (via _wall_cmd_cb)
            return

        label = best.get('label', '')
        behaviour = BEHAVIOUR_MAP.get(label)
        if behaviour is None:
            return

        now = self.get_clock().now()

        if behaviour == "STOP":
            self.stop_until = now + Duration(seconds=self.stop_timeout)
            self.speed_override = None
            self.override_expires = None
            # Immediately publish zero
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                f'Stop sign detected: holding zero velocity for {self.stop_timeout:.1f}s')

        elif behaviour == "SLOW":
            self.speed_override = self.slow_speed
            self.override_expires = now + Duration(seconds=self.override_duration)
            self.get_logger().info('Slow sign detected: reducing speed')

        elif behaviour == "FAST":
            self.speed_override = self.fast_speed
            self.override_expires = now + Duration(seconds=self.override_duration)
            self.get_logger().info('Fast sign detected: increasing speed')

        elif behaviour == "COUNT_OBJ":
            self.speed_override = self.count_speed
            self.override_expires = now + Duration(seconds=self.override_duration * 2)

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
                f'Current counts: {self.sector_counts}')

            # Publish counting status
            status_msg = String()
            status_msg.data = json.dumps(self.sector_counts)
            self.pub_count.publish(status_msg)


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
