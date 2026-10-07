"""
Goal Publisher — grounds YOLO detections to metric positions.

Pipeline:
    /yolo/detections_json (+ typed /yolo/detections)
        + /atlas/rgbd_camera/depth_image  (registered depth, metres or mm)
        + /atlas/rgbd_camera/camera_info  (intrinsics; fallback defaults otherwise)
    -> pinhole back-projection of the bounding-box centroid
    -> optical-axes -> body-axes conversion (x forward, y left, z up)
    -> TF transform camera frame -> map (when available)
    -> /goal_pose (geometry_msgs/PoseStamped, map frame, z = 0)
    -> store_landmark service (object position, not robot position)

Default topics and frames match the official atlas robot bridged by
topic2_official_system.launch.py and the static camera TF published by
topic2_octomap_with_nav2.launch.py (atlas/base_link -> atlas/realsense).
"""

import json
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from yolo_msgs.msg import DetectionArray
    HAS_TYPED_DETECTIONS = True
except Exception:
    DetectionArray = None
    HAS_TYPED_DETECTIONS = False
try:
    from yolo_msgs.srv import StoreLandmark
    HAS_STORE = True
except Exception:
    StoreLandmark = None
    HAS_STORE = False


def rotate_by_quaternion(vec, q):
    """Rotate a 3-vector by quaternion (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = vec
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w * t + cross(q.xyz, t)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Topics / frames (official atlas defaults)
        self.declare_parameter('depth_topic', '/atlas/rgbd_camera/depth_image')
        self.declare_parameter('camera_info_topic', '/atlas/rgbd_camera/camera_info')
        self.declare_parameter('detections_json_topic', '/yolo/detections_json')
        self.declare_parameter('detections_typed_topic', '/yolo/detections')
        self.declare_parameter('camera_frame', 'atlas/realsense')
        self.declare_parameter('map_frame', 'map')
        # The depth image uses optical axes (x right, y down, z forward); the
        # camera TF frame uses body axes (x forward, y left, z up).
        self.declare_parameter('optical_to_body', True)

        # Fallback intrinsics, overwritten by the first CameraInfo message.
        self.declare_parameter('camera_fx', 554.257)
        self.declare_parameter('camera_fy', 554.257)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        # Behaviour
        self.declare_parameter('min_confidence', 0.45)
        self.declare_parameter('fallback_depth', 1.0)
        self.declare_parameter('publish_goals', True)
        self.declare_parameter('goal_cooldown', 5.0)
        # Only sector posters become navigation goals; signs are still
        # stored as landmarks but do not redirect the robot.
        self.declare_parameter('goal_labels', ['orange', 'tree', 'vehicle'])

        gp = self.get_parameter
        self.depth_topic = gp('depth_topic').value
        self.camera_info_topic = gp('camera_info_topic').value
        self.camera_frame = gp('camera_frame').value
        self.map_frame = gp('map_frame').value
        self.optical_to_body = bool(gp('optical_to_body').value)
        self.fx = float(gp('camera_fx').value)
        self.fy = float(gp('camera_fy').value)
        self.cx = float(gp('camera_cx').value)
        self.cy = float(gp('camera_cy').value)
        self.min_confidence = float(gp('min_confidence').value)
        self.fallback_depth = float(gp('fallback_depth').value)
        self.publish_goals = bool(gp('publish_goals').value)
        self.goal_cooldown = float(gp('goal_cooldown').value)
        self.goal_labels = {str(label).lower() for label in gp('goal_labels').value}

        self.intrinsics_from_camera_info = False
        self.last_goal_time = 0.0
        self.last_tf_warn_time = 0.0

        self.bridge = CvBridge()
        self.last_depth = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(
            String, gp('detections_json_topic').value, self.detections_json_cb, 10)
        if HAS_TYPED_DETECTIONS:
            self.create_subscription(
                DetectionArray, gp('detections_typed_topic').value, self.detections_typed_cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.cli = None
        if HAS_STORE:
            self.cli = self.create_client(StoreLandmark, 'store_landmark')
            if not self.cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    'store_landmark service not available yet; landmarks will be '
                    'stored once landmark_db is running')

        self.get_logger().info(
            f'GoalPublisher: depth={self.depth_topic}, camera_info={self.camera_info_topic}, '
            f'camera_frame={self.camera_frame} -> {self.map_frame}, '
            f'goal_labels={sorted(self.goal_labels)}, publish_goals={self.publish_goals}')

    # ------------------------------------------------------------------ #
    #  Sensor callbacks
    # ------------------------------------------------------------------ #
    def camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics_from_camera_info:
            return
        k = msg.k
        if len(k) == 9 and k[0] > 0.0 and k[4] > 0.0:
            self.fx, self.fy, self.cx, self.cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            self.intrinsics_from_camera_info = True
            self.get_logger().info(
                f'Camera intrinsics from {self.camera_info_topic}: '
                f'fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return
        # 16UC1 depth is in millimetres; 32FC1 is in metres.
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        self.last_depth = depth

    def detections_json_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except Exception:
            return
        if isinstance(detections, list):
            self.handle_detections(detections)

    def detections_typed_cb(self, msg):
        self.handle_detections([
            {
                'label': det.label,
                'conf': det.confidence,
                'xyxy': [det.x1, det.y1, det.x2, det.y2],
            }
            for det in msg.detections
        ])

    # ------------------------------------------------------------------ #
    #  Grounding
    # ------------------------------------------------------------------ #
    def depth_at(self, cx, cy):
        """Median depth in a small window around the centroid; None if unknown."""
        if self.last_depth is None:
            return None
        h, w = self.last_depth.shape[:2]
        if not (0 <= cy < h and 0 <= cx < w):
            return None
        y0, y1 = max(0, cy - 2), min(h, cy + 3)
        x0, x1 = max(0, cx - 2), min(w, cx + 3)
        window = self.last_depth[y0:y1, x0:x1].astype(np.float32).ravel()
        window = window[np.isfinite(window) & (window > 0.0)]
        if window.size == 0:
            return None
        return float(np.median(window))

    def back_project(self, cx, cy, z):
        """Pixel (cx, cy) at depth z -> point in the camera TF frame."""
        x_opt = (cx - self.cx) * z / self.fx
        y_opt = (cy - self.cy) * z / self.fy
        z_opt = z
        if self.optical_to_body:
            return (z_opt, -x_opt, -y_opt)
        return (x_opt, y_opt, z_opt)

    def to_map(self, point_cam):
        """Transform a camera-frame point into the map frame; None if TF unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, Time())
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 2.0:
                self.get_logger().warn(
                    f'TF {self.map_frame} <- {self.camera_frame} unavailable: {exc}')
                self.last_tf_warn_time = now
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        rx, ry, rz = rotate_by_quaternion(point_cam, (q.x, q.y, q.z, q.w))
        return (rx + t.x, ry + t.y, rz + t.z)

    def handle_detections(self, detections):
        detections = [d for d in detections if float(d.get('conf', 0.0)) >= self.min_confidence]
        if not detections:
            return
        best = max(detections, key=lambda d: float(d.get('conf', 0.0)))
        label = str(best.get('label', '')).strip().lower()
        x1, y1, x2, y2 = map(int, best.get('xyxy', [0, 0, 0, 0]))
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        z = self.depth_at(cx, cy)
        depth_source = 'depth'
        if z is None:
            z = self.fallback_depth
            depth_source = 'fallback'

        point_cam = self.back_project(cx, cy, z)
        point_map = self.to_map(point_cam)

        if point_map is None:
            frame, point = self.camera_frame, point_cam
        else:
            frame, point = self.map_frame, point_map

        self.get_logger().info(
            f'{label} ({float(best.get("conf", 0.0)):.2f}) grounded at '
            f'({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) in {frame} [{depth_source}]')

        self.store_landmark(label, point, float(best.get('conf', 0.0)))

        if self.publish_goals and label in self.goal_labels and point_map is not None:
            now = time.monotonic()
            if now - self.last_goal_time >= self.goal_cooldown:
                self.publish_goal(point_map, frame)
                self.last_goal_time = now

    def publish_goal(self, point, frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0.0  # 2D navigation goal
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)
        self.get_logger().info(
            f'Published /goal_pose ({point[0]:.2f}, {point[1]:.2f}) in {frame}')

    def store_landmark(self, label, point, confidence):
        if self.cli is None or not self.cli.service_is_ready():
            return
        req = StoreLandmark.Request()
        req.label = label
        req.x, req.y, req.z = (float(v) for v in point)
        req.confidence = float(confidence)
        future = self.cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                self.get_logger().debug(f'Landmark stored: {res.message}')
            except Exception as e:
                self.get_logger().warn(f'Landmark store failed: {e}')

        future.add_done_callback(_done)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
