import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.declare_parameter('camera_fx', 554.257)
        self.declare_parameter('camera_fy', 554.257)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.fx = float(self.get_parameter('camera_fx').get_parameter_value().double_value)
        self.fy = float(self.get_parameter('camera_fy').get_parameter_value().double_value)
        self.cx = float(self.get_parameter('camera_cx').get_parameter_value().double_value)
        self.cy = float(self.get_parameter('camera_cy').get_parameter_value().double_value)

        self.bridge = CvBridge()
        self.last_depth = None
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.sub_det_json = self.create_subscription(String, '/yolo/detections_json', self.detections_json_cb, 10)
        if HAS_TYPED_DETECTIONS:
            self.sub_det_typed = self.create_subscription(DetectionArray, '/yolo/detections', self.detections_typed_cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # service client to store landmarks
        if HAS_STORE:
            self.cli = self.create_client(StoreLandmark, 'store_landmark')
            self.get_logger().info('Waiting for store_landmark service...')
            if not self.cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('store_landmark service not available')

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth = depth
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

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
        if not detections:
            return
        # Filter to confident detections only
        detections = [d for d in detections if d.get('conf', 0) >= 0.45]
        if not detections:
            return
        # pick best detection
        best = max(detections, key=lambda d: d.get('conf',0))
        x1, y1, x2, y2 = map(int, best.get('xyxy', [0,0,0,0]))
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        z = None
        if self.last_depth is not None:
            h, w = self.last_depth.shape
            if 0 <= cy < h and 0 <= cx < w:
                z = float(self.last_depth[cy, cx])
        if z is None or z == 0 or np.isnan(z):
            # fallback distance
            z = 1.0
        # backproject to camera frame
        X = (cx - self.cx) * z / self.fx
        Y = (cy - self.cy) * z / self.fy
        Z = z
        pose = PoseStamped()
        pose.header.frame_id = 'camera_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(X)
        pose.pose.position.y = float(Y)
        pose.pose.position.z = float(Z)
        # orientation left as identity
        self.pub.publish(pose)
        self.get_logger().info(f'Published goal pose at ({X:.2f},{Y:.2f},{Z:.2f})')
        # optionally call landmark DB
        if HAS_STORE and hasattr(self, 'cli') and self.cli is not None and self.cli.service_is_ready():
            req = StoreLandmark.Request()
            req.label = best.get('label','')
            req.x = float(X)
            req.y = float(Y)
            req.z = float(Z)
            req.confidence = float(best.get('conf',0.0))
            fut = self.cli.call_async(req)
            # not waiting synchronously; log when done
            def _cb(f):
                try:
                    res = f.result()
                    self.get_logger().info(f'Landmark stored: {res.message}')
                except Exception as e:
                    self.get_logger().error(f'Failed to store landmark: {e}')
            fut.add_done_callback(_cb)


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
