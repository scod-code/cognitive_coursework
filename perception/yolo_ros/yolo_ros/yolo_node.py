import os
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
try:
    from yolo_msgs.msg import DetectionArray
    from yolo_ros.typed_publisher import convert
    TYPED_MSGS = True
except Exception:
    DetectionArray = None
    convert = None
    TYPED_MSGS = False

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('model_path', '/home/somto/ros2_coursework_ws/results/trafficsignv2/weights/best.pt')
        self.declare_parameter('confidence', 0.40)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value
        self.model = None
        if YOLO is not None:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'Loaded YOLO model: {model_path}')
            except Exception as e:
                self.get_logger().warn(f'Could not load YOLO model at {model_path}: {e}')
        else:
            self.get_logger().warn('ultralytics YOLO not installed; node will not run inference')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        if TYPED_MSGS:
            self.pub_typed = self.create_publisher(DetectionArray, '/yolo/detections', 10)
            self.pub_json = self.create_publisher(String, '/yolo/detections_json', 10)
        else:
            self.pub_json = self.create_publisher(String, '/yolo/detections', 10)
        self.pub_dbg = self.create_publisher(Image, '/yolo/dbg_image', 10)

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge convert failed: {e}')
            return

        detections = []
        if self.model is not None:
            try:
                results = self.model(cv_image, imgsz=640)
                # results is a list; take first
                r = results[0]
                boxes = r.boxes
                for box in boxes:
                    conf = float(box.conf[0])
                    if conf < self.confidence:
                        continue
                    xyxy = box.xyxy[0].tolist()
                    cls = int(box.cls[0])
                    label = self.model.names.get(cls, str(cls))
                    detections.append({'label': label, 'conf': conf, 'xyxy': xyxy})
            except Exception as e:
                self.get_logger().error(f'YOLO inference failed: {e}')

        # Keep a JSON fallback topic for older local consumers while the typed topic is integrated.
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.pub_json.publish(det_msg)

        # publish typed detections if available
        if TYPED_MSGS and convert is not None:
            try:
                typed = convert(detections)
                self.pub_typed.publish(typed)
            except Exception as e:
                self.get_logger().error(f'Failed to publish typed detections: {e}')

        # draw dbg image
        dbg = cv_image.copy()
        for d in detections:
            x1, y1, x2, y2 = map(int, d['xyxy'])
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(dbg, f"{d['label']} {d['conf']:.2f}", (x1, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            self.pub_dbg.publish(dbg_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
