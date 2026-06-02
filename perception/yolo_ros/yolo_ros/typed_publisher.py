# Helper to convert JSON detections to typed yolo_msgs.DetectionArray
import rclpy
from yolo_msgs.msg import Detection as DetectionMsg, DetectionArray


def convert(detections):
    arr = DetectionArray()
    for d in detections:
        m = DetectionMsg()
        m.label = d.get('label','')
        m.confidence = float(d.get('conf',0.0))
        xyxy = d.get('xyxy',[0,0,0,0])
        m.x1 = float(xyxy[0])
        m.y1 = float(xyxy[1])
        m.x2 = float(xyxy[2])
        m.y2 = float(xyxy[3])
        arr.detections.append(m)
    return arr
