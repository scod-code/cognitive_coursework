# ROS2 Topic Interface Contract
## cognitive_coursework — Agreed between Person A and Person B

This file is the single source of truth for all shared ROS2 topic names.
Both people must use these exact names. Do not rename without updating this file.

---

## Topics

| Topic | Publisher | Subscriber | Message Type | Notes |
|---|---|---|---|---|
| `/camera/image_raw` | Person A (Gazebo bridge) | Person B (yolo_node) | `sensor_msgs/Image` | Camera feed from robot URDF |
| `/yolo/detections` | Person B (yolo_node) | Both (debug/logging) | `yolo_msgs/DetectionArray` | Raw YOLO output — bboxes, labels, confidence |
| `/yolo/dbg_image` | Person B (debug_node) | Both (RViz) | `sensor_msgs/Image` | Annotated camera feed with bounding boxes |
| `/detection_cmd` | Person B (sign response node) | Person A (nav stack) | `std_msgs/String` | Commands: `"STOP"`, `"FAST"`, `"SLOW"`, `"CLEAR"` |
| `/goal_pose` | Person B (goal detection node) | Person A (Nav2) | `geometry_msgs/PoseStamped` | Published when goal marker is detected |

---

## Command Values for `/detection_cmd`

| Value | Meaning | Expected nav behaviour |
|---|---|---|
| `"STOP"` | Stop sign detected | Robot halts for 3 seconds |
| `"FAST"` | Fast sign detected | Increase linear velocity |
| `"SLOW"` | Slow sign detected | Reduce linear velocity |
| `"CLEAR"` | No sign in frame | Resume default wall-following speed |

---

## Notes

- `/goal_pose` uses the `geometry_msgs/PoseStamped` frame `map` — Nav2 standard input
- `/detection_cmd` uses plain strings for simplicity; upgrade to a custom action if time allows
- Person A's wall-following node should default to normal speed until a `/detection_cmd` is received
