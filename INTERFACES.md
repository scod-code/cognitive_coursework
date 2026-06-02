# ROS2 Topic Interface Contract
## cognitive_coursework — Agreed between Person A and Person B

This file is the single source of truth for all shared ROS2 topic names.
Both people must use these exact names. Do not rename without updating this file.

---

## Topics

| Topic | Publisher | Subscriber | Message Type | Notes |
|---|---|---|---|---|
| `/camera/image_raw` | Person A (Gazebo bridge) | Person B (yolo_node) | `sensor_msgs/Image` | Camera feed from robot URDF |
| `/yolo/detections` | Person B (yolo_node) | Person A adapter, Person B nodes | `yolo_msgs/DetectionArray` | Typed YOLO output with `detections[]` |
| `/yolo/detections_json` | Person B (yolo_node) | Person B fallback nodes | `std_msgs/String` | JSON fallback: `[{"label": "...", "conf": 0.0, "xyxy": [x1, y1, x2, y2]}]` |
| `/yolo/dbg_image` | Person B (debug_node) | Both (RViz) | `sensor_msgs/Image` | Annotated camera feed with bounding boxes |
| `/cmd_vel` | Person B (sign_controller) | Gazebo diff drive | `geometry_msgs/Twist` | Stop/slow/fast response commands; stop is held for 3 seconds |
| `/goal_pose` | Person B (goal detection node) | Person A (Nav2/adapter) | `geometry_msgs/PoseStamped` | Published from detections using depth when available |

---

## `yolo_msgs/DetectionArray`

| Field | Type | Meaning |
|---|---|---|
| `detections` | `yolo_msgs/Detection[]` | One entry per YOLO bounding box |

## `yolo_msgs/Detection`

| Field | Type | Meaning |
|---|---|---|
| `label` | `string` | Class label, currently `fast`, `slow`, or `stop` |
| `confidence` | `float32` | YOLO confidence score |
| `x1` | `float32` | Bounding box left pixel |
| `y1` | `float32` | Bounding box top pixel |
| `x2` | `float32` | Bounding box right pixel |
| `y2` | `float32` | Bounding box bottom pixel |

---

## Notes

- `/goal_pose` currently uses the `camera_link` frame because `goal_publisher` back-projects from the camera image/depth stream.
- `sign_controller` currently publishes directly to `/cmd_vel`; a dedicated mux or `/navigation_enabled` gate would be cleaner if both wall-following and sign control run at the same time.
