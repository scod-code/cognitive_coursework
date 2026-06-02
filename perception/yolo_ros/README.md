YOLO ROS2 integration package

Contains:
- `yolo_node`: subscribes to `/camera/image_raw`, runs YOLOv8 inference, publishes `/yolo/detections` (JSON string) and `/yolo/dbg_image` (annotated Image)
- `sign_controller`: subscribes to `/yolo/detections`, publishes `/cmd_vel` responses for `stop`/`slow`/`fast`
- `goal_publisher`: converts detection bbox + `/camera/depth/image_raw` into `/goal_pose` (geometry_msgs/PoseStamped)
- `landmark_db`: ROS2 service to persist detected landmarks (`store_landmark` service)
- `resource_monitor`: logs CPU/memory to CSV and publishes `/resource_monitor` messages (psutil)

Defaults assume model at `/home/somto/ros2_coursework_ws/results/traffic_sign_v1-3/weights/best.pt`.

Build:

```bash
cd ~/ros2_coursework_ws
colcon build --packages-select yolo_ros
source install/setup.bash
ros2 launch yolo_ros yolo_launch.py
```

Dependencies: `ultralytics`, `opencv-python`, `cv_bridge`, `rclpy`.

Note: Build `yolo_msgs` before `yolo_ros` so the typed messages are available:

```bash
cd ~/ros2_coursework_ws
colcon build --packages-select yolo_msgs yolo_ros
```
