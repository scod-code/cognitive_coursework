# yolo_ros — YOLOv8 ROS 2 integration (Person B)

Nodes (all `ros2 run yolo_ros <name>`):

| Node | Subscribes | Publishes / calls | Purpose |
| --- | --- | --- | --- |
| `yolo_node` | `/atlas/rgbd_camera/image` | `/yolo/detections_json` (String JSON), `/yolo/detections` (`yolo_msgs/DetectionArray`), `/yolo/dbg_image` | YOLOv8n inference, six classes |
| `sign_controller` | `/yolo/detections_json`, `/yolo/detections`, `/wall_follower/cmd_vel` | `/atlas/cmd_vel`, `/sign_controller/sector_counts` | Forwards wall-follower motion with STOP / SLOW / FAST / count-crawl overrides |
| `goal_publisher` | `/yolo/detections_json`, `/atlas/rgbd_camera/depth_image`, `/atlas/rgbd_camera/camera_info` | `/goal_pose` (map frame), `store_landmark` | Depth back-projection of detections to map coordinates; sector posters become Nav2 goals |
| `landmark_db` | — | `store_landmark` service | SQLite persistence (`~/ros2_coursework_ws/perception/landmarks.db`) |
| `resource_monitor` | — | `/resource_monitor`, CSV | 1 Hz CPU / RAM metacognition log |
| `curiosity_explorer` | `/projected_map` (OctoMap 2D projection), `/odom` | `/goal_pose`, `/curiosity_explorer/metrics` | Frontier vs novelty-weighted exploration goals |

In the Topic 2 demo, `ros2 launch wall_follower topic2_full_demo.launch.py` starts `yolo_node`
(override weights with `model_path:=`), and `goal_publisher` / `curiosity_explorer` with
`enable_goal_publisher:=true` / `enable_curiosity:=true`.

Full topic contract: `../../INTERFACES.md`.

## Model

Default model path: `~/ros2_coursework_ws/results/trafficsignv2/weights/best.pt`
(the committed six-class model). Override with `-p model_path:=...`.
Training evidence for this model is `evidence/results.csv` (100 epochs) and
`evidence/training_results_summary.md`; the earlier 50-epoch run is in
`results/traffic_sign_v1-3/`.

## Build and run

```bash
cd ~/ros2_coursework_ws
colcon build --packages-select yolo_msgs yolo_ros   # yolo_msgs first
source install/setup.bash
ros2 launch yolo_ros yolo_launch.py
```

`yolo_launch.py` also starts `wall_follower_node`, which needs `/scan`; run
`ros2 launch wall_follower topic2_official_adapters.launch.py` beforehand on the official
atlas robot. Because `sign_controller` drives `/atlas/cmd_vel`, do not run it while Nav2
is executing goals.

Dependencies: `ultralytics`, `opencv-python`, `psutil`, `cv_bridge`, `tf2_ros`, `rclpy`.
