# ROS 2 Interface Contract

## cognitive_coursework — shared topic, service, and frame names

This file is the single source of truth for the names the nodes in this
repository use to talk to each other and to the official atlas robot. If a
name changes in code, change it here in the same commit.

The robot is the official `atlas` model spawned by the module-provided
`ntu_robotsim` package (cwmaze world) and bridged by
`wall_follower/launch/topic2_official_system.launch.py`. The whole graph below is started by
`ros2 launch wall_follower topic2_full_demo.launch.py`.

---

## Frames

| Frame | Published by | Notes |
| --- | --- | --- |
| `map` → `odom` | `topic2_nav2_tf_helper` | Fixed identity. Localisation is ground-truth odometry, not AMCL. |
| `odom` → `atlas/base_link` | `topic2_nav2_tf_helper` | From `/atlas/odom_ground_truth` |
| `atlas/base_link` → `atlas/realsense` | `topic2_octomap_with_nav2.launch.py` (static) | (0.10, 0, 0.19), zero rotation; body axes (x forward) |

---

## Sensor and actuator topics (Gazebo bridge)

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/atlas/rgbd_camera/image` | `sensor_msgs/Image` | Gazebo → ROS | RGB input to `yolo_node` |
| `/atlas/rgbd_camera/depth_image` | `sensor_msgs/Image` | Gazebo → ROS | Registered depth (32FC1 m or 16UC1 mm); input to `goal_publisher` |
| `/atlas/rgbd_camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | Intrinsics; read once by `goal_publisher` |
| `/atlas/rgbd_camera/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS | Input to `topic2_pointcloud_filter` and the optional `/scan` adapter |
| `/atlas/odom_ground_truth` | `nav_msgs/Odometry` | Gazebo → ROS | Input to `topic2_nav2_tf_helper` |
| `/atlas/imu/data` | `sensor_msgs/Imu` | Gazebo → ROS | Bridged, currently unused |
| `/atlas/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | **The only velocity topic the robot listens on.** Published by Nav2 `controller_server`/`behavior_server` (remapped) and by `sign_controller` |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | `use_sim_time: true` everywhere |

---

## Navigation and mapping

| Topic | Type | Publisher | Subscriber(s) | Notes |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | `topic2_nav2_tf_helper` | Nav2, `curiosity_explorer` | Republished ground truth in `odom`/`atlas/base_link` frames |
| `/map` | `nav_msgs/OccupancyGrid` | Nav2 `map_server` (latched) | Nav2 costmaps | Static pre-built map `topic2_nav2_clean_map.yaml` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose, `goal_publisher`, `curiosity_explorer` | `bt_navigator`, `topic2_goal_pose_bridge` | Must be in `map` frame. Note `bt_navigator` already consumes `/goal_pose` natively; the bridge duplicates this and each click produces two `NavigateToPose` goals (second preempts first). The bridge is kept by default; disable it with `use_goal_pose_bridge:=false` on `topic2_full_demo.launch.py` / `topic2_nav2.launch.py` |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | — | `topic2_goal_pose_bridge`, `pomdp_goal_selector`, `autonomous_mission` (clients) | `autonomous_mission` keeps at most one goal active and waits for its result |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | — | `autonomous_mission` (client) | Candidate validation and planned-path-length ranking |
| `/autonomous_mission/outcome` | `std_msgs/String` (JSON) | `autonomous_mission` (latched) | monitoring | `COMPLETE` / `PARTIAL` / `FAILED_SAFE` / `ABORTED_BY_OPERATOR` + reason. The node also reads `/map`, `/yolo/detections_json`, `/traffic_rule_state`, and watches `/goal_pose` to detect a competing goal source |
| `/speed_limit` | `nav2_msgs/SpeedLimit` | `topic2_nav2_traffic_rules` | Nav2 `controller_server` | Absolute (`percentage=false`). Requires DWB `max_vel_x` ≥ highest limit (0.28) |
| `/traffic_rule_state` | `std_msgs/String` (JSON `{"rule","speed_limit"}`) | `topic2_nav2_traffic_rules` | monitoring | NORMAL 0.18 / FAST 0.28 / SLOW 0.05 / STOP 0.01, 2.5 s expiry |
| `/atlas/rgbd_camera/points_filtered` | `sensor_msgs/PointCloud2` | `topic2_pointcloud_filter` | `octomap_server` | |
| `/octomap_binary`, `/octomap_full` | `octomap_msgs/Octomap` | `octomap_server` | RViz | |
| `/occupied_cells_vis_array` | `visualization_msgs/MarkerArray` | `octomap_server` | RViz | The "blue voxels" |
| `/projected_map` | `nav_msgs/OccupancyGrid` | `octomap_server` (latched) | `curiosity_explorer` | 2D projection that grows as the robot explores |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` via `topic2_official_adapters.launch.py` | `wall_follower_node` | **Only exists if the adapters launch is running.** Not part of `topic2_full_demo.launch.py` |
| `/wall_follower/cmd_vel` | `geometry_msgs/Twist` | `wall_follower_node` | `sign_controller` | Baseline motion for the Person-B stack |
| `/atlas/footprint_marker` | `visualization_msgs/Marker` | `topic2_nav2_tf_helper` | RViz | |

---

## Perception and cognition

| Topic / Service | Type | Publisher | Subscriber(s) | Notes |
| --- | --- | --- | --- | --- |
| `/yolo/detections_json` | `std_msgs/String` | `yolo_node` | `topic2_nav2_traffic_rules`, `topic2_yolo_counter`, `topic2_yolo_landmark_bridge`, `sign_controller`, `goal_publisher`, `pomdp_goal_selector` | JSON list: `[{"label": "...", "conf": 0.0, "xyxy": [x1, y1, x2, y2]}]`. Published every frame, empty list when nothing is detected |
| `/yolo/detections` | `yolo_msgs/DetectionArray` | `yolo_node` | `sign_controller`, `goal_publisher` | Typed twin of the JSON topic |
| `/yolo/dbg_image` | `sensor_msgs/Image` | `yolo_node` | rqt_image_view / RViz | Annotated frames |
| `/counting/status` | `std_msgs/String` (JSON `{"orange","tree","vehicle"}`) | `topic2_yolo_counter` | monitoring | **Per-frame** counts above 0.45 confidence |
| `/sign_controller/sector_counts` | `std_msgs/String` (JSON) | `sign_controller` | monitoring | Max-per-frame tallies; deliberately a different topic from `/counting/status` |
| `/landmark_observation` | `std_msgs/String` (`<goal_name>:<confidence>`) | manual / scripts | `pomdp_goal_selector` | Test channel; live observations come from `/yolo/detections_json` |
| `/resource_monitor` | `std_msgs/String` (`ts,cpu%,mem%`) | `resource_monitor` | monitoring | Also appended to `perception/resource_usage.csv` |
| `/curiosity_explorer/metrics` | `std_msgs/String` | `curiosity_explorer` | monitoring | |
| `store_landmark` | `yolo_msgs/srv/StoreLandmark` | `landmark_db` (server) | `topic2_yolo_landmark_bridge`, `goal_publisher` (clients) | SQLite at `~/ros2_coursework_ws/perception/landmarks.db`. The bridge stores the **robot's** map pose at observation time (z = 0); `goal_publisher` stores the **object's** depth-grounded position |

## YOLO classes

`dataset/data.yaml`: `fastsign`, `orange`, `slowsign`, `stopsign`, `tree`, `vehicle`.
`topic2_nav2_traffic_rules` also accepts the legacy `fast` / `slow` / `stop` labels.

## `yolo_msgs/Detection`

| Field | Type |
| --- | --- |
| `label` | `string` |
| `confidence` | `float32` |
| `x1`, `y1`, `x2`, `y2` | `float32` pixel bounds |

## `yolo_msgs/srv/StoreLandmark`

Request: `string label`, `float32 x`, `float32 y`, `float32 z`, `float32 confidence`.
Response: `bool success`, `string message`.
