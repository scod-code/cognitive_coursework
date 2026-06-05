# Cognitive Robotics Coursework

This repository contains our Cognitive Robotics coursework project. The project uses ROS 2, Gazebo, Nav2, OctoMap, YOLO object detection, and a landmark database to make a simulated robot navigate around a maze, detect objects and signs, and store useful information about what it sees.

The main idea of the project is to show a robot system that connects movement, mapping, perception, and memory. The robot is not only moving around the maze; it can also use camera data, detect signs and objects, react to them, and save landmark information.

## Project Features

The project includes:

- ROS 2 workspace and custom packages
- Gazebo simulation with the maze and robot
- Nav2 navigation with RViz goal selection
- Wall-following/navigation support
- OctoMap 3D mapping setup
- YOLO object detection
- Traffic sign response
- Object counting
- Landmark database using SQLite
- ROS 2 topics, services, actions, and TF frames

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/scod-code/cognitive_coursework.git
cd cognitive_coursework
```

If using the final demo branch:

```bash
git checkout person-a-final-demo-ready
```

### 2. Start the Docker container

From Windows PowerShell:

```powershell
cd D:\Cognitive_Coursework
docker start ros2_gui
docker exec -it ros2_gui bash
```

Inside the Docker terminal:

```bash
source /opt/ros/humble/setup.bash
source /work/install/setup.bash
cd /work
```

### 3. Build the workspace

```bash
cd /work
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /work/install/setup.bash
```

If only the main project packages need rebuilding:

```bash
colcon build --packages-select wall_follower yolo_msgs yolo_ros --symlink-install
source /work/install/setup.bash
```

## Dependencies

The project needs:

- ROS 2 Humble
- Gazebo / Ignition Gazebo
- Python 3
- Colcon
- RViz2
- Nav2
- OctoMap
- ROS-Gazebo bridge
- OpenCV / cv_bridge
- Ultralytics YOLO
- SQLite

Main ROS 2 packages used include:

rclpy
sensor_msgs
geometry_msgs
nav_msgs
std_msgs
tf2_ros
tf2_msgs
visualization_msgs
octomap_msgs
nav2_msgs
robot_state_publisher
rviz2
octomap_server
nav2_map_server
nav2_planner
nav2_controller
nav2_bt_navigator
nav2_lifecycle_manager
ros_gz_bridge
```

Custom packages:

wall_follower
yolo_ros
yolo_msgs
```

## Usage Guide

Open a separate Docker terminal for each main part of the demo.

In every Docker terminal, run this first:

```bash
source /opt/ros/humble/setup.bash
source /work/install/setup.bash
cd /work
```

### 1. Start Gazebo and the robot

```bash
ros2 launch wall_follower topic2_official_system.launch.py
```

This starts the Gazebo maze, spawns the robot, and creates the ROS-Gazebo bridge topics.

### 2. Start Nav2 and RViz

```bash
ros2 launch wall_follower topic2_nav2.launch.py
```

Use 2D Goal Pose in RViz to send the robot to a goal.

Important: do not run `topic2_relay_adapters` during the main Nav2 demo, because the Nav2 launch already handles the needed TF and command setup.

### 3. Start OctoMap mapping

ros2 launch wall_follower topic2_octomap_with_nav2.launch.py

This starts the point cloud filter and OctoMap server.

The point cloud filter reads:

/atlas/rgbd_camera/points

and publishes:

/atlas/rgbd_camera/points_filtered

OctoMap is included as part of the mapping pipeline. The server launches, the TF setup is available, and the RealSense-style point cloud topic is connected through the filter. In the simulation, the mapping result depends on whether the camera is publishing usable point cloud data at that moment. The current setup is correct for the project architecture and can be tested through the point cloud and OctoMap topics.

### 4. Start YOLO object detection

```bash
ros2 run yolo_ros yolo_node --ros-args -p model_path:=/work/results/trafficsignv2/weights/best.pt
```

The YOLO model detects:

fastsign
slowsign
stopsign
orange
tree
vehicle
```

Main YOLO topics:

/yolo/detections_json
/yolo/dbg_image
```

### 5. View YOLO detections

```bash
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image
```

This opens the debug camera image with detection boxes.

### 6. Start the object counter

```bash
ros2 run wall_follower topic2_yolo_counter
```

This reads YOLO detections and publishes object counts to:

/counting/status

### 7. Start the landmark database

```bash
ros2 run yolo_ros landmark_db --ros-args -p db_path:=/work/perception/landmarks.db
```

This starts the SQLite landmark database.

The database service is:

/store_landmark

### 8. Start the YOLO landmark bridge

```bash
ros2 run wall_follower topic2_yolo_landmark_bridge
```

This connects YOLO detections to the landmark database.

The flow is:

/yolo/detections_json
-> topic2_yolo_landmark_bridge
-> TF lookup: map <- atlas/base_link
-> /store_landmark
-> /work/perception/landmarks.db

The database stores the object label, robot map position, confidence, and timestamp. At the moment, it stores the robot position when the object was seen, not the exact 3D centre of the object.

## Useful Test Commands

Check Nav2 actions:

```bash
ros2 action list | grep navigate
```

Check YOLO output:

```bash
ros2 topic echo /yolo/detections_json --once
```

Check traffic rule output:

```bash
ros2 topic echo /traffic_rule_state --once
ros2 topic echo /speed_limit --once
```

Check object counter:

```bash
ros2 topic echo /counting/status --once
```

Check landmark service:

```bash
ros2 service list | grep landmark
ros2 service type /store_landmark
```

Check robot velocity commands:

```bash
ros2 topic echo /atlas/cmd_vel --once
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map atlas/base_link
```

Query the landmark database:

```bash
python3 - <<'EOF'
import sqlite3

conn = sqlite3.connect("/work/perception/landmarks.db")
cur = conn.cursor()

cur.execute("""
SELECT label, x, y, z, confidence, datetime(ts,'unixepoch')
FROM landmarks
ORDER BY id DESC
LIMIT 10;
""")

for row in cur.fetchall():
    print(row)

conn.close()
EOF
```

## Reset Commands

If the robot gets stuck or needs to be reset, stop the traffic rule node first:

```bash
pkill -f topic2_nav2_traffic_rules
```

Reset the robot position:

```bash
ign service -s /world/cwmaze/set_pose \
--reqtype ignition.msgs.Pose \
--reptype ignition.msgs.Boolean \
--timeout 3000 \
--req 'name: "atlas", position: {x: -3.0, y: -3.0, z: 0.2}, orientation: {x: 0, y: 0, z: 0, w: 1}'
```

Clear Nav2 costmaps:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

## MoSCoW Requirements Checklist

Must Integrated simulation bring-up, baseline navigation, and 3D mapping Complete

Should Real-time object detection and classification Complete

Could Goal-directed navigation and traffic-rule adaptation Complete

could Persistent landmark memory and semantic counting Partial

would Visual-odometry-only localisation and VLA navigation Future work