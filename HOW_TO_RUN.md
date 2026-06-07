# How to Run the Complete System

This document covers how to run the full cognitive robotics system in the **official Topic 2 maze** (the cwmaze coursework environment with the atlas robot).

---

## Prerequisites

```bash
# Ubuntu 22.04 with ROS2 Humble
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-octomap-server ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-state-publisher ros-humble-tf2-ros \
  ros-gz-bridge ros-gz-sim

# Python packages
pip3 install ultralytics opencv-python psutil numpy==1.24.4
```

The `ntu_robotsim` package must be installed (provided by the university). It provides the `cwmaze.launch.py` and `spawn_robot.launch.py` for the official maze world.

---

## Build the Workspace

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Option A: Full Topic 2 Official Maze (Primary Demo)

This is the main system — the atlas robot navigates the official cwmaze using Nav2, detects signs with YOLO, adapts speed to traffic rules, and counts objects.

### Terminal 1 — Start the official maze simulation

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_official_system.launch.py
```

This launches:
- Ignition Gazebo with the cwmaze world
- The atlas robot (spawns at x=-3, y=-3)
- The ROS-Ignition bridge (clock, odom, RGB-D camera, IMU, cmd_vel)

Wait ~15 seconds for everything to load.

### Terminal 2 — Start Nav2 + RViz

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_nav2.launch.py
```

This launches:
- TF helper (map → odom → atlas/base_link)
- Map server (loads `topic2_nav2_clean_map.yaml`)
- Nav2 planner (NavFn with A*)
- Nav2 controller (DWB local planner)
- BT navigator
- Behavior server (spin, backup, wait recoveries)
- Goal pose bridge
- Lifecycle manager (auto-starts all Nav2 nodes)
- RViz with Nav2 visualization

### Terminal 3 — Start YOLO perception

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run yolo_ros yolo_node --ros-args -p model_path:=$HOME/ros2_coursework_ws/results/trafficsignv2/weights/best.pt
```

### Terminal 4 — Start traffic rule adaptation

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wall_follower topic2_nav2_traffic_rules
```

This adapts Nav2 speed based on YOLO detections:
- `fastsign` → speed_limit = 0.28
- `slowsign` → speed_limit = 0.05
- `stopsign` → speed_limit = 0.01
- no sign   → speed_limit = 0.18

### Terminal 5 — Start object counter

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wall_follower topic2_yolo_counter
```

Counts oranges, trees, and vehicles when visible and publishes to `/counting/status`.

### Terminal 6 (optional) — Start OctoMap 3D mapping

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_octomap_with_nav2.launch.py
```

### How to navigate

Once everything is running:
1. In **RViz**, use the **2D Goal Pose** tool to click a destination in the maze
2. Nav2 will plan a path and the robot will autonomously navigate there
3. As it passes signs, YOLO detects them and traffic rules adapt the speed
4. When it sees counting posters, the counter publishes object counts

### Verify it's working

```bash
# Check robot is moving
ros2 topic echo /atlas/cmd_vel --once

# Check YOLO is detecting
ros2 topic echo /yolo/detections_json --once

# Check traffic rules
ros2 topic echo /traffic_rule_state

# Check counting
ros2 topic echo /counting/status

# Check OctoMap
ros2 topic echo /occupied_cells_vis_array --once
```

---

## Option B: Simple Maze with Gazebo Classic (Fallback)

If `ntu_robotsim` is not available, use the built-in maze with the simple_robot URDF.

### Terminal 1 — Launch Gazebo with maze world

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower maze.launch.py
```

This opens Gazebo Classic with `simple_world.world` (a 12m×12m maze with corridors) and spawns the robot at the centre.

### Terminal 2 — Launch perception + wall-following

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yolo_ros yolo_launch.py
```

The robot will wall-follow autonomously through the maze. When it sees signs (if they are in this world), it reacts.

---

## Option C: Detection Arena (Sign Testing Only)

For testing YOLO detection with coloured walls and all 6 sign posters:

### Terminal 1

```bash
ros2 launch simple_robot_description gazebo.launch.py
```

### Terminal 2

```bash
ros2 launch yolo_ros yolo_launch.py
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    OFFICIAL TOPIC 2 SYSTEM                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Ignition Gazebo (cwmaze)                                       │
│       │                                                         │
│       ├── /atlas/rgbd_camera/image ──────→ yolo_node            │
│       │                                     │                   │
│       │                           /yolo/detections_json         │
│       │                                     │                   │
│       │                           ┌─────────┼──────────┐        │
│       │                           │         │          │        │
│       │                 traffic_rules    counter   goal_pub     │
│       │                           │                             │
│       │                    /speed_limit ──→ Nav2 controller     │
│       │                                                         │
│       ├── /atlas/odom_ground_truth ──→ TF helper                │
│       │                                  (map→odom→base_link)   │
│       │                                                         │
│       ├── /atlas/rgbd_camera/points ──→ pointcloud_filter       │
│       │                                      │                  │
│       │                              /points_filtered           │
│       │                                      │                  │
│       │                              OctoMap server             │
│       │                              (3D occupancy grid)        │
│       │                                                         │
│       └── /atlas/cmd_vel ◀── Nav2 controller ◀── Nav2 planner  │
│                                                    ▲            │
│                                              RViz 2D Goal       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Topics

| Topic | Source | Purpose |
|-------|--------|---------|
| `/atlas/rgbd_camera/image` | Ignition bridge | RGB camera feed for YOLO |
| `/atlas/rgbd_camera/points` | Ignition bridge | Point cloud for OctoMap |
| `/atlas/odom_ground_truth` | Ignition bridge | Robot odometry |
| `/atlas/cmd_vel` | Nav2 controller | Movement commands |
| `/yolo/detections_json` | yolo_node | YOLO detection results |
| `/yolo/dbg_image` | yolo_node | Annotated camera for debugging |
| `/traffic_rule_state` | traffic_rules node | Current rule (FAST/SLOW/STOP/NORMAL) |
| `/speed_limit` | traffic_rules node | Current speed limit value |
| `/counting/status` | counter node | Object counts (orange/tree/vehicle) |
| `/octomap_binary` | OctoMap server | 3D map data |
| `/occupied_cells_vis_array` | OctoMap server | Visualization markers |

---

## Troubleshooting

### ntu_robotsim not found
```bash
# Check if it's installed:
ros2 pkg list | grep ntu_robotsim

# If not, it needs to be in your workspace or installed system-wide.
# Ask your instructor for the package or check the course materials.
```

### Robot doesn't move after sending Nav2 goal
```bash
# Check Nav2 lifecycle nodes are active:
ros2 lifecycle nodes

# Check TF is publishing:
ros2 run tf2_ros tf2_echo map atlas/base_link

# Check Nav2 action is available:
ros2 action list | grep navigate
```

### YOLO not detecting anything
```bash
# Check camera is publishing:
ros2 topic hz /atlas/rgbd_camera/image

# View the debug image:
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image

# Robot needs to be facing a sign for detection
```

### OctoMap shows nothing
```bash
# Check point cloud filter is running:
ros2 topic hz /atlas/rgbd_camera/points_filtered

# Check TF is correct for OctoMap:
ros2 run tf2_ros tf2_echo map atlas/base_link
```

---

## Quick Demo Script (What to show assessor)

1. Launch Terminals 1-5 (official system + Nav2 + YOLO + traffic + counter)
2. In RViz, set a 2D Goal Pose in the maze → robot navigates autonomously
3. Show `/yolo/dbg_image` in rqt_image_view → bounding boxes on signs
4. Show `ros2 topic echo /traffic_rule_state` → rules change as robot passes signs
5. Show `ros2 topic echo /counting/status` → counts objects in posters
6. Show OctoMap blue voxels in RViz → 3D mapping happening live
7. Explain: "Nav2 plans the route, YOLO detects signs, traffic adapter changes speed, counter tallies objects, OctoMap builds a 3D map — all integrated through ROS2 topics"
