# Topic 2 Cognitive Robotics - Complete Guide

**Status:** ✅ Ready for Demo and Submission  
**Branch:** `main` (default). `merge/integrate-person-a` is kept level with `main`.

---

## 📋 ONE DOCUMENT FOR EVERYTHING

This is your **only** guidance document. Use it for:
- ✅ Setup and prerequisites
- ✅ Building the workspace
- ✅ Running the complete demo
- ✅ Understanding what's implemented
- ✅ Troubleshooting
- ✅ Coursework submission

---

## 🔧 Prerequisites (One-Time Setup)

**External package required: `ntu_robotsim` (official cwmaze simulation).**
The demo launch includes `ntu_robotsim/launch/cwmaze.launch.py` and `spawn_robot.launch.py`.
Run every command in an Ubuntu 22.04 (WSL) shell, not Windows PowerShell: `colcon` and
`ros2` are not on the PowerShell PATH.
This package is provided by the COMP40761 module (it contains the `cwmaze` world and the
`atlas`/jetbot model) and is **not** part of this repository. Clone it into the same
workspace before building:

```bash
mkdir -p ~/ros2_coursework_ws && cd ~/ros2_coursework_ws
# this repository
git clone https://github.com/scod-code/cognitive_coursework.git .
# module-provided simulation package (obtain the URL from the module's NOW learning room)
git clone <ntu_robotsim-repository-url> ntu_robotsim
```

Also required: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress with `ros_gz`, Nav2,
`octomap_server`, `pointcloud_to_laserscan`, and the Python packages `ultralytics`,
`opencv-python`, `psutil`.

```bash
# Ubuntu 22.04 with ROS2 Humble (already installed)
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y

# Critical: Fix NumPy version for cv_bridge
pip3 install "numpy<2"

# Build the four demo packages only. pcl_ros, octomap_server etc. come from
# /opt/ros/humble; building the copies vendored inside ntu_robotsim is not
# needed and can run out of memory in WSL.
colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower
source install/setup.bash
```

**Verify build succeeded:**
```bash
ros2 pkg list | grep -E "wall_follower|yolo_ros|yolo_msgs|ntu_robotsim"
```
All four packages must be listed.

---

## 🚀 Running the Complete System (One Command)

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_full_demo.launch.py
```

The launch file starts everything in order using timers:

| Time | Starts | ✅ Expect |
|------|--------|----------|
| 0 s | Gazebo cwmaze + atlas robot + `ros_gz` bridge | Gazebo window, robot at (-3, -3) |
| ~20 s (`nav2_delay`) | Nav2 + TF helper + RViz | RViz with the maze map and robot footprint |
| ~30 s (`perception_delay`) | OctoMap, `yolo_node`, `topic2_nav2_traffic_rules`, `topic2_yolo_counter` | `Model ready for inference`, `Topic 2 Nav2 traffic rules started`, `Topic 2 YOLO counter started`; blue voxels in RViz once the robot moves |

### Launch arguments

| Argument | Default | Effect |
|----------|---------|--------|
| `use_rviz` | `true` | Start RViz |
| `model_path` | `$HOME/ros2_coursework_ws/results/trafficsignv2/weights/best.pt` | YOLO weights |
| `nav2_delay` | `20.0` | Seconds after launch before Nav2/RViz start |
| `perception_delay` | `30.0` | Seconds after launch (not after Nav2) before OctoMap, YOLO, traffic rules, counter start. Must be larger than `nav2_delay`, so raise both together |
| `use_goal_pose_bridge` | `true` | Start `topic2_goal_pose_bridge` (see `INTERFACES.md`) |
| `enable_pomdp` | `false` | Start `pomdp_goal_selector` |
| `pomdp_dry_run` | `true` | `false` lets the POMDP dispatch Nav2 goals |
| `enable_goal_publisher` | `false` | Start `goal_publisher` (publishes `/goal_pose`) |
| `optical_to_body` | `true` | `goal_publisher` camera-axis conversion |
| `enable_curiosity` | `false` | Start `curiosity_explorer` (publishes `/goal_pose` every 5 s) |

Examples:
```bash
# Another model (the earlier 50-epoch v1-3 weights)
ros2 launch wall_follower topic2_full_demo.launch.py model_path:=$HOME/ros2_coursework_ws/results/traffic_sign_v1-3/weights/best.pt
# POMDP goal selection, dispatching goals
ros2 launch wall_follower topic2_full_demo.launch.py enable_pomdp:=true pomdp_dry_run:=false
# YOLO goal publisher with the alternative axis convention
ros2 launch wall_follower topic2_full_demo.launch.py enable_goal_publisher:=true optical_to_body:=false
# Curiosity exploration, no duplicate goal bridge
ros2 launch wall_follower topic2_full_demo.launch.py enable_curiosity:=true use_goal_pose_bridge:=false
# Slow machine / WSL
ros2 launch wall_follower topic2_full_demo.launch.py nav2_delay:=30.0 perception_delay:=45.0
```

Individual components can still be (re)started on their own, e.g.
`ros2 launch wall_follower topic2_official_system.launch.py`, `topic2_nav2.launch.py`,
`topic2_octomap_with_nav2.launch.py`, or `ros2 run yolo_ros yolo_node`,
`ros2 run wall_follower topic2_nav2_traffic_rules`, `ros2 run wall_follower topic2_yolo_counter`.

---

## 🎮 DEMO: Navigating the Robot

**Keep the launch terminal running (Ctrl+C stops everything).**

### Step 1: Click 2D Goal Pose in RViz
- Look at RViz toolbar (top of window)
- Find the button that looks like a **green target/crosshair** (labeled "2D Goal Pose")
- Click it (it will highlight/stay pressed)

### Step 2: Send Robot to Orange Sector
- In the RViz map, click on a **grey floor area** (not black walls) near the **west wall** (left side)
- This is where the orange poster is
- **Result:** Green arrow appears, blue path shows, robot drives there

### Step 3: Send Robot to Trees Sector
- Click 2D Goal Pose again
- Click near the **east wall** (right side of maze) — trees are there
- **Result:** Robot navigates to trees

### Step 4: Send Robot to Vehicles Sector
- Click 2D Goal Pose again
- Click near the **north wall** (top of maze) — vehicles are there
- **Result:** Robot navigates to vehicles

### Step 5: Send Robot to Exit (Optional)
- Click 2D Goal Pose again
- Click near the **south wall** (bottom of maze)
- **Result:** Robot navigates to exit

---

## 📊 What You Should Observe

### While Robot Moves:
- ✅ Robot footprint moves in RViz
- ✅ Green path appears from start to goal
- ✅ Blue OctoMap voxels build in real-time
- ✅ Gazebo shows robot moving

### When Robot Faces Posters:
- ✅ YOLO detects objects on `/yolo/detections_json`
- ✅ Object counter publishes on `/counting/status`:
  - `{"orange": 5, "tree": 0, "vehicle": 0}` ← at orange sector
  - `{"orange": 0, "tree": 3, "vehicle": 0}` ← at trees sector
  - `{"orange": 0, "tree": 0, "vehicle": 8}` ← at vehicles sector

### When Robot Passes Signs:
- ✅ Traffic rules adapt speed on `/speed_limit` (consumed by Nav2 `controller_server`):
  - `0.28` (FAST sign)
  - `0.05` (SLOW sign)
  - `0.01` (STOP sign)
  - `0.18` (normal, no sign)
  DWB `max_vel_x` is 0.28 so every one of these limits takes effect; the robot
  cruises at the NORMAL limit and only reaches 0.28 after a FAST sign.

---

## ✅ Coursework Requirements Met

| Requirement | Implementation |
|------------|-----------------|
| **Basic Navigation** | Nav2 path planning + DWB local control |
| **Basic Mapping** | OctoMap 3D occupancy grid from the RGB-D point cloud (visualisation and `/projected_map`; Nav2 plans on the pre-built static map) |
| **Object Detection** | YOLO detects 6 classes (signs, oranges, trees, vehicles) |
| **Traffic Rules** | Speed adaptation based on detected signs |
| **Enhanced Navigation** | Nav2 with NavFn planner, DWB controller, static-map costmaps; localisation from ground-truth odometry (`topic2_nav2_tf_helper`), not AMCL |
| **Advanced Object Detection** | YOLO counts objects in sectors (oranges, trees, vehicles) |
| **Sector Recognition** | Confirms robot location by object detections |

---

## 🔍 Optional Monitoring (extra terminal)

If you want to see what's happening in detail, open additional terminals:

### Monitor Object Counts
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /counting/status
```

### Monitor Traffic Rules
```bash
ros2 topic echo /traffic_rule_state
```

### Monitor YOLO Detections
```bash
ros2 topic echo /yolo/detections_json
```

### View Annotated Camera (with bounding boxes)
```bash
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image
```

---

## 🛠️ Troubleshooting

### Robot doesn't move when I click 2D Goal Pose
**Fix:**
1. Check RViz's fixed frame is "map" (left panel, "Global Options")
2. Make sure you clicked on a grey floor area, not a wall
3. Check Nav2 is active: `ros2 lifecycle nodes` should show all nav2_* nodes

### YOLO not detecting anything
**Fix:**
1. Check camera is publishing: `ros2 topic hz /atlas/rgbd_camera/image`
2. Robot must face a poster to detect (move closer with 2D Goal Pose)
3. Verify model path exists: `ls $HOME/ros2_coursework_ws/results/trafficsignv2/weights/best.pt`

### OctoMap not showing blue voxels
**Fix:**
1. In RViz, add "OccupiedCells Vis Array" display
2. Move robot first so it captures point cloud data
3. Check topic: `ros2 topic hz /occupied_cells_vis_array`

### Build fails with colcon, or `not found: .../install/pcl_ros/.../local_setup.bash`
A previous full-workspace build left a broken `pcl_ros` entry (usually an out-of-memory
failure). Rebuild only the demo packages from a clean overlay:
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower
source install/setup.bash
```

### `file 'topic2_full_demo.launch.py' was not found`
The workspace you built is older than this repository version. Run `git pull` in that
workspace, then rebuild as above.

### "!rclpy.ok()" error when checking nodes
**Fix:**
1. Make sure `topic2_full_demo.launch.py` is running
2. Wait ~30 seconds after launching before querying nodes

### Nav2 starts before Gazebo is ready
**Fix:** raise the delays, e.g. `nav2_delay:=30.0 perception_delay:=45.0`.

---

## 📝 For Your Report / Submission

**What to describe:**
1. **Architecture:** Gazebo simulates cwmaze, Nav2 plans paths, YOLO detects objects, traffic rules adapt speed, OctoMap builds 3D map
2. **Integration:** All subsystems communicate via ROS2 topics
3. **Demo flow:** Manual navigation via RViz 2D Goal Pose → robot autonomously reaches sectors → YOLO confirms location → traffic rules change speed
4. **Evidence:** Show screenshots of:
   - RViz with robot at each sector
   - `/counting/status` output showing orange/tree/vehicle counts
   - `/traffic_rule_state` changing as robot passes signs
   - OctoMap blue voxels in RViz

**Key files to reference:**
- `wall_follower/launch/topic2_full_demo.launch.py` — single entry point for the demo
- `wall_follower/launch/topic2_official_system.launch.py` — Gazebo integration
- `wall_follower/launch/topic2_nav2.launch.py` — Nav2 stack
- `wall_follower/wall_follower/topic2_nav2_traffic_rules.py` — Traffic rule logic
- `wall_follower/wall_follower/topic2_yolo_counter.py` — Object counting logic

---

## 🎯 Quick Checklist Before Demo

- [ ] `topic2_full_demo.launch.py` running
- [ ] Gazebo window shows maze
- [ ] RViz opened automatically
- [ ] Robot can be moved with 2D Goal Pose clicks
- [ ] Robot moves to oranges → counter shows oranges
- [ ] Robot moves to trees → counter shows trees
- [ ] Robot moves to vehicles → counter shows vehicles
- [ ] Speed changes when robot passes signs (observable in speed_limit topic)
- [ ] Blue OctoMap voxels visible in RViz

---

## ⚙️ System Architecture

```
Gazebo (cwmaze + atlas robot)
    ├─ RGB camera → YOLO (6-class detection)
    │              └─ /yolo/detections_json
    │                 ├─ topic2_nav2_traffic_rules (/speed_limit)
    │                 └─ topic2_yolo_counter (/counting/status)
    │
    ├─ Point cloud → topic2_pointcloud_filter → OctoMap (/occupied_cells_vis_array)
    │
    ├─ Ground-truth odometry → topic2_nav2_tf_helper → map→odom→atlas/base_link (TF chain, no AMCL)
    │
    └─ Nav2 Stack (planner + controller) ← RViz 2D Goal Pose
       ├─ /speed_limit ← topic2_nav2_traffic_rules
       └─ /atlas/cmd_vel (to Gazebo robot)
```

### Optional: Person-B cognition stack

Not part of the default demo. `ros2 launch yolo_ros yolo_launch.py` starts
`sign_controller`, `goal_publisher`, `landmark_db`, `resource_monitor` and a second
`yolo_node`; run `ros2 launch wall_follower topic2_official_adapters.launch.py` first so
`wall_follower_node` gets a `/scan`. `sign_controller` drives `/atlas/cmd_vel` directly and
will fight Nav2 for the robot, so use it instead of — not alongside — RViz goals.
`pomdp_goal_selector`, `goal_publisher` and `curiosity_explorer` are started by the demo
launch with `enable_pomdp:=true`, `enable_goal_publisher:=true` and `enable_curiosity:=true`.
The POMDP only dispatches Nav2 goals with `pomdp_dry_run:=false`; `goal_publisher` and
`curiosity_explorer` publish `/goal_pose` as soon as they are enabled. See `INTERFACES.md` for every topic.

---

## ✨ Final Notes

- **No waypoint tuning needed** — Uses manual RViz navigation (autonomous goal dispatch from `pomdp_goal_selector` / `goal_publisher` is optional)
- **NumPy critical** — Must be <2.0 or YOLO crashes
- **Ordering is automatic** — the launch timers start Gazebo, then Nav2, then perception
- **One terminal** — keep the launch running for the complete demo
- **Components still independent** — individual launch files / `ros2 run` commands still work

**You're ready for demo and submission.**

