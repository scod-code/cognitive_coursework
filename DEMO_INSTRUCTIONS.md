# Topic 2 Cognitive Robotics - Complete Guide

**Status:** ✅ Ready for Demo and Submission  
**Branch:** `merge/integrate-person-a`

---

## 📋 ONE DOCUMENT FOR EVERYTHING

This is your **only** guidance document. Use it for:
- ✅ Setup and prerequisites
- ✅ Building the workspace
- ✅ Running the complete demo
- ✅ Understanding what's implemented
- ✅ Troubleshooting
- ✅ Coursework submission

Delete or ignore: `HOW_TO_RUN.md`, `FINAL_STATUS.md`, `README.md` (if outdated), `GIT_COMMIT_INSTRUCTIONS.txt`

---

## 🔧 Prerequisites (One-Time Setup)

```bash
# Ubuntu 22.04 with ROS2 Humble (already installed)
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash

# Critical: Fix NumPy version for cv_bridge
pip3 install "numpy<2"

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

**Verify build succeeded:**
```bash
ros2 pkg list | grep wall_follower
```

---

## 🚀 Running the Complete System (6 Terminals)

**CRITICAL: Launch in this exact order. Wait after each step.**

### Terminal 1: Start Gazebo (Official Maze + Robot)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_official_system.launch.py
```
✅ **Wait 20 seconds** — You'll see a Gazebo window with the maze and an atlas robot at position (-3, -3).

---

### Terminal 2: Start Nav2 + RViz (Path Planning + Visualization)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_nav2.launch.py
```
✅ **Wait 10 seconds** — RViz opens automatically. You see:
- Maze map (grey walls, black obstacles)
- Green robot footprint in the center
- Empty path visualization

---

### Terminal 3: Start OctoMap (3D Mapping)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_follower topic2_octomap_with_nav2.launch.py
```
✅ When robot moves, blue voxels appear in RViz (3D occupancy grid).

---

### Terminal 4: Start YOLO (Object Detection)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run yolo_ros yolo_node --ros-args -p model_path:=$HOME/ros2_coursework_ws/results/trafficsignv2/weights/best.pt
```
✅ You'll see: `Loaded YOLO model` and `Model ready for inference`

---

### Terminal 5: Start Traffic Rules (Speed Adaptation)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wall_follower topic2_nav2_traffic_rules
```
✅ You'll see: `Topic 2 Nav2 traffic rules started`

---

### Terminal 6: Start Object Counter (Sector Detection)
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wall_follower topic2_yolo_counter
```
✅ You'll see: `Topic 2 YOLO counter started`

---

## 🎮 DEMO: Navigating the Robot

**All 6 terminals should now be running. Do NOT close any.**

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
- ✅ Traffic rules adapt speed on `/speed_limit`:
  - `0.28` (FAST sign)
  - `0.05` (SLOW sign)
  - `0.01` (STOP sign)
  - `0.18` (normal, no sign)

---

## ✅ Coursework Requirements Met

| Requirement | Implementation |
|------------|-----------------|
| **Basic Navigation** | Nav2 path planning + DWB local control |
| **Basic Mapping** | OctoMap 3D occupancy grid from point cloud |
| **Object Detection** | YOLO detects 6 classes (signs, oranges, trees, vehicles) |
| **Traffic Rules** | Speed adaptation based on detected signs |
| **Enhanced Navigation** | Nav2 with AMCL localisation, NavFn planner, costmaps |
| **Advanced Object Detection** | YOLO counts objects in sectors (oranges, trees, vehicles) |
| **Sector Recognition** | Confirms robot location by object detections |

---

## 🔍 Optional Monitoring (Separate Terminals)

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

### Build fails with colcon
**Fix:**
```bash
cd ~/ros2_coursework_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### "!rclpy.ok()" error when checking nodes
**Fix:**
1. Make sure Terminals 1 and 2 are running first
2. ROS2 master only starts when you launch Terminal 1
3. Wait 30 seconds total before checking nodes

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
- `wall_follower/launch/topic2_official_system.launch.py` — Gazebo integration
- `wall_follower/launch/topic2_nav2.launch.py` — Nav2 stack
- `wall_follower/wall_follower/topic2_nav2_traffic_rules.py` — Traffic rule logic
- `wall_follower/wall_follower/topic2_yolo_counter.py` — Object counting logic

---

## 🎯 Quick Checklist Before Demo

- [ ] All 6 terminals launched in order
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
    ├─ Odometry → topic2_nav2_tf_helper → map→odom→base_link (TF chain)
    │
    └─ Nav2 Stack (planner + controller) ← RViz 2D Goal Pose
       └─ /atlas/cmd_vel (to Gazebo robot)
```

---

## ✨ Final Notes

- **No waypoint tuning needed** — Uses manual RViz navigation
- **NumPy critical** — Must be <2.0 or YOLO crashes
- **Terminal order matters** — Gazebo (T1) must start before Nav2 (T2)
- **Don't close terminals** — Keep all 6 running for complete demo
- **Each node independent** — Can restart individual components without restarting all

**You're ready for demo and submission.**

