# How to Run the Complete System

This gets the robot moving in the detection arena with signs, wall-following, and YOLO perception.

---

## Prerequisites

**System**: Ubuntu 22.04 with ROS2 Humble + Gazebo Classic 11

**Python packages** (install once):
```bash
pip3 install ultralytics opencv-python psutil numpy==1.24.4
```

Note: `numpy==1.24.4` is needed because ROS2 Humble's `cv_bridge` was compiled against numpy 1.x. If you have numpy 2.x installed, cv_bridge will print errors.

---

## Step 1: Build the workspace

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you get errors about missing packages, install them:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher ros-humble-cv-bridge
```

---

## Step 2: Launch Gazebo with the detection arena

Open **Terminal 1**:
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simple_robot_description gazebo.launch.py
```

**What you should see**: Gazebo opens with a 30m×30m arena with coloured walls (red, green, blue, yellow) and 6 sign/poster models on the walls. The robot (blue box) spawns at the centre.

**If you see the plain maze instead**: You may have launched `maze.launch.py` by mistake. Use `gazebo.launch.py`.

**If signs/posters are missing**: The GAZEBO_MODEL_PATH wasn't set. The launch file should handle this automatically. If not:
```bash
export GAZEBO_MODEL_PATH=~/ros2_coursework_ws/install/simple_robot_description/share/simple_robot_description/models:$GAZEBO_MODEL_PATH
```

---

## Step 3: Launch all perception + navigation nodes

Open **Terminal 2**:
```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yolo_ros yolo_launch.py
```

**What this starts**:
- `wall_follower_node` — drives the robot (wanders until it finds walls, then follows them)
- `yolo_node` — detects signs and objects from camera feed
- `sign_controller` — modifies speed based on detections (stop/slow/fast/count)
- `goal_publisher` — logs 3D positions of detected objects
- `landmark_db` — stores landmarks in SQLite database
- `resource_monitor` — logs CPU/RAM usage

**The robot should start moving immediately** — it wanders forward until it reaches a wall, then follows the wall around the arena.

---

## Step 4: Verify the system is working

Open **Terminal 3** and check topics:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Robot should be receiving velocity commands:
ros2 topic echo /cmd_vel --once

# Wall-follower should be publishing:
ros2 topic echo /wall_follower/cmd_vel --once

# LiDAR should be active:
ros2 topic hz /scan

# Camera should be active:
ros2 topic hz /camera/image_raw

# YOLO should be detecting (when robot faces a sign):
ros2 topic echo /yolo/detections_json --once
```

---

## What Should Happen

1. **First 30-60 seconds**: Robot drives forward from the centre toward a wall (wander mode). The arena is large (15m to each wall) and LiDAR range is 8m, so it takes time to reach a wall.

2. **Once near a wall**: Robot switches to wall-following — it balances left/right distances and cruises along the wall.

3. **When it sees a sign**:
   - **Stop sign** → Robot stops for 3 seconds, then resumes
   - **Slow sign** → Robot slows to 0.05 m/s for 2 seconds
   - **Fast sign** → Robot speeds to 0.3 m/s for 2 seconds
   - **Orange/Tree/Vehicle poster** → Robot slows to a crawl, counts objects, publishes to `/counting/status`

4. **Continuously**: Landmarks are stored in SQLite, resource usage is logged to CSV.

---

## Troubleshooting

### Robot doesn't move at all
```bash
# Check if wall_follower is running:
ros2 node list | grep wall_follower

# Check if LiDAR is publishing:
ros2 topic echo /scan --once

# Check if sign_controller is receiving:
ros2 topic echo /wall_follower/cmd_vel --once

# Check if diff_drive is receiving:
ros2 topic echo /cmd_vel --once
```

If `/scan` is not publishing, the robot didn't spawn correctly. Kill Gazebo and relaunch.

### Signs don't appear in Gazebo
The models directory wasn't found. Check:
```bash
ls ~/ros2_coursework_ws/install/simple_robot_description/share/simple_robot_description/models/
# Should show: fast_sign_poster/ orange_poster/ slow_sign_poster/ stop_sign_poster/ tree_poster/ vehicle_poster/
```

If empty, rebuild:
```bash
colcon build --packages-select simple_robot_description
source install/setup.bash
```

### YOLO model not loading
```bash
ls ~/ros2_coursework_ws/results/trafficsignv2/weights/best.pt
# Should exist (~6.2 MB)
```

If missing, check that Git LFS pulled the file (or that it's not gitignored).

### Robot moves but YOLO doesn't detect anything
- The robot needs to be **facing a sign** (within ~5m) for detection
- Check the debug image: `ros2 run rqt_image_view rqt_image_view /yolo/dbg_image`
- YOLO confidence threshold is 0.40 — signs far away may not trigger

---

## Optional: Watch the counting results
```bash
ros2 topic echo /counting/status
```

## Optional: Check landmark database
```bash
sqlite3 ~/ros2_coursework_ws/perception/landmarks.db "SELECT * FROM landmarks ORDER BY ts DESC LIMIT 10;"
```

## Optional: Run curiosity explorer (requires /map topic)
```bash
ros2 run yolo_ros curiosity_explorer --ros-args -p mode:=icm
```

---

## Summary of Launch Commands

| What | Command |
|------|---------|
| Build workspace | `colcon build --symlink-install` |
| Launch Gazebo + robot | `ros2 launch simple_robot_description gazebo.launch.py` |
| Launch all nodes | `ros2 launch yolo_ros yolo_launch.py` |
| View YOLO detections | `ros2 run rqt_image_view rqt_image_view /yolo/dbg_image` |
| Check counts | `ros2 topic echo /counting/status` |
| Check velocity | `ros2 topic echo /cmd_vel` |
