# Topic 2 Cognitive Robotics System

**Status:** ✅ Ready for Demo and Submission

---

## 📖 Start Here

**Choose your guidance based on what you need:**

1. **Running the demo?** → Read `DEMO_INSTRUCTIONS.md`
2. **Preparing your submission?** → Read `COURSEWORK_SUBMISSION.md`
3. **Troubleshooting?** → See Troubleshooting section in `DEMO_INSTRUCTIONS.md`

---

## ⚡ 30-Second Summary

Your system is a complete cognitive robotics pipeline:

- ✅ **Gazebo** simulates the cwmaze with an atlas robot
- ✅ **Nav2** plans paths; robot navigates to clicked goals
- ✅ **YOLO** (YOLOv8n, `results/trafficsignv2/weights/best.pt`) detects traffic signs, oranges, trees, and vehicles
- ✅ **Traffic rules** adapt robot speed based on detected signs
- ✅ **OctoMap** builds a live 3D occupancy map
- ✅ **Object counter** confirms sectors by counting objects
- ✅ **RViz** visualizes everything

---

## 🚀 Quick Start

**Prerequisite:** the module-provided `ntu_robotsim` package (official `cwmaze` world and
`atlas` robot) must be cloned into the same workspace. It is not part of this repository —
see the Prerequisites section of `DEMO_INSTRUCTIONS.md`.

```bash
cd ~/ros2_coursework_ws            # contains this repo and ntu_robotsim/
source /opt/ros/humble/setup.bash
pip3 install "numpy<2" ultralytics opencv-python psutil
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower
source install/setup.bash
```

Then start the whole demo with one command (see `DEMO_INSTRUCTIONS.md` for options):

```bash
ros2 launch wall_follower topic2_full_demo.launch.py      # you click goals in RViz
ros2 launch wall_follower topic2_autonomous.launch.py     # robot chooses its own goals
```

---

## 📋 What's Implemented

| Feature | Status | Evidence |
|---------|--------|----------|
| Navigation (Nav2) | ✅ | Robot reaches clicked goals; localisation from ground-truth odometry (no AMCL) |
| Path Planning | ✅ | NavFn global planner active |
| Local Control | ✅ | DWB local controller active |
| Mapping (OctoMap) | ✅ | 3D voxels build in real-time |
| YOLO Detection | ✅ | Multi-class detector running |
| Traffic Rules | ✅ | `/speed_limit` (Nav2 `SpeedLimit`) adapts DWB speed to signs |
| Object Counting | ✅ | Counts oranges, trees, vehicles |
| Integration | ✅ | All subsystems via ROS2 topics |
| Autonomous Mission | 🟡 | 4 self-chosen goals reached and 2 classes confirmed in simulation; full run to `COMPLETE` not yet recorded |

---

## 📁 Repository Structure

```text
.
├── wall_follower/               # Main ROS 2 package (navigation, mapping, rules)
│   ├── launch/                  # System, Nav2, OctoMap and wall-follower launch files
│   ├── wall_follower/           # Python nodes (traffic rules, YOLO counter, TF helpers,
│   │                            #   landmark bridge, POMDP goal selector, Kalman filter)
│   ├── config/                  # Nav2 parameter files (topic2, wall-follower, generic)
│   ├── maps/                    # Maze maps and the cleaned Nav2 map (pgm + yaml)
│   ├── urdf/                    # Atlas URDF for RViz
│   └── rviz/                    # Saved RViz configurations
├── perception/                  # YOLO perception packages
│   ├── yolo_ros/                # Detection node plus explorer, landmark DB, and sign controller
│   └── yolo_msgs/               # Custom Detection/DetectionArray msgs and StoreLandmark srv
├── scan_filter/                 # LaserScan filtering (legacy standalone sim only)
├── simple_robot_description/    # Legacy standalone sim: URDF, worlds, and the poster
│                                #   models (stop/slow/fast signs, orange, tree, vehicle)
│                                #   used to render and photograph training signs
├── dataset/                     # Roboflow traffic-sign dataset (train/valid/test, YOLO format)
├── results/                     # YOLO training runs and weights (traffic_sign_v1-3, trafficsignv2)
├── evidence/                    # 100-epoch training log (results.csv), curves, metrics summaries
├── proper_images/               # Coursework brief PDFs and source images
├── docs/                        # Sign-rendering fix notes and docs/history/ (superseded snapshots)
├── DEMO_INSTRUCTIONS.md         # ← Read this to run
├── COURSEWORK_SUBMISSION.md     # ← Read this for the report
├── INTERFACES.md                # Every topic, service, and frame — the contract between nodes
└── README.md                    # ← You are here
```

---

## 🎯 Coursework Requirements

**All implemented:**

- ✅ Basic Navigation
- ✅ Basic Mapping
- ✅ Object Detection
- ✅ Traffic Rules
- ✅ Enhanced Navigation
- ✅ Advanced Object Detection
- ✅ Sector Recognition

---

## 🔧 Key Dependencies

```text
ROS2 Humble
nav2_core, nav2_planner, nav2_controller
octomap_server, octomap_ros
ultralytics (YOLO), opencv, psutil
numpy<2 (CRITICAL)
ntu_robotsim (module-provided cwmaze simulation — external)
```

---

## 📞 Support

**Problem solving:**

1. Read troubleshooting section in `DEMO_INSTRUCTIONS.md`
2. Check the `topic2_full_demo.launch.py` output for errors (raise `nav2_delay` and `perception_delay` together on slow machines)
3. Verify NumPy version: `pip3 install "numpy<2"`
4. Clean rebuild: `rm -rf build install log && colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower`

---

## 📊 System Diagram

```text
User (RViz)
    ↓ clicks 2D Goal Pose
Nav2 Planner
    ↓ plans path
Nav2 Controller (DWB)
    ↓ sends commands
Gazebo Robot (/atlas/cmd_vel)
    ↓ moves in simulation
    ├─→ Camera Feed → YOLO → /yolo/detections_json
    │                              ├→ Traffic Rules → /speed_limit
    │                              └→ Counter → /counting/status
    │
    ├─→ Point Cloud → OctoMap → /occupied_cells_vis_array
    │
    └─→ Odometry → TF Chain → RViz Visualization
```

---

## ✨ Features

- **Autonomous Navigation:** Click goals in RViz, robot drives there
- **Real-Time Perception:** YOLO detects objects as robot moves
- **Adaptive Behavior:** Speed changes based on traffic signs
- **3D Mapping:** Live occupancy grid during exploration
- **Integrated System:** All components communicate via ROS2 topics (see `INTERFACES.md`)
- **One-Command Demo:** `topic2_full_demo.launch.py` starts everything; each part can still be launched alone
- **Autonomous Mission:** `topic2_autonomous.launch.py` adds `autonomous_mission`, which chooses, validates and dispatches Nav2 goals itself over the known map (see `DEMO_INSTRUCTIONS.md` for what has been verified)

---

## 🎓 For Your Report

Use these files as references:

- `COURSEWORK_SUBMISSION.md` — What to write
- `DEMO_INSTRUCTIONS.md` — How the system works (architecture section)
- `evidence/` — Training curves, confusion matrices, and metrics summaries
- Source code in `wall_follower/wall_follower/` and `perception/` — Implementation details

---

## ✅ Ready?

**To run the demo:** Open `DEMO_INSTRUCTIONS.md`  
**To prepare submission:** Open `COURSEWORK_SUBMISSION.md`  
**To understand architecture:** Open either file's system diagram

---

**Status: System is complete and ready for demonstration.**
