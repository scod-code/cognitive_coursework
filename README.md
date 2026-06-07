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
- ✅ **YOLO** detects 6 classes (traffic signs, oranges, trees, vehicles)
- ✅ **Traffic rules** adapt robot speed based on detected signs
- ✅ **OctoMap** builds a live 3D occupancy map
- ✅ **Object counter** confirms sectors by counting objects
- ✅ **RViz** visualizes everything

---

## 🚀 Quick Start

```bash
cd ~/ros2_coursework_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
pip3 install "numpy<2"
colcon build --symlink-install
```

Then open 6 terminals and follow `DEMO_INSTRUCTIONS.md` for launch commands.

---

## 📋 What's Implemented

| Feature | Status | Evidence |
|---------|--------|----------|
| Navigation (Nav2) | ✅ | Robot reaches clicked goals |
| Path Planning | ✅ | NavFn global planner active |
| Local Control | ✅ | DWB local controller active |
| Mapping (OctoMap) | ✅ | 3D voxels build in real-time |
| YOLO Detection | ✅ | 6-class detector running |
| Traffic Rules | ✅ | Speed adapts to signs |
| Object Counting | ✅ | Counts oranges, trees, vehicles |
| Integration | ✅ | All subsystems via ROS2 topics |

---

## 📁 Repository Structure

```
wall_follower/
├── launch/                          # Launch files
│   ├── topic2_official_system.launch.py
│   ├── topic2_nav2.launch.py
│   └── topic2_octomap_with_nav2.launch.py
├── wall_follower/                   # Python nodes
│   ├── topic2_nav2_tf_helper.py
│   ├── topic2_nav2_traffic_rules.py
│   ├── topic2_yolo_counter.py
│   └── [other nodes]
├── config/                          # Configuration
│   └── topic2_nav2_params.yaml
├── maps/                            # Navigation maps
│   ├── topic2_nav2_clean_map.yaml
│   └── topic2_nav2_clean_map.pgm
├── setup.py                         # Package setup
├── package.xml                      # Dependencies
└── CMakeLists.txt                   # Build config

DEMO_INSTRUCTIONS.md                 # ← Read this to run
COURSEWORK_SUBMISSION.md             # ← Read this for report
README.md                            # ← You are here
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

```
ROS2 Humble
nav2_core, nav2_planner, nav2_controller
octomap_server, octomap_ros
yolo_ros
tensorflow, ultralytics, opencv
numpy<2 (CRITICAL)
```

---

## 📞 Support

**Problem solving:**

1. Read troubleshooting section in `DEMO_INSTRUCTIONS.md`
2. Check that all 6 terminals launched in correct order
3. Verify NumPy version: `pip3 install "numpy<2"`
4. Clean rebuild: `rm -rf build install && colcon build --symlink-install`

---

## 📊 System Diagram

```
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
- **Integrated System:** All components communicate via ROS2 topics
- **Easy to Demo:** Single command to launch everything

---

## 🎓 For Your Report

Use these files as references:
- `COURSEWORK_SUBMISSION.md` — What to write
- `DEMO_INSTRUCTIONS.md` — How system works (architecture section)
- Source code in `wall_follower/wall_follower/` — Implementation details

---

## ✅ Ready?

**To run the demo:** Open `DEMO_INSTRUCTIONS.md`  
**To prepare submission:** Open `COURSEWORK_SUBMISSION.md`  
**To understand architecture:** Open either file's system diagram

---

**Status: System is complete and ready for demonstration.**
