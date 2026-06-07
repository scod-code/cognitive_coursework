# Topic 2 Cognitive Robotics - Coursework Submission Guide

**Read this alongside `DEMO_INSTRUCTIONS.md` for complete reference.**

---

## 📌 Submission Checklist

- [ ] System builds cleanly: `colcon build --symlink-install`
- [ ] NumPy installed correctly: `pip3 install "numpy<2"`
- [ ] All 6 components launch without errors
- [ ] Robot responds to 2D Goal Pose clicks in RViz
- [ ] YOLO detects objects at sectors
- [ ] Traffic rules adapt speed on sign detection
- [ ] OctoMap builds 3D occupancy map
- [ ] Object counting works (oranges, trees, vehicles)
- [ ] Git commits are clean and meaningful

---

## 🏗️ System Architecture for Report

Your system implements:

```
┌─ SIMULATION LAYER ─┐
│ Gazebo (cwmaze)    │
│ + atlas robot      │
└────────────────────┘
          ↓
┌─ PERCEPTION LAYER ─┐
│ YOLO (6-class)     │
│ + Counter          │
└────────────────────┘
          ↓
┌─ REASONING LAYER ──┐
│ Traffic Rules      │
│ + Speed Adapter    │
└────────────────────┘
          ↓
┌─ NAVIGATION LAYER ─┐
│ Nav2 (planner)     │
│ + Local Controller │
└────────────────────┘
          ↓
┌─ MAPPING LAYER ────┐
│ OctoMap            │
│ + 3D Occupancy     │
└────────────────────┘
          ↓
┌─ VISUALIZATION ────┐
│ RViz               │
│ + Manual Goals     │
└────────────────────┘
```

---

## 📝 What to Write in Your Report

### Executive Summary (1 paragraph)
"The system integrates simulation (Gazebo), perception (YOLO), reasoning (traffic rules), navigation (Nav2), and mapping (OctoMap) in a unified ROS2 architecture. The robot autonomously navigates to user-clicked goals, detects traffic signs to adapt speed, counts objects to confirm sector location, and builds a live 3D map of the environment."

### Architecture Section
Reference the system architecture above. Explain each layer.

### Implementation Details
- **Navigation:** Nav2 with NavFn global planner, DWB local controller, AMCL localisation
- **Perception:** YOLOv8 fine-tuned on 6 classes (fastsign, slowsign, stopsign, orange, tree, vehicle)
- **Traffic Rules:** Speed mapping (FAST=0.28, SLOW=0.05, STOP=0.01, NORMAL=0.18)
- **Mapping:** OctoMap processes point clouds from RGB-D camera
- **Integration:** ROS2 topics connect all subsystems

### Evidence
Include screenshots or terminal output showing:
1. RViz with robot at each sector (orange, trees, vehicles)
2. `/counting/status` output: `{"orange": X, "tree": Y, "vehicle": Z}`
3. `/traffic_rule_state` changing as robot passes signs
4. OctoMap blue voxels building in RViz
5. Robot successfully navigating 3+ different goals

### Coursework Requirements Met

| Requirement | How Met | Evidence |
|------------|--------|----------|
| Basic Navigation | Nav2 path planning | Robot reaches clicked goals |
| Basic Mapping | OctoMap 3D grid | Blue voxels in RViz |
| Object Detection | YOLO detector | Bounding boxes on `/yolo/dbg_image` |
| Traffic Rules | Speed adaptation | `/speed_limit` changes on sign detection |
| Enhanced Navigation | Nav2 + costmaps | Smooth obstacle avoidance |
| Advanced Object Detection | YOLO counting | `/counting/status` updates at sectors |
| Sector Recognition | Detected objects confirm location | Counts match expected posters |

---

## 🔬 Key Implementation Files

**Describe these in your report:**

| File | Purpose | Lines |
|------|---------|-------|
| `wall_follower/launch/topic2_official_system.launch.py` | Gazebo integration + ROS bridge | ~50 |
| `wall_follower/launch/topic2_nav2.launch.py` | Nav2 stack initialization | ~80 |
| `wall_follower/wall_follower/topic2_nav2_traffic_rules.py` | Traffic rule logic | ~120 |
| `wall_follower/wall_follower/topic2_yolo_counter.py` | Object counting | ~90 |
| `wall_follower/config/topic2_nav2_params.yaml` | Nav2 parameters | ~150 |
| `wall_follower/maps/topic2_nav2_clean_map.yaml` | Navigation map | ~5 |

---

## 📊 Performance Metrics (Optional)

If your report requires metrics, you can measure:

```bash
# Navigation success rate (% of goals reached)
ros2 topic echo /navigate_to_pose/result

# Average detection confidence (from YOLO)
ros2 topic echo /yolo/detections_json | grep confidence

# Object counting accuracy (match poster types)
ros2 topic echo /counting/status

# OctoMap update rate (Hz)
ros2 topic hz /octomap_binary
```

---

## 🎓 What Assessors Will Look For

1. **System Integration:** Can all 6 components work together?
2. **Navigation:** Does robot reach goals autonomously?
3. **Perception:** Do detections match actual objects?
4. **Reasoning:** Does traffic adapt robot speed correctly?
5. **Mapping:** Is 3D occupancy grid building?
6. **Code Quality:** Is it clean, commented, follows ROS conventions?
7. **Documentation:** Is the system easy to understand and reproduce?

---

## ✅ Submission Files Required

Ensure these are in your repository:

```
wall_follower/
├── launch/
│   ├── topic2_official_system.launch.py     ✅
│   ├── topic2_nav2.launch.py                ✅
│   └── topic2_octomap_with_nav2.launch.py   ✅
├── wall_follower/
│   ├── topic2_nav2_tf_helper.py             ✅
│   ├── topic2_nav2_traffic_rules.py         ✅
│   ├── topic2_yolo_counter.py               ✅
│   └── [other nodes]
├── config/
│   └── topic2_nav2_params.yaml              ✅
├── maps/
│   ├── topic2_nav2_clean_map.yaml           ✅
│   └── topic2_nav2_clean_map.pgm            ✅
├── setup.py                                  ✅
├── package.xml                               ✅
└── CMakeLists.txt                            ✅
```

---

## 🚀 Final Git Commit

When ready to submit:

```bash
cd ~/ros2_coursework_ws
git add -A
git commit -m "Topic 2 submission: integrated navigation, perception, traffic rules, and 3D mapping"
git push origin merge/integrate-person-a
```

---

## 📚 References for Your Report

- ROS2 Documentation: https://docs.ros.org/
- Nav2 Stack: https://navigation.ros.org/
- OctoMap: https://octomap.github.io/
- YOLO: https://docs.ultralytics.com/
- Gazebo Fortress: https://gazebosim.org/

---

## ⚡ Quick Demo for Assessors

**Show this in 3 minutes:**

1. Launch all 6 terminals (60 seconds)
2. Click 2D Goal Pose → robot moves to orange sector (30 seconds)
3. Show `/counting/status` with orange count (20 seconds)
4. Click goal → robot moves to trees (30 seconds)
5. Show `/counting/status` with tree count (20 seconds)
6. Explain: "Navigation is automatic via Nav2. Perception identifies sectors via YOLO. Traffic rules adapt speed. OctoMap builds 3D map." (30 seconds)

---

## ✨ Quality Checklist

- [ ] Code is readable and commented
- [ ] ROS naming conventions followed (snake_case for topics/nodes)
- [ ] All dependencies documented in `package.xml`
- [ ] Build completes without warnings
- [ ] System runs without crashes
- [ ] Demo can be repeated reliably
- [ ] Report explains what each component does
- [ ] Git history is clean (meaningful commit messages)

---

**You're ready for submission.**
