# ROS2 Cognitive Robotics Coursework

## Overview
This repository contains a complete ROS2 cognitive robotics system for autonomous navigation and perception. The system implements a robot capable of:
- Real-time object detection using YOLOv8 for traffic sign recognition
- Autonomous wall-following navigation
- Gazebo simulation with realistic physics and sensor models
- ROS2 communication between perception (Person B) and navigation (Person A) components

## System Architecture

### Core Components

**1. Perception System (Person B)**
- **YOLOv8 Object Detection**: Real-time detection of traffic signs (stop, fast, slow, tree, vehicle, orange, fastsign, slowsign)
- **Typed ROS2 Messages**: Custom `yolo_msgs` package for structured detection data
- **Landmark Database**: Persistent storage of detected objects and their positions
- **Debug Visualization**: Annotated camera feed with bounding boxes

**2. Navigation System (Person A)** 
- **Wall Following**: Basic reactive navigation along arena walls
- **Gazebo Simulation**: Physics-based robot model with LiDAR and camera
- **Robot Description**: URDF model with differential drive, camera, and LiDAR sensors

**3. Communication Interface**
- Defined topic contracts in `INTERFACES.md`
- Typed message exchange using `yolo_msgs/DetectionArray`
- Fallback JSON messaging for compatibility

### Key Metrics
- **YOLO Model Performance**: mAP@50=0.845, Precision=0.796, Recall=0.865
- **Class-specific mAP@50**:
  - stopsign: 0.945 (target: ≥0.70) ✅
  - vehicle: 0.638 (target: ≥0.60) ✅  
  - tree: 0.614 (target: ≥0.60) ✅
- **Training Dataset**: 107 train images, 22 validation images, 11 test images
- **Model Size**: YOLOv8n (nano variant) optimized for real-time inference

## Quick Start

### Prerequisites
```bash
# Ubuntu 22.04 with ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-pip

# Python dependencies
pip3 install ultralytics opencv-python torch torchvision
```

### Installation
```bash
# Clone repository
git clone <repository-url>
cd ros2_coursework_ws

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Running the System

Depending on the task, you can launch either the **Arena Environment** (for sign detection and YOLO training/evaluation) or the **Maze Environment** (for wall-following and Nav2 path planning).

#### Option A: Arena Environment (Object Detection & Data Collection)
This environment includes traffic signs (stop, fast, slow) and object posters (orange, tree, vehicle) for perception and classification.

1. **Launch the Arena Simulation**:
   ```bash
   ros2 launch simple_robot_description gazebo.launch.py
   ```
2. **Start the YOLO Perception Node**:
   ```bash
   ros2 run yolo_ros yolo_node
   ```
3. **Start Wall Following Navigation (optional)**:
   ```bash
   ros2 run wall_follower wall_follower_node
   ```
4. **Visualize in RViz**:
   ```bash
   ros2 run rviz2 rviz2 -d install/simple_robot_description/share/simple_robot_description/rviz/robot.rviz
   ```

#### Option B: Maze Environment (Autonomous Navigation & Mapping)
This environment includes a complete maze layout. You can run it with either basic wall-following or the advanced Nav2 navigation stack.

1. **Launch the Maze Simulation (Basic Wall-Following)**:
   ```bash
   ros2 launch wall_follower maze.launch.py
   ```
   Then start the wall follower node:
   ```bash
   ros2 run wall_follower wall_follower_node
   ```

2. **Launch the Maze Simulation with Nav2 Stack**:
   ```bash
   ros2 launch wall_follower nav2_sim.launch.py
   ```
   This starts the maze simulation, RViz with path visualization, and the full Nav2 navigation suite (AMCL localization, map server, path planner, and obstacle avoidance controller). You can command the robot using the **2D Goal Pose** tool in RViz to make it plan a path and navigate to the selected goal.

3. **Start the YOLO Perception Node (optional)**:
   You can also run YOLOv8 in the maze to detect signs and publish traffic rules:
   ```bash
   ros2 run yolo_ros yolo_node
   ```

## Project Structure

```
ros2_coursework_ws/
├── dataset/                    # Training dataset (YOLO format)
│   ├── train/images/          # Training images
│   ├── valid/images/          # Validation images  
│   ├── test/images/           # Test images
│   └── data.yaml              # Dataset configuration
├── perception/
│   └── yolo_ros/              # YOLO ROS2 package
├── simple_robot_description/  # Robot URDF and Gazebo world
├── wall_follower/             # Navigation package
├── results/                   # Training results and models
│   ├── trafficsignv2/         # Original model training
│   └── trafficsignv3/         # Retrained model (v2 dataset)
├── evidence/                  # Evaluation metrics and visualizations
├── INTERFACES.md             # ROS2 topic contracts
├── PERSON_A_PROGRESS.md      # Navigation task tracking
├── SIGN_RENDERING_FIX_SUMMARY.md  # Gazebo rendering fixes
└── README.md                 # This file
```

## Training Results

### Original Model (trafficsignv2)
- **mAP@50**: 0.684
- **Precision**: 0.763  
- **Recall**: 0.635
- **Training Time**: ~3 hours

### Retrained Model (trafficsignv3) - **IMPROVED**
- **mAP@50**: 0.845 (+23.5% improvement)
- **Precision**: 0.796
- **Recall**: 0.865 (+36.2% improvement)
- **Training Time**: 1.16 hours (faster convergence)
- **Class Performance**: All targets exceeded

### Training Progression
- **Epochs**: 100
- **Final Loss**: box=0.892, cls=0.861, dfl=0.896
- **Convergence**: Stable by epoch 50, continued refinement to epoch 100

## Evidence Folder Contents
The `evidence/` folder contains:
- **Training Metrics**: CSV files with epoch-by-epoch performance
- **Confusion Matrices**: Visualizations of classification performance
- **Demo Videos**: Gazebo simulation recordings
- **Performance Plots**: mAP, precision, recall progression over training
- **Model Comparisons**: Original vs retrained model performance

## Technical Achievements

### 1. YOLO Pipeline Implementation
- Custom ROS2 node for YOLOv8 inference
- Typed message interface with fallback JSON support
- Real-time debug visualization
- Model deployment with configurable confidence thresholds

### 2. Gazebo Simulation Fixes
- Resolved PBR material rendering issues for OGRE1
- Added collision geometry to traffic sign models
- Created Ogre material scripts for texture loading
- Fixed red sign blending with red wall background

### 3. Dataset Management
- Roboflow integration for dataset versioning
- Automated training pipeline with Ultralytics
- Dataset augmentation and balancing
- Performance tracking against class-specific targets

### 4. ROS2 Communication
- Contract-driven interface design
- Custom message definitions (`yolo_msgs`)
- Topic name standardization
- Clear separation of concerns between components

## Coursework Task Coverage

### Completed (Person B - Perception)
- [x] YOLOv8 model training and deployment
- [x] ROS2 node for real-time inference
- [x] Custom message definitions and publishers
- [x] Debug visualization with annotated camera feed
- [x] Dataset management and version control
- [x] Model retraining with improved dataset
- [x] Performance evaluation and metrics tracking

### Completed (Person A - Navigation)  
- [x] Robot URDF with sensors (RGB camera, depth camera, LiDAR, differential drive)
- [x] Gazebo world with traffic signs
- [x] Wall following algorithm (PID control with obstacle avoidance)
- [x] Kalman filter for sensor noise reduction
- [x] Scan filter node (EMA smoothing on LaserScan)
- [x] Basic navigation control
- [x] RViz configuration for visualization

### In Progress / Future Work
- [ ] Advanced navigation (OctoMap/Nav2)
- [ ] POMDP-based goal selection
- [ ] Visual odometry implementation
- [ ] Full SLAM pipeline

## Troubleshooting

### Common Issues

**1. YOLO Model Not Loading**
```bash
# Check model path in yolo_node.py
# Verify ultralytics is installed: pip3 install ultralytics
# Check file permissions: ls -la results/trafficsignv2/weights/best.pt
```

**2. Gazebo Signs Not Rendering**
```bash
# Rebuild robot description package
colcon build --packages-select simple_robot_description
# Check material scripts exist in install space
```

**3. ROS2 Topics Not Appearing**
```bash
# Source workspace properly
source install/setup.bash
# Check node is running: ros2 node list
# List topics: ros2 topic list
```

## License
This project is part of a coursework submission for Cognitive Robotics. Dataset sourced from Roboflow with CC BY 4.0 license.

## Contributors
- **Person A**: Navigation and simulation components
- **Person B**: Perception and machine learning components

## Repository Status
- **Git Branch**: `main` (default, unified with `person-b/yolo-pipeline`)
- **Last Commit**: All work unified and pushed
- **Synced**: Local and remote branches fully synchronized
- **Documentation**: Complete with performance metrics and evidence