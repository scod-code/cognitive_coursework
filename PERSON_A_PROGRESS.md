# Person A — Navigation & Mapping Progress

## Foundation Setup

- [x] ROS2 workspace structure
- [x] Robot description package
- [x] URDF robot base
- [x] Differential drive wheels
- [x] Caster wheel
- [x] Camera link
- [x] LiDAR link
- [x] RViz launch file
- [x] RViz config
- [x] Gazebo world file
- [x] Gazebo material references
- [x] Differential drive Gazebo plugin
- [x] Gazebo LiDAR sensor plugin
- [x] Gazebo launch file
- [x] Package dependencies

## Remaining Foundation Tests

- [ ] Install / access ROS2 runtime
- [ ] Build workspace with colcon
- [ ] Launch RViz
- [ ] Launch Gazebo
- [ ] Verify /scan topic
- [ ] Verify /cmd_vel topic
- [ ] Verify /odom topic
- [ ] Verify TF tree

## Coursework Tasks

- [x] Task 1: Wall-Following Navigation ✅
  - **Implementation**: `wall_follower/wall_follower/wall_follower_node.py`
  - **Features**: PID control, depth/LiDAR sensor fusion, configurable parameters
  - **Topics**: `/wall_follower/cmd_vel` (Twist commands), `/camera/depth/image_raw`, `/scan`

- [x] Task 2: Kalman Filter for Sensor Noise ✅  
  - **Implementation**: `wall_follower/wall_follower/kalman_filter.py`
  - **Features**: 1D Kalman filter for depth smoothing, process/measurement noise tuning
  - **Usage**: Integrated into wall_follower_node for noise reduction

- [ ] Task 3: 3D Occupancy Grid / OctoMap
- [ ] Task 4: Enhanced Navigation / Nav2
- [ ] Task 5: POMDP-Inspired Goal Selection
- [ ] Task 6: Visual Odometry
- [ ] Task 7: Full SLAM