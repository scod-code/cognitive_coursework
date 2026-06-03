# wall_follower official maze workflow

These commands keep the original `nav2_sim.launch.py` and `maze_map.yaml` workflow unchanged.

## Official mapping

Start Gazebo Classic, the Person A robot, SLAM Toolbox, and RViz:

```bash
ros2 launch wall_follower official_mapping.launch.py
```

Drive the robot while RViz is fixed to `map` and showing `/map`, `/scan`, TF, and the robot model:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the official map to a new path from the workspace root:

```bash
ros2 run nav2_map_server map_saver_cli -f wall_follower/maps/official_maze_map
```

This creates:

- `wall_follower/maps/official_maze_map.yaml`
- `wall_follower/maps/official_maze_map.pgm`

Do not overwrite `wall_follower/maps/maze_map.yaml` or `wall_follower/maps/maze_map.pgm`.

## Official Nav2 simulation

After saving and rebuilding the package, start the official Nav2 workflow:

```bash
ros2 launch wall_follower official_nav2_sim.launch.py
```

In RViz, use `2D Pose Estimate` to initialize AMCL, then use `2D Goal Pose` to send navigation goals in the official maze.

## POMDP official goal placeholders

The goal selector can load YAML goals:

```bash
ros2 run wall_follower pomdp_goal_selector --ros-args -p goal_config:=$(ros2 pkg prefix wall_follower)/share/wall_follower/config/official_goals.yaml
```

`official_goals.yaml` currently contains placeholder coordinates. Replace them only after deriving final coordinates from the generated official map.
