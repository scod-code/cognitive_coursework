#!/usr/bin/env bash
# Launch or validate the autonomous mission (run from an Ubuntu/WSL terminal).
#
#   bash scripts/run_autonomy.sh launch [extra ros2 launch args...]   # default
#   bash scripts/run_autonomy.sh validate                             # unit tests + launch check
#
# ROS2_WS overrides the workspace (default: the repository containing this script).
# The workspace must already be built:
#   colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower
# (do not run a bare `colcon build`; the vendored pcl_ros exhausts memory).
set -eo pipefail

WS="${ROS2_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MODE="${1:-launch}"
[ "$#" -gt 0 ] && shift

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble not found at /opt/ros/humble (run this inside WSL Ubuntu 22.04)." >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "No install/ in $WS. Build first:" >&2
  echo "  cd $WS && colcon build --symlink-install --packages-select ntu_robotsim yolo_msgs yolo_ros wall_follower" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

case "$MODE" in
  launch)
    exec ros2 launch wall_follower topic2_autonomous.launch.py "$@"
    ;;
  validate)
    cd "$WS/wall_follower"
    echo "== unit tests (ROS-free logic + mocked navigator) =="
    python3 -m pytest test/test_candidate_generator.py test/test_goal_selection.py \
      test/test_mission_metrics.py test/test_mission_transitions.py -q -p no:cacheprovider
    echo "== launch file resolves =="
    ros2 launch wall_follower topic2_autonomous.launch.py --show-args > /dev/null
    echo "== executable installed =="
    ros2 pkg executables wall_follower | grep autonomous_mission
    echo "VALIDATION OK (simulation behaviour is not covered; see DEMO_INSTRUCTIONS.md, Fully Autonomous Mission)"
    ;;
  *)
    echo "usage: $0 [launch|validate] [launch args]" >&2
    exit 2
    ;;
esac
