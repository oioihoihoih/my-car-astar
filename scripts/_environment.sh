#!/usr/bin/env bash
# Shared environment for scripts launched from any working directory.
set -e
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
robot_workspace="${MY_CAR_WORKSPACE:-$project_root}"
source /opt/ros/humble/setup.bash
tb3_underlay="${TB3_UNDERLAY:-$HOME/turtlebot3_ws}"
if [[ -f "$tb3_underlay/install/local_setup.bash" ]]; then
  source "$tb3_underlay/install/local_setup.bash"
fi
if [[ ! -f "$robot_workspace/install/local_setup.bash" ]]; then
  echo "Build the workspace first: cd '$robot_workspace' && colcon build --symlink-install --packages-select my_bot" >&2
  exit 1
fi
source "$robot_workspace/install/local_setup.bash"
mkdir -p "$robot_workspace/.runtime/ros-logs" "$robot_workspace/.runtime/tmp"
export ROS_LOG_DIR="$robot_workspace/.runtime/ros-logs"
export TMPDIR="$robot_workspace/.runtime/tmp"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
