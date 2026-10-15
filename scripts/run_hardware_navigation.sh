#!/usr/bin/env bash
set -e
if [[ $# -lt 1 ]]; then
  echo 'Usage: bash scripts/run_hardware_navigation.sh /absolute/path/map.yaml [input_topic:=/camera/image_raw]' >&2
  exit 2
fi
robot_map="$1"
shift
if [[ ! -f "$robot_map" ]]; then
  echo "Map YAML does not exist: $robot_map" >&2
  exit 2
fi
robot_map="$(realpath -- "$robot_map")"
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-16}"
export LDS_MODEL="${LDS_MODEL:-LDS-03}"
exec ros2 launch my_bot tb3_astar_yolo_hardware.launch.py "map:=$robot_map" rviz:=false viewer:=false "$@"
