#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.sh"
exec ros2 launch my_bot tb3_astar_yolo_simulation.launch.py "$@"
