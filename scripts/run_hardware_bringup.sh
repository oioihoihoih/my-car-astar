#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/_environment.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-16}"
export LDS_MODEL="${LDS_MODEL:-LDS-03}"
exec ros2 launch my_bot tb3_safe_bringup.launch.py "$@"
