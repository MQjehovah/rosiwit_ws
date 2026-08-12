#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/visualize_xzmap.py" &
PUB_PID=$!
trap 'kill $PUB_PID 2>/dev/null' EXIT
sleep 3
ros2 run rviz2 rviz2 -d "$SCRIPT_DIR/xzmap_rviz.rviz"
