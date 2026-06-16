#!/bin/bash
# ============================================================
# rosiwit_navigation - Docker Entrypoint
# ============================================================
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

echo "============================================="
echo " rosiwit_navigation - ROS2 Humble"
echo " Differential Drive Robot Navigation"
echo "============================================="

exec "$@"
