#!/bin/bash
# ============================================================
# rosiwit_slam - Docker Entrypoint Script
# Configures ROS2 Humble environment with Cyclone DDS
# ============================================================
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if available
if [ -f "${ROS_WS:-/ros2_ws}/install/setup.bash" ]; then
    source "${ROS_WS:-/ros2_ws}/install/setup.bash"
fi

# Configure Cyclone DDS (use environment variable if set)
if [ -n "${RMW_IMPLEMENTATION}" ] && [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
fi

# Set default ROS_DOMAIN_ID if not set (SEC-004 fix)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Print environment info
echo "=== rosiwit_slam Container ==="
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "User: $(whoami)"
echo "=============================="

# Execute CMD
exec "$@"
