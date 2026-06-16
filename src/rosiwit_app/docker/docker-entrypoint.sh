#!/bin/bash
# ============================================================
# rosiwit_app - Docker Entrypoint Script
# Configures ROS2 Humble environment for unified dispatch system
# ============================================================
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if available
if [ -f "${ROS_WS:-/ros2_ws}/install/setup.bash" ]; then
    source "${ROS_WS:-/ros2_ws}/install/setup.bash"
fi

# Configure DDS (use environment variable if set)
if [ -n "${RMW_IMPLEMENTATION}" ] && [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
fi

# Set default ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Ensure map storage directory exists
mkdir -p "${MAP_PATH:-/tmp/rosiwit_sim_map}"

# Print environment info
echo "============================================="
echo " rosiwit_app - Unified Robot Dispatch System"
echo " ROS_DISTRO:     ${ROS_DISTRO}"
echo " ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo " RMW_IMPLEMENT:  ${RMW_IMPLEMENTATION:-default}"
echo " Map Path:       ${MAP_PATH:-/tmp/rosiwit_sim_map}"
echo " User:           $(whoami)"
echo "============================================="

# Execute CMD
exec "$@"
