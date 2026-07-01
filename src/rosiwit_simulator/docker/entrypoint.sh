#!/bin/bash
# ============================================================
# rosiwit_simulator - Docker Entrypoint (ROS2 Humble)
# ============================================================
# Sets up ROS2 environment and launches the simulation
# ============================================================

set -e

# Source ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Source workspace (if install exists)
if [ -f /colcon_ws/install/setup.bash ]; then
    source /colcon_ws/install/setup.bash
fi

# ROS2 DDS configuration
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOG_LEVEL=${ROS_LOG_LEVEL:-INFO}

# Gazebo model path
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/colcon_ws/src/rosiwit_simulator/models

# GPU acceleration (NVIDIA)
if [ -f /etc/vulkan/icd.d/nvidia_icd.json ] || [ -d /usr/lib/nvidia ]; then
    echo "[INFO] NVIDIA GPU detected, enabling GPU acceleration"
    export LIBGL_ALWAYS_SOFTWARE=0
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
else
    echo "[WARN] No NVIDIA GPU detected, using software rendering"
    export LIBGL_ALWAYS_SOFTWARE=1
    export SVGA_VGPU10=0
fi

# Display configuration
export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}

echo "============================================================"
echo " rosiwit_simulator - 3D LiDAR Simulation (ROS2 Humble)"
echo " ROS_DISTRO:     humble"
echo " ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo " RMW_IMPLEMENT:  ${RMW_IMPLEMENTATION}"
echo "============================================================"

# Execute command
exec "$@"
