# ============================================================
# rosiwit_simulator - ROS2 Humble + Gazebo 11 Docker Image
# Multi-stage build for mbot simulation with 3D LiDAR support
# ============================================================
# Migration: ROS1 Noetic → ROS2 Humble
# ============================================================

# ==================== Stage 1: Base Image ====================
FROM ros:humble-ros-base-jammy AS base

LABEL maintainer="AI Development Team <ai_dev_team@example.com>"
LABEL version="2.0.0"
LABEL description="rosiwit_simulator - Gazebo simulation for mbot robot with 3D LiDAR (ROS2 Humble)"

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Set timezone
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ==================== Stage 2: Builder ====================
FROM base AS builder

# Install build and runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Gazebo 11 (full install for GPU ray sensor support)
    gazebo11 \
    libgazebo11-dev \
    # ROS2 Gazebo packages (Humble)
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    # ROS2 sensor packages
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    # Robot description tools
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # Visualization
    ros-humble-rviz2 \
    # Launch dependencies
    ros-humble-launch \
    ros-humble-launch-ros \
    # Nav2 dependencies
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-bringup \
    # GPU acceleration support (NVIDIA)
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa \
    # Utilities
    bash-completion \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update || true

# ==================== Stage 3: Build workspace ====================
FROM builder AS workspace

# Create colcon workspace
WORKDIR /colcon_ws

# Copy source code
COPY .. /colcon_ws/src/

# Source ROS2 and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select simulator \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install"

# ==================== Stage 4: Runtime ====================
FROM base AS runtime

# Install only runtime dependencies (smaller image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-bringup \
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user for security (fixing High-01 from security audit)
RUN groupadd -g 1000 rosuser && \
    useradd -u 1000 -g rosuser -m -s /bin/bash rosuser && \
    usermod -aG plugdev rosuser

# Copy build artifacts from workspace stage
COPY --from=workspace /colcon_ws/install /colcon_ws/install
COPY --from=workspace /colcon_ws/src /colcon_ws/src

# Set working directory
WORKDIR /colcon_ws

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set ownership
RUN chown -R rosuser:rosuser /colcon_ws

# Environment variables
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
ENV GAZEBO_MODEL_PATH=/colcon_ws/src/rosiwit_simulator/models
ENV ROS_LOG_LEVEL=INFO

# Health check
HEALTHCHECK --interval=30s --timeout=10s --retries=5 --start-period=60s \
    CMD ros2 topic list || exit 1

# Switch to non-root user (fixing High-01)
USER rosuser

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "simulator", "simulator_gazebo_3d.launch.py"]
