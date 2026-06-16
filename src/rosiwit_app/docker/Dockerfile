# ============================================================
# rosiwit_app - ROS2 Humble Docker Image
# Unified dispatch system for mobile robot
# Multi-stage build: dev (full tools) + runtime (slim)
# ============================================================
# This image orchestrates the complete robot system:
#   - rosiwit_simulator (sensor simulation)
#   - rosiwit_slam (Fast-LIO2 SLAM)
#   - diffbot_navigation (Nav2-based navigation)
#   - rosiwit_app (dispatch & state machine)
# ============================================================

# ==================== Stage 1: Base ====================
FROM ros:humble-ros-base-jammy AS base

LABEL maintainer="AI Development Team <ai-dev-team@example.com>"
LABEL version="1.0.0"
LABEL description="rosiwit_app - Unified mobile robot dispatch system (ROS2 Humble)"

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ==================== Stage 2: Dev (default) ====================
FROM base AS dev

# Install build tools and development dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials
    build-essential \
    cmake \
    git \
    ca-certificates \
    wget \
    curl \
    # Python tools for ROS2
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Code quality tools (Python)
    python3-flake8 \
    python3-pydocstyle \
    # Debugging tools
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble dependencies for the full stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS2 Python
    ros-humble-rclpy \
    # Standard messages
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    # TF2
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    # Navigation2 stack
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-smoother \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-simple-commander \
    # SLAM dependencies
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    libpcl-all-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    # Simulator dependencies
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-launch \
    ros-humble-launch-ros \
    # Visualization
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --ignore-installed \
    pyyaml>=6.0 \
    numpy>=1.21.0

# Setup rosdep
RUN rosdep init || true && rosdep update || true

# Create workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Setup ROS2 environment on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo 'export PS1="\[\033[01;32m\]rosiwit-app\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "'' >> /root/.bashrc

# Configure colcon defaults
RUN mkdir -p /root/.colcon && \
    echo '{"build": {"symlink-install": true, "cmake-args": ["-DCMAKE_BUILD_TYPE=RelWithDebInfo"], "parallel-workers": "auto"}}' \
    > /root/.colcon/defaults.yaml

# Copy all workspace source (full stack for monorepo build)
COPY . ${ROS_WS}/src/

# Install project dependencies via rosdep
RUN cd ${ROS_WS} && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build the complete workspace (all 4 packages)
# rosiwit_app is ament_python, others are ament_cmake
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${ROS_WS} && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

# ==================== Stage 3: Runtime (slim) ====================
FROM base AS runtime

# Install runtime-only dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS2 Python
    ros-humble-rclpy \
    # Standard messages
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-srvs \
    # TF2
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    # Navigation2 runtime
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-simple-commander \
    # SLAM runtime
    ros-humble-pcl-conversions \
    libpcl-all \
    libyaml-cpp0.7 \
    # Simulator runtime
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # Visualization
    ros-humble-rviz2 \
    # Python YAML
    python3-yaml \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src

# Copy only built artifacts from dev stage
COPY --from=dev ${ROS_WS}/install ${ROS_WS}/install
COPY --from=dev ${ROS_WS}/src/rosiwit_app/config ${ROS_WS}/src/rosiwit_app/config
COPY --from=dev ${ROS_WS}/src/rosiwit_app/launch ${ROS_WS}/src/rosiwit_app/launch
COPY --from=dev ${ROS_WS}/src/rosiwit_app/rviz ${ROS_WS}/src/rosiwit_app/rviz

# Create map storage directory
RUN mkdir -p /tmp/rosiwit_sim_map

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

# Default entry: launch full system
WORKDIR ${ROS_WS}
COPY docker/docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "rosiwit_app", "system_bringup.launch.py"]
