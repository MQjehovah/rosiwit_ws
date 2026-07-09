# ============================================================
# rosiwit_navigation - ROS2 Humble Docker Image
# Multi-stage build: dev (full tools) + runtime (slim)
# ============================================================

# ==================== Stage 1: Base ====================
FROM ros:humble-ros-base-jammy AS base

LABEL maintainer="AI Development Team <ai-dev-team@example.com>"
LABEL version="0.1.0"
LABEL description="rosiwit_navigation - ROS2 differential drive robot navigation with obstacle avoidance"

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
    # Python tools for ROS2
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-rosinstall-generator \
    # Code quality tools
    clang-format \
    clang-tidy \
    cppcheck \
    # Debugging / profiling
    gdb \
    valgrind \
    # Runtime libs for testing
    libgtest-dev \
    # Clean apt cache
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-cmake-python \
    ros-humble-ament-index-cpp \
    ros-humble-ament-lint-auto \
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
    && rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN rosdep init && rosdep update

# Create workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Setup ROS2 environment on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc && \
    echo 'export PS1="\[\033[01;32m\]rosiwit-navigation\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /root/.bashrc

# Configure colcon defaults
RUN mkdir -p /root/.colcon && \
    echo '{"build": {"symlink-install": true, "cmake-args": ["-DCMAKE_BUILD_TYPE=RelWithDebInfo", "-DBUILD_TESTING=ON"], "parallel-workers": "auto"}}' \
    > /root/.colcon/defaults.yaml

# Copy project source
COPY . ${ROS_WS}/src/rosiwit_navigation/

# Install project dependencies
RUN cd ${ROS_WS} && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build the project
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${ROS_WS} && \
    colcon build --packages-select rosiwit_navigation --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
    && colcon test --packages-select rosiwit_navigation --event-handlers console_direct+ || true"

# ==================== Stage 3: Runtime (slim) ====================
FROM base AS runtime

# Install runtime-only dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ament-index-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src

# Copy only built artifacts from dev stage
COPY --from=dev ${ROS_WS}/install ${ROS_WS}/install
COPY --from=dev ${ROS_WS}/src/rosiwit_navigation/config ${ROS_WS}/src/rosiwit_navigation/config
COPY --from=dev ${ROS_WS}/src/rosiwit_navigation/launch ${ROS_WS}/src/rosiwit_navigation/launch
COPY --from=dev ${ROS_WS}/src/rosiwit_navigation/maps ${ROS_WS}/src/rosiwit_navigation/maps

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

# Default entry: launch navigation
WORKDIR ${ROS_WS}
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "rosiwit_navigation", "navigation.launch.py"]
