# ============================================================
# rosiwit_slam (FAST-LIO2) - ROS2 Humble Docker Image
# Multi-stage build for optimized image size
# Security-hardened per security_report.md findings
# ============================================================
# Fixes: SEC-001 (USER slam_user), SEC-004 (ROS_DOMAIN_ID=42),
#        SEC-006 (package name), SEC-008 (commit hash pinning),
#        SEC-010 (narrow port range)
# ============================================================

# ==================== Stage 1: Base Image ====================
FROM ros:humble-ros-base-jammy AS base

LABEL maintainer="AI Development Team <ai_dev_team@example.com>"
LABEL version="2.0.0"
LABEL description="rosiwit_slam - FAST-LIO2 SLAM ROS2 Node (Humble) - Multi-sensor fusion localization and mapping"

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Set timezone
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ==================== Stage 2: Builder ====================
FROM base AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libpcl-common-dev \
    libpcl-io-dev \
    libpcl-filters-dev \
    libpcl-kdtree-dev \
    libpcl-features-dev \
    libpcl-segmentation-dev \
    libpcl-surface-dev \
    libpcl-registration-dev \
    libpcl-visualization-dev \
    libpcl-sample-consensus-dev \
    libpcl-search-dev \
    libpcl-outofcore-dev \
    libpcl-stereo-dev \
    pcl-tools \
    libyaml-cpp-dev \
    libboost-all-dev \
    libceres-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Sophus (pinned to commit hash for supply chain security — SEC-008)
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    git checkout a79ba54d07b13f58105c6e3f4e472d4968a3e728 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Sophus

# Install GTSAM (pinned to commit hash — SEC-008)
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && \
    git checkout 316c4b40f4f26a7e4b0d89cc8d1f6a4fc3a0f40e && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/gtsam

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-message-filters \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src

# Copy package files (FIX: use rosiwit_slam package name — SEC-006)
COPY . ${ROS_WS}/src/rosiwit_slam/

# Build the package
WORKDIR ${ROS_WS}
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build \
        --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        --parallel-workers $(nproc)"

# ==================== Stage 3: Runtime ====================
FROM base AS runtime

# Install runtime dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-dev \
    libpcl-common-dev \
    libpcl-io-dev \
    libpcl-filters-dev \
    libpcl-kdtree-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    libceres-dev \
    libtbb-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-message-filters \
    && rm -rf /var/lib/apt/lists/*

# Copy built libraries from builder
COPY --from=builder /usr/local/lib/libSophus* /usr/local/lib/
COPY --from=builder /usr/local/lib/libgtsam* /usr/local/lib/
COPY --from=builder /usr/local/lib/cmake /usr/local/lib/cmake
COPY --from=builder /usr/local/include /usr/local/include

# Copy built ROS2 workspace
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Update library cache
RUN ldconfig

# Create non-root user for security (SEC-001 fix)
RUN groupadd -g 1000 slam_user && \
    useradd -u 1000 -g slam_user -m -s /bin/bash slam_user && \
    usermod -aG plugdev slam_user

# Create directories for data and config
RUN mkdir -p /data/maps /data/bags /config && \
    chown -R slam_user:slam_user /data /config /ros2_ws

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set environment variables (SEC-004 fix: ROS_DOMAIN_ID=42)
ENV ROS_WS=/ros2_ws
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOG_LEVEL=INFO

# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> /home/slam_user/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /home/slam_user/.bashrc

# Narrow DDS discovery ports (SEC-010 fix)
EXPOSE 7400/udp 7401/udp 7402/udp 7403/udp
EXPOSE 7400/tcp 7401/tcp 7402/tcp 7403/tcp

# Working directory
WORKDIR /ros2_ws

# Health check
HEALTHCHECK --interval=30s --timeout=10s --retries=3 --start-period=60s \
    CMD ros2 node list 2>/dev/null | grep -q rosiwit_slam || exit 1

# Switch to non-root user (SEC-001 fix — CRITICAL)
USER slam_user

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "rosiwit_slam", "fast_lio2.launch.py"]
