# rosiwit_slam 离线建图镜像
# 基于 ros:humble-ros-base (Ubuntu 22.04 + ROS2 Humble)
#
# 构建上下文 = 仓库根目录, 源码直接取 src/rosiwit_slam (单一来源, 不复制):
#   docker build -f docker/slam-mapping/Dockerfile -t rosiwit/slam-mapping:humble .
# 根目录 .dockerignore 已限定上下文只含 docker/ 与 src/rosiwit_slam, 避免打包 build/install/log 等。
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TERM=xterm

# ---- ROS 签名密钥 (rosdistro 官方续期版, 有效期至 2030; 现场下载会被网络中间层的旧缓存污染) ----
COPY docker/slam-mapping/ros.asc /tmp/ros.asc

# ---- 第三方依赖 (与 WSL 中环境一致, 全部走 apt) ----
# 老的 ros:humble-ros-base 镜像内 ROS 源签名密钥已过期:
# 注入续期版密钥(转二进制, 老容器 gpgv 不认 armored 格式)后再装包;
# rosbag2 全家桶需显式列出版本一致的整套, 避免镜像内旧版(0.15.9)与新 rosbag2-py ABI 不匹配
RUN mv /etc/apt/sources.list.d/ros2-latest.list /tmp/ros2-latest.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gnupg \
    && gpg --show-keys /tmp/ros.asc \
    && gpg --batch --quiet --dearmor < /tmp/ros.asc > /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    && mv /tmp/ros2-latest.list /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        python3-yaml \
        python3-pil \
        python3-colcon-common-extensions \
        libeigen3-dev \
        libpcl-dev \
        libceres-dev \
        libyaml-cpp-dev \
        ros-humble-sophus \
        ros-humble-gtsam \
        ros-humble-pcl-conversions \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage \
        ros-humble-rosbag2-cpp \
        ros-humble-rosbag2-transport \
        ros-humble-rosbag2-compression \
        ros-humble-rosbag2-interfaces \
        ros-humble-rosbag2-py \
        ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS1 bag -> rosbag2 转换工具 (rosbags-convert, 纯 Python) ----
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && pip3 install --no-cache-dir "rosbags>=0.9,<1" \
    && rm -rf /var/lib/apt/lists/*

# ---- 拷贝源码并编译 (仅需 rosiwit_slam, 其算法模块已内置) ----
WORKDIR /ws/src
COPY src/rosiwit_slam ./rosiwit_slam

WORKDIR /ws
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- 启动脚本 ----
COPY docker/slam-mapping/entrypoint.sh /entrypoint.sh
COPY docker/slam-mapping/scripts/normalize_bag.py /ws/scripts/normalize_bag.py
RUN chmod +x /entrypoint.sh /ws/scripts/normalize_bag.py

# ---- 默认运行参数 (可用 docker run -e 覆盖) ----
ENV BAG_PATH=/data/bag \
    CONFIG_FILE=livox.yaml \
    LIDAR_TOPIC=/livox/lidar \
    IMU_TOPIC=/livox/imu \
    RATE=1.0 \
    OUTPUT_DIR=/output \
    MAP_NAME=map \
    GRID_MAP=1

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
