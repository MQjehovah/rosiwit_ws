# ROS2 Coverage Planner - Dockerfile
# 多阶段构建：开发环境 + 生产环境镜像

# ==================== 阶段1: 开发环境 ====================
FROM ros:humble AS development

LABEL maintainer="AI Development Team"
LABEL description="ROS2 Coverage Planner Development Environment"
LABEL version="1.0"

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV COLCON_WS=/ros2_ws

# 安装开发工具和依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 编译工具
    build-essential \
    cmake \
    git \
    # ROS2开发工具
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # 测试框架
    googletest \
    libgtest-dev \
    # 其他工具
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# 初始化rosdep
RUN rosdep init && rosdep update

# 创建工作空间
RUN mkdir -p ${COLCON_WS}/src

# 设置工作目录
WORKDIR ${COLCON_WS}

# 复制源代码
COPY . ${COLCON_WS}/src/ros2_coverage_planner

# 安装依赖（使用rosdep）
RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

# 编译项目
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 设置ROS2环境变量
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

# 开发环境默认命令
CMD ["bash"]

# ==================== 阶段2: 生产环境 ====================
FROM ros:humble-ros-base AS production

LABEL maintainer="AI Development Team"
LABEL description="ROS2 Coverage Planner Production Runtime"
LABEL version="1.0"

ENV ROS_DISTRO=humble
ENV COLCON_WS=/ros2_ws

# 从开发阶段复制编译产物
COPY --from=development ${COLCON_WS}/install ${COLCON_WS}/install
COPY --from=development ${COLCON_WS}/log ${COLCON_WS}/log

# 设置ROS2环境变量
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

WORKDIR ${COLCON_WS}

# 生产环境启动命令
CMD ["ros2", "launch", "ros2_coverage_planner", "coverage_planner.launch.py"]

# ==================== 阶段3: 测试环境 ====================
FROM development AS testing

# 运行单元测试
RUN colcon test --packages-select ros2_coverage_planner
RUN colcon test-result --verbose

# 测试环境默认命令
CMD ["colcon", "test", "--packages-select", "ros2_coverage_planner"]