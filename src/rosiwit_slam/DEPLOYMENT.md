# rosiwit_slam - Deployment Guide

本文档提供 rosiwit_slam (FAST-LIO2) 的完整部署指南。

## 目录
- [部署概览](#部署概览)
- [本地部署](#本地部署)
- [Docker 部署](#docker-部署)
- [全栈部署（Simulator + SLAM）](#全栈部署)
- [CI/CD 部署](#cicd-部署)
- [安全加固说明](#安全加固说明)
- [故障排查](#故障排查)

---

## 部署概览

### 架构
```
┌──────────────────────────────────────────────────┐
│                Docker Compose Stack              │
│                                                  │
│  ┌──────────┐   /velodyne_points   ┌──────────┐  │
│  │ Simulator │ ─── /imu ────────→  │   SLAM   │  │
│  │ (Gazebo)  │ ←── /cmd_vel ─────  │(FAST-LIO2│  │
│  │           │                      │  Node)   │  │
│  └──────────┘                      └────┬─────┘  │
│                                         │        │
│                                    /cloud_registered  │
│                                    /Odometry          │
│                                         │        │
│                                    ┌────▼─────┐  │
│                                    │  RViz2   │  │
│                                    │ (Viz)    │  │
│                                    └──────────┘  │
│                                                  │
│  Network: host (DDS discovery)                   │
│  DDS: Cyclone DDS, ROS_DOMAIN_ID=42             │
└──────────────────────────────────────────────────┘
```

### 服务列表
| 服务 | 镜像 | 端口 | 说明 |
|------|------|------|------|
| simulator | rosiwit-simulator:humble-2.0 | — | Gazebo 仿真器 |
| slam | rosiwit-slam:humble-2.0 | — | FAST-LIO2 SLAM 节点 |
| slam-viz | ros:humble-ros-base-jammy | — | RViz2 可视化 |
| demo | rosiwit-slam:humble-2.0 | — | 全栈集成 Demo |
| slam-devel | rosiwit-slam:devel-2.0 | — | 开发环境 |

---

## 本地部署

### 前置条件
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- 依赖库安装见 [DEPENDENCIES.md](DEPENDENCIES.md)

### 构建步骤

```bash
# 1. 创建工作空间
mkdir -p ~/rosiwit_ws/src
cd ~/rosiwit_ws/src

# 2. 克隆代码
git clone <repository_url>/rosiwit_slam.git
git clone <repository_url>/rosiwit_simulator.git

# 3. 安装依赖
cd ~/rosiwit_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true

# 4. 编译
colcon build --packages-select rosiwit_slam \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# 5. Source 环境
source install/setup.bash
```

### 运行 SLAM 节点

```bash
# Standalone SLAM (需要外部传感器数据)
ros2 launch rosiwit_slam fast_lio2.launch.py \
    config_file:=/path/to/velodyne_vlp16.yaml \
    lidar_topic:=/velodyne_points \
    imu_topic:=/imu

# Livox Avia 模式
ros2 launch rosiwit_slam livox_avia.launch.py \
    config_file:=/path/to/livox_avia.yaml
```

---

## Docker 部署

### 构建镜像

```bash
cd rosiwit_slam/docker

# 生产镜像 (多阶段构建，最小化)
./build.sh

# 开发镜像 (包含调试工具)
./build.sh --devel

# 自定义 tag
./build.sh --tag v2.0.0

# 推送到仓库
./build.sh --push
```

### 运行容器

```bash
cd rosiwit_slam/docker

# 仅 SLAM 节点
./run.sh --slam

# Simulator + SLAM
./run.sh --sim

# 全栈 Demo（含自动运动）
./run.sh --demo

# 开发环境
./run.sh --devel

# 停止所有容器
./run.sh --stop

# 查看日志
./run.sh --logs
```

### 手动 Docker 命令

```bash
# 构建
docker build -f docker/Dockerfile -t rosiwit-slam:humble-2.0 .

# 运行
docker run -d \
    --name rosiwit-slam \
    --network host \
    --env ROS_DOMAIN_ID=42 \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --user 1000:1000 \
    --cap-drop ALL \
    --cap-add NET_RAW \
    --security-opt no-new-privileges:true \
    --memory 8g \
    --cpus 4.0 \
    rosiwit-slam:humble-2.0
```

---

## 全栈部署

### Simulator + SLAM 集成

```bash
# 使用 docker compose 启动全栈
cd rosiwit_slam/docker
docker compose up -d simulator slam

# 或者使用集成的 demo launch
docker compose up -d demo
```

### 话题映射

| 话题 | 发布者 | 订阅者 | 消息类型 |
|------|--------|--------|----------|
| /velodyne_points | Simulator | SLAM | sensor_msgs/PointCloud2 |
| /imu | Simulator | SLAM | sensor_msgs/Imu |
| /cmd_vel | SLAM/Auto-motion | Simulator | geometry_msgs/Twist |
| /cloud_registered | SLAM | RViz2 | sensor_msgs/PointCloud2 |
| /Odometry | SLAM | RViz2 | nav_msgs/Odometry |

### 参数配置

SLAM 节点通过 `config/velodyne_vlp16.yaml` 配置:

```yaml
# 关键参数
common:
    lid_topic: "/velodyne_points"   # 激光雷达话题
    imu_topic: "/imu"               # IMU 话题
    time_sync_en: false             # 时间同步

preprocess:
    lidar_type: 2                   # 1=Avia, 2=Velodyne
    scan_line: 16                   # VLP-16 扫描线数
    blind: 1.0                      # 最小距离 (m)

mapping:
    max_iteration: 4                # 最大迭代次数
    dense_map_enable: true          # 密集地图
    filter_size_corner: 0.5         # 角点滤波大小
    filter_size_surf: 0.5           # 面点滤波大小
```

---

## CI/CD 部署

### GitLab CI

Pipeline 包含 4 个阶段:

1. **lint**: CMake/Python/C++ 代码检查
2. **build**: colcon 编译 + Docker 镜像构建
3. **test**: 单元测试 + Docker 安全验证
4. **deploy**: 镜像推送到 staging/production

```bash
# 手动触发部署
# 1. staging: merge to main → manual deploy
# 2. production: git tag → manual deploy
```

### Jenkins

Jenkinsfile 包含 5 个阶段:

1. **Checkout**: 代码检出
2. **Lint**: 并行代码检查 (Python + C++)
3. **Build**: colcon 编译 (含 Sophus/GTSAM 安装)
4. **Test**: colcon test + 结果收集
5. **Docker Build & Deploy**: 镜像构建 + 安全验证 + 推送

---

## 安全加固说明

以下安全修复已应用到部署配置中:

| 编号 | 严重度 | 问题 | 修复方案 |
|------|--------|------|----------|
| SEC-001 | Critical | Dockerfile 未切换 USER | 添加 `USER slam_user` 到 Dockerfile |
| SEC-002 | High | docker-compose privileged:true | 移除 privileged, 使用 cap_add 最小权限 |
| SEC-004 | High | ROS_DOMAIN_ID=0 默认域 | 改为 ROS_DOMAIN_ID=42 |
| SEC-005 | High | Jenkinsfile --privileged | 移除 --privileged 标志 |
| SEC-006 | Medium | COPY 路径引用旧包名 | 使用 rosiwit_slam 包名 |
| SEC-007 | Medium | network_mode:host + ipc:host | host 网络仅用于 DDS 发现, 不使用 privileged |
| SEC-008 | Medium | git clone 无版本锁定 | Sophus/GTSAM 锁定 commit hash |
| SEC-010 | Low | 宽端口范围 | DDS 端口缩窄到 7400-7403 |

### 容器安全特性

- **非 root 用户**: 所有容器以 `slam_user` (UID 1000) 运行
- **最小权限**: `cap_drop: ALL` + 仅 `NET_RAW`
- **禁止提权**: `no-new-privileges: true`
- **资源限制**: CPU/内存上限
- **日志轮转**: json-file driver, 50MB × 5 文件
- **健康检查**: 自动检测节点状态
- **供应链安全**: Git 依赖锁定到 commit hash

---

## 故障排查

### 常见问题

**Q: colcon build 失败 — "Sophus not found"**
```bash
# 手动安装 Sophus
git clone https://github.com/strasdat/Sophus.git /tmp/Sophus
cd /tmp/Sophus && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install
sudo ldconfig
```

**Q: Docker 容器无法与宿主机 DDS 通信**
```bash
# 确保 ROS_DOMAIN_ID 一致
export ROS_DOMAIN_ID=42
# 确保 Cyclone DDS 已安装
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Q: Gazebo 在 WSL2 中无法启动 GUI**
```bash
# 使用 headless 模式
export GAZEBO_HEADLESS=1
ros2 launch rosiwit_simulator simulator_gazebo_3d.launch.py gui:=false
```

**Q: 编译时 "target name conflicts with rosidl_generate_interfaces"**
```bash
# 确保使用 fast_lio2_node 作为可执行目标名
# CMakeLists.txt 中: set(NODE_NAME fast_lio2_node)
```

**Q: Docker build 内存不足 (GTSAM 编译)**
```bash
# 减少并行编译数
make -j2  # 替代 make -j$(nproc)
# 或增加 Docker 内存限制 (Docker Desktop → Settings → Resources)
```
