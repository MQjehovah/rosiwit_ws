# rosiwit_slam 项目使用指南

## 项目状态

✅ **项目已成功编译并可以运行**

> **注意**: 包名已从 `fast_lio2_slam` 变更为 `rosiwit_slam`，可执行文件名为 `fast_lio2_node`。
> 以下文档内容中部分命令仍使用旧名，请以最新命令为准。

节点 `/fast_lio2_node` 提供完整的ROS2接口：
- **订阅**: `/velodyne_points`, `/imu`
- **发布**: `/cloud_map`, `/odom_estimated`, `/path_estimated`
- **服务**: `/save_map`, `/save_pcd`

## 快速开始

### 1. 编译项目

```bash
cd /home/jmq/agent/workspace/projects/rosiwit_ws/src/rosiwit_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动节点

**仿真时间模式（用于测试）**:
```bash
ros2 run rosiwit_slam fast_lio2_node --ros-args -p use_sim_time:=true
```

**真实时间模式（用于实际运行）**:
```bash
ros2 run rosiwit_slam fast_lio2_node
```

**使用launch文件**:
```bash
ros2 launch rosiwit_slam fast_lio2.launch.py
```

### 3. 运行测试

**综合建图测试**:
```bash
# 方式1: 自动化测试（推荐）
./test_mapping_auto.sh

# 方式2: 手动测试
# 终端1: 启动节点
ros2 run rosiwit_slam fast_lio2_node --ros-args -p use_sim_time:=true

# 终端2: 运行测试脚本
python3 test_mapping_comprehensive.py
```

**Livox格式测试**:
```bash
python3 test_livox.py
```

**其他测试脚本**:
- `test_mapping.py` - 基础建图测试
- `test_mapping_simple.py` - 简单建图测试
- `test_sim_time.py` - 仿真时间测试

### 4. 可视化

```bash
# 启动RViz2
rviz2

# 添加显示项:
# 1. Add -> PointCloud2 -> Topic: /cloud_map
# 2. Add -> Path -> Topic: /path_estimated
# 3. Add -> Odometry -> Topic: /odom_estimated
# 4. 设置Fixed Frame: map
```

## 数据格式要求

### LiDAR点云格式 (Livox格式)

点云消息需包含以下字段：
```
PointCloud2 fields:
  - x (FLOAT32)
  - y (FLOAT32)
  - z (FLOAT32)
  - intensity (FLOAT32)
  - normal_x (FLOAT32) - 法向量X分量
  - normal_y (FLOAT32) - 法向量Y分量
  - normal_z (FLOAT32) - 法向量Z分量
  - curvature (FLOAT32) - 曲率
```

### IMU数据格式

标准ROS2 sensor_msgs/Imu消息：
- 线性加速度 (m/s²)
- 角速度 (rad/s)
- 姿态四元数
- 协方差矩阵

### 数据频率要求

- **LiDAR**: 10-20 Hz
- **IMU**: 100-200 Hz

## 使用真实数据

### 播放Rosbag

```bash
# 播放数据包（使用仿真时间）
ros2 bag play your_livox_data.bag --clock

# 播放时指定速率
ros2 bag play your_data.bag --clock --rate 1.0
```

### 使用真实Livox雷达

1. 安装Livox SDK和驱动:
```bash
# 参考: https://github.com/Livox-SDK/livox_ros_driver2
```

2. 配置雷达话题映射:
```bash
ros2 run rosiwit_slam fast_lio2_node --ros-args \
  -p lidar_topic:=/livox/lidar \
  -p imu_topic:=/livox/imu
```

## 配置参数

### 命令行参数

```bash
ros2 run rosiwit_slam fast_lio2_node --ros-args \
  -p use_sim_time:=true \
  -p lidar_topic:=/lidar_points \
  -p imu_topic:=/imu/data \
  -p point_filter_num:=2 \
  -p filter_size_map:=0.5
```

### 配置文件

配置文件位于 `config/` 目录：
- `default.yaml` - 默认配置
- `livox_avia.yaml` - Livox Avia雷达配置
- `velodyne_vlp16.yaml` - Velodyne VLP16配置

## 服务接口

### 保存地图

```bash
# 调用保存地图服务
ros2 service call /save_map std_srvs/srv/Trigger

# 保存PCD文件
ros2 service call /save_pcd std_srvs/srv/Trigger
```

地图文件保存位置：
- 默认: `~/.ros/fast_lio2_maps/`
- 可通过参数 `map_save_path` 指定

## 故障排除

### 1. 节点启动失败

检查依赖:
```bash
# 确保ROS2环境已设置
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查依赖库
ldd install/rosiwit_slam/lib/rosiwit_slam/fast_lio2_node
```

### 2. 没有输出数据

可能原因:
1. **仿真时间未同步**: 确保发布 `/clock` 话题或设置 `use_sim_time:=false`
2. **点云格式错误**: 确保点云包含Livox格式字段
3. **QoS不匹配**: 使用 `RELIABLE` 可靠性策略
4. **数据频率过低**: IMU需100Hz以上，LiDAR需10Hz以上

### 3. 仿真时间问题

```bash
# 方式1: 使用rosbag的--clock选项
ros2 bag play data.bag --clock

# 方式2: 手动发布clock
python3 -c "
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

rclpy.init()
node = rclpy.create_node('clock_pub')
pub = node.create_publisher(Clock, '/clock', 10)
start = time.time()
while rclpy.ok():
    msg = Clock()
    t = time.time() - start
    msg.clock.sec = int(t)
    msg.clock.nanosec = int((t-int(t))*1e9)
    pub.publish(msg)
    time.sleep(0.01)
"
```

## 项目结构

```
fast_lio2_slam/
├── include/fast_lio2_slam/     # 头文件
│   ├── common/                  # 通用类型和工具
│   ├── data_preprocessor/       # 数据预处理
│   ├── fast_lio2_core/          # IEKF核心算法
│   ├── map_manager/             # 地图管理
│   ├── loop_closure/            # 闭环检测
│   ├── odom_fusion/             # 里程计融合
│   └── ros_interface/           # ROS2接口
│
├── src/                         # 源文件
│   └── main.cpp                 # 主入口
│
├── launch/                      # Launch文件
│   ├── fast_lio2.launch.py
│   └── livox_avia.launch.py
│
├── config/                      # 配置文件
│   ├── default.yaml
│   ├── livox_avia.yaml
│   └── velodyne_vlp16.yaml
│
├── rviz/                        # RViz配置
│   └── fast_lio2.rviz
│
├── test_*.py                    # 测试脚本
├── diagnose.sh                  # 诊断脚本
└── CMakeLists.txt               # 构建文件
```

## 技术架构

```
数据输入 (LiDAR + IMU)
    ↓
数据预处理 (滤波 + 去畸变)
    ↓
IEKF状态估计 (FAST-LIO2核心)
    ↓
地图更新 (iKD-Tree)
    ↓
输出 (里程计 + 路径 + 地图)
```

## 性能指标

- **处理速度**: 实时处理（依赖点云密度和硬件性能）
- **定位精度**: 厘米级（在良好环境下）
- **内存占用**: 取决于地图大小和点云密度
- **CPU使用**: 多线程优化

## 更多资源

- [API参考文档](docs/API_REFERENCE.md)
- [架构设计文档](docs/architecture.md)
- [关键函数说明](docs/KEY_FUNCTIONS.md)
- [测试报告](TEST_REPORT.md)
- [部署指南](DEPLOYMENT.md)

## 联系与支持

遇到问题请查看:
1. [TEST_REPORT.md](TEST_REPORT.md) - 测试报告
2. [DEPENDENCIES.md](DEPENDENCIES.md) - 依赖说明
3. [DEPLOYMENT.md](DEPLOYMENT.md) - 部署指南

---
**最后更新**: 2026-04-24
**版本**: 1.0.0