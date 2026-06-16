# rosiwit_slam 环境配置报告

> ⚠️ **[待更新: 文件路径已变更]**
> 本文档中部分路径仍引用旧路径 `project/fast_lio2_slam`。实际项目路径已迁移至 `projects/rosiwit_ws/src/rosiwit_slam`，包名从 `fast_lio2_slam` 变更为 `rosiwit_slam`，可执行文件名为 `fast_lio2_node`，`ROS_DOMAIN_ID` 已改为 `42`。请以代码为准。

## 执行时间
2026-05-05 (更新标记)

## 一、环境检查结果

### 1. ROS2 环境状态
| 项目 | 状态 | 说明 |
|------|------|------|
| ROS2 版本 | ✅ 已安装 | ROS2 Humble Hawksbill |
| ros2 命令 | ✅ 可用 | |
| colcon | ✅ 可用 | 构建工具 |

### 2. 系统依赖
| 依赖 | 状态 | 版本 |
|------|------|------|
| Eigen3 | ✅ 已安装 | 3.4.0 |
| PCL | ✅ 已安装 | 1.12.1 |
| yaml-cpp | ✅ 已安装 | 0.7.0 |
| Sophus | ✅ 已安装 | 项目内置 (include/Sophus) |

### 3. ROS2 包
| 包名 | 状态 |
|------|------|
| ros-humble-rclcpp | ✅ |
| ros-humble-sensor-msgs | ✅ |
| ros-humble-nav-msgs | ✅ |
| ros-humble-geometry-msgs | ✅ |
| ros-humble-tf2-ros | ✅ |
| ros-humble-std-srvs | ✅ |
| ros-humble-pcl-conversions | ✅ |
| ros-humble-pcl-ros | ⚠️ 未安装 (非必需) |
| ros-humble-rosbag2 | ✅ |

### 4. 项目构建状态
| 项目 | 状态 |
|------|------|
| build/ 目录 | ✅ 存在 |
| install/ 目录 | ✅ 存在 |
| install/setup.bash | ✅ 存在 |
| fast_lio2_slam 可执行文件 | ✅ 已编译 |

### 5. 配置文件
| 文件 | 状态 |
|------|------|
| config/default.yaml | ✅ |
| config/ouster_os1.yaml | ✅ |
| launch/fast_lio2.launch.py | ✅ |

### 6. 数据集
| 项目 | 状态 |
|------|------|
| datasets/Trayectory1/ | ✅ 存在 |
| rosbag2_2024_05_23-15_43_25_0.db3 | ✅ 3.0GB |
| LiDAR_baseline.csv | ✅ |

## 二、环境配置脚本

已创建以下脚本：

### 1. scripts/setup_env.sh
完整的环境配置脚本，执行以下操作：
- Source ROS2 Humble 环境
- Source 项目 workspace
- 设置环境变量 (FAST_LIO2_ROOT, FAST_LIO2_CONFIG)
- 验证环境状态

**使用方法:**
```bash
source /home/jmq/agent/workspace/project/fast_lio2_slam/scripts/setup_env.sh
```

### 2. scripts/verify_env.sh
环境验证脚本，检查所有依赖和配置是否正确。

**使用方法:**
```bash
bash /home/jmq/agent/workspace/project/fast_lio2_slam/scripts/verify_env.sh
```

## 三、环境变量配置

运行 `source scripts/setup_env.sh` 后将设置以下环境变量：

```bash
FAST_LIO2_ROOT=/home/jmq/agent/workspace/project/fast_lio2_slam
FAST_LIO2_CONFIG=/home/jmq/agent/workspace/project/fast_lio2_slam/config
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
```

## 四、快速启动指南

### 方式一：使用环境脚本
```bash
# 1. 设置环境
source /home/jmq/agent/workspace/project/fast_lio2_slam/scripts/setup_env.sh

# 2. 启动节点
ros2 launch rosiwit_slam fast_lio2.launch.py

# 3. 播放数据集（另一个终端）
source /home/jmq/agent/workspace/project/fast_lio2_slam/scripts/setup_env.sh
ros2 bag play datasets/Trayectory1/rosbag2_2024_05_23-15_43_25_0.db3 --clock
```

### 方式二：使用现有测试脚本
```bash
cd /home/jmq/agent/workspace/project/fast_lio2_slam
./run_mapping_test.sh
```

## 五、验证结果摘要

```
✅ 通过: 26 项
❌ 失败: 0 项
⚠️ 警告: 1 项 (pcl-ros 可选依赖)
```

**环境状态: 就绪** ✅

可以进行建图测试。