# rosiwit_navigation

> ROS2 Humble 差速轮驱动机器人导航功能包 — 丝滑单点导航、动态绕障、窄道通行

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green)](LICENSE)
[![Build](https://img.shields.io/badge/build-ament__cmake-orange)](CMakeLists.txt)
[![C++17](https://img.shields.io/badge/C++-17-blue)](CMakeLists.txt)

---

## 目录

- [快速开始](#快速开始)
- [环境要求](#环境要求)
- [构建 & 安装](#构建--安装)
- [运行](#运行)
- [测试](#测试)
- [Docker 部署](#docker-部署)
- [CI/CD](#cicd)
- [项目结构](#项目结构)
- [配置说明](#配置说明)
- [开发指南](#开发指南)

---

## 快速开始

```bash
# 1. 克隆仓库
cd ~/ros2_ws/src
git clone https://github.com/ai-dev-team/diffbot_navigation.git rosiwit_navigation

# 2. 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 构建
colcon build --packages-select diffbot_navigation --symlink-install

# 4. 运行
source install/setup.bash
ros2 launch diffbot_navigation navigation.launch.py
```

---

## 环境要求

| 依赖 | 版本 | 说明 |
|------|------|------|
| **操作系统** | Ubuntu 22.04 LTS | 推荐；Docker 支持跨平台 |
| **ROS2** | Humble Hawksbill | 必须 |
| **CMake** | ≥ 3.8 | 构建系统 |
| **C++ 标准** | C++17 | 编译器需支持 |
| **内存** | ≥ 4 GB | 含 Nav2 推荐 8 GB |

### 核心依赖

```
rclcpp, rclcpp_lifecycle          # ROS2 客户端库
geometry_msgs, nav_msgs           # 消息类型
sensor_msgs, tf2_ros              # 传感器 / TF
ament_cmake, ament_cmake_gtest    # 构建 & 测试
```

### 可选依赖 (需设置 `NAV2_ENABLED=1`)

```
navigation2, nav2_msgs, nav2_util, nav2_core
nav2_costmap_2d, nav2_planner, nav2_controller
```

---

## 构建 & 安装

### 从源码构建

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url> rosiwit_navigation

# 安装 ROS 依赖
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译（带测试）
colcon build --packages-select diffbot_navigation --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 加载环境
source install/setup.bash
```

### 构建类型

| 类型 | 标志 | 用途 |
|------|------|------|
| **Debug** | `-O0 -g` + ASan/UBSan | 开发调试、内存检测 |
| **RelWithDebInfo** | `-O2 -g` + 安全加固 | 日常开发（默认） |
| **Release** | `-O2 -DNDEBUG` + 全加固 | 生产部署 |

```bash
# Debug 模式（含 AddressSanitizer + UndefinedBehaviorSanitizer）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON

# Release 模式（安全加固：FORTIFY_SOURCE + stack-protector + PIE）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 禁用 Sanitizers（Debug 模式）

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=OFF -DENABLE_UBSAN=OFF
```

---

## 运行

### 启动导航节点

```bash
source install/setup.bash
ros2 launch diffbot_navigation navigation.launch.py
```

### 启动 RViz 可视化

```bash
ros2 launch diffbot_navigation rviz.launch.py
```

### 发送导航目标

```bash
# 通过命令行发送目标位姿 (x, y, yaw)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

---

## 测试

### 运行全部测试

```bash
colcon test --packages-select diffbot_navigation --event-handlers console_direct+
colcon test-result --all --verbose
```

### 按标签运行

```bash
# 仅单元测试
colcon test --packages-select diffbot_navigation --ctest-args -L unit

# 仅性能测试
colcon test --packages-select diffbot_navigation --ctest-args -L performance
```

### 单独运行某个测试

```bash
# 运行 PID 控制器测试
./build/diffbot_navigation/test_pid_controller --gtest_filter='*AntiWindup*'

# 运行 A* 性能测试
./build/diffbot_navigation/test_astar_planner_perf --gtest_filter='*LargeGrid*'
```

### 测试文件清单

| 测试文件 | 模块 | 用例数 | 说明 |
|---------|------|--------|------|
| `test_state_machine.cpp` | core | ~15 | 状态机转换 |
| `test_error_manager.cpp` | core | ~12 | 错误处理流程 |
| `test_event_bus.cpp` | core | ~18 | 事件总线发布/订阅 |
| `test_parameter_manager.cpp` | core | ~10 | 参数加载/验证 |
| `test_narrow_passage_detector.cpp` | narrow_passage | ~8 | 窄道检测 |
| `test_narrow_passage_enhanced.cpp` | narrow_passage | ~10 | 窄道增强场景 |
| `test_diff_drive_controller.cpp` | controller | ~12 | 差速轮控制 |
| `test_pid_controller.cpp` | controller | ~10 | PID抗饱和/反算 |
| `test_trajectory_generator.cpp` | navigation | ~15 | 轨迹生成 |
| `test_trajectory_generator_edge.cpp` | navigation | ~8 | 边界路径处理 |
| `test_astar_planner.cpp` | planners | ~12 | A* 规划基础 |
| `test_astar_planner_perf.cpp` | planners | ~8 | A* 大网格性能 |
| `test_velocity_limiter.cpp` | controller | ~10 | 速度约束 |
| `test_obstacle_detector.cpp` | obstacle_avoidance | ~15 | 障碍物检测 |
| `test_obstacle_avoidance.cpp` | obstacle_avoidance | ~10 | 避障策略 |
| `test_single_point_navigation.cpp` | navigation | ~15 | 单点导航端到端 |
| `test_integration.cpp` | 集成 | ~8 | 多模块集成 |
| `test_performance.cpp` | 性能 | ~10 | 性能基准 |
| `test_performance_benchmark.cpp` | 性能 | ~12 | 详细性能分析 |
| `test_narrow_passage.cpp` | narrow_passage | ~6 | 窄道通行 |

**总计: 20 个测试文件, ~204 个测试用例**

---

## Docker 部署

### 开发环境

```bash
# 构建并启动开发容器
docker compose up dev --build

# 进入容器
docker compose exec dev bash

# 在容器内构建
source /opt/ros/humble/setup.bash
colcon build --packages-select diffbot_navigation --symlink-install
```

### 运行测试

```bash
# CI 模式测试
docker compose run --rm test

# 代码检查
docker compose run --rm lint
```

### 生产部署

```bash
# 构建运行时镜像
docker compose build runtime

# 启动导航节点
docker compose up runtime
```

### VS Code Dev Container

1. 安装 [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. 打开项目文件夹
3. 点击左下角 `><` → "Reopen in Container"
4. 自动构建并配置好 C++ / ROS2 开发环境

---

## CI/CD

CI 流水线配置于 `.github/workflows/ci.yml`，包含四个阶段：

| 阶段 | 触发条件 | 说明 |
|------|---------|------|
| **Lint** | Push / PR | clang-format + cppcheck 静态分析 |
| **Build** | Push / PR | Debug / RelWithDebInfo / Release 三矩阵编译 |
| **Test** | Push / PR | 单元 / 集成 / 性能测试 |
| **Security** | Push / PR | 环境变量注入检测 / 危险模式扫描 |

性能测试默认仅在手动触发 (`workflow_dispatch`) 时运行。

---

## 项目结构

```
rosiwit_navigation/
├── include/diffbot_navigation/    # 公共头文件
│   ├── controller/               # PIDController, DiffDriveController
│   ├── controllers/              # 控制器工具
│   ├── core/                     # 状态机, 事件总线, 错误管理, 参数
│   ├── narrow_passage/           # 窄道检测器
│   ├── navigation/               # 导航协调器, 轨迹生成器
│   ├── obstacle_avoidance/       # 障碍物检测, 避障
│   └── planners/                 # A* 规划器
│
├── src/                          # 实现文件 (18 .cpp)
│   ├── controller/               # diff_drive_controller.cpp
│   ├── controllers/              # velocity_limiter.cpp
│   ├── core/                     # state_machine, error_manager, event_bus, parameter_manager
│   ├── narrow_passage/           # narrow_passage_detector.cpp
│   ├── navigation/               # trajectory_generator, single_point_navigation
│   ├── obstacle_avoidance/       # obstacle_detector, obstacle_avoidance
│   └── planners/                 # astar_planner.cpp
│
├── test/                         # 测试文件 (20 .cpp, ~204 用例)
├── config/                       # ROS2 参数配置 YAML
│   ├── controller_params.yaml    # PID / 控制器参数
│   ├── planner_params.yaml       # A* 配置 (迭代上限, 超时, ε)
│   ├── navigation_params.yaml    # 导航参数
│   ├── costmap_params.yaml       # 代价地图
│   └── obstacle_params.yaml      # 避障参数
│
├── launch/                       # ROS2 Launch 文件
│   ├── navigation.launch.py
│   └── rviz.launch.py
│
├── maps/                         # 测试/演示地图
├── docs/                         # 文档
│   ├── GETTING_STARTED.md
│   ├── ARCHITECTURE.md
│   ├── API_REFERENCE.md
│   └── test_report.md
│
├── Dockerfile                    # 多阶段 Docker 构建
├── docker-compose.yml            # Docker Compose (dev/test/runtime)
├── docker-entrypoint.sh          # 容器入口脚本
├── .dockerignore
├── .devcontainer/                # VS Code Dev Container
├── .github/workflows/ci.yml      # GitHub Actions CI/CD
├── .clang-format                 # C++ 格式化配置
├── .clang-tidy                   # 静态分析配置
├── CMakeLists.txt                # ament_cmake 构建
└── package.xml                   # ROS2 包清单
```

---

## 配置说明

### 关键参数

所有运行时参数通过 `config/` 目录下的 YAML 文件配置。

| 参数 | 默认值 | 位置 | 说明 |
|------|--------|------|------|
| `k_p, k_i, k_d` | 1.0, 0.1, 0.01 | controller_params.yaml | PID 增益 |
| `anti_windup_mode` | 1 (条件积分) | controller_params.yaml | 抗饱和模式: 0=None, 1=Conditional, 2=BackCalc |
| `integral_max/min` | ±1.0 | controller_params.yaml | 积分限幅 |
| `output_max/min` | ±1.0 | controller_params.yaml | 输出限幅 |
| `max_iterations` | 50000 | planner_params.yaml | A* 迭代上限 |
| `planning_timeout_ms` | 2000 | planner_params.yaml | 规划超时 (ms) |
| `heuristic_epsilon` | 1.5 | planner_params.yaml | 加权启发式系数 (>1 加速) |
| `fallback_on_timeout` | true | planner_params.yaml | 超时返回 best-effort 路径 |

### 自定义配置

```bash
# 使用自定义参数文件
ros2 launch diffbot_navigation navigation.launch.py \
    params_file:=/path/to/custom_params.yaml
```

---

## 开发指南

### 代码风格

- 基于 Google 风格，2 空格缩进，100 字符行宽
- 格式化: `clang-format -i <file>`
- 静态检查: `clang-tidy -p build/diffbot_navigation <file>`

### 添加新模块

1. 在 `include/diffbot_navigation/<module>/` 下添加头文件
2. 在 `src/<module>/` 下添加实现
3. 在 `CMakeLists.txt` 中注册库/可执行文件
4. 添加对应的单元测试到 `test/`
5. 运行 `clang-format` 格式化

### 安全注意事项

- **禁止使用** `std::getenv` — 改用 ROS2 参数服务器获取配置
- **dt 除零防护**: 所有 `dt` 除法前检查 `dt > 1e-6`
- **EventBus 回调**: 避免在回调中调用 `subscribe()`/`publish()`，防止死锁
- **Release 构建**自动启用 `-D_FORTIFY_SOURCE=2` + `-fstack-protector-strong`

### 已知限制 & 待优化

| 项目 | 状态 | 说明 |
|------|------|------|
| PID 积分抗饱和 | ✅ 已修复 | 条件积分 + 反算模式 |
| 空路径/单点路径 | ✅ 已修复 | 显式边界语义 |
| A* 大网格性能 | ✅ 已优化 | 迭代上限 + 超时回退 + ε 启发式 |
| `std::getenv` 移除 | ✅ 已消除 | 全量替换为常量/参数 |
| 窄道检测器测试 | ⚠️ 覆盖率偏低 | 已补充增强测试 |
| Windows 环境 | ⚠️ 不支持 | 仅 Linux / Docker |
| 集成测试 | ⚠️ 需扩展 | 当前 1 个文件 |

---

## 许可证

Apache-2.0 — 详见 [LICENSE](LICENSE) 文件。

---

> **维护者**: AI Development Team <ai-dev-team@example.com>
> **仓库**: https://github.com/ai-dev-team/diffbot_navigation
