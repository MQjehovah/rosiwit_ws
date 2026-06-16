# rosiwit_navigation — 项目文档

> **版本**: v1.1 (优化后)  
> **文档生成时间**: 2026-05-04  
> **项目路径**: `E:\xzkj\agent\workspace\projects\rosiwit_ws\src\rosiwit_navigation`

---

## 1. 项目概述

`rosiwit_navigation` 是一个基于 **ROS2 Humble** 的差速轮驱动机器人导航功能包，提供：

- **丝滑单点导航** (Smooth Single-Point Navigation)
- **动态绕障** (Dynamic Obstacle Avoidance)  
- **窄道通行** (Narrow Passage Navigation)
- **模块化控制框架**：PID 控制器、Pure Pursuit 轨迹跟踪、速度限制器
- **A\* 路径规划**（含加权启发式和大网格优化）

### 技术栈

| 技术 | 版本/说明 |
|------|----------|
| ROS2 | Humble |
| 语言 | C++17 |
| 构建系统 | CMake / ament_cmake |
| 测试框架 | GTest |
| 代码格式 | clang-format |
| 命名空间 | `diffbot_navigation`（内部）、`rosiwit_navigation`（ROS 包名） |

### 模块结构

```
rosiwit_navigation/
├── include/diffbot_navigation/
│   ├── core/           # 核心：事件总线、状态机、异常、错误管理、参数管理
│   ├── controller/     # 控制：PID、Pure Pursuit、速度限制器、差速驱动
│   ├── planners/       # 规划：A*、路径平滑、代价地图
│   ├── navigation/     # 导航：路径规划接口、轨迹生成器、平滑导航
│   ├── obstacle_avoidance/  # 避障：障碍物检测器、避障规划器
│   └── narrow_passage/ # 窄道：窄道检测器、窄道规划器
├── src/                # 实现文件 (18 .cpp)
├── tests/              # 测试文件 (15+ 测试文件, ~204 用例)
├── docs/               # 项目内置文档
├── .clang-format       # 代码格式化配置
└── CMakeLists.txt      # 构建配置
```

---

## 2. 快速入门

### 2.1 环境要求

- **操作系统**: Linux (Ubuntu 22.04 推荐) / Windows 11
- **ROS2**: Humble Hawksbill
- **编译器**: GCC 11+ / Clang 14+ / MSVC 2022+
- **CMake**: 3.20+

### 2.2 安装依赖

```bash
# ROS2 核心依赖
sudo apt install ros-humble-ros-base ros-humble-nav2-bringup

# 编译依赖
sudo apt install build-essential cmake python3-colcon-common-extensions
```

### 2.3 构建

```bash
cd E:\xzkj\agent\workspace\projects\rosiwit_ws
colcon build --packages-select rosiwit_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **注意**: 如果 Nav2 不可用 (`nav2_core_DIR` 为空)，`diff_drive_controller` 库将自动跳过编译（CMakeLists.txt 已处理）。

### 2.4 运行测试

```bash
colcon test --packages-select rosiwit_navigation
colcon test-result --all --verbose
```

### 2.5 运行导航节点

```bash
ros2 run rosiwit_navigation smooth_navigation_main
```

---

## 3. 文档导航

| 文档 | 说明 | 路径 |
|------|------|------|
| 📘 **API 参考** | 所有公共 API 的详细签名和用法 | [api.md](./api.md) |
| 📐 **架构设计** | 模块架构、数据流、设计决策 | [architecture.md](./architecture.md) |
| 📋 **变更日志** | 优化轮次中的变更记录 | [changelog.md](./changelog.md) |
| 🔒 **安全审计** | 安全漏洞分析与修复 | [../security_report.md](../security_report.md) |
| 🧪 **测试报告** | 测试覆盖率和通过情况 | [../test_report.md](../test_report.md) |

### 项目内置文档

| 文档 | 路径 |
|------|------|
| 文档索引 | `docs/README.md` |
| 架构说明 | `docs/ARCHITECTURE.md` [待更新: 对应代码已变更] |
| API 参考 | `docs/API_REFERENCE.md` [待更新: 对应代码已变更] |
| 快速开始 | `docs/GETTING_STARTED.md` |
| 研究报告 | `docs/research_report.md` |
| 测试报告 | `docs/test_report.md` |

---

## 4. 关键设计文档

### 4.1 已完成的修复 (v1.1)

| 修复ID | 问题 | 状态 |
|--------|------|------|
| FIX-1 | PID 积分抗饱和（AntiWindup） | ✅ 已修复 |
| FIX-2 | 轨迹生成器空路径/单点路径边界处理 | ✅ 已修复 |
| FIX-3 | std::getenv 环境变量注入安全 | ✅ 已修复 |
| FIX-4 | VelocityLimiter dt≈0 除零保护 | ✅ 已修复 |
| FIX-5 | A* 大网格性能优化（加权启发式 + 超时回退） | ✅ 已修复 |

### 4.2 构建库清单

| 库名 | 内容 | 编译条件 |
|------|------|---------|
| `diffbot_core_lib` | 事件总线、状态机、异常、错误管理、参数管理 | 始终 |
| `diffbot_planners_lib` | A* 规划器、RRT 规划器、代价地图 | 始终 |
| `diffbot_narrow_passage_lib` | 窄道检测器、窄道规划器 | 始终 |
| `diffbot_controllers_lib` | Pure Pursuit 控制器 | 始终 |
| `diffbot_controller_utils_lib` | 速度限制器 | 始终 |
| `diffbot_controller_lib` | 差速驱动控制器 | Nav2 可用时 |
| `diffbot_navigation_lib` | 轨迹生成、路径规划、避障、导航协调器 | 始终 |
| `diffbot_controller_fixture` | PID 测试夹具库 (仅测试) | BUILD_TESTING |

---

## 5. 性能目标

| 场景 | 指标 | 目标 |
|------|------|------|
| A* 100×100 无障碍 | 耗时 | < 50ms |
| A* 500×500 无障碍 | 耗时 | < 500ms |
| A* 1000×1000 | 耗时 | < 2s (ε-weighted 或超时回退) |
| PID 控制 | 周期 | < 1ms |
| 轨迹生成 | 100点路径 | < 10ms |

---

## 6. 安全要点

- ✅ `std::getenv` 已替换为 `rclcpp::Node::declare_parameter` (CVE-1 已修复)
- ✅ A* 超时和迭代上限防止 DoS (CVE-2 已修复)
- ✅ VelocityLimiter dt≈0 除零保护 (CVE-3 已修复)
- ✅ EventBus 超时机制防止死锁 (CVE-4 已修复)
- ✅ 析构函数异常保护 (CVE-5 已修复)

详见 [安全审计报告](../security_report.md)。
