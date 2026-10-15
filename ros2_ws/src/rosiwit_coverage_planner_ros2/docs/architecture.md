# ROS2 Coverage Planner 架构说明

> **版本**: v2.0
> **最后更新**: 2026-04-29
> **来源**: architecture.md (整理架构师输出)
> **修复**: TurnOptimizer合并条件已优化（v2.0）

---

## 目录

1. [架构概述](#架构概述)
2. [系统架构图](#系统架构图)
3. [核心模块](#核心模块)
4. [设计模式](#设计模式)
5. [数据流](#数据流)
6. [技术决策](#技术决策)

---

## 架构概述

### 设计目标

开发一个模块化、可扩展的ROS2全覆盖路径规划功能包：

- **双算法支持**: 弓字形（Zigzag）和回字形（Spiral）都是核心功能
- **策略模式架构**: 算法可插拔，便于扩展新算法
- **ROS2标准接口**: 遵循ROS2 Humble规范
- **高测试覆盖**: 单元测试覆盖率≥80%

---

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Coverage Planner                        │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │             CoveragePlannerNode (ROS2接口层)             │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐  │  │
│  │  │ Subscription │  │  Publisher   │  │  Param Server  │  │  │
│  │  │   /map       │  │/coverage_path│  │coverage_mode   │  │  │
│  │  │  /initialpose│  │              │  │robot_radius    │  │  │
│  │  └──────────────┘  └──────────────┘  └────────────────┘  │  │
│  └────────────────────────┬─────────────────────────────────┘  │
│                           │                                     │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │           PlannerContext (策略上下文)                    │  │
│  │  - selectPlanner(coverage_mode) → IPlanner               │  │
│  └────────────────────────┬─────────────────────────────────┘  │
│                           │                                     │
│               ┌───────────┴───────────┐                        │
│               ▼                       ▼                         │
│  ┌────────────────────┐    ┌────────────────────┐              │
│  │  ZigzagPlanner     │    │  SpiralPlanner     │              │
│  │  (弓字形算法)       │    │  (回字形算法)       │              │
│  │  - BSA扫描线       │    │  - 区域分解        │              │
│  │  - 障碍物恢复      │    │  - 螺旋生成        │              │
│  └────────────────────┘    └────────────────────┘              │
│               │                       │                         │
│               └───────────┬───────────┘                        │
│                           ▼                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                 Common Utils (公共工具)                   │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐  │  │
│  │  │  MapUtils    │  │  PathUtils   │  │ CoverageStats  │  │  │
│  │  │  - 膨胀      │  │  - 平滑      │  │  - 覆盖率      │  │  │
│  │  │  - 可达性    │  │  - 统计      │  │  - 转弯数      │  │  │
│  │  └──────────────┘  └──────────────┘  └────────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 核心模块

### 模块列表

| 模块 | 职责 | 优先级 | 文件位置 |
|------|------|--------|----------|
| **CoveragePlannerNode** | ROS2节点主类，管理生命周期和接口 | P0 | coverage_planner_node.cpp |
| **IPlanner接口** | 规划器抽象接口，定义统一API | P0 | i_planner.hpp |
| **PlannerContext** | 策略上下文，根据参数选择算法 | P0 | planner_context.cpp |
| **ZigzagPlanner** | 弓字形算法实现 | P0 | zigzag_planner.cpp |
| **SpiralPlanner** | 回字形算法实现 | P0 | spiral_planner.cpp |
| **MapUtils** | 地图处理工具（膨胀、可达性） | P0 | coverage_utils.cpp |
| **PathUtils** | 路径处理工具（平滑、统计） | P1 | coverage_utils.cpp |
| **CoverageStats** | 覆盖率统计计算 | P1 | coverage_utils.cpp |
| **ZoneDecomposer** | 分区规划器，复杂地图区域分解 | P0 | zone_decomposer.cpp |
| **TurnOptimizer** | 转弯优化器，减少转弯次数 | P0 | turn_optimizer.cpp |
| **MapPreprocessor** | 地图预处理（形态学处理） | P1 | map_preprocessor.cpp |
| **ScanDirectionOptimizer** | 扫描方向优化（PCA分析） | P1 | scan_direction_optimizer.cpp |

---

## 设计模式

### 策略模式（Strategy Pattern）

本项目采用策略模式实现算法可插拔：

```
IPlanner（抽象策略接口）
    ↑
    ├── ZigzagPlanner（具体策略A）
    └── SpiralPlanner（具体策略B）

PlannerContext（策略上下文）
    - selectPlanner(mode) → IPlanner
```

**优势**:
- 算法切换无需修改代码，仅需修改参数
- 易于扩展新算法（如牛耕式、遗传算法）
- 符合开闭原则

### 降级策略

SpiralPlanner在复杂情况下的降级机制：

```
SpiralPlanner.plan()
    ├── 非凸区域分解成功 → 螺旋路径
    └── 区域分解失败 → 自动降级到ZigzagPlanner
```

---

## 数据流

### 规划流程

```
1. ROS2节点启动
   ↓
2. 订阅/map话题 → 接收OccupancyGrid
   ↓
3. 订阅/initialpose → 接收起始位置
   ↓
4. 触发/plan_coverage服务
   ↓
5. PlannerContext.selectPlanner(mode)
   ↓
6. IPlanner.plan(map, start_pose, config)
   ├── ZigzagPlanner → BSA扫描线规划
   └── SpiralPlanner → 螺旋覆盖规划
   ↓
7. PlannerResult → 路径 + 统计信息
   ↓
8. 发布/coverage_path话题 → nav_msgs/Path
```

### 地图处理流程

```
OccupancyGrid输入
    ↓
MapUtils.inflateMap() → 障碍物膨胀
    ↓
MapUtils.getReachableCells() → BFS可达性检查
    ↓
MapUtils.getFreeCells() → 空闲栅格提取
    ↓
规划器处理 → 路径点序列
```

---

## 技术决策

### ADR-001: 算法优先级修正

**状态**: 已采纳

**背景**:
用户原始需求明确要求"两种算法"，之前错误地将回字形算法降级为P1。

**决策**:
两种算法（弓字形、回字形）都是P0核心功能，必须全部实现。

**理由**:
- 用户需求：明确要求两种算法并列，而非备选
- 场景差异：不同算法适用于不同环境（开阔空间 vs 复杂障碍物）
- 竞品参考：ROS1方案提供多种算法选择

### ADR-002: 策略模式架构

**状态**: 已采纳

**背景**:
需要支持多种规划算法，且便于后续扩展。

**决策**:
采用策略模式，通过`IPlanner`抽象接口和`PlannerContext`上下文类管理算法切换。

**理由**:
- 开闭原则：新增算法无需修改现有代码
- 运行时切换：通过参数选择算法
- 单一职责：每个规划器只负责一种算法

### ADR-003: Spiral降级策略

**状态**: 已采纳

**背景**:
回字形算法在非凸区域分解时可能失败。

**决策**:
SpiralPlanner失败时自动降级到ZigzagPlanner，确保鲁棒性。

**理由**:
- 保证功能可用性：失败时仍有路径输出
- 用户无感知：降级过程自动完成
- 日志记录：方便问题诊断

### ADR-004: 简单膨胀优先

**状态**: 已采纳

**背景**:
障碍物膨胀处理复杂度较高。

**决策**:
P0实现简单圆形膨胀，P1优化轮廓膨胀和动态膨胀。

**理由**:
- MVP原则：满足80%场景的简单实现
- 技术风险控制：复杂优化后置
- 验证后再优化：基于实际测试数据改进

---

## 目录结构

```
ros2_coverage_planner/
├── package.xml              # ROS2包描述
├── CMakeLists.txt           # 构建配置
├── config/
│   └ coverage_params.yaml   # 参数配置文件
├── launch/
│   └ coverage_planner.launch.py  # Launch文件
├── include/coverage_planner/
│   ├── coverage_planner.hpp      # ROS2节点头文件
│   ├── i_planner.hpp             # 规划器接口
│   ├── planner_context.hpp       # 策略上下文
│   ├── zigzag_planner.hpp        # 弓字形头文件
│   ├── spiral_planner.hpp        # 回字形头文件
│   └── coverage_utils.hpp        # 工具类头文件
├── src/
│   ├── coverage_planner_node.cpp # ROS2节点实现
│   ├── zigzag_planner.cpp        # 弓字形实现
│   ├── spiral_planner.cpp        # 回字形实现
│   ├── coverage_utils.cpp        # 工具类实现
│   └ planner_context.cpp         # 策略上下文实现
├── test/
│   ├── test_zigzag_planner.cpp   # 弓字形测试
│   ├── test_spiral_planner.cpp   # 回字形测试
│   └ test_coverage_utils.cpp     # 工具类测试
│   ├── test_integration.cpp      # 集成测试
│   └ test_performance.cpp        # 性能测试
└── README.md                   # 项目文档
```

---

## 算法详解

### ZigzagPlanner（弓字形算法）

**核心流程**:

1. **BSA扫描线分割**
   - 将地图分割为水平或垂直扫描线
   - 根据障碍物打断扫描线

2. **障碍物打断恢复**
   - 扫描线被障碍物打断后，在另一侧继续
   - 计算最近可达点作为恢复起点

3. **最优方向选择**
   - 计算水平/垂直方向的转弯次数
   - 选择转弯次数较少的方向

**关键参数**:
- `robot_radius`: 机器人半径，用于障碍物膨胀
- `coverage_resolution`: 路径点间隔
- `direction_optimization`: 扫描方向优化策略

### SpiralPlanner（回字形算法）

**核心流程**:

1. **区域边界计算**
   - 确定可覆盖区域的最小/最大边界
   - 处理非凸区域的分解

2. **螺旋生成**
   - 从外边界向内螺旋
   - 每一圈收缩一定距离

3. **降级策略**
   - 区域分解失败时切换到ZigzagPlanner
   - 保证至少有一种算法能输出路径

**关键参数**:
- `spiral_direction`: 螺旋方向（顺时针/逆时针）
- `spiral_step`: 螺旋收缩步长

---

## 性能设计

### 性能目标

| 地图大小 | 规划耗时目标 | 内存占用目标 |
|----------|-------------|-------------|
| 100x100 | <500ms | <10MB |
| 500x500 | <3s | <50MB |
| 1000x1000 | <10s | <100MB |

### 性能优化策略

1. **地图预处理**
   - 障碍物膨胀一次完成，避免重复计算
   - 可达性检查使用BFS，效率高于DFS

2. **路径生成**
   - 扫描线批量生成，减少逐点计算
   - 避免不必要的坐标转换

3. **内存管理**
   - 使用`std::vector<bool>`存储覆盖标记（节省内存）
   - 预分配内存避免动态扩容开销

---

## 安全设计

### 输入验证

| 输入 | 验证规则 | 失败处理 |
|------|----------|----------|
| 地图宽度 | ≤46340（整数溢出防护） | 拒绝处理，报错 |
| 地图高度 | ≤46340（整数溢出防护） | 拒绝处理，报错 |
| 地图分辨率 | >0（除零防护） | 拒绝处理，报错 |
| 机器人半径 | >0 | 参数验证失败，使用默认值 |

### 异常处理

- 规划失败返回空路径，`success=false`
- 错误信息记录在`error_message`字段
- ROS日志输出警告和错误

---

## 测试架构

### 测试层次

```
单元测试（gtest）
    ├── ZigzagPlannerTest
    ├── SpiralPlannerTest
    └ MapUtilsTest / PathUtilsTest

集成测试
    ├── 算法切换验证
    ├── 覆盖率验收测试
    └ 路径连续性测试

性能测试
    ├── 规划耗时测试
    ├── 内存占用估算
    └ 大地图稳定性测试
```

### 测试覆盖率

| 模块 | 目标覆盖率 | 实际估算 |
|------|-----------|---------|
| ZigzagPlanner | ≥90% | ~92% |
| SpiralPlanner | ≥90% | ~95% |
| MapUtils | ≥95% | ~98% |
| PathUtils | ≥85% | ~90% |
| **总体** | ≥80% | ~90% |

---

## 相关文档

- [API参考](api.md) - 完整API文档
- [变更记录](changelog.md) - 版本历史
- [README](../README.md) - 使用指南