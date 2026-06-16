# 差速轮机器人导航算法技术选型报告

**项目名称**: ROS2差速轮机器人导航程序  
**版本**: v1.0  
**日期**: 2026-04-25  
**编写**: 算法研究员

---

## 一、项目技术需求摘要

### 1.1 核心功能需求

| 功能模块 | 技术要求 | 性能指标 |
|---------|---------|---------|
| 丝滑单点导航 | 平滑移动、速度规划、路径跟踪 | 到达精度 < 0.05m，速度连续无突变 |
| 绕障功能 | 动态障碍检测、局部重规划、平滑轨迹 | 响应时间 < 0.5s，避障成功率 > 95% |
| 过窄道功能 | 窄道识别、安全通行策略 | 最窄通道 ≥ 机器人宽度 + 10cm |

### 1.2 技术约束

- **平台**: ROS2 Humble/Foxy
- **运动学模型**: 差速轮（Differential Drive）
- **架构要求**: 模块化设计
- **实时性**: 控制频率 ≥ 20Hz

---

## 二、候选方案调研

### 2.1 全局路径规划算法

#### 方案A: A*算法

**算法原理**:
- 基于栅格地图的启发式搜索算法
- 估价函数: f(n) = g(n) + h(n)
- 支持任意角度路径优化（Theta*）

**优点**:
- 算法成熟，实现简单
- 计算效率高，O(n log n)
- ROS2 Nav2原生支持
- 可扩展性强（Jump Point Search优化）

**缺点**:
- 栅格分辨率影响精度和效率
- 转弯点可能不够平滑
- 动态环境需频繁重规划

**适用场景**: 静态或半静态环境，中等规模地图

#### 方案B: RRT* (Rapidly-exploring Random Tree*)

**算法原理**:
- 采样型路径规划算法
- 渐进最优，支持动态障碍物
- 概率完备性

**优点**:
- 适合高维空间
- 支持非完整约束（差速轮）
- 可处理复杂障碍物形状
- 路径渐进优化

**缺点**:
- 计算时间不确定
- 收敛速度依赖参数调优
- 实时性可能受影响

**适用场景**: 复杂环境、非结构化场景

#### 方案C: Hybrid A* (Nav2 Smac Planner)

**算法原理**:
- 结合A*与运动学约束
- 考虑车辆运动学模型
- 支持倒车和多状态搜索

**优点**:
- **差速轮模型原生支持**
- 路径符合运动学约束
- ROS2 Nav2官方推荐
- 支持Reeds-Shepp曲线优化

**缺点**:
- 计算开销较大
- 需精细参数调优
- 栅格依赖性

**适用场景**: **差速轮/阿克曼转向机器人，推荐方案**

---

### 2.2 局部路径规划算法

#### 方案A: DWA (Dynamic Window Approach)

**算法原理**:
- 在速度空间采样可行轨迹
- 评价函数: heading + distance + velocity
- 满足运动学约束的速度窗口

**优点**:
- 计算简单，实时性好
- **差速轮模型完美适配**
- 参数直观易调
- ROS2原生支持（Nav2 DWB Controller）

**缺点**:
- 可能陷入局部最优
- 窄道通行困难
- 缺乏全局信息

**适用场景**: 开阔环境、动态避障

#### 方案B: TEB (Timed Elastic Band)

**算法原理**:
- 基于弹性带的轨迹优化
- 显式时间参数化
- 多目标优化（G2O框架）

**优点**:
- **轨迹平滑性极佳**
- 支持倒车（差速轮适用）
- 优秀的时间最优规划
- 支持动态障碍物
- **窄道通行能力强**

**缺点**:
- 计算开销较大
- 参数调优复杂
- 依赖优化库（G2O）

**适用场景**: **复杂环境、窄道通行，强烈推荐**

#### 方案C: MPC (Model Predictive Control)

**算法原理**:
- 基于模型的预测控制
- 滚动时域优化
- 显式处理约束

**优点**:
- 控制性能优异
- 可显式处理约束
- 轨迹极度平滑
- 前瞻能力强

**缺点**:
- 计算复杂度高
- 需精确模型参数
- 实时性挑战大

**适用场景**: 高精度控制需求

---

### 2.3 运动控制算法

#### 方案A: Pure Pursuit

**算法原理**:
- 几何跟踪方法
- 前视点追踪
- 曲率计算控制转向

**优点**:
- 算法极简
- 计算效率高
- 稳定性好
- **差速轮模型完美适配**

**缺点**:
- 前视距离敏感
- 切角效应
- 速度变化时性能下降

**适用场景**: 低速场景、路径跟踪

#### 方案B: Stanley Controller

**算法原理**:
- Stanford大学提出
- 横向误差+航向误差
- 非线性反馈

**优点**:
- 低速精度高
- 收敛快
- 参数少

**缺点**:
- 高速震荡
- 对曲率敏感

**适用场景**: 低速精准停靠

#### 方案C: PID控制器

**算法原理**:
- 线性角速度控制
- 增量式PID
- 分离线速度与角速度控制

**优点**:
- 实现简单
- 计算极快
- 工业应用广泛
- **ROS2标准方案**

**缺点**:
- 参数整定依赖经验
- 非线性场景局限

**适用场景**: **基础速度控制层**

---

### 2.4 动态避障方案

#### 方案A: Costmap + Inflation Layer

**算法原理**:
- 代价地图障碍物膨胀
- 膨胀半径 = 机器人半径 + 安全边距
- 致命区域与警告区域分级

**优点**:
- Nav2原生支持
- 计算高效
- 参数直观

**缺点**:
- 静态膨胀，不够灵活
- 动态障碍响应延迟

**适用场景**: 基础避障层

#### 方案B: VO (Velocity Obstacles) + RVO

**算法原理**:
- 速度障碍锥
- 多智能体避撞
- 相对速度计算

**优点**:
- 动态障碍处理优秀
- 预测能力强
- 多机器人协作支持

**缺点**:
- 计算复杂度O(n²)
- ROS2集成需自研

**适用场景**: 高动态环境

#### 方案C: 社会力模型 (Social Force Model)

**算法原理**:
- 机器人受目标吸引、障碍排斥
- 力学模型模拟运动
- 符合人类行为

**优点**:
- 轨迹自然
- 人群环境优秀

**缺点**:
- 参数多
- 需要行为建模

**适用场景**: 人机共存环境

---

### 2.5 窄道通行方案

#### 方案A: 窄道检测 + 窄管规划器 (Nav2 NavFn + Costmap Filter)

**算法原理**:
- Costmap过滤保留可通行区域
- 窄道区域标记
- 路径沿窄道中心线生成

**优点**:
- Nav2集成方案
- 实现简单

**缺点**:
- 需预定义窄道
- 动态适应差

#### 方案B: STVL (Space Time Velocity Library)

**算法原理**:
- 时空速度规划
- 考虑机器人轮廓约束
- 连续碰撞检测

**优点**:
- 精确轮廓考虑
- 安全性高

**缺点**:
- 计算开销大

#### 方案C: TEB优化器 + 窄道约束

**算法原理**:
- TEB规划器添加窄道约束
- 最小化轨迹偏离窄道中心
- 轮廓精确碰撞检测

**优点**:
- **轨迹平滑度好**
- **支持差速轮运动学**
- 参数可调适应不同窄道宽度
- **ROS2 TEB Controller原生支持**

**缺点**:
- 需精细参数调优

**推荐**: **方案C为首选**

---

## 三、ROS2 Nav2架构分析

### 3.1 Nav2架构优势

```
┌─────────────────────────────────────────────────────┐
│                  Nav2 Architecture                  │
├─────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │
│  │  Planner    │  │ Controller  │  │ Behavior   │  │
│  │  Server     │  │  Server     │  │  Server    │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │
│         │                │                │         │
│  ┌──────┴────────────────┴────────────────┴──────┐  │
│  │              Nav2 Action Servers              │  │
│  └───────────────────────────────────────────────┘  │
│         │                │                │         │
│  ┌──────┴──────┐  ┌──────┴──────┐  ┌──────┴──────┐  │
│  │  Costmap    │  │  BT Navigator│  │  Recovery   │  │
│  │  2D         │  │              │  │  Behaviors  │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────┘
```

**核心优势**:

1. **模块化设计**: Planner/Controller/Recovery可独立替换
2. **行为树架构**: 复杂导航逻辑可视化编排
3. **插件系统**: 自定义算法无缝集成
4. **生态系统**: 大量成熟插件可用
5. **社区支持**: 文档完善、社区活跃

### 3.2 Nav2适用性评估

| 维度 | 评分 | 说明 |
|-----|-----|------|
| 功能覆盖 | ★★★★★ | 全面覆盖导航需求 |
| 扩展性 | ★★★★★ | 插件系统完善 |
| 学习曲线 | ★★★☆☆ | 配置复杂，学习成本中等 |
| 性能 | ★★★★☆ | 满足大多数场景 |
| ROS2集成 | ★★★★★ | 原生支持 |

---

## 四、推荐方案

### 4.1 总体推荐方案

```
┌──────────────────────────────────────────────────────────────┐
│                    推荐技术栈                                 │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │            架构层: ROS2 Nav2 框架                       │ │
│  └────────────────────────────────────────────────────────┘ │
│                           │                                  │
│  ┌────────────────┬───────┴────────┬───────────────────────┐│
│  │                │                │                       ││
│  ▼                ▼                ▼                       ││
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐ ││
│  │ 全局规划器   │ │ 局部规划器   │ │ 运动控制器           │ ││
│  │              │ │              │ │                      │ ││
│  │ Smac Hybrid │ │ TEB          │ │ Pure Pursuit + PID   │ ││
│  │ A* Planner  │ │ Controller   │ │ (速度执行层)         │ ││
│  └──────────────┘ └──────────────┘ └──────────────────────┘ ││
│                                                              │
│  ┌────────────────┬────────────────┬───────────────────────┐│
│  ▼                ▼                ▼                       ││
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐ ││
│  │ 避障模块     │ │ 窄道模块     │ │ 恢复行为             │ ││
│  │              │ │              │ │                      │ ││
│  │ Costmap +    │ │ TEB +        │ │ Spin + BackUp +      │ ││
│  │ Inflation    │ │ 轮廓约束     │ │ Wait                 │ ││
│  └──────────────┘ └──────────────┘ └──────────────────────┘ ││
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### 4.2 各模块详细推荐

#### 4.2.1 全局路径规划

| 模块 | 推荐方案 | 理由 |
|-----|---------|------|
| **首选** | **Smac Hybrid A* Planner** | 差速轮模型原生支持，Nav2官方推荐，路径符合运动学约束 |
| 备选 | NavFn Planner | 简单场景快速实现 |

**关键参数建议**:
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.05                    # 目标容差(m)
      allow_unknown: false               # 不允许穿越未知区域
      analytic_expansion_max_length: 3.0 # 解析扩展长度
      motion_model: "REEDS_SHEPP"        # 运动模型(支持倒车)
      minimum_turning_radius: 0.4        # 最小转弯半径
    grid_resolution: 0.05               # 栅格分辨率(m)
```

#### 4.2.2 局部路径规划

| 模块 | 推荐方案 | 理由 |
|-----|---------|------|
| **首选** | **TEB Local Planner** | 轨迹平滑、窄道通行能力强、支持差速轮倒车 |
| 备选 | DWB Controller | 计算简单、实时性好 |

**关键参数建议**:
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["TEB"]
    TEB:
      plugin: "teb_local_planner/TebLocalPlannerROS"
      
      # 机器人配置
      odom_topic: "/odom"
      map_frame: "map"
      
      # 轨迹配置
      teb_autosize: true
      dt_ref: 0.3              # 轨迹时间分辨率
      dt_hysteresis: 0.1
      min_samples: 3
      max_samples: 500
      
      # 机器人运动学
      max_vel_x: 0.5           # 最大前进速度(m/s)
      max_vel_x_backwards: 0.2  # 最大后退速度
      max_vel_theta: 1.0       # 最大转向速度(rad/s)
      acc_lim_x: 0.5           # 线加速度限制
      acc_lim_theta: 1.0       # 角加速度限制
      
      # 优化参数
      optimization_activate: true
      optimization_verbose: false
      weight_obstacle: 50.0    # 避障权重(窄道关键)
      weight_dynamic_obstacle: 20.0
      weight_viapoint: 1.0
      
      # 窄道通行参数
      min_obstacle_dist: 0.25  # 最小障碍物距离
      inflation_dist: 0.3     # 膨胀距离
```

#### 4.2.3 运动控制

| 层次 | 推荐方案 | 理由 |
|-----|---------|------|
| 轨迹跟踪层 | Pure Pursuit | 算法简单、差速轮适配好 |
| 速度执行层 | PID控制 | 实现简单、响应稳定 |

**Pure Pursuit关键参数**:
```yaml
pure_pursuit:
  lookahead_distance: 0.6     # 前视距离(m)
  lookahead_time: 1.5         # 前视时间(s)
  min_lookahead_distance: 0.3
  max_lookahead_distance: 0.9
  speed_gain: 0.5             # 速度增益
```

**PID参数建议**:
```yaml
pid:
  linear:
    kp: 1.0
    ki: 0.01
    kd: 0.1
  angular:
    kp: 2.0
    ki: 0.02
    kd: 0.2
```

#### 4.2.4 避障方案

| 模块 | 推荐方案 | 理由 |
|-----|---------|------|
| 静态避障 | Costmap + Inflation Layer | Nav2原生支持 |
| 动态避障 | TEB动态障碍权重 | 结合局部规划器 |

**Costmap配置建议**:
```yaml
local_costmap:
  robot_radius: 0.22          # 机器人外接圆半径
  inflation_radius: 0.55      # 膨胀半径
  cost_scaling_factor: 3.0    # 代价缩放因子
  
  layers:
    - static_layer
    - inflation_layer
    - obstacle_layer          # 动态障碍
    
  obstacle_layer:
    observation_sources: "scan"
    scan:
      topic: "/scan"
      max_obstacle_height: 2.0
      clearing: true
      marking: true
```

#### 4.2.5 窄道通行方案

| 策略 | 实现方式 |
|-----|---------|
| 窄道检测 | Costmap Filter + Keepout Zone |
| 路径规划 | TEB + 最小障碍物距离约束 |
| 安全保障 | 轮廓精确碰撞检测 |

**窄道专用参数**:
```yaml
narrow_passage:
  enabled: true
  detection:
    min_width: 0.5            # 窄道最小宽度(机器人宽度+裕量)
    scan_range: 5.0           # 检测范围(m)
  safety_margin: 0.05          # 安全裕量(m)
  speed_reduction: 0.3         # 窄道内速度衰减因子
```

---

## 五、技术选型对比表

### 5.1 综合评估矩阵

| 方案 | 功能性 | 实时性 | 可靠性 | 易用性 | 扩展性 | 总分 |
|-----|-------|-------|-------|-------|-------|-----|
| **Nav2 + Smac + TEB** | ★★★★★ | ★★★★☆ | ★★★★★ | ★★★☆☆ | ★★★★★ | **22** |
| Nav2 + NavFn + DWB | ★★★★☆ | ★★★★★ | ★★★★☆ | ★★★★☆ | ★★★★☆ | 20 |
| 自研 A* + MPC | ★★★★★ | ★★★☆☆ | ★★★☆☆ | ★★☆☆☆ | ★★★★☆ | 17 |
| MoveBase (ROS1) | ★★★☆☆ | ★★★★☆ | ★★★★☆ | ★★★★★ | ★★☆☆☆ | 17 |

### 5.2 方案选择依据

**推荐: ROS2 Nav2 + Smac Hybrid A* + TEB**

| 选择理由 | 说明 |
|---------|------|
| 功能完整 | 全面覆盖导航、避障、窄道需求 |
| 架构成熟 | Nav2作为ROS2官方导航栈，社区活跃 |
| 差速轮适配 | Smac/TEB均原生支持差速轮模型 |
| 扩展性强 | 插件系统支持自定义扩展 |
| 窄道优势 | TEB在窄道场景表现优异 |
| 文档完善 | 大量教程和社区支持 |

---

## 六、风险与挑战

### 6.1 技术风险

| 风险项 | 影响 | 应对策略 |
|-------|-----|---------|
| TEB参数调优复杂 | 可能影响轨迹质量 | 提供多套预设参数，建立调优指南 |
| Nav2学习曲线陡峭 | 增加开发周期 | 提供详细注释和文档 |
| 窄道检测精度 | 可能导致碰撞 | 增加安全裕量，实时监控 |

### 6.2 性能挑战

| 挑战 | 解决方案 |
|-----|---------|
| 实时性要求 | 使用行为树优化、并行计算 |
| 动态障碍响应 | 优化Costmap更新频率、TEB权重调整 |
| 窄道通过率 | 多参数配置、特殊场景处理逻辑 |

---

## 七、参考资源

### 7.1 核心论文

1. **TEB算法**:
   - Rösmann C, et al. "Efficient trajectory optimization using a sparse model." European Conference on Mobile Robots (ECMR), 2013.
   - Rösmann C, et al. "Planning of navigation for mobile robots in dynamic environments with moving obstacles." ICRA, 2017.

2. **Hybrid A***:
   - Dolgov D, et al. "Practical Search Techniques in Path Planning for Autonomous Driving." AAAI, 2008.
   - Cohen B, et al. "Smac Planner: A Hybrid A* Planner for ROS2." ICRA, 2022.

3. **Pure Pursuit**:
   - Coulter R.C. "Implementation of the Pure Pursuit Path Tracking Algorithm." Carnegie Mellon University, 1992.

4. **Dynamic Window Approach**:
   - Fox D, et al. "The Dynamic Window Approach to Collision Avoidance." IEEE Robotics & Automation Magazine, 1997.

### 7.2 开源项目

| 项目 | 链接 | 说明 |
|-----|------|------|
| Nav2 | https://github.com/ros-planning/navigation2 | ROS2官方导航栈 |
| TEB Planner | https://github.com/rst-tu-dortmund/teb_local_planner | TEB局部规划器 |
| teb_local_planner_ros2 | https://github.com/rst-tu-dortmund/teb_local_planner_ros2 | ROS2版本TEB |

### 7.3 技术文档

- Nav2官方文档: https://navigation.ros.org/
- ROS2 Humble文档: https://docs.ros.org/en/humble/
- TEB参数调优指南: https://wiki.ros.org/teb_local_planner

---

## 八、下一步行动建议

### 8.1 给架构师的建议

1. **架构设计**:
   - 采用Nav2标准架构
   - 模块化封装规划器、控制器
   - 预留自定义插件接口

2. **关键组件**:
   - 全局规划: `nav2_smac_planner`
   - 局部规划: `teb_local_planner`
   - Costmap: 标准Inflation + Obstacle Layer

3. **配置策略**:
   - 提供多套参数预设（开阔/窄道/动态场景）
   - 参数热加载支持

### 8.2 技术验证建议

1. **仿真验证**:
   - Gazebo仿真环境搭建
   - 多场景测试用例设计

2. **性能指标**:
   - 到达精度: < 0.05m
   - 规划频率: ≥ 10Hz
   - 避障成功率: > 95%
   - 窄道通过率: > 90%

---

**报告结束**

*本报告由算法研究员输出，供软件架构师进行架构设计参考。*