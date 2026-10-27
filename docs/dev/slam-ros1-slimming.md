# ROS1 接口层：按"内聚"重组（2026-10-27/28）

> 原则（本轮确定）：**行数不是问题，一个文件只做一件事。**
> 允许单文件很大（`services.cpp` 538 行就是 17 个服务处理器，一个职责），
> 但不允许一个文件里混两件互不相干的事，也不允许靠"跨文件共享隐式状态"耦合。

## 1. 现在的分层：每个文件一句话

### engine/ —— 算法实例这一侧

| 文件 | 行数 | 这一件事 |
|---|---|---|
| `slam_algorithm.cpp` | 322 | 把一个 ROS-free 核心"跑起来"并管住它的三条线程 |
| `config_bridge.cpp` | 378 | ROS 参数 / legacy JSON / 核心 YAML → 注入给核心的 override YAML |
| `mount_frames.cpp` | 221 | 雷达/IMU/车体之间的外参（legacy r_il/t_il + base_link←imu/lidar 快照） |
| `cloud_utils.cpp` | 179 | 点云转换与机身自过滤（纯数据，无状态机） |
| `input_pipeline.cpp` | 172 | 传感器输入的串行处理链 + 每秒输入统计 |
| `core_gateway.cpp` | 277 | **与核心的唯一边界**：锁语义 + lane 投递 + 只读访问 |

### interface/ —— 兼容层这一侧

| 文件 | 行数 | 这一件事 |
|---|---|---|
| `services.cpp` | 538 | 17 个 ROS 服务的处理（大，但内聚） |
| `pose_publisher.cpp` | 503 | 兼容层的位姿 / 健康 / TF 发布 |
| `outputs.cpp` | 287 | 兼容层之外的补充输出（`/lio_odom`、`/world_cloud`） |
| `assembly.cpp` | 263 | 纯接线：构造 + 注册发布者/订阅者/服务/定时器 |
| `planar_constraint.cpp` | 191 | 2D 平面约束**算法**（只产出修正量） |
| `cloud_publisher.cpp` | 187 | 点云 / 栅格输出 |
| `mode_state.cpp` | 141 | 接口层模式状态机 + 切模式重建核心 |
| `subscribers.cpp` | 122 | 兼容层自己订阅的输入回调（底盘/升降/原始点云） |
| `realtime_grid.cpp` | 75 | 建图模式实时栅格的工作线程（单一定周期任务） |

其余（`config/config.cpp`、`io/*`、`monitor/*`、`node/*`）本来就是单一职责，未动。

## 2. 本轮按"内聚"做的四处修正

拆大文件只是表象，真正判定标准是"这个文件能不能用一句话说清、改它的时候会不会牵动别的事"。

### 2.1 `computeXz3dExtrinsic` 归位到 `mount_frames.cpp`

上一轮拆分时我把它放在了 `config_bridge.cpp`，但它是**从 TF 推外参**，
属于"安装关系"而不是"配置装配"。`config_bridge` 只保留"调用它并把结果写进 YAML"。

### 2.2 `planarConstraintStep`（148 行）独立成 `planar_constraint.cpp`

它原来住在 `pose_publisher.cpp` 里 —— 但它是**算法**（栅格相关求修正量），
不是发布。放在发布文件里的后果是：调算法会牵动发布链路的编译和心智负担。
现在发布文件只剩"这一帧在哪里、可不可信"，算法改动不影响发布。

### 2.3 `assembly.cpp` 拆出 `mode_state.cpp`

`assembly` 原本混了两件事：启动时跑一次的**接线**（initPub/initSub/initServer…）
和运行期随时调用的**模式状态机**（核心模式同步、切模式重建核心、定位 latch 复位、
地面高度）。后者是业务规则，改动频率与接线完全不同 → 独立成文件。

### 2.4 `cloud_publisher.cpp` 拆出 `subscribers.cpp`

原来是"输入回调（底盘里程计/升降/原始点云）+ 输出发布"混在一起。
输入侧改动不该影响发布 → `subscribers.cpp` 只管回调，`cloud_publisher.cpp` 只管输出。

### 每个文件头部都写明"不在这里做什么"

例如 `slam_algorithm.cpp` 的头注释：

```
// 不在这里做的事: 配置装配 (config_bridge)、外参解析 (mount_frames)、
// 点云转换 (cloud_utils)、与核心的持锁边界 (core_gateway)。
```

这是防止后续改动"顺手放进来"的最低成本手段。

## 3. 顺带删掉的重复与死代码（上一轮）

* **重复输出**（老接口已有）：`/odom`、`/map`、`/cloud_map`、`/grid_map` 删除，
  连带删掉为它们服务的 cpu-lane 快照机器、`publishTransform` + 静态 TF（一直在
  `suppress_conflicts` 保护下的死代码）。
* **配置镜像**：`RealtimeGrid` 逐字段照抄核心 grid 配置的 26 个字段（`setConfig()` 写入后
  从未被读）整块删除；`assembly.cpp` 只保留本层自己的三个参数。

## 4. 验证

```
catkin_make: 0 error
catkin_test_results: 250 tests, 0 failures
定位模式 3 目标: 3/3 SUCCEEDED (matching 0.934)
rosnode info /rosiwit_slam: /odom /cloud_map /grid_map 已消失; /map 只剩兼容层一份
无 process has died / 段错误
```

## 5. 还没做（需要契约信息）

1. `/body_cloud`、`/lio_path`、`/slam_calibration`：仓库内**无消费者**。不是老接口的
   重复项，先留；确认 App 也不用后可以再删 3 个发布者。
2. `services.cpp` 的 17 个服务里，`/merge_map`、`/save_grid_map`、`/set_slam_mode`
   与老接口的 `/stop_map_operation`、`/switch_map`、`/slam_mode_switch` 功能重叠。
   需要确认 App 到底调哪一套，再决定删哪套。
3. `config/config.cpp`（271 行，legacy JSON + 环境适配）与 `engine/config_bridge.cpp`
   的边界可以再讨论：前者"读 legacy 配置"，后者"把它变成核心配置"，目前是合理的两件事。
