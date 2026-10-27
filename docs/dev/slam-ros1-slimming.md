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
   与老接口的 `/stop_map_operation`、`/switch_map`、`/slam/mode_switch` 功能重叠。
   需要确认 App 到底调哪一套，再决定删哪套。
3. `config/config.cpp`（271 行，legacy JSON + 环境适配）与 `engine/config_bridge.cpp`
   的边界可以再讨论：前者"读 legacy 配置"，后者"把它变成核心配置"，目前是合理的两件事。


## 6. 配置：只剩一处，而且只吃 YAML（2026-10-28）

原来是**两套配置 + 大量兼容逻辑**：

```
/opt/xzrobot/configs/sensor_config.json  ─┐
conf/slam_conf.json (legacy, 40+ 字段)   ─┼─> Xz3dConfig (机型判断/旧键回落)  ─┐
核心 YAML                                 ─┘                                   ├─> 算法
                                          `SlamAlgorithm::Cfg` (YAML) ──────────┘
```

现在只有一份 YAML：

| 删除 | 内容 |
|---|---|
| `src/config/config.{h,cpp}`（271 行） | legacy JSON 读取、`deepMerge` 旧键回落、`isSkywalker50/GT` 机型判断、`laser_type` 推导 |
| `test/test_config.cpp`（40 个用例） | 测的是上面那条已删除的链路 |
| `sensor_config.json` / `slam_conf.json` 的读取 | 不再读；`/opt/...` 路径常量一并消失 |
| `laser_type → 输入话题` 的硬编码映射 | mid360→livox/*、vanjee→... 这类机型特判不再需要 |

改为从**同一份 YAML** 读（`SlamAlgorithm::Cfg`）：

```yaml
global:
  input:   { imu_topic, lidar_topic, use_odom, ... }
  is_debug: false
  debug_publish: false
  pipeline_mode: mapping        # 决定初始模式
frames:
  body_frame / world_frame
  use_tf: true                  # 原来在 legacy JSON
mapping:
  rt_grid_map_publish_hz: 1     # 原来在 legacy JSON
  publish_hz: 20                # 仅日志
  frontend.self_filter.*
```

仍然会**写**一份 override YAML 给核心（把 TF 推导出的 `r_il/t_il` 注入进去），但基底只认 YAML。

> **机器人迁移注意**：如果现场原来依赖 `laser_type=mid360` 自动选话题，现在必须在 YAML 里
> 显式写 `global.input.imu_topic / lidar_topic`。这是"只吃 YAML"的代价，上机前确认一次。

## 7. App 只调老接口 → 新接口只留导航要的两路（2026-10-28）

契约明确：**App 调的是老接口（xz_3d_slam 兼容层）**，新接口不需要再"对 App 友好"。

删除（没有任何调用方 / 与老接口重复）：

| 删除项 | 说明 |
|---|---|
| 5 个服务：`/save_map` `/load_map` `/merge_map` `/save_grid_map` `/set_slam_mode` | App 用的是 `/stop_map_operation` `/switch_map` `/slam/mode_switch`；全仓库无调用方 |
| `/body_cloud` | 无消费者；老接口已有 `curr_scan_point` |
| `/lio_path` | 无消费者；老接口已有 `mapping_path` |
| `/slam_calibration` | 无消费者；标定的**更新**保留（2 s 节奏），只去掉这条话题 |
| `PreparedPose::body_cloud` 的深拷贝 | 只为上面那条话题服务，一并去掉（少一次全点云拷贝） |

保留（本仓库导航节点消费）：

| 话题 | 消费者 |
|---|---|
| `/lio_odom` | `rosiwit_navigation_ros1`（`odom_topic`） |
| `/world_cloud` | `rosiwit_navigation_ros1`（体素障碍层） |

`outputs.cpp`：440 → **143 行**。现在它的职责一句话说完：
"把核心输出的位姿/点云变成导航要的两路话题"。

### 实测话题表（`rosnode info /rosiwit_slam`）

```
/curr_localization_pose  /curr_scan_point  /curr_submap_point  /global_grid_map
/global_map  /lio_odom  /localization/state  /loop_constraint  /map  /mapping_path
/or_curr_scan_point  /origin_point  /points_2d  /scan_f_map  /slam/info
/slam_warning_code  /tf  /world_cloud
```

## 8. 验证（本轮）

```
catkin_make (-j6): 0 error     # -j16 会 OOM: c++: internal compiler error: Killed
catkin_test_results: 210 tests, 0 failures
                     # 250 -> 210: 删掉的 40 个是 legacy 配置链的用例
rosnode info: /odom /cloud_map /grid_map /body_cloud /lio_path /slam_calibration 均已消失
导航: 目标 1/2 SUCCEEDED;  目标 3 仍在窄通道口 FAILED_SAFETY_STOP (已知问题, 见下)
无 process has died / 段错误
```

## 9. 仍然开着的问题

1. **窄通道执行**（nav 侧）：目标 3（返回 (-1.5,0)）在 (2.6~2.7, -1.4) 附近被 safety 判停，
   最小间隙 0.20~0.25 m。改前是稳定失败，nav 改单线程 spinner 后曾连续 3 次通过，
   现在约 1/3 通过 —— 属于**临界**行为，需要单独调（安全阈值 / 通过策略），与本次接口瘦身无关。
2. **`HEALTH_INIT` 仍按"可继续行驶"处理**（nav 契约问题，未做）。
3. 若现场 YAML 缺 `global.input.imu_topic / lidar_topic`，节点会订阅错误话题 —— 需要一次上机确认。
