# ROS1 接口层瘦身（S1/S2/S3，2026-10-27）

> 背景：ROS1 层 7038 行 / 33 文件，是核心（18756 行）的 37%，一个"薄接口包"不该这么重。
> 本次按三条做：S1 按职责拆文件、S2 去掉与老接口重复的输出、S3 删掉配置镜像。

## 结论先说

| 指标 | 改前 | 改后 |
|---|---|---|
| `slam_algorithm.cpp`（最大单文件） | **1314 行** | **322 行**（拆成 6 个文件，最大 417） |
| `outputs.cpp` | 440 行 | **287 行** |
| `realtime_grid.cpp` | ~200 行（含 26 处配置镜像） | **75 行** |
| ROS1 包总行数 | 7038 | 6993 |
| 发布者（新接口一侧） | 9 个话题 | **5 个**（去掉 4 个重复项） |
| 重复/死代码 | — | 删掉 ~350 行（配置镜像 90、快照机器 90、TF 抑制死代码 40、重复发布 60+） |

总量几乎没变——因为 S1 给每个新文件加了 30 行公共头。**真正的收益是内聚和"删掉的东西"**：
最大文件从 1314 → 417，重复输出与死代码消失。

## S1：`slam_algorithm.cpp` 按职责拆成 6 个文件

| 文件 | 行数 | 职责 |
|---|---|---|
| `engine/slam_algorithm.cpp` | 322 | 生命周期、三条线程（FE/tick/pub）、输出快照、初始位姿 |
| `engine/config_bridge.cpp` | 417 | legacy JSON/YAML 读取、override YAML 生成、外参推导、兼容诊断 |
| `engine/core_gateway.cpp` | 277 | 与核心的边界：持锁包装、lane 投递、只读访问 |
| `engine/mount_frames.cpp` | 180 | base_link ← imu/lidar 的 TF 解析与快照 |
| `engine/cloud_utils.cpp` | 179 | PointCloud2 → PCL、机身自过滤 |
| `engine/input_pipeline.cpp` | 172 | 入队、统计、IMU/LiDAR/odom/vo/absolute 处理 |

构造函数从 194 行降到 12 行，只保留"按顺序装配"：

```cpp
initCalibrationConfig();
bool vo = false, odom = false, abs = false;
loadInputConfig(vo, odom, abs);
prepareCoreConfig();
algo_ = SlamFactory::create(algo_name);
...
```

**纯搬移，行为不变**：250 个单测全绿，定位模式跑通。

## S2：去掉与老接口重复的输出

契约澄清：**老接口（xz_3d_slam 兼容层）就是机器实际在跑的接口**，新接口只是补充。
所以"语义重复"的新输出全部删除：

| 删除的话题 | 老接口里的对应项 | 说明 |
|---|---|---|
| `/odom` | 底盘 `/odom` | 本来就 `!suppress_conflicts` 才发，实际从不发 |
| `/map` | 兼容层 `/map`（1 Hz 实时栅格） | 重复发布，实际被抑制 |
| `/cloud_map` | `global_map`（latched 点云图） | 同一份全局点云 |
| `/grid_map` | `global_grid_map` | 同一份全局栅格 |

连带删掉：
* 为 `cloud_map`/`grid_map` 服务的"cpu lane 快照机器"（90 行）——这两个话题没了，它就没有消费者；
* `publishTransform()` + `static_tf_` 静态 TF 块 + `tf_broadcaster_`（40 行）——这些本来就在
  `suppress_conflicts` 保护下永不执行，而 TF 树由兼容层独占。

保留（不与老接口重复，且有人消费）：

| 话题 | 消费者 |
|---|---|
| `/lio_odom` | 导航（`odom_topic`） |
| `/world_cloud` | 导航（体素障碍层） |
| `/body_cloud`、`/lio_path`、`/slam_calibration` | 仓库内暂无消费者；不是老接口的重复项，先留（见"待确认"） |

## S3：删掉配置镜像（26 个字段）

`RealtimeGrid` 里曾把核心的 grid 配置**逐字段照抄**一遍（设计文档叫"镜像配置 D15"），
核心改默认值这边就得同步。检查后发现：**这份配置从来没被读过**——`setConfig()` 写入后，
`runOnce()` 只用 `m_grid_source_`（真正算图的是核心的 `generateGridMap`）。

于是整块删除：`loadGridConfigFromYaml()`（90 行、26 个 `gv("…")` 字段）+ `setConfig()` +
3 个成员。`assembly.cpp` 里只保留"本层自己的"三个参数（存图分辨率、实时图分辨率、平面约束开关）。

## 验证

* `catkin_make` 0 error；`catkin_test_results`：**250 tests, 0 failures**。
* 定位模式 3 目标：目标 1/2 SUCCEEDED（目标 3 在窄通道口偶发 `FAILED_SAFETY_STOP`，
  与本次改动无关——SLAM 侧只删了没人用的输出）。
* 节点发布话题表里 `/odom` `/map` `/cloud_map` `/grid_map` 已不再出现。
* 无 `process has died` / 段错误。

## 待确认（需要 App/现场信息）

1. `/body_cloud`、`/lio_path`、`/slam_calibration`：仓库内没有消费者。如果 App 也不用，
   可以再删 3 个发布者（约 60 行）。
2. `services.cpp`（538 行 / 17 个服务）：`/merge_map`、`/save_grid_map`、`/set_slam_mode`
   与老接口的 `/stop_map_operation`、`/switch_map`、`/slam_mode_switch` 功能重叠，
   需要确认 App 到底调哪一套。
3. `pose_publisher.cpp`（660 行）仍是最大单文件，全部是老接口位姿/TF/健康话题的逐字复刻。
   可以按话题族拆成 3 个文件（位姿 / TF / 健康），纯搬移、零行为风险。
