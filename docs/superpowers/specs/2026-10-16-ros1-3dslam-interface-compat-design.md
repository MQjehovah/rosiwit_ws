# rosiwit_slam_ros1 旧接口兼容层 — 设计文档

日期：2026-10-16
主题：在 `ros1_ws/src/rosiwit_slam_ros1` 内让新 SLAM 直接扮演旧的 `xz_3d_slam` 节点
状态：待评审

---

## 1. 意图与为什么

旧的 `xz_3d_slam`（源码 `E:\workspace_slam\3d_slam\src\xz_3d_slam`）已经被重写为
`rosiwit_slam`（算法核心 `src/rosiwit_slam`，ROS1 薄层 `ros1_ws/src/rosiwit_slam_ros1`）。

重写后的薄层对外只提供**新的一套**接口（`lio_odom` / `cloud_map` / `save_map` …），
而整机系统（导航、覆盖规划、感知、上位机、以及现场 `/opt/xzrobot/configs` 里的配置）
是照着旧 `xz_3d_slam` 的接口与配置写的。

本设计要做的事：**在不改动下游任何一方的前提下，让替换 SLAM 实现这件事在接口层不可见。**
具体做法是在 `rosiwit_slam_ros1` 的薄层里补齐旧 `xz_3d_slam` 的完整对外面：
话题、消息类型、服务、TF、参数、配置文件路径、告警码。

一句话：**新核心照跑，旧外壳照旧。**

---

## 2. 目标 / 非目标 / 反目标

### 目标

1. 替换后可执行文件同时暴露**两套接口**：新接口（现状）+ 旧 `xz_3d_slam` 接口（本次新增）。
2. 旧接口的**话题名、消息类型、服务名、服务类型、TF 边、参数名、配置文件路径**与 `xz_3d_slam` 逐个对齐。
3. 新核心不具备的能力，**保留通道与语义**（话题/服务照注册），实现可留空/降级，并打日志。
4. 现有新接口的使用者（`rosiwit_perception` 订阅 `/lio_odom`）不受影响。
5. 不引入 `livox_ros_driver2` / `livox_ros_driver` 依赖。

### 非目标

- 不修改算法核心 `src/rosiwit_slam`（本次改动只落在薄层）。
- 不追求算法指标（精度/鲁棒性）与旧实现一致，只追求**接口一致**。
- 不新增 3d_slam 头文件里声明但**从未注册**的接口（见 §5.9）。
- 不实现旧实现里本身就"空实现/死代码"的功能（`/global_localization`、`getMap`）。

### 反目标（即使技术跑通也算失败）

- 下游需要改代码、改 launch 的 remap、改 `/opt/xzrobot/configs` 才能跑。
- 为了"对上名"而把两套语义不同的发布挤到同一个话题上导致下游行为错乱（见 §5.3 的冲突处理）。
- 兼容层成为第二份算法实现（在薄层里重写建图/匹配逻辑）。

---

## 3. 约束

| 约束 | 来源 | 影响 |
|---|---|---|
| 不得依赖 `livox_ros_driver2` / `livox_ros_driver` | 用户明确要求 | 无法订阅 `livox_ros_driver2/CustomMsg`；`livox/lidar` 只能按 `sensor_msgs/PointCloud2` 处理（见 §6 偏差 D1） |
| 改动只落在 `ros1_ws/src/rosiwit_slam_ros1` | 用户明确要求 | 核心缺的 getter（如 fitness）本次不做，用降级值（见 §6 偏差 D3） |
| 同一个可执行同时暴露两套接口 | 用户决定（2026-10-16） | 需要解决 `/map`、`/odom`、`map→odom` 三处新旧冲突（见 §5.3、§5.6） |
| ROS1 = melodic / Ubuntu 18.04 | 环境实测 | C++17 已在用；`tf2_geometry_msgs` 可用 |
| `xzrobot_ros_dependency` 在 `ros1_ws/src` 内，可依赖 | 环境实测 | 所有旧 srv/msg 类型直接可用 |

---

## 4. 现状（已查证事实）

### 4.1 旧 `xz_3d_slam` 对外面

- 节点：`ros::init(..., "xz_3d_slam")`；可执行 `xz_3d_slam_node`；包 `xz_3d_slam`。
- 对外面**天然隔离**在三个文件：`xz_slam_ros1_interface.{h,cpp}`、`xz_slam_ros1_node.cpp`。
- 订阅 10 个、发布 14 个、注册 8 个服务、另有 3 个服务 client、1 个 ROS param、`map→odom` 动态 TF。
- 配置全部走 JSON：
  - `/machine/model`（ROS param）
  - `/opt/xzrobot/configs/sensor_config.json`（键 `type_3d`）
  - `<PROJECT_PATH>/conf/slam_conf.json`，再被 `/opt/xzrobot/configs/3d_slam/slam_conf.json` 深合并覆盖
  - `laser_params.<type>` 子树合并到根
  - `<PROJECT_PATH>/conf/warning_code.json`（无覆盖机制）
- `PROJECT_PATH` 编译期硬编码为 `/opt/ros/melodic/share/xz_3d_slam`。

### 4.2 新薄层现状

`ros1_ws/src/rosiwit_slam_ros1/`：`src/rosiwit_slam_ros1_node.cpp`（376 行，单类单文件）
公开 `lio_odom`/`odom`/`lio_path`/`body_cloud`/`world_cloud`/`cloud_map`/`grid_map`/`map`，
服务 `save_map`/`load_map`/`save_grid_map`/`set_slam_mode`（自定义 srv），
静态 `map→odom` + 动态 `world_frame→body_frame`。
核心由 `add_subdirectory(.../src/rosiwit_slam)` 内联构建（已实测路径可解析）。

### 4.3 系统真实消费者（关键）

全仓搜索旧接口专有字符串（`slam/info`、`slam_warning_code`、`global_grid_map`、
`curr_localization_pose`、`loop_constraint`、`points_2d`、`/slam/mode_switch`、
`/switch_map`、`/relocation`、`update_map_operation` …）**命中 0**。

真正被消费的只有：

| 消费者 | 依赖 |
|---|---|
| `rosiwit_navigation_ros1` | `/map`(OccupancyGrid)、`/scan`、`/odom`、TF `map→base_link` |
| `rosiwit_navigation_ros2` / `rosiwit_coverage_planner_ros2` | `/map`、`/odom`、`/scan` |
| `rosiwit_perception_ros2` | `/lio_odom`（新接口） |
| `docker/slam-mapping` | `/save_map` + `/save_grid_map`（新接口） |

含义：**兼容层的硬需求是 `/map` + `/odom` + TF 链**；旧接口的其余部分属于"现场/上位机/未来模块会用"的
覆盖性目标。这不改变本设计的逐项对齐要求，但决定了验证优先级（§7）。

### 4.4 新核心的能力缺口（决定哪些只能留空）

核心 `ISlamAlgorithm` 没有的能力：`points_2d` 2D 投影、回环 `MarkerArray`、
地图自更新写回（`start_task`/`finish_task`/扩图）、电梯/楼层联动、外部里程计标定
（`yaw_calib`/`x_calib`/`y_calib`）、车体框删点、z 范围裁剪按模式区分、模式 `FREE`。

核心有的相关能力（可用来填旧接口）：**原生读写旧 `xz_3d_slam` 的 `.xzmap` 地图格式**
（`io/xzmap_reader`、`io/xzmap_writer`，注释明确写"设备端 3D 点云地图格式"），
`setPipelineMode("idle"/"mapping"/"localization")`、
`loadMap(path)`、`saveMap(path)`、`saveGridMap(pgm,yaml,res)`、`getGlobalMap()`、
`getGridInfo()`、`setInitPose()`、`onOdom()` 运动先验、`SlamOutput.body_cloud/world_cloud`、
`MatchResult.fitness`（**未通过 ISlamAlgorithm 暴露**）。

---

## 5. 设计

### 5.1 文件布局、可执行与 launch 替换点

单个可执行，`ros1_ws/src/rosiwit_slam_ros1/src/` 下拆成三层，避免单文件继续膨胀：

```
ros1_ws/src/rosiwit_slam_ros1/
├── CMakeLists.txt                  [改] 拆 lib(rosiwit_slam_ros1_lib) + exe + gtest；
│                                        加 xzrobot_ros_dependency；装 conf/ 与 scripts/
├── package.xml                     [改] 加 <depend>xzrobot_ros_dependency</depend>、<test_depend>gtest</test_depend>
├── config/default.yaml             [不变]
├── launch/rosiwit_slam.launch      [不变]
├── launch/xz_3d_slam.launch        [新] 旧子系统 launch 的 drop-in（第 34 行换包/可执行）
├── launch/legacy_dev.launch        [新] 只起本节点(input_profile:=legacy)，供开发机单测
├── conf/slam_conf.json             [新] 旧 JSON 配置随包发布（默认值）
├── conf/warning_code.json          [新] 告警码表，从 3d_slam 原样搬
├── scripts/check_legacy_interface.sh [新] V2 一致性校验
├── scripts/legacy_interface.golden   [新] 期望的 topic/service/type 清单
├── srv/*.srv                       [不变]
├── test/                           [新] gtest：legacy_config / lost_monitor / legacy_convert / realtime_grid
└── src/
    ├── rosiwit_slam_ros1_node.cpp      [改] 只剩 main：装配 + spin
    ├── slam_engine.h/.cpp              [新] 核心驱动：配置、订阅缓冲、tryPopAndProcess 定时器、
    │                                        输出回调、地图/模式访问。ROS 依赖仅到 sensor 消息。
    ├── legacy_config.h/.cpp            [新] 旧 JSON 配置装载（纯逻辑，无 ROS；含地图路径契约）
    ├── legacy_convert.h/.cpp           [新] 旧消息构造（栅格行序/值域、SlamInfo、告警 JSON、点云）
    ├── lost_monitor.h/.cpp             [新] 定位丢失/恢复状态机（§5.11 B）
    ├── realtime_grid.h/.cpp            [新] 建图模式实时 /map（§5.11 A，独立 worker 线程）
    ├── rosiwit_interface.h/.cpp        [新] 现有新接口（lio_odom/… 8 发布 4 服务 2 TF）原样搬迁
    └── legacy_xz3dslam_interface.h/.cpp[新] 旧 xz_3d_slam 接口全套（含 §5.11 B 的丢失上报）
```

拆出 `legacy_config` / `legacy_convert` / `lost_monitor` 三个**纯逻辑**单元，是为了让
"旧默认值 / 深合并 / 地图路径契约 / 栅格行序 / 丢失状态机"这些最容易搞错、又最难在机器人上复现的部分
能用 gtest 在本机回归（见 `docs/superpowers/plans/2026-10-16-ros1-3dslam-interface-compat.md`）。

**为什么拆 `slam_engine`**：两套接口共享同一个核心实例与同一份位姿输出。若各建一个核心实例，
IMU/雷达会被喂两遍、地图会被建两份。`slam_engine` 是唯一的核心持有者，两套 interface 只读它的输出、
只把订阅回调转给它。

**launch 替换点（已查实：只有一处）**

旧包只有一个 launch：`E:\workspace_slam\3d_slam\src\xz_3d_slam\launch\xz_3d_slam.launch`。
它**不是单节点 launch，而是整个 SLAM 子系统的 launch**，共 8 个节点/包含：

| 行 | 内容 | 本设计是否涉及 |
|---|---|---|
| 11 | `param /machine/model = $(arg model)` | ✅ 兼容层读它 |
| 13 | `xzrobot_slam/abot_odom.py`（respawn） | ❌ 外部，保持不变 |
| 15-17 | `map_converter`，**remap `/scan_f_map` → `/scan_f_map_unused`** | ❌ 外部；顺带解释了 `scan_f_map` 为何是怪异的 `sensor_msgs/PointCloud` 类型 |
| 19 | `include robot_localization/launch/slip_sensitive.launch` | ❌ 外部；**它提供 `base_footprint→odom`，正是旧 `map→odom` 合成的输入**（§5.6 前提坐实） |
| 21 | `include whether_slip/...` | ❌ 外部 |
| 23-31 | `ar_track_alvar`（视觉标记） | ❌ 外部；与 `/get_charging_point` 同族 |
| 33 | `xz_3d_monitor` | ❌ 外部，保持不变 |
| **34** | `xz_3d_slam` ← **唯一要改的一行** | ✅ |

> **决定（M4，已定）**：只改第 34 行两个 token：
> ```xml
> <node name="xz_3d_slam" pkg="rosiwit_slam_ros1" type="rosiwit_slam_ros1_node" output="log" />
> ```
> 节点名 `xz_3d_slam` 保持不变，参数文件路径不变（§5.8）。
> 同时本包提供一份**同内容的 drop-in** `launch/xz_3d_slam.launch`（第 1-33 行原样复制、
> 第 34 行改掉），供"整机 bringup 把 `$(find xz_3d_slam)/launch/...` 指向本包"这种换法。
> 两条路等效，实现时都产出，由你现场挑。
>
> 你回"不清楚"没关系 —— 上表已经把答案从代码里读出来了，不需要你再确认。
> 另：第 33 行 `xz_3d_monitor` 与本文件里的 `ar_track_alvar` 一起启动，
> **这正是"外部服务（`/set_detect_mode`/`/get_charging_point`/`/switch_map_pro`）仍然存在"的证据**
> （D14 的假设因此成立，我按"还在"实现）。

**输入 profile**：一个可执行不能再订阅两份 IMU/雷达数据（会重复喂给核心），
所以**输入只有一套**，由参数选择解析方式：

| `~input_profile` | 用途 | 输入来源 |
|---|---|---|
| `rosiwit`（**默认**） | 现网新接口，行为不变 | `~config_file` 的 YAML（`imu_topic`/`lidar_topic`） |
| `legacy` | 替换旧节点，由 `launch/xz_3d_slam.launch` 显式传入 | `/machine/model` + `sensor_config.json` + `slam_conf.json` 的 `laser_sub_name`/`imu_sub_name` |

默认取 `rosiwit` 的原因：`launch/rosiwit_slam.launch` 标注为"不变"，
若默认是 `legacy` 它就会被悄悄改变行为（违反目标 4）。兼容场景由
`launch/xz_3d_slam.launch` 显式 `input_profile:=legacy` opt-in。（M12）

**输出则是两套同时都发**，符合"同时暴露两套接口"。

### 5.2 订阅（输入，全部走 `slam_engine`）

| 旧话题 | 类型 | 条件 | 实现 |
|---|---|---|---|
| `livox/imu` | `sensor_msgs/Imu` | `laser_type_ ∈ {mid360,new_mid360}` | 真实 → `core.onImu` |
| `livox/lidar` | `sensor_msgs/PointCloud2`（**类型偏差，见 D1**） | 同上 | 真实 → `core.onLidar` |
| `<imu_sub_name_>` | `sensor_msgs/Imu` | else 分支 | 真实 → `core.onImu` |
| `<laser_sub_name_>` | `sensor_msgs/PointCloud2` | else 分支 | 真实 → `core.onLidar` |
| `initialpose` | `PoseWithCovarianceStamped` | 始终 | 真实：缓存，定位模式下 `setInitPose`（z 强制 0，照旧） |
| `scan` | `sensor_msgs/LaserScan` | 始终 | **降级**：只收、只做频率统计与告警；2D 去畸变/2D 匹配留空 |
| `odom` | `nav_msgs/Odometry` | 始终 | 真实 → `core.onOdom`（运动先验） |
| `lift/state` | `std_msgs/String` | 始终 | **降级**：存字段，不影响行为 |
| `lift_type` | `std_msgs/String` | 始终 | **降级**：存 `area_type_`，打日志；核心无运行时切 2D 匹配的 API |

`laser_type_` 解析规则照抄 `loadConfigJson()`：`Livox_Mid360(s)→mid360/new_mid360`、
`WLR_722→vanjee/new_vanjee`、`Robosense→new_robosense`（后两者同时强制
`laser_sub_name_=rslidar_points`、`imu_sub_name_=rslidar_imu`），由 `/machine/model` 区分 50/GT。

### 5.3 发布（输出，两套并存）

旧接口 14 个话题全部注册。**三处新旧冲突必须显式解决**：

**冲突 1：`/map`**
旧 `xz_3d_slam` 的 `/map` = 建图模式下的**周期实时栅格**（roll后首行=最小 y，值域 0/100/**10**）。
新薄层的 `map` = 定位模式下的**一次性 latched 全局栅格**（首行=最大 y，值域 0/100/-1）。
两者话题名相同、语义与 latch 不同。

> **决定**：合并为**唯一一个 `/map` 发布器**，按模式分流：
> - MAPPING → 周期发实时栅格（旧语义，`rt_grid_map_publish_hz_`）——**必须带真实数据**
>   （2026-10-16 你已确认："建图时 /map 发布实时建图结果的栅格地图"）。实现路径见 §5.11 A，
>   做法是薄层复用核心的公开 `GridMapFactory` + `applyGridPostProcess`，**不需要改核心**。
> - LOCALIZATION → latched 发全局栅格一次（新语义，等于旧实现里 `global_grid_map` 的角色）
>
> 行序/值域按各自分支照旧（**不要统一**，否则下游渲染会上下颠倒）。
> 这是本设计里唯一一处"新旧语义按模式共存"的地方，需在实现里写清注释。

**冲突 2：`/odom`**
旧实现把 `/odom` 当**输入**（底盘里程计 → 运动先验）；新薄层把 `/odom` 当**输出**（`lio_odom` 的别名）。

> **决定**：`legacy` profile 下新接口的 `odom` 别名**默认关闭**（参数 `~publish_odom_alias`，默认 `false`），
> `lio_odom` 照常发。否则节点自己发 `/odom` 又订阅 `/odom`，形成自环。
> `rosiwit` profile 下 `publish_odom_alias` 默认 `true`（保持现网行为）。

**其余 13 个话题无冲突**，逐个按旧实现对齐（`frame_id="map"`、latch、发布条件、定时器频率）：

| 话题 | 类型 | 频率/条件 | frame | latch | 实现 |
|---|---|---|---|---|---|
| `global_map` | PointCloud2 | 定位:受 `debug_publish`、每进程一次；建图:同 | map | ✅ | 真实 `getGlobalMap()` |
| `global_grid_map` | OccupancyGrid | 同上，需 `set_grid_map_` | map | ✅ | 真实：加载的 pgm+yaml 或核心 grid |
| `curr_scan_point` | PointCloud2 | 10Hz | map | — | 真实：`world_cloud`（无则 body→map 变换） |
| `scan_f_map` | **`sensor_msgs/PointCloud`** | 10Hz | map | — | 真实：上面降采样后转老消息类型 |
| `curr_submap_point` | PointCloud2 | 10Hz + `debug_publish` | map | — | **空**：核心无子图接口（advertise 不发） |
| `or_curr_scan_point` | PointCloud2 | 10Hz，仅定位 | map | — | **降级**：发当前帧原始输入点云 |
| `curr_localization_pose` | PoseStamped | 100Hz | map | — | 真实：`SlamOutput.pose` |
| `mapping_path` | Path | 10Hz，仅建图 | map | — | 真实：累积位姿 |
| `loop_constraint` | MarkerArray | 10Hz，仅建图 | map | — | **空**：核心无回环可视化数据 |
| `slam/info` | `xzrobot_ros_dependency/SlamInfo` | 100Hz | 无 header | — | **部分真实**，字段映射见 §5.4 |
| `points_2d` | PointCloud2 | — | — | — | **空**：核心无 2D 投影；旧实现本身也没发过 |
| `slam_warning_code` | `std_msgs/String` | 1Hz | — | — | 真实：§5.7 |
| `origin_point` | PointCloud2 | 每帧雷达 | map | — | 真实：原始输入点云 |

`pub_time_gap` 硬编码 0.1s（`publish_hz` 无效）这一旧行为**照样保留**（§8 M2），否则下游按不同频率调参会对不上。

### 5.4 `/slam/info` 字段映射

`SlamInfo.msg`：`slam_state` / `location_status` / `update_request` / `matching_degree` /
`map_name` / `robot_pose` / `map_area`；常量 `MAPPING=0, LOCALIZATION=1, ERROR=2`、
`SUCCESS=0, RECOVERING=1, FAILED=2`。

| 字段 | 来源 | 备注 |
|---|---|---|
| `slam_state` | 模式：mapping→0，localization→1，无核心/错误→2；FREE 时期→1 | 照旧逻辑 |
| `location_status` | 重定位中→1；`core.getPoseCov()` 返回 false（即核心 `m_status==LOST`）→2；否则 0 | **真实**，取自核心真正的 `MonitorLevel`，见 §5.11 B |
| `update_request` | 恒 0 | 核心无；旧实现也恒 0 |
| `matching_degree` | **恒 0** | 核心 `fitness` 未经 `ISlamAlgorithm` 暴露；你在 2026-10-16 确认"现场没有用到"，故留 0（见 D3） |
| `map_name` | `curr_map_name_`（初值 `"NULL"`，`/switch_map` 时置 `request.map_name`） | 照旧 |
| `robot_pose` | 当前位姿 x/y/yaw | 照旧；NaN 归零 |
| `map_area` | 仅建图模式有效 | 用核心 grid 尺寸 × resolution² 估算；旧实现取自 `realtime_grid_map_area_` |

### 5.5 服务

**Server（8 个，全部注册，名与类型照旧）**

| 服务名 | 类型 | 实现 |
|---|---|---|
| `/slam/mode_switch` | `SlamModeSwitchService` | 真实：`mode==0→localization`、`mode==1→mapping`（**注意与 `SlamInfo` 常量相反**）；并照旧调用 `/set_detect_mode` client；非法值返回 false 且 `result` 留 0；**10s 限流**（距上次调用不足 10s 直接返回并报 `20003005`，旧代码 2121-2137 行） |
| `/switch_map` | `SwitchMap` | **部分**：三分支照旧 —— (a) `map_dir`/`map_name` 空 → `success=false` 直接返回；(b) `map_dir == curr_map_path_` 且已在定位 → 只重设初始位姿 + 转发 `/switch_map_pro`（1s 重试），`success` 取外部结果；(c) 换图 → 调 `/set_detect_mode`(mode=0, 0.5s 重试) 由外部结果决定 `success`，随后按 §5.10 A 算 `base` 并 `core.loadMap(base)` + 切 localization。**图加载失败时强制 `success=false` 并记 ERROR**（旧实现不检查加载结果，见 D13） |
| `/relocation` | `Relocation` | 真实：`setInitPose`（z 强制 0，`success` 恒 true） |
| `/stop_map_operation` | `StopMapOperation` | **部分**：`saveMap` + `saveGridMap` 到 `request.map_dir`；`charging_pile_json` 走 `/get_charging_point` client；`current_pose` 照填；`map_resolution`/`map_origin`/`map_negate`/`*_thresh`/`error` **照旧留空** |
| `/global_localization` | `std_srvs/Empty` | **空**：`return true`（旧实现本身就是空实现，完全一致） |
| `/finish_task` | `CommonBoolService` | **降级**：定位模式下返回 `status=true`，否则返回 false（旧实现透传 `localiza_ptr_->updateMap()`，核心无对应能力） |
| `/start_task` | `CommonBoolService` | **降级**：同上（旧实现透传 `setUpdateMapFlag()`） |
| `update_map_operation` | `UpdateMap` | **降级**：按 §5.10 契约尝试加载 `request.map_dir` 的地图 + 返回 `status=true`；扩图语义留空 |

**Client（3 个，照旧）**：`/set_detect_mode`(`CommonIntRequestService`)、
`/get_charging_point`(`TagDetect`)、`/switch_map_pro`(`SwitchMap`)。
调用时机与超时（1.0s / 0.5s + 100ms 重试）照抄。

**不注册**旧头文件里声明但从未 `advertiseService` 的成员：
`change_slam_model_server_`、`save_map_server_`、`change_pub_grid_map_`、
`switch_to_localization_`、`switch_to_mapping_`、`get_map_`、`lift_state_`，以及 client `set_map_`。
（对"对上"无影响，因为旧实现也没对外提供。）

### 5.6 TF

旧实现只发一条边：**`map → odom`，动态，100Hz**，算法为
`T_mo = T_mf * T_fo`，其中 `T_mf` 由 `curr_pose_` 构造（平移 `(x, y, -0.07)`，
旋转 `AngleAxis(-yaw, Z)`，`yaw = atan2(R(0,1), R(0,0))`），
`T_fo = lookupTransform("base_footprint", "odom")`（外部 `robot_localization`/`abot_odom` 提供）。
`lookupTransform` 失败则直接返回、不发 TF。

冲突：新薄层发**静态** `map→odom` + 动态 `world_frame→body_frame`（默认 `odom→base_link`）。

> **决定**：`legacy` profile 下
> - **停发**静态 `map→odom`（否则与动态边互撞）
> - **停发**动态 `world_frame→body_frame`（`odom→base_link` 由外部 `robot_localization` 提供，重复发会打架）
> - 只保留旧实现的动态 `map→odom`
>
> `rosiwit` profile 下保持现状。
> TF 树最终形状与旧实现一致：`map --(本节点)-- odom --(外部)-- base_footprint --(URDF)-- base_link`。
> 旧 `publishStaticTf()`（`base_link→scan`）**不实现**——旧实现的它从未被调用。

### 5.7 告警码

- 话题 `slam_warning_code`(`std_msgs/String`)，JSON 结构照抄：
  `{"faults":[{"type":"algorithm fault","module":"xz_3d_slam","code":"20003001","level":"WARNING","name":"...","content":"..."}]}`
- `module` 字符串固定 `xz_3d_slam`（下游可能按名字过滤）。
- `conf/warning_code.json` 从 3d_slam 原样搬入本包（**不放 `/opt`**，因为旧实现也是从包 share 读）。
- 实际会用到的码：`20003001`(lidar hz low)、`20003002`(imu hz low)、`20003005`(set mode frequent)、
  `20003500`(no lidar data)、`20003501`(no imu data)。
  `20003504`(pose lost) **本次不会触发**（核心无法对外表达丢失，见 D11）。
- **保留**旧行为的"启动/模式切换后前 100 个 tick 不发告警"（`warning_code_start_task_` 抑制窗口），
  否则下游告警风暴特征会变。

### 5.8 参数与配置

**ROS param**：只有 `/machine/model`（string），照旧全局读取。其余全部走 JSON。

**配置读取顺序**（照抄旧实现，逐字保持可覆盖关系；**每一步都 try/catch + 内置默认值，缺文件只 WARN 不抛异常**，见 M11）：

1. `/opt/xzrobot/configs/sensor_config.json` → `type_3d`；文件缺失时按 `Livox_Mid360` 处理（旧实现缺键时的默认）
2. `<slam_conf>` → 再被 `/opt/xzrobot/configs/3d_slam/slam_conf.json` 深合并覆盖
3. `laser_params.<base_type>`（`new_mid360→mid360` 等去前缀）合并到根
4. `<slam_conf>` 的定位：`legacy` profile 下按旧顺序取
   `PROJECT_PATH/conf/slam_conf.json`；本包内 `conf/slam_conf.json` 作为**兜底**（两者都用相对/绝对路径尝试，先存在的胜）
5. `conf/warning_code.json`（无覆盖机制）

**配置文件路径的落地方式**：不硬编码 `/opt/ros/melodic/share/xz_3d_slam`。
用一个参数 `~legacy_conf_dir`（默认：优先 `/opt/ros/melodic/share/xz_3d_slam/conf`，
不存在则用 `ros::package::getPath("rosiwit_slam_ros1") + "/conf"`）。
理由：本包不在 `xz_3d_slam` 名下，硬编码旧 share 路径会让"替换后立即崩"，
而配置**内容**与覆盖链保持完全一致。（§8 M1）

**读到的键与默认值**：照抄 §4/旧实现 `loadConfFromJsonFile()`，共 30 余项
（`slam_init_type`、`is_debug`、`debug_publish`、`roll_new/pitch_new/yaw_new/x_new/z_new`、
`imu_in_car_frame_yaw_livox/wj`、`laser_sub_name`、`imu_sub_name`、`publish_hz`、
`rt_grid_map_publish_hz`、`yaw_calib/x_calib/y_calib`、`use_tf`、`use_2d_undistort`、
`th_2d_range_min/max`、`use_delete_pts` + 6 个车体框、`use_3d_z_min/max`、
`mapping_z_min/max`、`loc_z_min/max`、`lidar_pts_monitor`、`lidar_pts_monitor_interval`、
`test_wait_time`、`print_xz_log_to_file`、`xz_log_path`）。
**默认值逐条照抄**，包括看起来不合理的（如 `yaw_new` 默认 90、`imu_in_car_frame_yaw_livox` 默认 65）。

`yaw_calib` 读入后乘 `M_PI/180` 转弧度——照旧。
`/initialpose` 与 `use_2d_undistort`/`th_2d_*`/`use_delete_pts`/车体框/z 范围这些**收下但不生效**
（核心不支持逐模式 z 裁剪与车体框删点），打一次启动日志说明"参数已读，当前核心未实现"。

### 5.9 落地的接口全集（验收基线）

以下清单就是"对上"的定义，将固化为测试的 golden 文件（§7）：

- **订阅 10**：见 §5.2（其中 `livox/lidar` 类型偏差 D1）
- **发布 14**：见 §5.3
- **服务 server 8**：见 §5.5
- **服务 client 3**：见 §5.5
- **TF 1 条**：`map→odom` 动态
- **参数 1 个**：`/machine/model`
- **节点名**：`xz_3d_slam`

### 5.10 地图路径契约与核心配置注入

这两件事不由话题/服务清单体现，但搞错会让"接口全对、功能全废"，所以必须显式定死。

**A. 地图路径契约**（已按 2026-10-16 的答复修正：`/switch_map` 由 App 控制节点在运行期调用，
`map_dir`=地图文件所在目录，`map_name`=地图名称，与 `is_debug` 无关）

已查证的三条事实：

1. **旧地图的目录布局**：目录内文件基名**固定为 `map`** ——
   旧代码 `curr_localization_map_ = request.map_dir.substr(0,32) + "map"` 把基名写死，
   即 `<map_dir>/map.pcd`、`<map_dir>/map.xzmap`、`<map_dir>/map.pgm`、`<map_dir>/map.yaml`
   （`map_updater.cpp` 也按 `path_name + "/" + map_name + ".xzmap"` 命名，落盘时 `map_name` 恒为 `"map"`）。
   **`map_name` 只是显示标签**（`curr_map_name_`），不参与文件路径 ——
   旧代码第 2546 行的 `request.map_dir + "/map.xzmap"` 同样印证基名是 `map`，与 `map_name` 无关。
   旧代码隐含要求 `map_dir` **以 `/` 结尾**，否则 `substr(0,32)+"map"` 会拼成 `...xxxmap`。

2. **核心原生兼容旧地图格式**：`include/io/xzmap_writer.h` / `xzmap_reader.h` 的注释明确写着
   "设备端 3D 点云地图格式 (xz_3d_slam 自定义 JSON, 单文件)"、"从设备端导出的 map.xzmap 载入关键帧"。
   **旧系统建好的 `map.xzmap` + `map.pcd` 可以直接被新核心定位使用** —— 这是本方案成立的关键前提，风险已消除。

3. **核心的两条路径拼接规则不同，必须分开看**：
   - `PcdMapManager::loadMap(name)` → `map_dir_ + "/" + name`；`map_dir_` 读 YAML 键
     `map_directory`（本包 `default.yaml` 里没有该键，为空时兜底成 `"."`）。
   - `LocalizationPipeline::setMap(p)` → 直接用 `p`，读 `p + ".xzmap"`（及 `p + ".scdb"`），
     **不拼 `map_directory`**。
   - `SlamPipeline::loadMap(path)` 把**同一个** `path` 喂给上面两者，成功后调用 `generateGridMap(0.05)`。

因此 `path` 必须是**绝对基名**（如 `/opt/xzrobot/maps/xxx/map`）才能让 `setMap` 找到图；
同时不能让 `map_dir_ + "/" + name` 破坏这个绝对路径。

> **决定（M8）**：注入 YAML 里把 `map_directory` 设为 `"/"`。
> 于是 `"/" + "/" + "/opt/xzrobot/maps/xxx/map"` = `"///opt/xzrobot/maps/xxx/map"`，
> 在 Linux 上多重前导斜杠等价于单个斜杠，解析为 `/opt/xzrobot/maps/xxx/map`。
> （这是为了让绝对路径穿过 `PcdMapManager` 的拼接；若日后给核心加一个"绝对路径直通"的
> `loadMap` 入口，本 hack 可去掉。见 §8 M8 备注。）
>
> 兼容层在收到 `/switch_map` 时按下面规则算出绝对基名：
>
> ```
> base = strip_trailing_slash(request.map_dir) + "/map"
> ```
>
> 然后：
> - `/switch_map` → `core.loadMap(base)`；并镜像旧逻辑"同一张图且已在定位 →
>   只重设初始位姿"分支（用 `base` 与 `curr_map_path_` 比较，对齐旧代码第 2162 行）
> - `/stop_map_operation` → `core.saveMap(base)` + `core.saveGridMap(base + ".pgm", base + ".yaml", res)`
> - `update_map_operation` → 旧代码用的是 `request.map_dir + "/map.xzmap"`（**不带尾斜杠**），
>   正好被 `strip_trailing_slash` 归一化，两种传法都能用
> - `curr_map_name_ = request.map_name`（纯标签，照旧）
>
> **与旧实现的一处有意差异**：旧代码 `substr(0,32)` 在 `map_dir` 长于 32 字符时会截断成错误路径。
> 兼容层改为"显式去尾斜杠 + 拼 `map`"：对短路径与旧实现结果**完全一致**，对长路径是修好了。
> 记为 D9（更正确，不是回归）。
>
> **副作用**：`map_directory="/"` 会让新接口的 `load_map`/`save_map` 服务（它们把 `req.path`
> 原样交给 `algo_raw_->loadMap`）也要求绝对路径。legacy profile 下这两个服务无人调用，
> 可接受；`rosiwit` profile 不注入该 override，行为不变。

**B. 核心配置注入（legacy 配置 → 核心 YAML）—— 2026-10-16 已按你的答复收紧**

核心的 IMU 归一化能力在 YAML 里：`acc_scale`（默认 1.0，注释原文"传感器若以 g 为单位
(如 Livox 内置 IMU), 设 9.81"）、`r_il`（9 元素旋转）、`t_il`（平移）。
旧实现是在 C++ 里做的：`rosIMUToxzIMU()` 对 `mid360`/`new_mid360` 把加速度 **×9.8**；
`processIMUBuffer()` 对 `new_*` 用 `lidar_base_rot_`（`roll_new`/`pitch_new`/`yaw_new`）
旋转 accel+gyro。

你的答复是**"按现在的来"**，因此：

> **决定（M13，已改）**：兼容层**不做**旧实现的 IMU 单位换算与坐标系旋转，
> **不**由 `roll_new`/`pitch_new`/`yaw_new`/`imu_in_car_frame_yaw_*` 推导 `r_il`/`t_il`/`acc_scale`。
> 这些 legacy 角**照旧读入但不生效**（归入 D7 的"已读未生效"清单）。
> 外参与单位一律沿用**当前新系统 `config/default.yaml` 的现值**。
>
> 唯一的 override 注入是把 `map_directory` 写成 `"/"`（§5.10 A 的需要），
> 其余键原样透传。
>
> **失败可诊断性（这很重要）**：启动时打印核心最终生效的 `acc_scale` / `r_il` / `t_il`
> 与配置来源路径。若现场 IMU 实际是 g 单位（Livox）而当前 YAML 没设 `acc_scale=9.81`，
> 症状就是"完全没有位姿"——有了这行日志，一眼可判，不必回来重读旧代码。
> 这是我认为你答"按现在的来"时最可能踩到的坑，所以我把日志写进设计而不是留给你猜。

**C. `map_directory` 与 `input_profile` 的次序**：注入 YAML 的生成必须在读取 legacy 配置之后、
`core.init()` 之前完成；`input_profile=rosiwit` 时不注入任何 override（保持现网行为）。

### 5.11 两个"以为要改核心、其实不用"的实现路径

这两项你在 2026-10-16 已确认**必须能工作**：建图时 `/map` 要带实时栅格；定位丢失要能报出来。
两者原先都被我判成"核心没有 → 只能留空 / 只能改核心"。查证后**两条都只靠公开头文件在薄层实现**。

**A. 建图模式的实时 `/map` —— 复用核心的栅格生成器**

核心的栅格能力其实有一半是公开的：

| 符号 | 位置 | 是否公开 |
|---|---|---|
| `IGridMapGenerator`（`build` / `buildFromObservations`） | `include/slam_core/i_grid_map_generator.h` | ✅ |
| `GridMapConfig` / `GridInfo` | 同上 / `include/slam_core/slam_types.h` | ✅ |
| `GridMapFactory::create(name)` | `include/algorithms/grid_map/grid_map_factory.h` | ✅ |
| `applyGridPostProcess(GridInfo&, const GridPostConfig&)` | `include/algorithms/grid_map/grid_map_postprocess.h` | ✅ |
| `ISlamAlgorithm::getGlobalMap(PointVec&)` | `i_slam_algorithm.h` | ✅ |
| `PcdMapManager::generateGridMap()` | — | ❌ 私有（只有 `saveGridMap` 能顺带刷新它） |

薄层因此可以这样算实时栅格（**内存内、不写盘、不改核心**）：

```
pts   ← core.getGlobalMap()                     // 累积世界点云
cloud ← 由 pts 组装 PointCloud
gen   ← GridMapFactory::create(<与核心相同的 grid_map_generator 名>)
gen->build(cloud, origin=Zero, cfg, info)
applyGridPostProcess(info, post_cfg)
      → 发布为 /map (OccupancyGrid, 建图语义: 首行=最小 y, 值域 0/100/10)
```

`cfg`/`post_cfg` 从**与核心相同的 YAML 键**读取（`grid_min_height`、`grid_max_height`、
`map_resolution`、`occupied_thresh`、`dilate_radius`、`point_spacing`、`grid_skeleton_width`、
`grid_line_min_len_m`、`grid_axis_snap` …），保证与 `grid_map`/`save_grid_map` 算出的结果一致。
（薄层镜像了这些键 → 若核心改默认值会漂移，记 D15。）

**必须放在独立工作线程**：全图重投影是 O(地图点数)，占用 ROS 回调线程会拖累 100 Hz 的
`curr_localization_pose` 与 `map→odom` TF。用一个 worker 线程按 `rt_grid_map_publish_hz_` 节流，
结果加锁交给发布定时器。

**备选（记录在案）**：`saveGridMap()` 写到 tmpfs 临时文件再 `getGridInfo()` 读回 —— 能拿到核心
内部完全一致的栅格（含私有后处理），代价是每周期一次多 MB 写。当薄层镜像配置出现漂移时切过去。

**B. 定位丢失上报 —— `getPoseCov()` 就是那个信号**

核心内部**有**真正的丢失判定：`LocalizationPipeline` 由 `MonitorLevel` 驱动 `m_status`
（`localization_pipeline.cpp:620-629`，`lost_fitness=0.5` + `patience` 连续判据），
`MonitorLevel::LOST → m_status = LOST`。问题只是 `ISlamAlgorithm` 没暴露 `getStatus()`。

但**已有的 `getPoseCov()` 恰好把这个状态漏了出来**：
`SlamPipeline::getPoseCov` → `LocalizationPipeline::getPoseWithCov` 在 `m_status == LOST`
时 `return false`（`localization_pipeline.cpp:641`）。

于是薄层无需改核心：

```
定时(如 10Hz) core.getPoseCov(pose, cov)
  定位模式 && 地图已加载 && 返回 false → location_status = 2  且报 20003504
  返回 true                          → location_status = 0
```

**两个必须处理的副作用**：
- `m_status == INIT`（刚加载地图、尚未定位上）时 `getPoseWithCov` 返回 **true** 并给出默认位姿。
  薄层要加一个"是否曾经 LOCALIZED 过"的 latch，避免把 INIT 误判成 SUCCESS。
- 核心把 `MonitorLevel::DEGRADED` 映射成 `LOCALIZED`（第 627 行），**区分不出 CREDIBLE 与 DEGRADED**。
  所以 `location_status` 的三态里，1（RECOVERING）只用于"重定位进行中"，不用于"降级"。记 D11。

---

## 6. 已知偏差（无法完全对齐的项）

全部为**明示偏差**，需在下游可见处记录：

| 编号 | 偏差 | 原因 | 缓解 |
|---|---|---|---|
| D1 | `livox/lidar` 的消息类型由 `livox_ros_driver2/CustomMsg` 改为 `sensor_msgs/PointCloud2` | 不允许依赖 livox 驱动 | 同名话题保留；驱动端设 `xfer_format=0` 输出 PointCloud2，或 remap 到本节点的 PointCloud2 输入。**需你在现场确认这一条可接受** |
| D2 | `curr_submap_point`、`loop_constraint`、`points_2d` 三个话题注册但不发（或发空） | 核心无对应输出 | 下游若有渲染需求，先表现为"话题在、无数据" |
| D3 | `matching_degree` 恒 0 | 核心 `fitness` 未经 `ISlamAlgorithm` 暴露 | 后续给核心加 getter 即可填实（不在本次范围） |
| D4 | `lift/state`、`lift_type` 收下不生效（无 2D 匹配切换、无楼层逻辑） | 核心无电梯/楼层能力 | 通道保留，日志说明 |
| D5 | `/start_task`、`/finish_task`、`update_map_operation` 为降级实现（无地图自更新写回） | 核心无此能力 | 返回成功语义，日志说明 |
| D6 | 无 `FREE` 模式；`/slam/mode_switch` 之外的 `slam_mode` 语义用 `idle` 代替 | 核心只有 `idle/mapping/localization` | `slam/info` 的 `slam_state` 照旧映射，不暴露 `idle` |
| D7 | `use_2d_undistort`/`th_2d_*`、`yaw_calib/x_calib/y_calib`、车体框删点、按模式 z 裁剪，以及 IMU 相关的 `roll_new`/`pitch_new`/`yaw_new`/`x_new`/`z_new`/`imu_in_car_frame_yaw_livox`/`_wj` **收下不生效** | 核心无对应能力/参数；且你在 2026-10-16 答"按现在的来"，明确不走旧实现的 IMU 单位换算与坐标系旋转 | 启动日志逐条列出"已读未生效"，并**打印核心最终生效的 `acc_scale`/`r_il`/`t_il`**（§5.10 B），便于排查"无位姿" |
| D8 | 旧 `doc/slam模块接口.md`、`doc/warning_code.md` 描述的接口（`changeSlamModel`、`/algorithm_fault`）与实际 ROS1 代码不一致 | 旧文档陈旧 | **以代码为准**，不以文档为准 |
| D9 | 旧实现 `map_dir.substr(0,32)+"map"` 的路径拼接改为"去尾斜杠 + 拼 `map`"；且 `map_directory` 被设为 `"/"` 以让绝对路径穿过核心的 `map_dir_ + "/" + name` | 旧 `substr(0,32)` 在 `map_dir` 长于 32 字符时生成错误路径；核心无"绝对路径直通"的地图加载入口 | 短路径下结果与旧实现完全一致；长路径下是修好的。`///` 依赖 Linux 折叠前导斜杠，已附注释说明 |
| D10 | 建图模式 `/map` 的栅格由**薄层**用核心公开的 `GridMapFactory` + `applyGridPostProcess` 重算，而非核心内部私有路径 | `PcdMapManager::generateGridMap()` 不公开，`ISlamAlgorithm` 上只有会产生磁盘写副作用的 `saveGridMap()` | 结果与核心自己的栅格一致（同一生成器+同一后处理+同一配置键）；若配置漂移则切 §5.11 A 的 tmpfs 备选方案。见 D15 |
| D11 | `location_status` 的 1（RECOVERING）只用于"重定位进行中"，**不用于** DEGRADED；且 INIT 阶段靠薄层 latch 排除 | 核心把 `MonitorLevel::DEGRADED` 映射成 `LOCALIZED`（`localization_pipeline.cpp:627`），对外无法区分 CREDIBLE/DEGRADED；INIT 时 `getPoseWithCov` 会返回 true | 三态语义与旧实现略有差别，但"丢失要能报出来"这一需求已满足（LOST → 2 + `20003504`），见 §5.11 B |
| D12 | 定位/建图模式下的栅格由核心从 PCD 重新生成（`generateGridMap`，height_projection），**不读**旧系统随地图存下的 `map.pgm`/`map.yaml` | 核心的 `loadMap` 走"PCD→重新投影"，没有"读旧 pgm/yaml"的入口 | 同一张地图，但 `/map`+`global_grid_map` 的分辨率/障碍判据会与旧系统不同（旧 pgm 是 `map_updater` 的多层融合栅格）。若必须逐像素一致，需在兼容层里自己解析 pgm+yaml 并直接发布（范围升级） |
| D13 | `/switch_map` 在地图加载失败时强制 `response.success=false`（旧实现无条件返回外部服务的结果，不检查加载是否成功） | 安全的单调收紧：假成功会让 App 以为定位已就绪，其实无图可用 | 只在失败路径上收紧，成功路径与旧实现一致；记 ERROR 日志 |
| D14 | `/set_detect_mode`、`/get_charging_point`、`/switch_map_pro` 三个 client 在**本仓库内不存在实现**（属旧 xzrobot 栈） | 本次只替换 SLAM，检测/地图管理模块未动 | 照旧调用；client 不可用时打 WARN 并按旧逻辑给出 `success=false`/空 `charging_pile_json`。**该假设已由旧 launch 文件证实**（§5.1 第 33 行 `xz_3d_monitor` 与 `ar_track_alvar` 同文件启动） |
| D15 | 建图实时栅格的 `GridMapConfig`/`GridPostConfig` 若干字段由薄层**镜像**自 YAML，而非从核心内部读取 | 核心没暴露"当前生效的栅格配置"的只读接口 | 逐字段对照同一批 YAML 键；核心若改默认值，薄层需同步（已在代码注释里标出耦合点） |
| D17 | 建图模式 `/map`（实时栅格）里**没有** `unknown=10` 这个取值，值域只有 `0`/`100`；且栅格范围是"已观测点的 bbox"，不含旧实现 `realtime_grid_map_area_` 那圈 unknown 外扩 | 薄层复用的核心 `height_projection` 生成器只有自由/占据两态，没有 unknown 概念 | 消费方（导航/App）读的是 `map_name`/位姿/障碍栅格，`10` 与 unknown 外扩只影响渲染观感；若下游确实需要 unknown，需在薄层把 bbox 外扩一圈回填 `10`（范围升级，届时再定） |
| D18 | `legacy` profile 下**定位模式**的 `/map` 由"发一次 + latch"改为"按 `rt_grid_map_publish_hz` 周期重发、不 latch" | 全进程只有一个 `/map` 发布器，而旧 MAPPING 语义（非 latch、周期）与新 LOCALIZATION 语义（latch、一次）不能共存于同一发布器 | 晚订阅者最多等一个周期（缺省 ≤1s）拿到数据；比 latch 更安全（不会拿到上一模式的残留栅格）。`rosiwit` profile 的 `/map` 行为完全不变（latch、2s 定时器发一次） |
| D19 | 为让核心在 melodic/bionic 下编过，核心有 **2 处向后兼容改动**（均不在薄层，**已获你批准**）：① `src/algorithms/fast_lio2/map_builder/ieskf.cpp` 用 C++17 编译期探测，有 `Sophus::SO3d::leftJacobian` 就走原实现，没有就走等价闭式解；② `src/algorithms/localization/localization_pipeline.cpp:302` 把 PCL 三参构造 `pcl::PointXYZINormal(x,y,z)` 改为默认构造 + 逐字段赋值 | 核心面向 ROS2 Humble/22.04 开发（`docker/slam-mapping/Dockerfile` = `ros:humble-ros-base`，`scripts/install_deps.sh` 全是 `ros-humble-*`），而 18.04 是 **PCL 1.8.1 + sophus 1.0.1 + Ceres 1.13**；`ros1_ws` 此前**从未编译过**（包内带 `COLCON_IGNORE`）。`make -k` 全量核对：核心 57 个编译单元只有这 2 个文件编不过 | **接口与算法语义零变化**。新版依赖仍走 Sophus 原函数（逐位不变）；闭式解 `J_l(φ)=I+(1-cosθ)/θ²φ^+(θ-sinθ)/θ³φ^²` 经有限差分独立验证（`JrInv` vs 右扰动 FD = 3.66e-08，`Jr·JrInv−I` = 7.29e-15，小角度级数分支 2.8e-14，3.7e-08 即 h=1e-7 FD 自身噪声底）；PCL 改动语义等价（默认构造已置零 `intensity`/`curvature`/`normal_*`、`data[3]=1`）。这是 ROS1 侧能落地的前提 |
| D20 | 薄层 `CMakeLists.txt` 在 `add_subdirectory(核心)` **之前**加一段 `Ceres::ceres` 垫片 | 核心写的是 `target_link_libraries(rosiwit_slam PUBLIC Ceres::ceres)`，而带命名空间的目标自 **Ceres 1.14** 才有；18.04 的 1.13 只导出无命名空间的导入目标 `ceres` 与变量 `CERES_INCLUDE_DIRS`/`CERES_LIBRARIES` | **纯薄层改动，核心一字未动。** 用 `NOT TARGET Ceres::ceres` 守卫 + `INTERFACE IMPORTED GLOBAL` 桥接；Ceres ≥1.14 时守卫为假 → 整段跳过，无副作用。（`ALIAS` 不可用：CMake 不允许对**非 GLOBAL** 的导入目标建 ALIAS。） |
| D21 | 主配置里 `laser_sub_name` / `imu_sub_name` **键缺失**时，我们保留 `sensor_config.json` 阶段推出的值（Robosense 会强制写 `rslidar_points`/`rslidar_imu`），而旧实现回落硬编码 `"rslidar_points"`/`"zyz_imu_data"` 把它盖掉 | 旧实现里主配置一定存在（文件缺失会直接抛异常崩掉），所以"键缺失"这条路在旧实现里几乎走不到；而在降级实现里它一定会走到。硬编码回落会丢掉 Robosense 的机型信息 | 只在主配置**缺这两个键**时才有差别。随包发行的 `conf/slam_conf.json` 两个键都在（`vanjee_points722` / `vanjee_lidar_imu_packets`），因此**实际部署行为与旧实现逐位一致**。用 conf 里的值优先，是"忠实"与"不丢信息"之间的取舍，取后者 |

---

## 7. 验证方案

三层，从便宜到贵：

**V1 编译验证**
```bash
cd ~/rosiwit_ws/ros1_ws && source /opt/ros/melodic/setup.bash
catkin_make
```
通过标准：零错误；`devel/lib/rosiwit_slam_ros1/rosiwit_slam_ros1_node` 存在。

**V2 接口一致性验证（本设计的核心验收）**
写一个 `scripts/check_legacy_interface.sh` + `test/legacy_interface.golden`：

1. 启动 `roslaunch rosiwit_slam_ros1 xz_3d_slam.launch`（无实车数据，只验证注册面）
2. 采集 `rostopic list -v`（含类型与发布/订阅者数）、`rosservice list` +
   `rosservice type <每个>`、`rosparam get /machine/model`、`rostopic echo -n1 /tf`
3. 与 golden 文件逐项 diff

通过标准：**10 个订阅、14 个发布、8 个服务（名+类型全对）、3 个 client、1 条 TF 边、1 个 param
全部命中，无多余的同名冲突话题**。
这是唯一能客观回答"全都对上了吗"的手段，因此列为必做。

**V3 行为冒烟（需要合成数据）**
喂一段合成数据（假 IMU 100 Hz + 假 PointCloud2 10 Hz 的 Python 发布器），确认：

1. **建图模式**：切到 mapping、喂几秒数据后 `/map` **周期出现非空栅格**
   （验证 §5.11 A 那条薄层重算路径真的通；检查 `info.width/height > 0` 且 `data` 含 0/100/10）
2. `curr_localization_pose` 有输出且 frame=`map`（定位模式）
3. `/slam/info` 的 `slam_state` 随 `/slam/mode_switch` 正确切换
4. `map→odom` TF 能查通（需同时伪造 `base_footprint→odom`）
5. `/slam_warning_code` 在断掉雷达/IMU 后能报出 `20003500`/`20003501`
6. **丢失上报**：定位模式下喂与地图不匹配的雷达数据，直到 `location_status==2` 且报 `20003504`
   （这条最难构造 —— 至少先验证薄层 latch 逻辑：INIT 阶段不得误报 0；作废数据时要能翻到 2）

通过标准：上述六项符合预期，且 ROS 日志无 crash/异常。**第 1、6 项是本次新补的能力，必须实测。**

**不可验证项（需现场）**：`map_converter`、`robot_localization`、`whether_slip`、
`xz_3d_monitor`、`abot_odom` 在本开发工作区**都不存在**，因此整机联调必须在目标机上做。

---

## 8. 我替你做的技术决定（请过目，可否决）

| 编号 | 决定 | 理由 |
|---|---|---|
| M1 | 配置文件目录用参数 `~legacy_conf_dir`，默认优先 `/opt/ros/melodic/share/xz_3d_slam/conf`，回落到本包 `conf/` | 不硬编码别人的 share 路径；配置内容与覆盖链完全照旧 |
| M2 | 保留旧实现的硬编码 0.1s 发布周期（`publish_hz` 无效）、告警码前 100 tick 抑制、`/slam/mode_switch` 的 10s 限流与告警码 `20003005` | 这些是下游可能已经习惯的时间特征；"对上"包含它们 |
| M3 | `/map` 用单一发布器按模式分流（§5.3 冲突 1），其余话题不加 remap | 避免两个发布器抢同名话题 |
| M4 | 改旧 launch **第 34 行**的 `pkg`/`type` 两个 token（节点名仍 `xz_3d_slam`），并另出一份同内容的 drop-in launch | 已查实旧 launch 是子系统 launch，SLAM 只占第 34 行；`pkg` 无论如何都得改（包名不同），故别名二进制省不了事。你回"不清楚"，我已从代码把答案定死。见 §5.1 |
| M5 | `legacy` profile 下关闭新接口的 `odom` 别名与全部新 TF 发布 | 防止自环与 TF 互撞 |
| M6 | 告警码表随包发布，不读 `/opt` | 旧实现也是读包 share，保持一致 |
| M7 | `points_2d` 照旧实现为"注册但不发" | 旧实现本身就是死话题，发空反而可能让下游误判 |
| M8 | 注入 YAML 里 `map_directory: "/"`；`/switch_map` 用 `base = strip_trailing_slash(map_dir) + "/map"` 算出绝对基名后 `core.loadMap(base)` | `LocalizationPipeline::setMap` 要求绝对路径且不拼 `map_directory`，而 `PcdMapManager` 会拼；`"/"` 让绝对路径穿过拼接。见 §5.10 A |
| M9 | `location_status` 的 2（LOST）由轮询 `core.getPoseCov()` 返回 false 判定，并报 `20003504`；**不改核心** | 这是核心内部真正的 `MonitorLevel` 唯一漏出来的信号（§5.11 B）。你已确认"丢失要能报出来" |
| M10 | 建图模式 `/map` 由薄层复用核心**公开**的 `GridMapFactory` + `applyGridPostProcess` 实时重算，跑在独立 worker 线程；**不改核心** | 你已确认"建图时 /map 发实时栅格"。核心私有的 `generateGridMap` 够不到，但生成器与后处理都在公开头文件里（§5.11 A） |
| M11 | 所有配置文件读取都 try/catch + 内置默认值，缺文件只 WARN 不抛异常 | 旧实现在 `/opt` 文件缺失时会 `json::parse` 抛异常直接退出；V2 验收要在没有 `/opt/xzrobot/configs` 的开发机上启动节点 |
| M12 | `~input_profile` **默认 `rosiwit`**；`launch/xz_3d_slam.launch` 显式传 `input_profile:=legacy` | 若默认 `legacy`，则标为"不变"的 `rosiwit_slam.launch` 会被悄悄改成读 `/opt` JSON + 订阅旧话题，违反目标 4。反过来显式 opt-in 则两边都名副其实 |
| M13 | **不做**旧实现的 IMU 单位换算与坐标系旋转；外参/单位沿用当前新系统 YAML 现值；唯一注入是把 `map_directory` 写成 `"/"` | 你答"按现在的来"。为让可能的"无位姿"可诊断，启动日志必须打印核心最终生效的 `acc_scale`/`r_il`/`t_il`。见 §5.10 B |

---

## 9. 已定与待确认（2026-10-16 第二轮更新）

本轮你的答复（2–8）已全部落进文档，只剩三件事需要你，**都不阻塞开工**。

### 已经定掉的（你答过的，不必再答）

| 你的答复 | 结论 | 落在哪 |
|---|---|---|
| 2. 建图时 `/map` 发实时栅格地图 | **必须实现**。薄层复用核心**公开**的 `GridMapFactory` + `applyGridPostProcess`，跑独立 worker 线程，**不改核心** | §5.11 A、M10、D10、D15 |
| 3. 丢失可以报出来 | **必须实现**。轮询 `core.getPoseCov()` 返回 false 即 LOST → `location_status=2` + `20003504`，**不改核心** | §5.11 B、M9、D11 |
| 4. `livox/lidar` 改 `PointCloud2` 可以 | 按此实现；现场需驱动端 `xfer_format=0`，或在 launch 里 remap | D1 |
| 5. IMU/外参"按现在的来" | 不移植旧实现的 ×9.8 与坐标系旋转；沿用当前 YAML 现值；启动打印生效的 `acc_scale`/`r_il`/`t_il` | §5.10 B、M13、D7 |
| 6. D2–D7 现场都没用到 | 三个空话题、`matching_degree`、电梯信号、地图自更新链路全部保持空/降级 | D2–D7 |
| 7. `pkg`/`type`"不清楚" | 已从旧 launch 代码读出答案：那是**子系统 launch**，只有**第 34 行**要改 | §5.1、M4 |
| 8. 部署形态"不清楚" | 两条路都产出，你现场挑（就地改第 34 行 / 换成我们的 drop-in） | §5.1 |

你**跳过了**上一轮的第 1 条（那三个外部服务是否还在）。我没有再追问，因为旧 launch 本身给了证据：
`xz_3d_monitor`（第 33 行）与 `ar_track_alvar`（第 23-31 行）与 SLAM 在同一个文件里启动，
说明那套外部服务仍在 → 按"还在"实现（D14），client 不可用时打 WARN 并按旧逻辑返回。

### 仍需你的（按重要性）

**A. 地图目录内文件基名的确认。**
我推导出是 `<map_dir>/map.{pcd,xzmap,pgm,yaml}`（旧代码把基名写死为 `map`，`map_name` 只做显示），
且 `map_dir` 以 `/` 结尾；已用 `strip_trailing_slash` 兼容两种传法。
**若 App 实际传的是别的形状（比如文件叫 `<map_name>.pcd`），请告诉我 —— M8 的拼路径规则要跟着改。**
这是本次唯一一处"我从代码推导出结论、但没有你的直接确认"的地方。

**B. 真机那两个配置文件的样本（脱敏即可）**：
`/opt/xzrobot/configs/sensor_config.json` 与 `/opt/xzrobot/configs/3d_slam/slam_conf.json`。
有它 §7 V3 能拿真参数冒烟；没有就只用合成数据，风险是 `laser_type_` 分支走错
（mid360 还是 vanjee/robosense 决定订阅哪个话题）。

**C. IMU 单位（我按"按现在的来"实现了，但请留意）**
若现场雷达是 Livox 且当前 YAML 没设 `acc_scale=9.81`，上机后症状是"完全没有位姿"。
我已把生效值打进启动日志 —— 这条不是要你回答，是要你**第一次上机时先看那一行**。
