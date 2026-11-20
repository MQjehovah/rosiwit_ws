# rosiwit_slam_ros1 线程模型（2026-11-20 审计）

范围：`ros1_ws/src/rosiwit_slam_ros1` 的进程内并发。依据：源码 + 容器内 `rosnode info` 实测。

## 1. 线程清单

| # | 线程 | 谁创建 | 周期/触发 | 干什么 |
|---|---|---|---|---|
| T1 | `io_spinner` | `main.cpp` `AsyncSpinner(1)` | 事件 | 默认回调队列：IMU/雷达/scan/odom/lift 订阅、`/initialpose`、0.1 s 点云定时器、1 s 栅格定时器、1 s 告警定时器 |
| T2 | `srv_spinner` | `main.cpp` `AsyncSpinner(1, srv_queue_)` | 事件 | 9 个服务的回调（与 T1 隔离，见 `docs/ros1_code_review.md` P1-1） |
| T3 | `slam_tick` | `PeriodicThread` | 20 ms | `Ros1OutputsInterface::onTick()`：`/lio_odom`、`/world_cloud`、每 2 s 在线标定 |
| T4 | `slam_pub` | `PeriodicThread` | 20 ms | `publishCurrPose()`：`/curr_localization_pose`、`/slam/info`、`/slam_warning_code`、TF `map->odom`、planar 约束、`/map` 兜底重发 |
| T5 | `frontend` | `SlamAlgorithm` | 输入驱动 | 消费 IMU/激光队列，`processImu/processLidar`、`base_->tryPopAndProcess()`（持 `core_mu_`） |
| T6 | `disk_write` lane | 线程池 | 任务 | 存图、扩图、**切模式重建核心** |
| T7 | `disk_read` lane | 线程池 | 任务 | `/switch_map` 地图解析 + 装载 |
| T8 | `cpu` lane ×2 | 线程池 | 任务 | 实时栅格、质量体检、缩略图 |
| T9 | `feature` lane ×1 | 线程池 | 任务 | 描述子建库（fpfh/scan_context） |
| T10 | `rt_grid` | `RealtimeGrid` | 1 Hz | 调核心 `generateGridMap` 出 `/map` 数据 |
| T11 | `tf2 listener` | `tf2_ros::TransformListener` | 事件 | 收 `/tf`、`/tf_static`，维护 buffer |
| T12 | roscpp 内部 | roscpp | — | rosout 追加、XMLRPC、TCPROS 连接 |

`rosnode info` 实测（容器内 roscore + 本体节点、无厂商节点）：

```
Services:    /slam/mode_switch /switch_map /relocation /stop_map_operation
             /global_localization /slam/dump_recorder /finish_task /start_task
             /update_map_operation        (9 个，全在 srv_queue_)
Publications: 18 个（含 /map /slam/info /lio_odom /world_cloud /tf）
```

## 2. 锁与所有权

### 2.1 引擎侧（`SlamAlgorithm`）

```
锁序（唯一方向）: core_excl_mu_  ->  core_mu_
```

| 锁 | 保护对象 | 规则 |
|---|---|---|
| `core_mu_` (mutex) | 核心指针 + 输入热路径 | T5 每帧短暂持有；`resetCore()` 里改为**有界 2 s 获取**（P0-2） |
| `core_excl_mu_` (timed_mutex) | 结构操作：重建/装图/改模式/导出 | `lockExcl(..., 200 ms)`，超时即失败 |
| `out_mutex_` | 最新输出快照 | `latestOutput()` 持锁拷贝 |
| `mount_mu_` / `child_mu_` | TF 安装关系缓存 | `computeBaseFrames(blocking=false)` 有界 |
| `calib_mutex_` | 在线标定 | T3 每 2 s |

### 2.2 接口侧（`Ros1SlamInterface`）

| 锁 | 保护对象 | 写者 | 读者 |
|---|---|---|---|
| `pose_mu_` | `curr_pose_` | T4、T2（切图清零） | T4、T2（存图取快照） |
| `static_map_mu_` | `realtime_grid_map_` | T4（1 Hz 定时器）、T2 | T4、T3 |
| `map_name_mu_` | `curr_map_name_` | T2 | T4 |
| `map_path_mu_` | `curr_map_path_` | T2 | T2 |
| `path_mu_` | `mapping_path_msg_` | T4 | T1（1 Hz 整条重发） |
| `planar_mu_` | `planar_pts_*` / `planar_field_` / `planar_*` | T1（激光点集）、T4（匹配/纠偏） | 同上 |
| `raw_input_mtx_` | 最近一帧原始点云 | T5（钩子） | T1 |
| `warning_code_mtx_` | `pending_warning_codes_` | T5、T4、T2 | T1（1 s 定时器） |

**新规则（本次建立）：发布侧 latch 单一写者**

`last_*` 时间戳、`odom_mon_`、`core_ever_localized_`、`last_core_loc_status_`、
`debug_map_published_`、`mapping_path_msg_` 这些"发布循环在用的状态"**只由 `slam_pub`(T4) 写**。
服务线程（T2）和 lane（T6）需要复位时只置位掩码：

```cpp
requestPublishReset(kPubResetPoseLatch | kPubResetLocStatus | kPubResetOdomMon);
// T4 在 publishCurrPose() 开头 applyPublishResetIfRequested() 落地
```

代价：复位最迟晚一个 tick（20 ms），换来的是这些字段**不需要任何锁**。

### 2.3 锁序（接口侧，禁止反向）

```
planar_mu_  ->  static_map_mu_ / pose_mu_ / warning_code_mtx_ / 引擎 core_mu_
pose_mu_    ->  （不嵌套任何其它锁）
```

规则：**任何接口锁内都不允许等待 lane 任务或服务**。所有"等待"（`switch_map` 12 s、
`mode_switch` 8 s、`stop_map` 180 s）都在无锁状态下等 atomic/future。

## 3. 本次修掉的问题

| # | 问题 | 线程对 | 后果 | 修复 |
|---|---|---|---|---|
| T-1 | `planar_pts_base_`（vector）激光回调 `swap` 与 `planarConstraintStep` 同时读写 | T1 ↔ T4 | **未定义行为**，点集翻倍时可能读已释放内存 | 两边都持 `planar_mu_`；`planarConstraintStep` 整段持锁 |
| T-2 | `planar_field_`（vector）`publishZeroPoseInfo` 清空 vs T4 读 | T2 ↔ T4 | 同上（`planar_constraint` 打开时才触发，默认关闭 → 潜伏） | 改为 `kPubResetPlanar` 掩码，T4 自己清 |
| T-3 | `curr_pose_` 读侧（`publishLocationInfo` / 高频位姿 / `sendTransform`）不加锁 | T2 写 ↔ T4 读 | 撕裂位姿（x 新 y 旧），偶发跳变 | 读侧统一 `currentPoseSnapshot()` |
| T-4 | `odom_mon_` / `last_*` 时间戳由服务线程直接复位 | T2 ↔ T4 | 撕裂的 `ros::Time`、监视器状态错乱 | 单一写者掩码机制 |
| T-5 | `have_imu_/have_laser_` 输入钩子（T5）写、定时器（T1）读 | T5 ↔ T1 | 数据竞争（bool，实害小） | `std::atomic<bool>` |
| T-6 | `warning_code_start_task_` / `suppress_ticks_` 服务写、定时器读 | T2 ↔ T1 | 数据竞争 | `std::atomic` |
| T-7 | `debug_map_published_` 服务写、定时器读写 | T2 ↔ T1 | 数据竞争；现在服务只置掩码，实际单写者 | `std::atomic<bool>` |
| T-8 | `set_grid_map_` / `global_grid_map_msg_` 从来没有置真 | — | 死代码，误导"global_grid_map 有生产者" | 删除（话题仍注册） |
| T-9 | `timerPublishLocalizationPose` 未注册却调 `publishCurrPose` | — | 一旦有人注册就会变成第二个写者 | 删除 |

## 4. 生命周期 / 关停顺序

```
main:
  SlamRosNode node            // 构造顺序: pool -> algo -> iface -> subs
  AsyncSpinner srv/io         // 声明在 node 之后 => 先于 node 析构
  waitForShutdown -> stop()
  ~node
      ~subs
      ~SlamRos1Interface      // 摘钩子 -> 停 rt_grid worker
          ~Ros1SlamInterface  // 服务/发布者/定时器
      ~SlamAlgorithm          // shutdown(): input_.close -> join frontend -> tick -> pub
      ~ThreadPool             // lanes 最后停 (算法关停时可能还要投递)
```

要点：

- **必须先摘钩子再销毁发布者**（`facade.h` 注释）：T3/T4/T5 每拍都会回调本对象
- `RealtimeGrid::stop()` 在接口析构里同步 join
- lane 任务体捕获 `this`（`switch_map` / `mode_switch` / 存图），
  `Ros1SlamInterface` 析构会等 `map_load_inflight_` 归零；`mode_switch` 任务由 lane 串行执行，
  进程退出时由 ThreadPool 停等

## 5. 结构性风险（未改，已知并接受）

| 风险 | 说明 |
|---|---|
| `slam_mode_` 在 T2/T4 都写 | 已是 `std::atomic<SlamMode>`，但"谁说了算"依赖 `updateModeFromCore()` 每拍校正 |
| T8 `cpu` lane 有 2 条 | 实时栅格与质量体检若同时访问同一核心导出接口，靠引擎 `core_excl_mu_` 串行 |
| T3 每 2 s 的 `updateCalibration()` 在 tick 线程 | 万一标定实现变慢会拖慢 `/lio_odom`；当前实测无影响，建议后续加耗时日志 |
| `stop_map` 在服务线程等 180 s | 现在只阻塞 T2（独立队列），但同一时间其它服务会排队 |
| `ros::Time` 参与跨线程比较的地方仍有个别（如 `last_odom_t_`） | 已 atomic<double>，新增代码请沿用"存秒数不存 ros::Time" |

## 6. 验证

| 项 | 结果 |
|---|---|
| 编译（amd64 melodic，`xzrobot_ros_compile:1.0.3-ceres`） | ✅ `Built target rosiwit_slam_ros1_node` |
| 单元测试 | ✅ 15 套件 / 93 用例 / 0 失败（`run_tests exit=0`；`test_reloc_disambig` 已固定种子，8/8 稳定） |
| 启动冒烟（容器 roscore + 本体节点，`default.yaml`） | ✅ 节点存活、9 服务 + 18 话题注册、`/slam/info` 约 9 Hz |
| `/slam/mode_switch {mode:0}` 无 `/set_detect_mode` | ✅ `result: 0`（失败如实上报） |
| `/slam/mode_switch {mode:0}` 有假服务回 true | ✅ `result: 1` |
| `/slam/mode_switch {mode:1}` | ✅ `result: 1` |
| 连续切模式后进程 | ✅ ALIVE |

**未覆盖**：`planar_mu_` 这条路径需要真实地图 + 激光（`planar_constraint` 只在定位模式且有 `/scan` 时
才进入），容器里没有数据源 —— 只做了编译与静态审查验证。上机验证时要看
`[locstate] ... planar=1` 且 `/slam/info` 不掉频。

## 7. 观察（不是并发问题，但值得记）

- **全局重定位是随机的**：`FpfhRansacGlobalLocalizer` 每轮用时钟种子抽 85% 子集
  （`fpfh_ransac_global_localizer.cpp:144`），这正是"同图多候选"的来源，但也让
  `test_reloc_disambig` 偶发（并行跑测试时实测 1/6 概率只出 1 个候选 → 假失败）。
  已加 `GlobalLocalizeConfig::random_seed`（0 = 时钟，生产不变），测试固定种子后 8/8 稳定。
  **产品含义**：单帧重定位返回的候选数本来就会波动，App 侧不要假设"一次调用就能看到全部歧义"。
- `sendTransform()` 的 `lookupTransform("base_footprint", "odom")` 失败时用 `ROS_ERROR`，
  在 T4 的 50 Hz 下会刷屏（容器里没有底盘 TF 时实测每秒 50 条）。建议改
  `ROS_ERROR_THROTTLE(5.0, ...)`；不影响接口面。
- `T3` 每 2 s 的 `updateCalibration()` 跑在 tick 线程，若标定变慢会拖 `/lio_odom`；
  建议后续记录其耗时并在超过 5 ms 时告警。
- `stop_map` 在服务线程最多等 180 s（大图导出），只阻塞 T2；同一时刻其它服务排队。
  若 App 有并发调用需求，可给每个服务单独的队列 + 线程。
