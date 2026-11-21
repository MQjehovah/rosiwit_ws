# rosiwit_slam_ros1 线程模型（2026-11-20 审计）

范围：`ros1_ws/src/rosiwit_slam_ros1` 的进程内并发。依据：源码 + 容器内 `rosnode info` 实测。

## 1. 线程清单

| # | 线程 | 谁创建 | 周期/触发 | 干什么 |
|---|---|---|---|---|
| T1 | `io_spinner` | `main.cpp` `AsyncSpinner(1)` | 事件 + 定时器 | 所有订阅回调（只入队）；20 ms 心跳（位姿/TF/`/slam/info`/告警码）；0.1 s / 1 s 定时器**只做触发**（投池） |
| T2 | `srv_spinner` | `main.cpp` `AsyncSpinner(1, srv_queue_)` | 事件 | 9 个服务：**只投管理命令 + 有界等待**，不做实际工作 |
| T3 | `slam_control` | `ControlLoop`（`main` 线程即可承载） | 命令驱动 | 载图 / 切模式 / 存图 / 扩图，**严格按命令顺序执行** |
| T4 | `pool` ×3 | `ThreadPool` | 任务 | 点云抽稀+转换+发布、栅格渲染+发布、2D 匹配、在线标定 |
| T5 | `frontend` | `SlamAlgorithm` | 输入驱动 | 运动先验 + 匹配 + 建图前端（持 `core_mu_`） |
| T6 | 建图后端 / `rt_grid` | 核心 / `RealtimeGrid` | 各自周期 | 后端优化、实时栅格生成 |
| 池内长任务 | 搜索/建库 | `LocalizationPipeline` | 按需投递 | 全局重定位搜索、FPFH 建库（原来是核心自带裸线程） |
| T7 | `tf2 listener` | tf2 | 事件 | TF buffer |
| T8 | roscpp 内部 | roscpp | — | rosout、XMLRPC、TCPROS |

设计原则（与 `rosiwit_common/README.md` 一致）：

1. **io 线程只做"入队 + 小消息发布"**：任何 O(n) 的转换都在池里；
2. **管理操作串行靠线程，不靠锁**：`ControlLoop` 一条命令队列，载图与存图天然不会并发；
3. **线程池只装"业务相关性小或会丢帧也没关系"的活**：转换/渲染/匹配/标定；
4. **大消息在哪里算就在哪里发**：ROS1 的 `publish()` 在调用线程序列化，与其转手不如就地发完。

`rosnode info` 实测。

## 1. 线程清单

| # | 线程 | 谁创建 | 周期/触发 | 干什么 |
|---|---|---|---|---|
| T1 | `io_spinner` | `main.cpp` `AsyncSpinner(1)` | 事件 | 默认回调队列：IMU/雷达/scan/odom/lift 订阅、`/initialpose`；3 个定时器**只做触发**（投递任务，不转换） |
| T2 | `srv_spinner` | `main.cpp` `AsyncSpinner(1, srv_queue_)` | 事件 | 9 个服务的回调（与 T1 隔离，见 `docs/ros1_code_review.md` P1-1） |
| T3 | `slam_tick` | `PeriodicThread` | 20 ms | `/lio_odom` + **大消息排空**（每次最多一条：点云/轨迹/栅格/整图） |
| T4 | `slam_pub` | `PeriodicThread` | 20 ms | 只发**小消息**：`/curr_localization_pose`、`/slam/info`、`/slam_warning_code`、TF `map->odom`；并**投递** planar 任务 |
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

### 2.0 大消息：池任务里就地发布（本轮核心改动）

之前的划分并不自洽：0.1 s 定时器（**IO 线程**）上做点云抽稀 + 序列化（实测可到 30 MB/s），
1 s 定时器（IO）上复制 640 KB 栅格（还复制两次），`planarConstraintStep`（2D 匹配）跑在
**发布线程**上，`updateCalibration` 跑在 tick 线程上。

现在：

```
IO 定时器 ──触发──> pool: buildScanMessages / buildGridMessage / buildGlobalMapMessage
                          └── 构造完**直接 publish**（不再经过任何输出线程）
        └── 触发 ──> pool: planar_step / calib
```

| 规则 | 实现 |
|---|---|
| 转换/拷贝/匹配不碰 io 线程 | `publish_jobs.cpp` 的 `build*` + `schedulePlanarJob` + `scheduleCalibJob` |
| 大消息不经过额外线程 | 池任务构造完就地 `publish`（ROS1 序列化本来就在调用线程） |
| 不排队、不积压 | 每类任务一个 `busy` 标志：上一份还没算完就丢掉本帧 |
| 析构安全 | `AuxJobGuard`：投递计数 + `DoneFn` 归零；析构时关闸并等在途 ≤2 s |

**`/world_cloud` 已删除**：导航的 `voxelCallback` 本来就做 `map<-lidar` 的 TF，
改订原始雷达话题（`voxel_cloud_topic`）即可，SLAM 侧不再序列化整帧点云。
点云/轨迹类调试话题统一由现有的 `debug_publish` 开关门控。

### 2.0b 管理操作：控制线程 + 命令队列

`/switch_map` `/slam/mode_switch` `/stop_map_operation` `/update_map_operation` 全部改为：

```
srv 线程: control_.postAndWait(命令, timeout)   → 拿到结果或"已受理"
控制线程: 载图解析(25 s) / resetCore / 导出存图(180 s) —— 顺序执行
```

- 服务的返回语义没变：完成给真实结果，超时给 `result=1`（已受理）
- 大图解析 25 s 期间，io/心跳与定位线程完全不受影响（实测切图失败路径 `/slam/info` 仍 9 Hz）
- 不再需要 `disk_read`/`disk_write`/`feature`/`cpu` 这些 lane 名字与 generation 握手

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
| T-10 | **IO 线程上做点云抽稀 + 序列化**（`startPub`） | T1 | IMU 200 Hz 回调被挤掉 → 丢帧/位姿抖 | 转换搬到 `cpu` lane，输出线程只 publish |
| T-11 | **IO 线程上复制 640 KB 栅格 ×2**（`publishRealtimeGridMap`） | T1 | 同上 | 搬到 `cpu` lane（`buildGridMessage`） |
| T-12 | **发布线程上跑 2D 匹配**（`planarConstraintStep`） | T4 | 匹配期间位姿/TF 停发（sim 里是 enabled） | 搬到 `cpu` lane，带防重入 |
| T-13 | tick 线程上跑在线标定 | T3 | 标定变慢会拖 `/lio_odom` | 搬到 `cpu` lane |
| T-14 | `/world_cloud` 的 `toROSMsg` 在输出线程 | T3 | 大点云转换顶住 50 Hz 输出 | 搬到 `cpu` lane + `drainWorldCloud` |
| T-15 | 池任务捕获 `this`，析构时可能还在跑 | — | 退出时 use-after-free | `AuxJobGuard` + 析构等待（实测 SIGINT 干净退出） |

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

## 6. 验证（2026-11-21，最终形态）

| 项 | 结果 |
|---|---|
| 编译（amd64 melodic，`xzrobot_ros_compile:1.0.3-ceres`） | ✅ `Built target rosiwit_slam_ros1_node` |
| 单元测试 | ✅ 15 套件 / 93 用例 / 0 失败（含固定种子的 `test_reloc_disambig`） |
| 启动冒烟 | ✅ 9 服务、`/slam/info` ≈9 Hz |
| `mode_switch 1 → 0` | ✅ 两次 `result: 1` |
| `switch_map` 指向不存在的目录 | ✅ `success: False`，不挂死，切图期间心跳保持 9 Hz |
| SIGINT 退出 | ✅ `EXITED-CLEAN`（control 先关闸，再等池任务，最后析构发布者） |
| 核心重定位线程迁移 | ✅ 编译通过 + 全部单测通过；**池化路径需真机重定位验证**（容器无地图/激光） |

**未覆盖**：`planar_mu_` 与 `/map` 发布路径需要真实地图/激光数据，容器里只做了编译与静态验证。

## 7. 观察

- **全局重定位是随机的**：`FpfhRansacGlobalLocalizer` 每轮用时钟种子抽 85% 子集，
  这是"同图多候选"的来源，也导致测试偶发（已加 `GlobalLocalizeConfig::random_seed`，测试固定种子）。
- **核心的两条裸线程已迁入 pool**（2026-11-21）：`relocBuildWorker`（FPFH 建库 ≈45 s）与
  `globalRelocWorker`（全局搜索）原来是 `std::thread` 常驻，现改为
  `IExecutor::submit("pool", …)` + 在途标志（有在途不重复投），任务体改为
  `runRelocBuildOnce()` / `runGlobalRelocOnce()`。
  * 注入点：`SlamAlgorithm` 创建核心后调 `algo_->setExecutor(executor_)`，
    `SlamPipeline` 转发给 `ILocalization`，`LocalizationPipeline` 存指针
  * **未注入时保持原有线程**（ROS2 与核心单测路径不受影响）
  * 语义不变：建库期间切图会在完成后自动再投一次；全局搜索的 generation/冷却/消歧逻辑原样保留
- `sendTransform()` 的 TF 查找失败用 `ROS_ERROR`，20 ms 心跳下会刷屏；建议改 `THROTTLE`。

## 8. 线程数对照

| | 之前 | 现在 |
|---|---|---|
| ROS 执行线程 | 2（io + srv） | 2（io + srv） |
| 发布线程 | 2（slam_tick + slam_pub） | 0（心跳在 io，大消息在池） |
| 管理/lane | 4 条 lane（disk_read/disk_write/cpu×2/feature，其中 feature 空转） | 1 条控制线程 |
| 线程池 | 5（含空转的 feature） | 3 |
| 业务 | 1（frontend） | 1（frontend） |
| 合计（不含库内部） | 12 | **7** |

## 9. 性能审计（2026-11-21）

审计口径：**每秒无谓拷贝的字节数 / 热路径上的锁 / 热路径上的日志**。不猜，只列能定位到代码的。

### 9.1 已改

| # | 问题 | 之前 | 现在 |
|---|---|---|---|
| P1 | `/world_cloud` 每帧整幅点云序列化 | 10 Hz × 全帧（数 MB/s），导航再 stride 抽样扔掉 | **删除**，导航订原始雷达（`voxelCloudCallback` 自带 `map<-lidar` TF） |
| P2 | 静态栅格每次读取整份拷贝 | `staticMapSnapshot()` 返回**值**：640 KB/次，兜底重发 1 Hz + planar ≤4 Hz + 栅格任务内再拷一次 → 最多 ~3 MB/s 分配+memcpy | 改 `shared_ptr<const OccupancyGrid>`，**零拷贝** |
| P3 | `/origin_point` 在**持 `core_mu_` 的输入热路径**上做整帧拷贝 + 序列化 | 10 Hz × 几万点拷贝 + 序列化，期间 `applyInitPose`/`resetCore` 全被顶住 | 无人订阅直接返回（不拷贝）；有人订阅时序列化投 `pool`，最新优先 |
| P4 | TF 查找失败每秒刷 50 条 `ROS_ERROR` | TF 树不完整时 ~5% CPU（ARM 控制台带 flush） | `ROS_ERROR_THROTTLE(5.0)` |
| P5 | 建图模式 `/curr_scan_point`、`/or_curr_scan_point` 无条件发 | 10 Hz × 整帧 | 统一受 `debug_publish`（默认 false）门控 |

### 9.2 量化（估算，按 20 万点 / 1600 m 地图）

| 项 | 之前 | 现在 |
|---|---|---|
| 点云序列化（非导航） | ~30 MB/s | 0（默认关闭） |
| 栅格拷贝 | ~3 MB/s | 0 |
| 输入热路径上的阻塞（点云发布） | 每帧 3~10 ms | 0 |
| TF 报错 | 50 条/s | 0.2 条/s |
| 实测 `/slam/info` | 8.6~8.9 Hz | 9.0~9.1 Hz |

### 9.3 看过但**没改**的（有理由）

| 项 | 现状 | 为什么不动 |
|---|---|---|
| `/map` 1 Hz 重发（内容不变也发） | 640 KB/s 序列化 | 占 CPU < 0.1%，但 App/工具可能靠"周期性刷新"判断新鲜度；改了就变成行为变更，收益不值 |
| `localizationStatus()/pipelineMode()/getMatchingDegree()` 要拿 `core_mu_` | 心跳最多等一个帧的处理时间（几十 ms） | 换成无锁快照能进一步降抖动，但要改核心访问器；当前 10 Hz 心跳有余量，先记录不改 |
| `frontendLoop` 持 `core_mu_` 时 `tryPopAndProcess()` | 建图时一次输入可能处理多个后端条目 | 属于核心设计（输入与结构操作互斥）；要改得动 `SlamPipeline` 的职责划分 |
| `processLidar` 全程持 `core_mu_` | 一帧内 `ros2Pcl` + `onLidar` | 同上，且这是"帧串行"的语义保证 |
| `/lio_odom` 每帧取 `bodyFrame()/worldFrame()`（返回 `std::string`） | 100 次/秒小分配 | 量级可忽略，缓存反而增加不一致风险 |

### 9.4 真机上要验的三条

1. `top -H` 看 `pool` 的 3 条线程在**建图 + RViz 连上**时的 CPU 占比（预期转换任务被 `debug_publish` 关掉后很低）
2. 建图 10 分钟后 `/slam/info` 的抖动（p99 间隔），确认心跳没被帧处理拖住
3. 重定位（判丢 → 全局搜索）期间，`/lio_odom` 与 `map->odom` TF 是否仍然连续（搜索已在线程池，理论上不再占 frontend）
