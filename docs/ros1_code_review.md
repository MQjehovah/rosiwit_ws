# rosiwit_slam_ros1 代码审查（2026-11-20）

范围：`ros1_ws/src/rosiwit_slam_ros1`（接口层）+ 与真机行为相关的引擎交互。
依据：源码静态审查 + 设备 14004361 上的实测现象。

## P0 — 现场会直接踩到的

### 1. `/switch_map` 在地图加载完成前就回 `success=true`

`services.cpp:278-290`

```cpp
// 换图：阶段一同步完成（服务立即返回），阶段二在后台加载 MapData。
MapSwitchContext ctx = beginMapSwitch(xzMapBase(req.map_dir), req);
startMapLoadJob(ctx, req.initial_pose);
res.success = ctx.detect_ok || ctx.pgm_ok;   // ← 此时 setKfMap 还没跑
```

**实测证据**：设备上 `switch_map` 返回 `success: True`，日志几秒后才出现
`[LocPipeline] setMapData: 2063848 pts, 931 keyframes` / `background localization map ready`。
期间 App 若立刻给初始位姿，会被"图还未就绪"拒绝（我们复现过 seed 失败）。

**影响**：切图后"立即定位"必失败一次，App 需要重试或等待。
**建议**：`res.success` 只在 MapData 就绪后置真（带超时，如 30 s），或新增明确
"LOADING" 语义；同时 `/slam/info.location_status` 在加载期保持 `1(RECOVERING)`。

### 2. `/slam/mode_switch` 可能长时间阻塞（设备上把终端卡死）

`services.cpp:79-107` → `resetCoreAndCaches()` + `callSetDetectMode()`（最多 0.5 s 重试）
全在服务回调线程里同步执行；`resetCoreAndCaches` 内部若等待 lane/线程 join
（同类 API 如 `/stop_map_operation` 用的是 `runOnLaneAndWait(..., 180000ms)`，`services.cpp:324`），
服务线程会被长期占用。

**实测证据**：设备上调用 `/slam/mode_switch` 后**终端整个卡住**，`Ctrl+C` 无效，只能
`disconnect+reconnect`。

**建议**：模式切换改成"受理→后台执行→由 `/slam/info` 反映进度"，回调里立即返回；
给 `resetCoreAndCaches` 加超时与阶段日志。

### 3. `/map` 未 latched

`assembly.cpp:142`

```cpp
map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);   // 无 latching
```

有 1 Hz 兜底重发（`pose_publisher.cpp:263`），所以**大概率能用**；但 App/导航节点若在
两次重发之间启动，会短暂拿不到地图（"地图空白"投诉的经典来源）。

**建议**：`advertise<...>("/map", 1, true)`；`/slam/info`、`/slam_warning_code` 同理评估
（故障码如果不 latched，瞬时故障 App 会漏掉）。

## P1 — 健壮性

### 4. 服务回调内的阻塞式重试/等待

| 位置 | 内容 |
|---|---|
| `services.cpp:40/51/62` | `callTagDetect`(1.0 s)、`callSetDetectMode`(0.5 s)、`callSwitchMapPro`(1.0 s) 都是 `sleep(0.1)` 重试 |
| `services.cpp:324` | `/stop_map_operation` 在 lane 上等待最长 **180 s** |

这些如果与服务本身同属 MutuallyExclusive 回调组，会串行阻塞其它服务调用。
**建议**：把"等待"移到后台任务，服务回 `accepted=true`；或给每个服务独立回调组。

### 5. `publish_hz` 参数无效（读了但被硬编码覆盖）

`assembly.cpp:162-170`（日志也打印了 `publish_hz=20 无效-硬编码 0.1`）。
**建议**：要么真正生效，要么从配置里删掉，避免运维以为能调。

### 6. 跨线程共享的普通成员

- `curr_map_path_`、`loaded_map_mtime_`：服务线程写、tick 线程读（`publishLocationInfo` 里用 `mapName()`）
- 部分 `last_*` 状态是 `std::atomic`（好的），但字符串类无法 atomic

**建议**：`curr_map_path_` 用 `std::mutex` 保护或改成 `shared_ptr<const std::string>` 快照。

## P2 — 整洁性/可维护性

7. **两个类名并存**：头文件是 `class Ros1SlamInterface`，而 `outputs.cpp` 里写的是
   `RosiwitInterface::RosiwitInterface(...)`。说明有 `using` 别名，容易读错，建议统一。
8. **单元测试全部被跳过**：`test/` 有 9 个 gtest（gicp matcher guard、localization pipeline、
   lost monitor、pose fusion、realtime grid、reloc 消歧…），但构建一直用
   `-DCATKIN_ENABLE_TESTING=OFF`。建议在容器里跑一次 `catkin_make run_tests`。
9. `scripts/localization_kidnap_check.py` 已弃用（依赖已删的 `LocalizationState`），
   建议移入 `scripts/deprecated/` 或删除。
10. **参数陷阱**：`acc_scale=1.0`（万集 m/s²）与 GenZ 那次的 Livox `9.81` 不可混用；
    `gyro_scale=1.036` 尚未在真机验证。

## 已确认没问题的（本轮整改后）

- 与厂商接口面一致：8 个服务 + 5 个必需话题；`/initialpose` 由引擎订阅（`slam_ros_node.cpp:62`）
- 无自建消息；厂商包为设备原版（32 msg / 39 srv / 1 action）
- `rosiwit_slam` / `rosiwit_slam_ros1` / `rosiwit_navigation_ros1` 在 melodic 容器中编译通过
- arm64 deb 在设备上 `dpkg -i` 成功、`ldd` 零缺失

## 修复进度（2026-11-20）

| # | 项目 | 状态 |
|---|---|---|
| P0-1 | `/switch_map` 等地图就绪再返回 | ✅ 已改：`map_load_done_gen_` / `map_load_ok_`，最多等 12 s；加载成功返回真实结果，超时仍按"已受理"返回 true 并打 WARN。4 条退出路径 + DoneFn 兜底都会写 done |
| P0-3 | `/map`、`/slam_warning_code` 改 latched | ✅ 已改：`advertise(/map, 1, true)`、`advertise(slam_warning_code, 1, true)` |
| P0-2 | `/slam/mode_switch` 异步化 | ✅ 已改：服务只做校验/去重/投递，重建放到 `disk_write` lane（与存图串行），回调有界等待 8 s；`resetCore()` 里 `core_mu_` 改为**有界获取 2 s**（原来无超时，这是"卡死数分钟"的根因） |
| P2-8 | 单元测试 | ✅ 已跑：见下表 |
| P1-1 | 服务回调内的阻塞重试/等待 | ✅ 已改：服务改为**独立回调队列 + 独立 spinner 线程**，切图/切模式/存图的等待只阻塞服务线程（详见下节） |
| P1-2 | `publish_hz` 读了不生效 | ✅ 已删：配置项、解析、字段一起移除；`0.1 s` 固定（与厂商真机一致），日志不再谎报可调 |
| P1-3 | `curr_map_path_` / `loaded_map_mtime_` 跨线程 | ✅ 已改：`map_path_mu_` + `currentMapPath()/setCurrentMapPath()`；mtime 改 `std::atomic<double>` |
| P2-7 | 类名 `RosiwitInterface` 易混 | ✅ 已改名 `Ros1OutputsInterface`（与 `Ros1SlamInterface` 分清） |
| P2-9 | 弃用脚本 | ✅ `scripts/localization_kidnap_check.py` → `scripts/deprecated/` |

### P1-1 为什么必须拆 spinner（这是"切图期间点云不动"的根因）

`main.cpp` 原来是 `ros::spin()`，**单线程**消费全局回调队列：话题、定时器、服务全在一个线程里。
只要有一个服务回调阻塞，IMU/雷达回调、`0.1 s` 发布定时器、`1 s` 栅格定时器就一起停摆。

现在：

```
io_spinner  (1 线程, 默认队列):   话题 + 定时器, 保持原来的串行顺序
srv_spinner (1 线程, 服务队列):   9 个服务, 阻塞只影响服务自己
```

- 服务的 `NodeHandle` 是 `srv_nh_`，构造里 `srv_nh_.setCallbackQueue(&srv_queue_)` 后注册，
  所以 9 个服务全在独立队列上；话题/定时器仍在 `nh_` 的默认队列
- `SlamRosNode::serviceQueue()` / `SlamRos1Interface::serviceQueue()` 把它透出给 `main`
- 服务之间仍严格串行（一个线程），不改变原先"服务互斥"的语义

### 一个必须记住的坑：`runOnLaneWithDone` 的 DoneFn 参数不是"成功"

```cpp
executor_->submit(..., [done](JobId, bool ok) { done(ok); });   // ok = 任务被执行过
```

`ok=false` 只表示**被取消/被丢弃/抛异常**，`ok=true` 不代表业务成功。
`/slam/mode_switch` 第一版就用 `ok` 当结果，实测「`/set_detect_mode` 不存在」时仍回 `result: 1`。
现在改为在任务体里 `mode_switch_ok_.store(applyModeSwitch(target))`，DoneFn 只在 `!ran` 时置 false。
（`runOnLaneAndWait` 没这个问题，它额外看了 `body_ok`。）

### 冒烟验证（容器内 roscore + 真节点，无厂商节点）

| 用例 | 结果 |
|---|---|
| 节点启动、9 个服务 + 18 个话题注册 | ✅ `rosnode info` 全在 |
| `/slam/mode_switch {mode:0}`，无 `/set_detect_mode` | ✅ `result: 0`（服务回 error），日志 `mode 0 -> FAILED` |
| `/slam/mode_switch {mode:0}`，假服务回 true | ✅ `result: 1`，日志 `mode 0 -> ok` |
| `/slam/mode_switch {mode:1}` | ✅ `result: 1`，日志 `mode 1 -> ok` |
| 连续调用后进程 | ✅ ALIVE（不再卡死） |

### P0-2 异步化的具体约定

`/slam/mode_switch` 的应答语义（对 App 可见）：

| 情形 | `result` | `return` |
|---|---|---|
| 切换在 8 s 内完成且成功 | 1 | true |
| 切换在 8 s 内完成但失败（核心重建失败 / `/set_detect_mode` 失败） | 0 | false |
| 8 s 内没跑完（极慢） | 1（已受理） | true |
| 已有同目标切换在跑，本次被忽略 | 1 | true |
| 已有**不同**目标切换在跑 | 0 | true |
| 非法 mode | 0 | false |

- 内核重建只做一次（`mode_switch_pending_` 去重），避免 App 重试把刚建好的状态反复清空
- `resetCore()` 的 `core_excl_mu_` 200 ms、`core_mu_` 2 s 都是上限，拿不到就明确失败
- 进度只能看日志（`[ros1_slam] /slam/mode_switch: mode X -> ok/FAILED`），
  `/slam/info` 语义未改（避免 App 误判）

### 单元测试结果（amd64 melodic 容器，`catkin_make run_tests`）

15 个 gtest 套件、93 个用例，**0 失败 0 错误**：

| 套件 | 用例 | 套件 | 用例 |
|---|---|---|---|
| test_calibration_controller | 3 | test_map_store | 8 |
| test_channel | 6 | test_online_calibrator | 2 |
| test_convert | 15 | test_pcd_map_manager | 8 |
| test_ekf_prior | 6 | test_pose_fusion | 8 |
| test_executor | 8 | test_realtime_grid | 4 |
| test_gicp_matcher_guard | 3 | test_reloc_disambig | 6 |
| test_lost_monitor | 8 | test_snapshot | 2 |
| test_thread | 6 | | |

跑测试的三个前提（每次都容易忘）：

0. **用 `xzrobot_ros_compile:1.0.3-ceres` 这个镜像**（已内置 `libceres-dev` + Sophus），
   不用再每次 `apt-get install`（新建容器里 apt 会静默失败，编译到 `ceres/ceres.h` 才炸）
1. 容器里 `apt-get install -y libceres-dev`（基础镜像 `xzrobot_ros_compile:1.0.3` 默认没有）
2. `cp -r third_party/sophus /usr/local/include/`（镜像没有 Sophus）
3. 首次 configure 必须**干净**：如果先在没有 ceres 的环境 configure 过，
   `Ceres_DIR-NOTFOUND` 会被写进 `CMakeCache.txt`，之后装了 ceres 也不会重新找，
   表现为 `librosiwit_slam.so: undefined reference to ceres::...` → 删 build 目录重来
