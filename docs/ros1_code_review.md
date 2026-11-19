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
