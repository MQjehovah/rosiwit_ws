# 导航优化批次：fail-safe / 参数化 / 重规划 / 自恢复 / 覆盖（2026-10-23）

> 针对评审列表 2、3、5、6、7、8、9、10、11、12 的落地记录。
> 实车 `/opt/ros` 尚未安装本版本；以下均为源码 + 隔离环境验证。

## 已完成

### 2. 传感器 fail-safe

* `map_timeout`（默认 0，静态地图/latch 不按新鲜度判；只要求地图存在）
* `scan_timeout`（0.5s）、`cloud_timeout`（1.0s）、`require_scan`、`require_cloud`
* 控制循环每周期检查；超时：
  * action：`failAction(-6 / -2, STOPPED_SCAN / STOPPED_MAP)`
  * 原生 goal：停车并置 `goal_status`
* `applySafety` 原有的 scan stale 逻辑保留为第二层。

### 3. 状态机自恢复

当前代码已有反向脱困（`RecoveryStage::FOLLOW` + Hybrid A* 反向 escape）；
本轮没有重复实现，只确认与 `task_sm_` 的 `RECOVERY_STARTED/DONE` 接线、
恢复期间禁止周期重规划，并去掉重复参数。

### 5/6. 性能（部分）

* scan 回调不再每个 scan 都做完整 costmap 聚类：
  `costmap_update_rate` 默认 5Hz（原为每次 scan）。
* 覆盖模式 A* 与正常 planner 分离；正常导航的 Hybrid A* 不再被覆盖 approach 拖慢。
* **未完成**：全量线程模型（传感器回调独立 queue / costmap worker）和
  `CostmapBuilder`/`SafetyPipeline` 抽取，留下一轮做结构性重构。

### 8. 话题/接口参数化

新增参数并替换硬编码：

```text
map_topic / scan_topic / goal_topic / nav_stop_service
global_path_topic / local_path_topic / global_costmap_topic / local_costmap_topic
goal_status_topic / diagnostics_topic / tracked_obstacles_topic
```

已有 `cmd_vel_topic` / `odom_topic` / `voxel_cloud_topic` / `rviz_goal_topic` 保持一致。

### 9. DWA 回退

`controller_name=dwa` + `footprint_aware_control=true` 时自动回退为
**圆形膨胀 + 点模型 DWA**（A/B 已证明 DWA+footprint 会绕圈），并打 WARN。
DWA 仍可作为 MPPI 的低算力回退。

### 10. 原生 goal 末端航向

`/goal_pose` 与 RViz `/move_base_simple/goal` 统一走 `startNativeGoal()`：
都作为 `core::ActionTask::NAVIGATE`，到位后按目标朝向对齐；
不再出现“action 会转、`/goal_pose` 不转”的不一致。

### 11. 重规划滞回

* `replan_max_interval`（5s）周期兜底；
* `replan_min_interval`（1s）最小间隔；
* `replan_on_blocked`：路径采样遇到致命格才重规划；
* 路径仍然有效时不做 1Hz 无脑重规划。

### 12. 覆盖效率

* 覆盖行段从“机器人投影到远端”开始，避免每行先退回到行端；
* 覆盖 approach 使用独立 A*（正常 planner 是 Hybrid A* + footprint，
  会把区域边缘目标判成 `Goal in obstacle`）；
* approach 失败时直接沿行段走，由控制器/安全层兜底，不把整块标不可达；
* `coverage_complete_rate` 默认 0.99，剩余格子标 `BLOCKED` 并写入报告。

隔离闭环（2m×1m，sweep=0.5，0.35m/s）：**99.1% 覆盖，80s 完成**，
剩余 0.0175m² 标为不可达（`/clean_map` 80 + status JSON）。

## 验证

```text
覆盖栅格单测       9/9
核心全量          13/13
协议单测          17/17
movebase 接口冒烟 16/16（隔离 master，require_scan=false）
覆盖闭环          99.1% / 80s / 不可达已标注
实车包编译        通过
```

## 未完成（下一轮）

1. **定位健康接入**：`/slam/info` LOST/DEGRADED、位姿跳变检测（安全底线）。
2. **线程模型**：传感器回调独立 callback queue / costmap worker；
   `CostmapBuilder`、`SafetyPipeline` 抽成可单测模块。
3. **3D 障碍进安全层**：目前 SafetyMonitor 只吃 `/scan` 2D 点。
4. **App 覆盖接入**：`navi_cmd=20` 目前需客户端直接发；App 绘制区域尚未接。
5. 电梯/对桩/延边等旧功能。

## 配置

新增参数已写入 `config/default.yaml` 与 `config/movebase_compat.yaml`；
实车沿用 `movebase_compat.yaml`。正常导航控制器仍是
`mppi + footprint + max_accel_theta=0.4`，覆盖模式单独 `pure_pursuit`。
