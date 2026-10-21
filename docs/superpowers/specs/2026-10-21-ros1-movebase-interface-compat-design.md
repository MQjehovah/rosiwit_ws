# rosiwit_navigation_ros1 对齐 robot_movebase 接口（2026-10-21）

> 目标：让 `rosiwit_navigation_ros1` 能顶替 `robot_movebase` 的上层接口，
> 同时保持“接口层薄、算法层 ROS 无关”的分层。
> 依据：`E:\workspace_slam\robot_movebase`（下文路径均相对该包）
> `src/RobotMovebaseNode.cpp`、`action/NaviCmdAction.action`、`include/robot_movebase/RobotMovebaseNode.h`。

## 1. 旧包的真实接口面

### 1.1 action（唯一正式控制入口）

`src/RobotMovebaseNode.cpp:315` 创建，名字 `RobotNaviCmdAction`（相对名，根命名空间下即
`/RobotNaviCmdAction`），消息类型来自厂商包：

```text
xzrobot_ros_dependency/NaviCmdActionAction
```

Goal（**以现场安装的 xzrobot_ros_dependency 为准**，见 §5-11）：
`navi_cmd / robot_vel / path_index / path_segment_index / backing_dist /
record_path(Pose2D[]) / tracking_path(Pose2D[]) / file_path`

Feedback：`execution_percent / execution_index / execution_path_segment_index /
clean_area / navi_state / execution_path(Pose2D[]) / local_path(Pose2D[]) /
access_turnstile_id`

Result：`execution_percent / execution_index / execution_path_segment_index /
clean_area / execution_path(Pose2D[]) / tracking_path_result`

关键语义（`ExecuteCb()`，`src/RobotMovebaseNode.cpp:2244`）：

* **成功和失败都用 `setSucceeded`**，失败只体现在 `tracking_path_result`（负数码）。
* 进入 action 先发一帧零速度（同函数开头）。
* 取消由 `preemptCB()` 立即 `setPreempted()`（`:3778`）。
* 控制循环按 `g_control_freq`（配置 10Hz）跑，反馈同频发布。

### 1.2 话题（`Robot_Move_Base` 构造函数内）

| 话题 | 类型 | 旧包行为 | 本包 |
|---|---|---|---|
| `/global_path` | `nav_msgs/Path` | 规划/重规划后发布 | 保留（已有） |
| `/local_path` | `nav_msgs/Path` | 每周期发布 | 保留（已有） |
| `/movebase_cmd_vel` | `geometry_msgs/Twist` | **底盘真正消费的控制量** | 新增，与 `/cmd_vel` 同值 |
| `/cmd_vel` | `geometry_msgs/Twist` | 对桩/调试用 | 保留（已有） |
| `/local_map` | `nav_msgs/OccupancyGrid` | 局部障碍图 | 新增，等同本包局部 costmap |
| `/makeplan_map` | `nav_msgs/OccupancyGrid` | 规划用障碍图 | 新增，等同本包全局 costmap |
| `/robot_footprint` | `geometry_msgs/PolygonStamped` | 当前车体框 | 新增 |
| `/prediction_robot_footprint` | `geometry_msgs/PolygonStamped` | 预测车体框 | 新增 |
| `/prediction_path` | `nav_msgs/Path` | 预测路径 | 新增 |
| `/movebae_robot_path` | `nav_msgs/Path` | 已走过轨迹（**旧包拼写如此**） | 新增，保持拼写 |
| `/back_off_path` | `nav_msgs/Path` | 倒车/脱困路径 | 新增，仅倒车时发布 |
| `/docking_path` | `nav_msgs/Path` | 对桩路径 | 仅 advertise + 启动时发一帧空路径 |
| `/re_point` | `geometry_msgs/PointStamped` | 前视参考点 | 新增 |
| `/area_type` | `std_msgs/String` | 区域业务 | 仅 advertise（旧包也只在特定业务下发） |
| `/clean_map` | `nav_msgs/OccupancyGrid` | 清洁业务 | 仅 advertise |
| `/clean_area` | `std_msgs/Float64` | 清洁业务 | 仅 advertise |
| `/clean_part_status` | `std_msgs/String` | 清洁业务 | 仅 advertise |
| `/RobotNaviState` | `robot_movebase/NaviState` | **只 advertise、从不 publish**（全包 grep 无 publish 调用） | **不实现**，见 §5 |

订阅：

| 话题 | 类型 | 本包处理 |
|---|---|---|
| `<robot_odom>`（配置默认 `odom`） | `nav_msgs/Odometry` | 已有 `odom_topic`（默认 `/odom`） |
| `/cmd_vel` | `geometry_msgs/Twist` | **不订阅**：旧包用于打滑/怼桩判断，本包订阅会与自身发布回环 |
| `/driver/chassis/info` | `xzrobot_ros_dependency/ChassisInfo` | 不订阅：只服务对桩 |

服务：**旧包没有 service**（只有 action）。本包原有的 `/nav_stop`（`std_srvs/Trigger`）保留。

### 1.3 navi_cmd 字典（`RobotCmdDectorNode()`，`:9201`）

| cmd | 旧包含义 | 本包实现 | 说明 |
|---|---|---|---|
| 0 | 取消 | ✅ | 直接成功返回并停车 |
| 1 | 单点导航 | ✅ | `tracking_path[path_index]`；>1 点时取索引点；支持 `file_path` |
| 2 | 两点导航 | ✅ | `tracking_path[0]` 为起点、`[1]` 为目标；从显式起点规划 |
| 3 | 路径跟踪 | ✅ | `tracking_path` 稠密点或 `file_path` JSON，从 `path_index` 起跟 |
| 4 | 入梯 | ❌ 明确失败 | 需要旧框架的梯控/闸机模块 |
| 5 | 出梯 | ❌ 明确失败 | 同上 |
| 6 | 原地旋转 180° | ✅ | 相对当前航向 +π |
| 7 | 倒车 | ✅ | `backing_dist`（>0.1 生效，否则 0.3m），-0.1m/s |
| 8 | 沥水路径 | ❌ 明确失败 | 清洁业务 |
| 9–13 | 旧包内部扩展命令 | ❌ 明确失败 | 语义依赖旧状态机 |

不支持的命令回 `tracking_path_result = -4`（协议里没有“不支持”码，沿用“无法规划”），
并打 `ROS_WARN`。**不做静默成功**。

### 1.4 tracking_path_result

| 值 | 旧包含义 | 本包何时产生 |
|---|---|---|
| 1 | 成功 | 全部支持的命令完成 |
| -1 | 获取坐标异常 | TF 失败、点数不足 |
| -2 | 获取全局地图异常 | 未收到 `/map` |
| -3 | 获取局部地图异常 | 控制器/局部层缺失 |
| -4 | 无法规划 / 不支持 | 规划失败、navi_cmd 不支持 |
| -5 | 无法加载路径文件 | `file_path` 打不开/JSON 非法/无 `path` |
| -6 | 感知异常超时 | odom 超时 0.6s |
| -16 | 长时间不动 | 任务看门狗超时（默认 60s） |
| -7/-8/-15 | 对桩/闸控/禁行区 | 本包不会产生（无对应业务） |

## 2. 分层：接口层 / 算法层

```
ros1_ws/src/rosiwit_navigation_ros1/src/
├── rosiwit_navigation_ros1_node.cpp   ROS 节点: 订阅/发布/定时器/安全过滤
├── movebase_protocol.{h,cpp}          接口协议: action 报文 -> 内部命令 (纯 C++)
└── movebase_interface.{h,cpp}         接口适配: actionlib 服务端 + 旧话题 (ROS 侧)

src/rosiwit_navigation/               算法核心 (零 ROS 依赖, ROS1/ROS2 共享)
├── include/rosiwit_navigation/nav_core/action_task.hpp
└── src/nav_core/action_task.cpp       任务状态机: 进度/到位/末端航向/倒车/卡死
```

边界约定：

* **算法层不知道 robot_movebase**：`core::ActionTask` 的输入是
  `ActionTaskCommand{kind, final_yaw, back_distance}`，没有 navi_cmd、result code、
  Pose2D[] 这些旧协议概念。
* **接口层不做行为决策**：`movebase_interface` 只把 ROS 消息搬进搬出；
  `movebase_protocol` 只做字段解析和路径文件读取；
  “是否到位 / 什么时候转航向 / 什么时候判卡死”全在 `ActionTask`。
* 因此 `test/test_action_task.cpp` 在无 ROS 环境可跑，
  `test/test_movebase_protocol.cpp` 只依赖核心类型头文件。

## 3. 行为对齐细节

* **单点导航**（cmd 1）：规划 → 跟随 → 位置到位后若航向误差 > `movebase_goal_yaw_tolerance`
  则原地对齐目标航向，再回成功（对应旧包 `ControlSate_Rota`）。
* **两点导航**（cmd 2）：用 `tracking_path[0]` 作为规划起点。旧包不要求车在起点，
  本包同样只把起点交给规划器；不做起步原地旋转。
* **路径跟踪**（cmd 3）：不调用全局规划，直接把路径给控制器；关闭周期重规划，
  避免把给定路径改掉。
* **旋转/倒车**：`core::ActionTask` 给出角速度/线速度 primitive，接口层只做安全过滤与发布。
* **反馈**：`execution_percent` 0..1；`execution_index` 用控制器当前点；
  `execution_path` 为已走过轨迹（>0.1m 采样）；`local_path` 为剩余参考路径前 200 点；
  `navi_state=2` 当安全层发生减速/停止。
* **结果**：一律 `setSucceeded`，与旧包一致；取消走 `setPreempted`。

## 4. 参数

见 `config/movebase_compat.yaml`。与兼容层直接相关的：

| 参数 | 默认 | 说明 |
|---|---|---|
| `movebase_compat_enabled` | `true` | 关掉后不建 action/旧话题 |
| `cmd_vel_topic` | `/cmd_vel` | 原生命令话题 |
| `movebase_cmd_vel_topic` | `/movebase_cmd_vel` | 旧包底盘控制话题 |
| `movebase_robot_footprint` | **实车 footprint** | 车体框，供 `/robot_footprint` |
| `movebase_default_speed` | `max_velocity_x` | `robot_vel<=0` 时的速度 |
| `movebase_back_speed` | `0.1` | 倒车速度 |
| `movebase_rotate_speed` | `0.6` | 旋转角速度上限 |
| `movebase_stuck_timeout` | `60` | 卡死判定（旧包 `check_stop_time` 默认 300s，见 §5） |
| `robot_radius` | `0.39` | 硬禁区（= 实车 `g_min_safe`，非几何半径） |
| `inflation_radius` | `0.62` | 软代价上限（= 实车 `g_max_safe` 的等效膨胀，见 §9） |

启动：

```bash
roslaunch rosiwit_navigation_ros1 movebase_compat.launch
```

## 5. 有意偏差 / 不实现（上机前必读）

1. **`/RobotNaviState` 不发布**。旧节点只 advertise 从不 publish（`grep robot_state_pub`
   只有 advertise 一处），实现它需要依赖 `robot_movebase/NaviState` 消息包，
   与“替换旧包”冲突。若现场确有订阅者（实际收不到数据），需要单独把该 msg 复制成
   中性消息包再补。
2. **局部控制器默认 DWA，不是 TEB**。旧包用 TEB；核心的 `teb` 是轻量梯度优化实现。
   需要时可切 `controller_name: teb`，但不承诺轨迹与旧 TEB 一致。
3. **不实现 `/cmd_vel` 订阅（打滑/怼桩判断）**：会形成自回环；对桩业务本身也不在本包范围。
4. **不实现入梯/出梯/沥水/延边/禁行区/对桩**：没有对应算法与外部服务。
   对应 navi_cmd 明确回 -4。
5. **卡死超时默认 60s**，旧包 `check_stop_time` 默认 300s。调到 300 只需改参数；
   默认取 60 是为了避免“卡住 5 分钟才报错”。
6. **`/area_type`、`/clean_*` 只有接口没有数据**：旧包也只在清洁/区域业务下发布。
7. **`execution_percent` 在路径跟踪时来自控制器进度**，不是旧包的“路径索引/总点数”，
   数值范围一致（0..1），但拐弯/重规划时可能有小幅跳变。
8. **`tracking_path` 视为 map 系 Pose2D**（旧协议没有 frame 字段），与旧包一致。
9. **现场消息包没有 `set_navi_mode`**：`robot_movebase/action/NaviCmdAction.action`
   源码里这个字段比机器人上安装的 `xzrobot_ros_dependency` 新。本包按现场包编译，
   同时不复制“停障模式先等待再脱困”的行为（本包没有脱困）。
10. **`robot_vel` 只作为速度上限**，且会被 `max_velocity_x` 收敛；日志会打印收敛值。
11. **清洁/闸机字段回中性值**：`clean_area=0`、`access_turnstile_id` 为空串；
   本包没有清洁与闸机业务。

## 6. 验证

本机（无 ROS1）可跑：

```bash
# 协议解析
g++ -std=c++17 -I src/rosiwit_navigation/include \
    ros1_ws/src/rosiwit_navigation_ros1/test/test_movebase_protocol.cpp \
    ros1_ws/src/rosiwit_navigation_ros1/src/movebase_protocol.cpp \
    -lgtest -pthread -o /tmp/t && /tmp/t

# 任务状态机
g++ -std=c++17 -I src/rosiwit_navigation/include \
    src/rosiwit_navigation/test/test_action_task.cpp \
    src/rosiwit_navigation/src/nav_core/action_task.cpp \
    -lgtest -lgtest_main -pthread -o /tmp/t2 && /tmp/t2
```

机器人上：

```bash
roslaunch rosiwit_navigation_ros1 movebase_compat.launch
# 1) 接口存在性
rostopic list | grep -E 'movebase_cmd_vel|local_map|makeplan_map|robot_footprint|prediction_path|movebae_robot_path|back_off_path|docking_path|re_point'
rosnode info /robot_move_base | grep -A3 RobotNaviCmdAction

# 2) 单点导航 (target x/y/theta)
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped ...   # 原生入口
# 或 actionlib 客户端: navi_cmd=1, tracking_path=[{x,y,theta}]
# 期望: /movebase_cmd_vel 有速度, action feedback percent 递增, result=1

# 3) 路径跟踪: navi_cmd=3, tracking_path 给 >=2 点
# 4) 旋转: navi_cmd=6 -> result=1 且朝向转 ~π
# 5) 倒车: navi_cmd=7, backing_dist=0.3 -> 后退 ~0.3m, result=1
# 6) 不支持: navi_cmd=4 -> 立即 result=-4
```

接口一致性脚本（机器人上真跑时用）：

```bash
rosrun rosiwit_navigation_ros1 check_movebase_interface.sh
# 期望: "检查项 23 个, 不符 0 个" / exit=0

# 隔离冒烟: 私有 roscore + /navtest, 造 /map//odom/TF, 15 项断言
rosrun rosiwit_navigation_ros1 smoke_movebase.sh
```

## 7. 本次验证结果

* `test_movebase_protocol`：15/15 通过（本机 g++，无 ROS）。
* `test_action_task`：10/10 通过；核心全量 `ctest` **11/11** 通过。
* ROS1 侧（`movebase_interface` + 节点）**未在本机编译**：当前开发环境只有
  ROS2 Humble，且 `xzrobot_ros_dependency` 不在仓库内（厂商包，`.gitignore` 已排除）。
  需要在机器人/`ros1_ws` 环境执行：

  ```bash
  cd ros1_ws && catkin_make --pkg rosiwit_navigation_ros1
  catkin_make run_tests_rosiwit_navigation_ros1
  ```

  编译通过后按 §6 的 6 步真机验证；其中第 2–5 步是接口语义的核心验收。


## 8. 真机实测记录（2026-10-21，192.168.21.166）

环境：Ubuntu 18.04 aarch64 / ROS Melodic / CMake 3.25 / GCC 7.5；
`robot_move_base` 正在 11311 上运行（本次**未改动**在跑导航）。

### 8.1 现场消息包与源码不一致（已按现场修正）

`robot_movebase/action/NaviCmdAction.action` 比机器人上安装的
`/opt/ros/melodic/share/xzrobot_ros_dependency/action/NaviCmdAction.action` **新**：

| 字段 | 源码 .action | 现场安装包 | 处理 |
|---|---|---|---|
| Goal `set_navi_mode` | 有 | **无** | 不访问 |
| Goal `record_path` | 无 | 有 | 忽略 |
| Result/Feedback `clean_area` | 无（`#if CLEAN_AREA`） | 有 | 回 0 |
| Feedback `access_turnstile_id` | 无 | 有 | 回空串 |

结论：**以现场安装的消息包为准编译**，否则 `catkin_make` 直接报
`has no member named 'set_navi_mode'`。

### 8.2 Action 生命周期：SimpleActionServer 不可用

第一版用 `SimpleActionServer`，控制循环由定时器驱动、goal 回调立即返回，
actionlib 会打印：

```
Your executeCallback did not set the goal to a terminal status.
For now, the ActionServer will set this goal to aborted
```

即所有 goal 立即被 abort。已改为 `actionlib::ActionServer` + `ServerGoalHandle`：
goal/cancel 回调在 `ros::spin` 线程内，完成时由定时器调用 `setSucceeded/setCanceled`。

另一个坑：**cancel 请求到达时 actionlib 已把状态改成 `PREEMPTING`**，
此时 `GoalStatus == ACTIVE` 判断为假；必须对 `ACTIVE || PREEMPTING` 调
`setCanceled()`，否则客户端永远等不到 result（表现为 `wait_for_result` 超时）。

### 8.3 取消后被旧规划结果重新拉起（已修）

`stopNavigation()` 原先只停速度，不处理在途的 `std::async` 规划。
复现：navi_cmd=2 取消后约 1s，旧规划结果被 `applyPlanningResult()` 应用，
`is_navigating_=true`，车重新开始走。现在取消时同时
`planning_active_=false`、`plan_generation_++`、`async_planner_->cancel()`。

### 8.4 校验脚本修正

初版 golden 把 `/RobotNaviCmdAction/{goal,cancel,feedback,result,status}` 写在
`[services]`。actionlib 这 5 个是**话题**，不是服务；已修正为 22 话题 + 1 服务
（`/nav_stop`），共 23 项。

### 8.5 实测结果

| 项 | 结果 |
|---|---|
| 真机 `catkin_make --pkg rosiwit_navigation_ros1` | **exit=0** |
| `catkin_make run_tests_rosiwit_navigation_ros1` | **15/15** |
| 隔离 master（127.0.0.1:114xx）端到端 15 项 | **全 PASS** |
| 接口一致性脚本（namespace 隔离） | **23/23，不符 0** |

端到端覆盖：action 可用、navi_cmd=4→-4 且无速度、navi_cmd=1 空点位→-1、
6 输出 +0.6rad/s、7 输出 -0.1m/s、2/3 规划/跟踪后输出正线速度、
四种命令 cancel→PREEMPTED、取消后保持停止。

### 8.6 部署位置与边界

* 源码：`/home/xzrobot/rosiwit_ws/src/rosiwit_navigation` +
  `/home/xzrobot/rosiwit_ws/ros1_ws/src/rosiwit_navigation_ros1`
* 构建：`/home/xzrobot/rosiwit_ws/ros1_ws/devel`（**未 install 到 /opt/ros**）
* 在跑的 `robot_move_base`（11311）未重启、未改 launch；本次所有运行验证都在
  独立 master（`127.0.0.1:114xx`）+ `/navtest` 命名空间内完成，不碰实车。
* 切换实车导航需要另行确认：停 `robot_move_base`、把 bringup 指向
  `roslaunch rosiwit_navigation_ros1 movebase_compat.launch`，并保留回滚命令。


## 9. 实车 footprint / 规划参数（192.168.21.166，2026-10-21）

**结论：不再使用 `robot_movebase` 源码里的 `robot_movebase_v1.yaml`。**
源码里的 footprint 是 `[-0.2,-0.35] … [0.6,0.35]`，而实车运行参数是另一份：

```text
/robot_move_base/footprint:
  [-0.25,-0.33] [0.45,-0.33] [0.55,-0.20]
  [ 0.55, 0.20] [0.45, 0.33] [-0.25, 0.33]
/robot_move_base/g_max_safe: 0.42
/robot_move_base/g_min_safe: 0.39
/robot_move_base/max_speed: 0.8
/robot_move_base/speed_lim_w: 0.5
/robot_move_base/accel_lim_v_x: 0.3
/robot_move_base/accel_lim_w: 0.2
/robot_move_base/min_lookahead_dist: 0.3
/robot_move_base/max_lookahead_dist: 0.9
/robot_move_base/xy_goal_tolerance: 0.1
/robot_move_base/yaw_goal_tolerance: 0.1
/robot_move_base/g_control_freq: 10
/robot_move_base/stop_dist: 0.12
/robot_move_base/g_robot_safe_dist: 0.3
```

映射到本包（见 `config/movebase_compat.yaml`）：

| 本包参数 | 值 | 来源/理由 |
|---|---|---|
| `movebase_robot_footprint` / `safety_footprint` | 实车 6 点 footprint | `/robot_move_base/footprint` |
| `robot_radius` | `0.39` | 实车 `g_min_safe`（旧节点第二档规划膨胀） |
| `inflation_radius` | `0.62` | 旧节点 `CostMapInflate(0.1)` + `g_max_safe(0.42)` + 2×cell(0.05) 的等效半径；本包用硬禁区 0.39 + 软代价到 0.62 表达旧包“先 max 后 min”的两档 |
| `safety_stop_polygon` | footprint 外扩 `stop_dist=0.12` | `[-0.37,-0.45] … [0.67,0.45]` |
| `safety_slowdown_polygon` | footprint 外扩 `g_robot_safe_dist=0.30` | `[-0.55,-0.63] … [0.85,0.63]` |
| `max_velocity_x` | `0.8` | `max_speed` |
| `max_velocity_theta` | `0.5` | `speed_lim_w` |
| `max_accel_x` / `max_accel_theta` | `0.3` / `0.2` | `accel_lim_v_x` / `accel_lim_w` |
| `goal_tolerance_xy` / `_yaw` | `0.1` / `0.1` | `xy/yaw_goal_tolerance` |
| `lookahead_distance` / min / max | `0.6` / `0.3` / `0.9` | `min/max_lookahead_dist` |
| `movebase_default_speed` | `0.8` | `max_speed` |
| `movebase_rotate_speed` | `0.5` | `speed_lim_w` |
| `controller_frequency` | `10` | `g_control_freq` |

节点启动时会打印一行，便于现场核对（不是仿真值）：

```text
[rosiwit_navigation_ros1] footprint(6 pts) x[-0.25,0.55] y[-0.33,0.33]
robot_radius=0.39 inflation_radius=0.62 safety_footprint=6 pts
```

如果 footprint 的 x/y 范围超出实车值，节点会额外打
`footprint 与实车 ... 不一致, 请确认不是仿真参数` 的 WARN。
