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
| `movebase_robot_footprint` | 旧包 footprint | 车体框，供 `/robot_footprint` |
| `movebase_default_speed` | `max_velocity_x` | `robot_vel<=0` 时的速度 |
| `movebase_back_speed` | `0.1` | 倒车速度 |
| `movebase_rotate_speed` | `0.6` | 旋转角速度上限 |
| `movebase_stuck_timeout` | `60` | 卡死判定（旧包 `check_stop_time` 默认 300s，见 §5） |

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

