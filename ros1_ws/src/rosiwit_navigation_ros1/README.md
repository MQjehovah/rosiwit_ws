# rosiwit_navigation_ros1

`rosiwit_navigation` 的 ROS1 薄接口包：ROS 报文/话题/action 与算法核心之间的桥接。
算法核心位于 `src/rosiwit_navigation`（零 ROS 依赖），由本包内联构建，与 ROS2 包共享。

## 两种入口

| 入口 | 说明 |
|---|---|
| 原生 | `/goal_pose`（`geometry_msgs/PoseStamped`）、`/nav_stop`（`std_srvs/Trigger`）、`/cmd_vel`、`/local_path`、`/global_path`、`/local_costmap`、`/global_costmap`、`/goal_status`、`/nav_diagnostics`、`/tracked_obstacles` |
| robot_movebase 兼容 | action `/RobotNaviCmdAction`（`xzrobot_ros_dependency/NaviCmdActionAction`）+ `/movebase_cmd_vel`、`/local_map`、`/makeplan_map`、`/robot_footprint`、`/prediction_robot_footprint`、`/prediction_path`、`/movebae_robot_path`、`/back_off_path`、`/docking_path`、`/re_point` 等旧话题 |

两套入口共用同一个控制循环，兼容层由参数 `movebase_compat_enabled`（默认 `true`）控制。
兼容语义、支持的命令（`navi_cmd` 1/2/3/6/7）与有意偏差见
[`docs/superpowers/specs/2026-10-21-ros1-movebase-interface-compat-design.md`](../../../docs/superpowers/specs/2026-10-21-ros1-movebase-interface-compat-design.md)。

## 分层

```
src/
├── rosiwit_navigation_ros1_node.cpp   订阅/发布/定时器/安全过滤（ROS 层）
├── movebase_protocol.{h,cpp}          robot_movebase action 报文解析（纯 C++）
└── movebase_interface.{h,cpp}         actionlib 服务端 + 旧话题适配（ROS 层）

src/rosiwit_navigation/                算法核心
├── nav_core/action_task.{hpp,cpp}     任务状态机：进度/到位/末端航向/倒车/卡死
└── nav_core/...                       规划器、控制器、安全层、costmap
```

## 运行

```bash
# 原生接口
roslaunch rosiwit_navigation_ros1 rosiwit_navigation.launch

# robot_movebase 兼容接口（节点名默认 robot_move_base）
roslaunch rosiwit_navigation_ros1 movebase_compat.launch
```

## 测试

```bash
# 接口一致性（节点运行时；机器人上最有用的自检）
rosrun rosiwit_navigation_ros1 check_movebase_interface.sh

# 协议解析（不依赖 ROS 运行时）
catkin_make run_tests_rosiwit_navigation_ros1
# 或直接:
g++ -std=c++17 -I src/rosiwit_navigation/include \
    test/test_movebase_protocol.cpp src/movebase_protocol.cpp \
    -lgtest -lgtest_main -pthread -o /tmp/t && /tmp/t

# 任务状态机（核心单测）
g++ -std=c++17 -I src/rosiwit_navigation/include \
    src/rosiwit_navigation/test/test_action_task.cpp \
    src/rosiwit_navigation/src/nav_core/action_task.cpp \
    -lgtest -lgtest_main -pthread -o /tmp/t2 && /tmp/t2
```
