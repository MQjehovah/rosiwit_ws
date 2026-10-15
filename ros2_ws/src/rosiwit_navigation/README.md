# rosiwit_navigation

差分驱动机器人导航系统，三层架构：**ros_interface → nav_core → algorithms**。

## 架构

```
ros_interface/       ← ROS2 生命周期节点、话题/服务/Action 接口
  navigation_node.hpp/cpp    平滑导航节点 (LifecycleNode)
  navigation_node_main.cpp   主入口

nav_core/            ← 抽象接口 + 工厂
  i_planner.hpp              全局规划器接口
  i_controller.hpp           控制器接口
  types.hpp                  核心数据类型
  exceptions.hpp             异常定义
  navigation_factory.hpp/cpp 工厂 (按名称创建组件)

algorithms/          ← 具体算法实现
  a_star/astar_planner        A* 全局规划
  navfn/navfn_planner         NavFn 波前规划
  pure_pursuit/               Pure Pursuit 路径跟踪
  diff_drive/                 差速驱控制器 (Nav2)
  path_planning/path_planner  路径规划器封装
  trajectory/                 轨迹生成器
  tracking/velocity_limiter   速度/加速度约束
  obstacle_avoidance/         障碍物检测 + 避障
  narrow_passage/             窄道检测 + 通行
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /rosiwit_ws
colcon build --packages-select rosiwit_navigation --symlink-install
```

## 运行

```bash
source install/setup.bash
ros2 launch rosiwit_navigation navigation.launch.py
```

支持两种目标下发方式：
- **RViz "2D Nav Goal"** → `/goal_pose` 话题
- **Nav2 Action** → `/navigate_to_pose` Action

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `global_frame` | `odom` | 全局坐标系 |
| `robot_frame` | `base_link` | 机器人本体坐标系 |
| `controller_frequency` | 20.0 | 控制频率 (Hz) |
| `planner_frequency` | 1.0 | 重规划频率 (Hz) |
| `max_velocity_x` | 0.5 | 最大线速度 (m/s) |
| `max_velocity_theta` | 1.0 | 最大角速度 (rad/s) |
| `goal_tolerance_xy` | 0.1 | 目标位置容差 (m) |
| `goal_tolerance_yaw` | 0.1 | 目标角度容差 (rad) |

## 依赖

- ROS2 Humble (`rclcpp`, `rclcpp_lifecycle`, `nav_msgs`, `geometry_msgs`)
- `tf2_ros`, `tf2_geometry_msgs`
- Nav2 (可选): `nav2_util`, `nav2_costmap_2d`, `nav2_core`
