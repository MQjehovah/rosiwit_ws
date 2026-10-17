# rosiwit_navigation 开发文档

## 三层架构

```
ros_interface/    ←  ROS 边界层：节点、话题、TF、Action
nav_core/         ←  抽象接口层：纯虚类、工厂、类型定义
algorithms/       ←  实现层：具体算法
```

### 数据流

```
/goal_pose (话题)  ──┐
                      ├──→ NavigationNode (LifecycleNode)
/navigate_to_pose (Action) ──┘         │
                                        ├──→ PathPlanner (A*/NavFn)
                                        ├──→ TrajectoryGenerator
                                        ├──→ PurePursuitController
                                        └──→ 发布 /cmd_vel
```

### 坐标系

```
odom → base_footprint → base_link → 传感器/轮子
  ↑
FAST-LIO2 SLAM (rosiwit_slam)
```

导航节点查 `global_frame` → `robot_frame` 的 TF。

## 工厂模式

`NavigationFactory` 按名称创建组件：

| 名称 | 创建对象 |
|------|---------|
| `astar` / `a_star` | `AStarPlanner` |
| `navfn` | `NavFnPlanner` |
| `pure_pursuit` | `PurePursuitController` |

接口定义在 `nav_core/`:
- `IPlannerStrategy` — 规划器抽象 (AStarPlanner, NavFnPlanner)
- `IControllerStrategy` — 控制器抽象 (PurePursuitController)

## 测试

```bash
colcon test --packages-select rosiwit_navigation --event-handlers console_direct+
colcon test-result --all --verbose
```

活跃的测试:
- `test_narrow_passage_detector` — 窄道检测
- `test_trajectory_generator` — 轨迹生成
- `test_velocity_limiter` — 速度限制器
- `test_narrow_passage` — 窄道通行集成

## 包名历史

| 阶段 | 包名 | 说明 |
|------|------|------|
| 原版 | `diffbot_navigation` | 上游仓库名 |
| 当前 | `rosiwit_navigation` | 统一 rosiwit 命名 |

C++ 命令空间统一为 `rosiwit_navigation`，内嵌 `core`、`navigation`、`planners`、`controllers`、`narrow_passage`、`obstacle_avoidance` 等子命名空间。
