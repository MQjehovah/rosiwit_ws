# nav2 参考实现摘录（离线存档）

WSL 环境无法直连 GitHub，这里把关键实现摘下来，改动前直接对照，不用再抓。

来源（Apache-2.0，ros-navigation/navigation2，main 分支）：

- `nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`
- `nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp`
- `nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp`

## 1. 跟踪律（computeVelocityCommands）

```cpp
lookahead_dist = getLookAheadDistance(speed);            // 速度缩放 + clamp
carrot = nav2_util::getLookAheadPoint(lookahead_dist, transformed_plan);  // 沿弧长
curvature = calculateCurvature(carrot);                  // kappa = 2*y / L^2（机器人系）

if (shouldRotateToGoalHeading(...))          // 到目标: v=0, 原地转
    rotateToHeading(linear, angular, angle_to_goal, speed);
else if (shouldRotateToPath(carrot, angle, sign))        // |angle| > 0.785
    rotateToHeading(linear, angular, angle_to_path, speed);
else {
    applyConstraints(curvature, speed, pose_cost, plan, linear, sign);
    angular = linear * curvature;            // 先限线速度, 再算角速度
}
```

## 2. 曲率限速 `heuristics::curvatureConstraint`

```cpp
radius = fabs(1.0 / curvature);
if (radius < min_radius)
    return raw_linear_vel * (1.0 - fabs(radius - min_radius) / min_radius);
return raw_linear_vel;
```

## 3. 接近终点收缩 `heuristics::approachVelocityConstraint`

```cpp
// 用**剩余弧长**判断是否进入收缩区, 用**到终点的欧氏距离**做平滑比例
remaining = calculate_path_length(path);
scaling = (remaining < approach_dist) ? hypot(last.x, last.y) / approach_dist : 1.0;
vel = constrained_vel * scaling;
if (vel < min_approach_velocity) vel = min_approach_velocity;
return min(constrained_vel, vel);
```

## 4. 原地转 `rotateToHeading`

```cpp
linear_vel = 0.0;
sign = angle > 0 ? 1 : -1;
angular = sign * rotate_to_heading_angular_vel;
// 加速度可行域
angular = clamp(angular, curr_w - max_ang_accel*dt, curr_w + max_ang_accel*dt);
// 防过冲
max_vel_to_stop = sqrt(2 * max_ang_accel * fabs(angle));
if (fabs(angular) > max_vel_to_stop) angular = sign * max_vel_to_stop;
```

## 5. 官方默认参数（小机器人需按比例缩小）

| 参数 | nav2 默认 |
|---|---|
| `lookahead_dist` | 0.6 |
| `min_lookahead_dist` | 0.3 |
| `max_lookahead_dist` | 0.9 |
| `lookahead_time` | 1.5 |
| `use_velocity_scaled_lookahead_dist` | false |
| `rotate_to_heading_angular_vel` | 1.8 |
| `rotate_to_heading_min_angle` | 0.785 |
| `min_approach_linear_velocity` | 0.05 |
| `approach_velocity_scaling_dist` | 0.6 |
| `regulated_linear_scaling_min_radius` | 0.90 |
| `regulated_linear_scaling_min_speed` | 0.25 |
| `min_linear_vel` / `max_linear_vel` | -0.5 / 0.5 |

## 6. 关键前提（我们踩过的坑）

nav2 的控制器假设**一条稳定的全局路径**，并且大角度/卡死由独立组件处理：

- `nav2_rotation_shim_controller`：|航向误差| 大 → 交给它原地转，转完再交回主控制器
- 行为树恢复：`Spin` / `BackUp` / `ClearCostmap`
- `GoalChecker` 决定到位，控制器不自己判

**所以把跟踪律照搬到"每行重规划 + approach 拼接"的覆盖层上会失效**：
覆盖速度 0.25 m/s 时速度缩放前视只有 0.55 m，比行宽 0.5 m 还大，前视点会落到
相邻行/approach 段上 → 机器人跟一条不存在的线（实测轨迹 162 m、覆盖率 22.6%）。

## 7. 覆盖路线生成参考

- `fields2cover`：`SwathGenerator`（按 `headland` 内缩生成平行行）+ `Route`
  （boustrophedon 排序，行间连接）
- `opennav_coverage`：一次生成整条 coverage route，再交给控制器跟踪

结论：**先把整条路线生成完（含行间转向），再启用 nav2 跟踪律**，顺序不能反。
