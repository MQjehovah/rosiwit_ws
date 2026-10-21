# ROS1 动态覆盖清洁（navi_cmd=20）设计与实测

> 2026-10-21。目标：不再“先规划一条长路径再盲跟踪”，而是维护覆盖状态、
> 动态生成短行段，完成区域覆盖，并标注障碍导致的不可覆盖区域。

## 1. 接口（冻结）

只扩展 `NaviCmdAction` 的一个命令，不新增 action/service/goal 字段：

```text
navi_cmd = 20           # 动态覆盖清洁
tracking_path           # 区域多边形顶点，map 系 Pose2D[]，>=3 点，theta 忽略
robot_vel               # 覆盖最大速度 m/s；<=0 用参数 coverage_max_speed
其余 goal 字段忽略
```

宽度等全部走私有参数：

```yaml
coverage_sweep_width: 0.5          # 清扫宽度 m
coverage_controller_name: "pure_pursuit"  # 覆盖模式单独控制器
coverage_max_speed: 0.4
coverage_resolution: 0.05
coverage_replan_period: 10.0       # 周期兜底；正常按行段走完/6s 不动重规划
coverage_horizon: 2.5              # 单次行段最大长度 m
coverage_retry_timeout: 10.0       # 无进展多久后把剩余标不可达
coverage_complete_rate: 0.995      # 覆盖率达到即收尾
coverage_blocked_inflation: 0.10   # 动态障碍点膨胀
coverage_plan_inflation: 0.0       # 覆盖 A* 不再圆形膨胀
coverage_plan_robot_radius: 0.0
coverage_report_path: ""           # 非空则落盘 coverage_report.json
```

## 2. 算法

```text
CoverageGrid (ROS 无关)
  状态: outside / unknown / uncovered / covered / blocked / obstacle
  按“实际位姿轨迹 + sweep_width/2 圆盘”累计 covered，不是按规划路径
  静态障碍来自 /map；动态障碍来自 /laser 点（0.1m 膨胀，每帧刷新）

行段调度 (v1)
  1. BFS 找最近未覆盖格
  2. 取它所在的整行/列连续可通行段（区域长边方向优先），截到 horizon
  3. 机器人离行端 >0.3m 时先用 A* 到行端
  4. 行段交给 coverage 控制器跟随
  5. 路径走完/6s 不动/10s 周期 -> 重新取行段
```

关键点：

* **覆盖边界是车体中心的硬边界**；预测下一步会越界时停线速度（保留转向），
  避免冲出去后回不来。
* **动态障碍不直接等于不可达**：被挡时等 `retry_timeout`，障碍消失后重新可达。
* **静态不可达**（墙后、太窄）用 BFS 可达性标 `BLOCKED`，输出矩形包络。
* 覆盖模式用独立的 `pure_pursuit` 控制器：行段是直线，pure_pursuit 比 MPPI 稳；
  正常导航仍用 MPPI + footprint，互不影响。安全层继续兜底。
* 最后 0.5% 的边界/盲区不再死磕：达到 `coverage_complete_rate` 后
  剩余未覆盖标 `BLOCKED` 并写入报告。

## 3. 输出（不新增话题）

| 话题 | 内容 |
|---|---|
| `/clean_map` | `-1` 区域外/unknown，`0` 未覆盖，`50` 已覆盖，`80` 不可达/被挡，`100` 障碍/动态障碍 |
| `/clean_area` | 已覆盖面积 m² |
| `/clean_part_status` | JSON：state / coverage_rate / covered_area / total_area / blocked_area / dynamic_cells / unreachable 矩形包络 |
| `/local_path` / `/global_path` | 当前行段 |
| action feedback/result | `execution_percent`=覆盖率，`clean_area`=已覆盖面积；完成（哪怕有不可达）回 `tracking_path_result=1` |

## 4. 实测（隔离 master + 仿真车体，不碰实车）

场景：2m×1m 区域，sweep=0.5m，速度 0.35m/s，地图全自由；
仿真车体按 `/navtest/movebase_cmd_vel` 积分并广播 TF/odom。

结果：

```text
result=1  percent=0.996  clean_area=1.993 m²  state=SUCCEEDED
blocked_area=0.0075 m²  unreachable=[]（3 个格子，clean_map 里为 80）
```

- 正常导航控制器：MPPI + footprint（未参与覆盖执行）
- 覆盖控制器：pure_pursuit
- 覆盖栅格单测 9/9；核心全量 13/13；协议单测 17/17
- 实车包编译通过

## 5. 已知限制 / 待现场验证

1. v1 行段是固定 x/y 方向，不规则区域效率一般；后续可接局部弓字/分区。
2. 覆盖执行用 pure_pursuit，不看局部 costmap；依赖 A* 覆盖图 + 安全层。
   如果现场安全层频繁停车，需要把覆盖控制器换成 DWA/MPPI 或调低速度。
3. 2m² 合成场景约 90s，效率还受控制器/仿真 TF 影响；真实车体需要实测。
4. 动态障碍只用当前 `/scan` 点，遮挡/盲区可能漏；靠 retry + 安全层。
5. 尚未接 App 的“绘制区域”入口；目前由客户端直接发 `navi_cmd=20`。
6. **实车尚未安装本版本**（`/opt/ros` 仍是之前 MPPI+footprint 的二进制）；
   需要 install + `rosnode kill /robot_move_base` 后才会生效。

## 6. 复现

```bash
# 核心单测
cd src/rosiwit_navigation && cmake -S . -B build && cmake --build build -j && ctest --test-dir build

# 实车隔离闭环（私有 master + /navtest）
rosrun rosiwit_navigation_ros1 rosiwit_navigation_ros1_node ...  # 或用临时 build
python ~/coverage_drive_smoke.py
```
