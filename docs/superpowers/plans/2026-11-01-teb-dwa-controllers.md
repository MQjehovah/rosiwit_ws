# TEB / DWA 控制器 + 四控制器对比 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `teb` 做成贴近 `teb_local_planner` 的定时弹性带、把 `dwa` 补到能公平参赛，并用固定场景对 `rpp/mppi/dwa/teb` 做可比对比。

**Architecture:** 自研块三对角 Gauss-Newton + LM 求解器（不引 g2o/Eigen）；障碍距离用现有 `euclideanDistanceCells()` 的 EDT 场按足迹采样查询；同伦多候选用"全局路径左右偏移的初值"近似；每拍 warm-start 增量优化。所有新增代码放在 core（无 ROS 依赖），用 `catkin_add_gtest` 单测。

**Tech Stack:** C++17 / catkin melodic / gtest / WSL2+Gazebo 仿真

**Spec:** `docs/superpowers/specs/2026-11-01-teb-controller-design.md`

## Global Constraints

- 不引入新第三方依赖（无 Eigen / g2o / Ceres）。
- `computeVelocityCommand()` ≤ 8 ms（控制 10 Hz）；`setPath()` 不构图。
- 复用 `core::sampleFootprint` / `core::footprintCost` / `core::euclideanDistanceCells`。
- core 代码不得依赖 ROS；单测在 `src/rosiwit_navigation/test/`，由 glob 自动纳入构建。
- 任何被判"可行"的带，窗口内每个顶点的足迹不得超过 lethal(100)。
- 中文注释保持 UTF-8；不得用 PowerShell `Get-Content/Set-Content` 改源文件。

## Review Focus

- 弹性带端点漂移（起点没跟上机器人 → 输出速度反向）；每拍必须重新锚定起点。
- 同伦候选来回跳（两族代价接近时），必须迟滞。
- EDT 窗口外查询（带子比窗口长）返回什么，不能让"未知"被当成"很安全"。
- 倒车段符号取错（cusp 处顶点位移与 `velocity_x` 不一致）。
- 全不可行时的输出（零速 vs 低速），不能悄悄输出"最不坏"的撞墙轨迹。

---

### Task 1: 块三对角求解器 `core/banded_solver.hpp` + 单测

**Files:** Create `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/banded_solver.hpp`;
Test `src/rosiwit_navigation/test/test_banded_solver.cpp`

**Interfaces (Produces):**
```cpp
namespace rosiwit_navigation { namespace core {
struct BlockTridiagonal {                     // 每顶点 3 维 (x,y,theta)
  int n = 0;
  std::vector<std::array<double, 9>> diag;    // n 个 3x3
  std::vector<std::array<double, 9>> upper;   // n-1 个 3x3 (i,i+1)
  std::vector<double> rhs;                    // 3n
};
/// 解 (H + lambda*diag(H)) d = -rhs; 成功返回 true, 并把 d 写回 out (3n)
bool solveBlockTridiagonal(BlockTridiagonal & sys, double lambda, std::vector<double> * out);
}}
```

- [ ] **Step 1: 写失败测试**（3 个用例：对角系统解析解、三对角耦合系统与稠密解一致、λ 增大使步长变小）
- [ ] **Step 2: 跑测试确认失败** — `cd ~/rosiwit_ws/ros1_ws && catkin_make -DCATKIN_ENABLE_TESTING=1 -j8 && make -C build/rosiwit_navigation test_banded_solver` → 缺头文件
- [ ] **Step 3: 实现**：先按对角块做列消元（Thomas 算法的块版本），再回代；`lambda` 加在 diag 上；
      对角线出现非正定（<1e-12）时返回 false
- [ ] **Step 4: 跑测试确认通过**（全 core 套件也跑一遍）
- [ ] **Step 5: 提交**

### Task 2: 弹性带边与代价 `core/teb_types.hpp` + 单测

**Files:** Create `.../nav_core/teb_types.hpp`; Test `.../test/test_teb_types.cpp`

**Interfaces (Produces):**
```cpp
struct TebVertex { double x, y, theta, dt; };
struct TebWeights { double path, obstacle, kinematics, velocity, accel, time, shortest;
                    double min_obstacle_dist, inflation_dist, max_deviation; };
/// 单条边的残差与 3x3 雅可比; 返回代价, 并把 grad/cost 累加进去
struct EdgeAccumulator { std::array<double,9> hess{}; std::array<double,3> grad{}; double cost{}; };
EdgeAccumulator edgePath(const TebVertex& v, const Path& path, double w);
EdgeAccumulator edgeObstacle(const TebVertex& v, double dist_m, const TebWeights& w);
EdgeAccumulator edgeKinematics(const TebVertex& a, const TebVertex& b, double w);
EdgeAccumulator edgeVelocity(const TebVertex& a, const TebVertex& b, const TebWeights& w, double max_v);
EdgeAccumulator edgeAccel(const TebVertex& a, const TebVertex& b, const TebVertex& c, const TebWeights& w, double max_a);
EdgeAccumulator edgeTime(const TebVertex& v, double w);
double shortestPathCost(const TebVertex& a, const TebVertex& b, const TebVertex& c, double w, std::array<double,3>* g_prev, std::array<double,3>* g_next);
```

- [ ] **Step 1: 写失败测试**：路径偏离代价在 v 在路径上时为 0、偏移 1 m 时按二次增长；
      障碍代价在 dist>=min_obstacle_dist 时为 0、dist 减小时单调增；kinematics 在直线段为 0、
      90° 折角时 >0；time 代价 = w*dt；accel 在 |Δv|<=max_a*dt 时为 0
- [ ] **Step 2-5**：同 Task 1 的节奏（失败 → 实现 → 通过 → 提交）

### Task 3: 障碍距离场 `core/obstacle_distance.hpp` + 单测

**Files:** Create `.../nav_core/obstacle_distance.hpp`; Test `.../test/test_obstacle_distance.cpp`

**Interfaces (Produces):**
```cpp
class ObstacleDistance {
 public:
  void update(const CostmapGrid& grid);                 // 用 euclideanDistanceCells 建场
  bool ready() const;
  bool inside(double x, double y) const;                // 窗口外 -> false
  double distanceAt(double x, double y) const;          // 米; 窗口外返回 -1
  /// 车体足迹的最小间隙(米): 对 footprint 采样点取最小 distanceAt; 任一点在窗口外返回 -1
  double footprintClearance(const Pose2D& pose,
                            const std::vector<std::pair<double,double>>& samples) const;
};
```
- [ ] **Step 1: 写失败测试**：单格 lethal 中心的距离为 0、相邻格为 res；
      `footprintClearance` 在贴近障碍时变小、远离时变大；窗口外返回 -1
- [ ] **Step 2-5**：同上节奏

### Task 4: 重写 `teb_controller.cpp` + 单测

**Files:** Modify `src/rosiwit_navigation/src/algorithms/local_planner/teb_controller.{hpp,cpp}`;
Test `src/rosiwit_navigation/test/test_teb_controller.cpp`

**Interfaces (Consumes):** Task 1-3 全部 + `core::sampleFootprint`/`footprintCost`
**Interfaces (Produces):** `TebController` 实现 `supportsFootprint()==true`、`supportsMultiDirection()==true`、
`setLocalCostmap()`、`getDiagnostics()`（`"teb_candidates=3,best=1,iters=2,infeasible=0"` 形式）

- [ ] **Step 1: 写失败测试**：① 直线路径上 3 个控制周期内 `isGoalReached()` 从 false 变 true 且 v>0；
      ② 路径中段放一个 lethal 障碍 → 优化后的带在该点绕开（窗口内 clearance 全 >0）且仍朝目标前进；
      ③ 障碍占满窗口 → 输出 `{0,0,0}` 且 diagnostics 含 `infeasible=1`；
      ④ 路径带 `velocity_x<0` 的段 → 输出 v<0；
      ⑤ `getCurrentWaypointIndex()` 单调不减且不超 `path.points.size()`
- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**：构图（3 条候选：原路径 + ±homotopy_offset）→ 每拍 warm-start 优化
      （LM，max_iterations 次，ΔT 由弧长/限速闭式更新）→ 裁剪身后顶点/末端补点 →
      代价最低且窗口内可行的候选胜出（迟滞：上一拍赢家优先）→ 取带首段的 v=Δs/dt、ω=Δθ/dt
- [ ] **Step 4: 跑测试确认通过**（含全 core 套件）
- [ ] **Step 5: 提交**

### Task 5: DWA 补齐（兜底/倒车/末段）+ 单测

**Files:** Modify `dwa_controller.cpp`; Test `test_dwa_controller.cpp`

- [ ] **Step 1: 写失败测试**：① 窗口内全是 lethal 时仍返回非空命令且 v <= 0.3*max（不再 `{0,0,0}` 卡死）；
      ② `allow_reverse=true` 且目标在身后 → 输出 v<0；③ 距目标 0.4 m 时 v < 0.3*max
- [ ] **Step 2-5**：同上节奏

### Task 6: 固定场景 + 对比脚本

**Files:** Create `ros1_ws/src/rosiwit_simulation_ros1/world/doorway.world`;
Modify `scripts/sim/run_nav_sim.sh`（`--goal x,y,yaw`、`--keepout-at t,x,y`）与 `scripts/sim/nav_sim_goal.py`（
`min_clearance_m`、输出 `controller`/`profile`）；Create `scripts/sim/compare_controllers.sh`

- [ ] **Step 1: 加 `--goal`**：给了就跳过 BFS 选点直接用（并校验在自由区）
- [ ] **Step 2: 加 `min_clearance_m`**：用 `/map` 的 lethal 格对本体轨迹取最近距离
- [ ] **Step 3: `doorway.world`**：两道墙留 0.9 m 门洞，门后 3 m 自由区，spawn 在门前 3 m
- [ ] **Step 4: `compare_controllers.sh`**：`for profile in clean free; for ctrl in rpp mppi dwa teb; for scene in ...; for i in 1 2 3` →
      生成 `results.csv` + 打印 markdown 表
- [ ] **Step 5: 提交**

### Task 7: 跑矩阵 + 出报告

- [ ] **Step 1:** 先单场景冒烟（teb × corridor × clean），确认不崩、≤8 ms
- [ ] **Step 2:** 跑完 4×2×5×3 = 120 次（每次数十秒，分批跑）
- [ ] **Step 3:** 写 `docs/dev/controller-comparison-2026-11.md`：表格 + 归因（控制层/规划层/安全层）
- [ ] **Step 4:** 提交
