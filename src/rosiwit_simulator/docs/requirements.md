# 需求文档：rosiwit_slam 编译修复与仿真集成 Launch

> **版本**: 1.0  
> **日期**: 2026-05-05  
> **作者**: AI开发团队 - 产品经理  
> **状态**: 待开发

---

## 1. 需求概述

修复 `rosiwit_slam` 包的 CMake 目标名冲突和 launch 文件包名错误，使其能在 ROS2 Humble 下成功编译（`colcon build` 零错误通过），并创建一个集成 launch 文件，将仿真器（Gazebo + 3D 雷达机器人）与 SLAM 节点及自动运动控制串联起来，实现一键启动"仿真→建图"的完整闭环演示。

---

## 2. 用户故事

### US-1: 编译通过（开发者）
> 作为 `rosiwit_slam` 的开发者，我希望执行 `colcon build --packages-select rosiwit_slam` 能够零错误通过，这样我才能继续后续的 SLAM 功能开发和调试。

### US-2: Launch 文件正确引用包名（开发者）
> 作为开发者，我希望 `fast_lio2.launch.py` 中所有的包名和可执行名正确指向 `rosiwit_slam` 包，这样我能独立启动 SLAM 节点进行测试。

### US-3: 一键启动仿真+建图（算法工程师）
> 作为 SLAM 算法工程师，我希望通过一个 launch 文件就能同时启动 Gazebo 仿真、机器人模型、SLAM 节点和自动运动控制，这样我可以在无显示器的 WSL2 环境下快速验证算法效果，无需手动逐个启动节点。

### US-4: 机器人自动移动产生建图数据（测试人员）
> 作为测试人员，我希望机器人在仿真环境中按预设轨迹（8字形/圆形）自动移动，这样 SLAM 算法可以接收到不同视角的激光雷达和 IMU 数据，完成建图过程，而不需要手动遥控。

---

## 3. 功能优先级表

| 优先级 | 功能项 | 描述 | 预计影响 |
|--------|--------|------|----------|
| **P0** | CMake 目标名冲突修复 | 将 `add_executable(${PROJECT_NAME} ...)` 改为 `add_executable(fast_lio2_node ...)`，避免与 `rosidl_generate_interfaces` 的目标名 `rosiwit_slam` 冲突 | **不修复则完全无法编译** |
| **P0** | Launch 文件包名/可执行名修复 | `fast_lio2.launch.py` 中 3 处 `fast_lio2_slam` → `rosiwit_slam`/`fast_lio2_node` | **不修复则 launch 无法启动节点** |
| **P0** | `colcon build` 零错误通过 | 修复后执行完整编译验证 | 核心交付标准 |
| **P0** | 集成 Launch 文件 | 创建 `simulator_slam_demo.launch.py`，串联 Gazebo + robot_state_publisher + spawn + SLAM + 自动运动 | 核心交付物 |
| **P1** | Headless 模式支持 | 集成 launch 默认 `GAZEBO_HEADLESS=1`、`gui:=false`，适配 WSL2 无显示器环境 | WSL2 环境必需 |
| **P1** | 自动运动节点 | 内嵌 Python 节点定时发布 `/cmd_vel`，让机器人按 8 字形/圆形轨迹移动 | 无此项则需手动遥控 |
| **P1** | 话题匹配验证 | 确认仿真器发布的 `/velodyne_points`、`/imu` 与 SLAM 订阅的话题一致 | 数据流通必需 |
| **P2** | `use_sim_time` 全链路统一 | 所有节点统一使用 `use_sim_time:=true`，确保仿真时钟同步 | 长期运行稳定性 |

---

## 4. 质疑回答

### Q1: 用户真正的痛点是什么？
**痛点**: `rosiwit_slam` 包当前完全无法编译（CMake 目标名冲突是硬阻塞），已有的仿真器（`rosiwit_simulator`）和 SLAM 包之间没有集成方案，每次测试需要手动逐个启动 4-5 个节点，在无显示器的 WSL2 环境下效率极低。真正的需求是**快速、可靠地验证 SLAM 算法在仿真环境中的建图效果**。

### Q2: 不做这个会怎样？有没有更简单的替代方案？
**不做**: `rosiwit_slam` 永远无法编译，之前的所有代码开发白费；SLAM 与仿真器无法集成，无法验证算法效果。  
**替代方案**: 手动逐个 `ros2 run` 启动各节点——但这在 WSL2 headless 环境下极其繁琐，且每次启动顺序、参数都需要手动对齐，容易出错。集成 launch 是标准 ROS2 做法且复杂度极低（单个 Python 文件），没有理由不做。

### Q3: 这个功能的成功标准是什么？如何衡量？
| 验收项 | 衡量标准 |
|--------|----------|
| 编译成功 | `colcon build --packages-select rosiwit_slam` 返回码 0，零错误 |
| Launch 语法正确 | `python3 -c "import simulator_slam_demo.launch"` 无语法错误 |
| 节点可启动 | `ros2 launch rosiwit_simulator simulator_slam_demo.launch.py` 启动后 `ros2 node list` 可见所有 5 个节点 |
| 话题流通 | `ros2 topic echo /velodyne_points` 和 `/imu` 有数据输出 |
| 机器人移动 | `ros2 topic echo /cmd_vel` 有周期性速度指令 |

### Q4: 技术上最大的风险点在哪？
1. **CMake 目标名冲突**：`rosidl_generate_interfaces` 生成的库目标与 `add_executable` 同名是已知 ROS2 构建系统限制，修复方案成熟（重命名可执行文件）但需确保所有引用处同步更新。
2. **Headless Gazebo 在 WSL2 下的稳定性**：无 GPU 加速时 Gazebo 可能运行缓慢甚至崩溃，需要 `GAZEBO_HEADLESS=1` + `verbose=true` 来减少渲染开销。
3. **第三方依赖（PCL、Eigen、Sophus）的编译兼容性**：这些不在本次修复范围内，但如果系统缺少依赖库会导致编译失败，需要事先确认。

### Q5: 如果要 1 天内交付，会砍掉什么？
- **砍掉**: P2 的 `use_sim_time` 全链路统一（可后续优化）
- **砍掉**: 自动运动轨迹的复杂配置（圆形轨迹足够，不需要 8 字形参数化）
- **砍掉**: RViz 可视化（WSL2 headless 下不需要）
- **保留**: 所有 P0 和 P1 项

### Q6: 有没有竞品已经做过？我们比他们好在哪里？
这不是竞品产品，而是内部开发工具。ROS2 生态中 `fast_lio2` 的官方仓库已提供参考 launch 文件，我们的集成将 simulator + SLAM + 自动运动合并为单一 launch，这是针对我们自身机器人平台（mbot + VLP-16）的定制化整合，外部没有完全匹配的方案。

---

## 5. 范围边界

### ✅ 做什么（In Scope）

1. **修改 `rosiwit_slam/CMakeLists.txt`**：
   - 引入 `NODE_NAME` 变量，将可执行文件重命名为 `fast_lio2_node`
   - 更新所有 `ament_target_dependencies`、`target_link_libraries`、`target_compile_definitions`、`install(TARGETS ...)` 引用

2. **修改 `rosiwit_slam/launch/fast_lio2.launch.py`**：
   - `FindPackageShare('fast_lio2_slam')` → `FindPackageShare('rosiwit_slam')`
   - `package='fast_lio2_slam'` → `package='rosiwit_slam'`
   - `executable='fast_lio2_slam'` → `executable='fast_lio2_node'`
   - `name='fast_lio2_slam'` → `name='fast_lio2_node'`

3. **创建 `rosiwit_simulator/launch/simulator_slam_demo.launch.py`**：
   - 启动 Gazebo（headless 模式，加载 house.world）
   - 启动 robot_state_publisher（mbot_with_lidar3d_gazebo.xacro）
   - spawn 机器人到 Gazebo
   - 启动 rosiwit_slam 节点（velodyne_vlp16.yaml，use_sim_time=true）
   - 内嵌自动运动节点（发布 /cmd_vel，圆形轨迹）

4. **编译验证**：`colcon build` 全部通过

### ❌ 不做什么（Out of Scope）

- 不修改 `rosiwit_simulator` 包的任何现有代码（仅新增 launch 文件）
- 不修改 `velodyne_vlp16.yaml` 配置文件
- 不修改 `package.xml`（已包含 member_of_group）
- 不添加 RViz 可视化节点
- 不处理 Sophus / GTSAM 缺失的编译降级逻辑（已有 QUIET 查找）
- 不做 CI/CD 流水线更新
- 不做性能优化或代码重构
- 不处理真实硬件适配（仅仿真）

---

## 6. 验收标准

### AC-1: CMake 编译成功
```bash
cd /home/jmq/agent/workspace/projects/rosiwit_ws
rm -rf build/rosiwit_slam install/rosiwit_slam
source /opt/ros/humble/setup.bash && colcon build --packages-select rosiwit_slam
```
**预期**: 返回码 0，零错误，零警告（关于目标名冲突的警告消除）。

### AC-2: fast_lio2.launch.py 独立可用
```bash
ros2 launch rosiwit_slam fast_lio2.launch.py config_file:=velodyne_vlp16.yaml use_sim_time:=true
```
**预期**: 节点 `fast_lio2_node` 成功启动，无 `Package not found` 或 `Executable not found` 错误。

### AC-3: 集成 launch 文件存在且语法正确
```bash
python3 -c "exec(open('/path/to/simulator_slam_demo.launch.py').read())"
```
**预期**: 无语法错误，`generate_launch_description()` 函数可调用。

### AC-4: 集成 launch 启动后节点列表正确
```bash
ros2 launch rosiwit_simulator simulator_slam_demo.launch.py
# 另一终端
ros2 node list
```
**预期**: 至少包含以下节点：
- `/gazebo` (Gazebo 仿真器)
- `/robot_state_publisher`
- `/fast_lio2_node` (SLAM)
- `/auto_move_node` (自动运动，名称可自定义)

### AC-5: 话题数据流通
```bash
ros2 topic hz /velodyne_points   # 预期: ~10Hz
ros2 topic hz /imu               # 预期: ~100-200Hz
ros2 topic hz /cmd_vel           # 预期: ~10Hz
```
**预期**: 三个话题均有稳定的周期性数据输出。

### AC-6: 不修改 simulator 包现有代码
```bash
git diff src/rosiwit_simulator/ --stat  # 仅新增 launch 文件
```
**预期**: simulator 包下仅有 `launch/simulator_slam_demo.launch.py` 一个新增文件，无修改文件。

---

## 7. 技术约束与参考

| 项目 | 约束 |
|------|------|
| ROS 版本 | ROS2 Humble Hawksbill |
| 构建系统 | ament_cmake + colcon |
| 运行环境 | WSL2 Ubuntu 22.04（无显示器，需 headless） |
| Python venv | `/home/jmq/agent/.venv/`（已安装 empy 3.3.4） |
| 参考文件 | `simulator_gazebo_3d.launch.py`（Gazebo+spawn 结构） |
| 参考文件 | `fast_lio2.launch.py`（SLAM 节点启动结构） |
| 参考配置 | `velodyne_vlp16.yaml`（VLP-16 参数 + 话题名） |

---

## 8. 修改清单（传递给架构师/开发者）

| # | 文件 | 修改类型 | 具体变更 |
|---|------|----------|----------|
| 1 | `rosiwit_slam/CMakeLists.txt` | **修改** | L99: `set(NODE_NAME fast_lio2_node)` + `add_executable(${NODE_NAME} ...)`，后续所有 `${PROJECT_NAME}` 可执行目标引用改为 `${NODE_NAME}` |
| 2 | `rosiwit_slam/launch/fast_lio2.launch.py` | **修改** | L38/L45/L46/L47: 4处 `fast_lio2_slam` → `rosiwit_slam`/`fast_lio2_node` |
| 3 | `rosiwit_simulator/launch/simulator_slam_demo.launch.py` | **新建** | 集成 launch: Gazebo(headless) + robot_state_publisher + spawn + SLAM + auto_move |

**总计**: 修改 2 个文件 + 新建 1 个文件，预计代码量 < 200 行。
