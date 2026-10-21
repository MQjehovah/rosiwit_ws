# DWA / MPPI footprint-aware A/B 报告（2026-10-21）

> 机器人：192.168.21.166，Ubuntu 18.04 aarch64 / ROS Melodic。
> 车体：长方形，`footprint = [-0.25,-0.33] [0.45,-0.33] [0.55,-0.20] [0.55,0.20] [0.45,0.33] [-0.25,0.33]`。
> 目的：回答两个问题
> 1. 给控制器加 footprint 代价是否值得（A）；
> 2. DWA 与 MPPI 在长方形车体下谁更适合（B）。

## 1. 实现（A）

核心新增 `nav_core/footprint.{hpp,cpp}`：

* `sampleFootprint()`：把多边形离散成“顶点 + 边加密(默认 0.05m) + 中心”；
* `footprintCost()`：把采样点按位姿旋转/平移后查 costmap，取**最大**代价；
* 没有 footprint 时退化为原来的“只看中心点”，旧行为不变。

接入：

* `ControllerConfig` 新增 `footprint` / `footprint_sample_spacing`；
* **DWA**：`simulate()` 的 costmap 与障碍物距离都改成按 footprint 采样点计算；
* **MPPI**：rollout 的 `localCostAt(p)` 改成 `footprintCost(...)`；
* 节点 `footprint_aware_control=true` 时：
  * 把实车 footprint 传给控制器；
  * 给控制器注入**未膨胀**的局部 costmap（否则等于把车体算两遍）；
  * 全局规划仍用 `robot_radius=0.39 / inflation_radius=0.62` 的圆形膨胀。
* **默认关闭**（`footprint_aware_control: false`），验证后再开。

单测：`test/test_footprint.cpp` 9/9；核心全量 **12/12** 通过。
此外，MPPI + footprint 在隔离 master 的 16 项接口冒烟 **16/16** 通过。
真机包编译（`cmake --build`）通过。

## 2. 场景（B）

合成地图 12m×8m，0.05m 栅格，8m 直线路径，真实 footprint：

* 右侧障碍 `x∈[2.4,2.8], y∈[0.25,0.55]`；
* 左侧障碍 `x∈[5.2,5.6], y∈[-0.55,-0.25]`；
* `x=4.0~4.2` 处 1.5m 宽门（车宽 0.66m）。

评价用**真实 footprint** 对**原始障碍图**做离线检查（与控制器看到的图相同），
统计碰撞步数、最小间隙、横向跟踪误差 RMS、平均速度、单次控制耗时。
`max_accel_theta` 默认用实车的 0.2，另测 0.3/0.4/0.5/0.6/0.8。

## 3. 结果（ARM 实机，10Hz 预算 100ms）

### 3.1 完整场景（two blocks + gate）

| 控制器 | 到达 | 碰撞步数 | 最小间隙 | 横向RMS | 平均速度 | 单次耗时 |
|---|---:|---:|---:|---:|---:|---:|
| DWA point | ✅ | **28** | 0.00 | 0.003 | 0.706 | 0.10–0.53 ms |
| MPPI point | ✅ | **30** | 0.00 | 0.033 | 0.621 | 1.95 ms |
| DWA + footprint (wacc 0.2) | ❌ 800步 | 0 | 0.05 | 1.203 | 0.312 | 0.84 ms |
| DWA + footprint (wacc 0.8) | ❌ 800步 | 14 | 0.00 | 2.152 | 0.666 | 0.80 ms |
| MPPI + footprint (wacc 0.2) | ❌ 800步 | 6 | 0.00 | 0.282 | 0.058 | 2.84 ms |
| **MPPI + footprint (wacc 0.3)** | **✅ 132步** | **0** | **0.05** | **0.180** | **0.611** | **3.59 ms** |
| MPPI + footprint (wacc 0.4) | ✅ 128步 | 0 | 0.05 | 0.092 | 0.620 | 3.62 ms |
| MPPI + footprint (wacc 0.5) | ✅ 128步 | 0 | 0.05 | 0.076 | 0.619 | 3.58 ms |
| MPPI + footprint (wacc 0.8) | ✅ 128步 | 0 | 0.05 | 0.066 | 0.619 | 3.58 ms |

### 3.2 空地图（验证 footprint 不改变正常跟踪）

| 控制器 | 到达 | 碰撞 | 横向RMS | 单次耗时 |
|---|---:|---:|---:|---:|
| DWA point | ✅ | 0 | 0.003 | 0.11 ms |
| DWA + footprint | ✅ | 0 | 0.003 | 0.85 ms |
| MPPI point | ✅ | 0 | 0.033 | 1.95 ms |
| MPPI + footprint | ✅ | 0 | 0.033 | 3.61 ms |

空地图下 footprint 版与点模型输出几乎一致，说明代价函数没有引入额外偏置。

### 3.3 关键现象

1. **点模型确实会漏掉长方形的角**：直线路径段上，点模型完成导航，但真实
   footprint 有 **28–30 个控制步压在障碍上**（车角/车侧擦过 0.25–0.55m 的凸出物）。
2. **DWA + footprint 会绕圈**：虽然 0 碰撞，但避开第一个障碍后没有回到路径，
   `w` 持续饱和，800 步也未到终点；把角加速度提高到 0.8 反而在完整场景撞了 14 步。
   DWA 的局部极小值 + 低角加速度问题比 footprint 代价本身更突出。
3. **MPPI + footprint 对 `max_accel_theta` 很敏感**：
   * 0.2（当前实车 `accel_lim_w`）：避障后速度掉到 0.058m/s，800 步未到终点；
   * **0.3**：刚好能完成，0 碰撞，横向 RMS 0.18；
   * 0.4–0.8：横向 RMS 0.09→0.066，完成步数稳定 128。
4. **算力不是问题**：MPPI+footprint 在 ARM 上 **3.6ms/次**，占 10Hz 预算的 3.6%。

## 4. 结论与建议

* **A 的结论**：footprint 代价值得做，且开销可接受（DWA +0.7ms，MPPI +1.7ms）。
  但它必须配合能“从避障偏移中恢复”的控制器；直接给现版 DWA 打开会绕圈。
* **B 的结论**：长方形车体下，**MPPI + footprint** 是可行组合；
  DWA 不适合直接叠加 footprint。
* **推荐参数**（先隔离/低速验证，再切实车）：

  ```yaml
  controller_name: "mppi"
  footprint_aware_control: true
  footprint_sample_spacing: 0.05
  max_accel_theta: 0.3      # 实车 accel_lim_w=0.2, max_w_inc=0.3; 先按 0.3
  max_velocity_x: 0.8
  max_velocity_theta: 0.5
  ```

  0.3 是 A/B 的临界值；建议现场先用 **0.4** 留一点余量，再按体感降。
  实车 `accel_lim_w=0.2` 与 `max_w_inc=0.3` 两个旧参数本身不完全一致，
  上机前要确认底盘实际允许的角加速度。

## 5. 已知边界

* 合成场景，无动态障碍；需要在真实地图/录包上再做一次 A/B。
* footprint 采样是“边界 + 中心”，完全落在车体内部、又不碰边界的薄障碍
  仍可能漏检（安全层 footprint 兜底）。
* footprint-aware 时控制器用**原始局部 costmap**，全局规划仍是 0.39/0.62
  圆形膨胀；两者语义不同，现场要观察窄通道通过性。
* MPPI 的 `H=20/K=160/lambda/sigma/权重` 仍是硬编码，没有 ROS 参数入口。
* **实车已按推荐值上车（2026-10-21）**：
  `controller_name=mppi`、`footprint_aware_control=true`、`max_accel_theta=0.4`。
  仍属现场验证阶段；回退只需把这两个参数改回
  `controller_name=dwa` / `footprint_aware_control=false` 并重载参数/重启节点。

## 6. 复现

```bash
# ARM 上
cmake -S ~/rosiwit_ws/src/rosiwit_navigation -B /tmp/nav_core_ab -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/nav_core_ab -j8
g++ -std=c++17 -O2 -pthread -I ~/rosiwit_ws/src/rosiwit_navigation/include \
    ~/rosiwit_ws/src/rosiwit_navigation/tools/bench_footprint_ab.cpp \
    /tmp/nav_core_ab/librosiwit_navigation.a -o ~/bench_ab
~/bench_ab full       # 也支持 empty / one_block / no_gate
```
