# 重定位提速（P0）— 实测与改动记录

> 目标（README TODO #2）：冷启动/被搬动/切图后的全局重定位 **p95 < 5 s、召回 > 95%、误锁 < 1%**。
> 本文记录 2026-11-01 这一轮的基线复现、瓶颈定位、改动、实测结果，以及还没做完的部分。

## 1. 基线复现（先证明问题存在）

新增基准工具 `tools/reloc_bench`（core，无 ROS 依赖）：

```bash
reloc_bench <map.pcd> <traj.tum> [--localizer bbs|scan_context|fpfh_ransac|all] \
            [--scenarios 20] [--scan-radius 25] [--submap 30] [--topk 5] \
            [--build-sc-db 200] [--pos-tol 0.5] [--yaw-tol 5] [--noise 0.02] [--drop 0.2]
```

场景构造（可复现，不需要实车）：沿一条轨迹等间隔取 N 个真值位姿 → 在该位姿处从地图裁 ±25 m 的点、
变换到传感器系并加噪声/随机丢点（模拟遮挡与变化）→ 全局重定位从零求解 → 与真值比平移/航向误差。
命中判定 `pos<0.5 m 且 yaw<5°`；报了结果但误差 `>2 m` 记为**误锁**。

数据集（本机可得）：NTU VIRAL eee_03 建图产物 `map.pcd`（**251,878** 点）+ 同一次运行的轨迹（474 位姿），
20 个场景（轨迹等间隔取样）。

基线（`--submap 0`，即改动前"整图 GICP"）：

| localizer | 召回 | 误锁 | p50 | p95 |
|---|---|---|---|---|
| `bbs`（20 场景） | 95% | 0% | 38.2 s | 42.7 s |
| `scan_context`（20 场景） | 100% | 0% | 19.6 s | 55.1 s |
| `fpfh_ransac` | 旧默认（50000 迭代 × 15 轮）**2 个场景跑 15 min 没跑完** | | > 90 s/场景 | |

复现了 README 里"22–45 s"的现场数字。

## 2. 瓶颈（实测拆解）

1. **每个候选都在整图上跑 GICP**：`localizeTopK` 给 `3*top_k`（默认 15）个候选各做一次
   `setInputTarget(全图)` + GICP + 覆盖打分。251k 点地图单次 ~0.7 s，15 个 ≈ 10 s；
   生产 110 万点地图就是 30–45 s。**这是随地图线性放大的主因**。
2. **`fpfh_ransac` 在整图上做 3D 特征 + RANSAC**：`3*top_k` 轮、每轮 `fpfh_max_iter=50000`，
   每轮收敛后还要一次整图 GICP。地图 FPFH 建库（~45 s）已在后台线程，不计入重定位延时，但搜索本身仍然极慢。
3. **`bbs` 的分支定界**：金字塔每层都按 `theta_bins(60) × cells` 调 `scoreNode`，每次遍历全部 2D 扫描点 →
   单场景 ~15 s，与候选数几乎无关（`topk=1` 仍要 15 s）。

## 3. 改动

| 改动 | 位置 | 效果 |
|---|---|---|
| **候选处裁局部子图** `reloc_submap_radius`（默认 30 m，0=旧行为） | `localization_utils` 新增 `cropRadius()`（KD-tree 半径搜索 + 点数上限）；`scan_context`/`bbs`/`fpfh_ransac` 的 GICP target、管线二级精配都用它 | 单候选 GICP 从 ~0.7 s 降到几十 ms |
| **ScanContext 候选级联**：先用候选位姿直接打覆盖分（0.8 m 容差，2000 次 1-NN ≈ 10 ms）淘汰错误候选，只对前 3 个做全精度 GICP；高置信（≥0.88）且已有 2 个候选即提前结束 | `scan_context_global_localizer.cpp` | p50 19.6 s → **0.95 s** |
| KD-tree 只建一次（原来每次 `localizeTopK` 重建） | 同上 | 每次省 30–80 ms |
| 精配源/目标点数上限（0.4 m 源、1 万点目标） | 同上 | 把最坏耗时从 27 s 压到 4 s |
| 管线二级精配按候选裁子图（原先把全图设成 matcher target） | `localization_pipeline.cpp` | 后处理阶段不再碰整图 |
| `fpfh_ransac` 默认收一收：`fpfh_max_iter 50000→5000`、轮数 `3k→k+2`、新增 `reloc_budget_ms`（默认 5000） | `localization_stage_types.h`、`fpfh_ransac_global_localizer.cpp` | 无 `.scdb` 兜底路径从"分钟级"降到"预算内返回" |
| 记一笔失败的尝试 | `bbs_global_localizer.cpp` | 粗层只用 1/10 点打分：提速 1.6x 但**召回 100%→12.5%**，已回退（注释留在代码里） |

## 4. 结果（同一地图 / 同一批 20 场景）

| localizer | 召回 | 误锁 | p50 | p95 | max |
|---|---|---|---|---|---|
| `bbs`（整图，基线） | 95% | 0% | 38200 ms | 42700 ms | 60900 ms |
| `bbs`（现状，6 场景） | 100% | 0% | 29179 ms | 31294 ms | 31294 ms |
| `scan_context`（整图，基线） | 100% | 0% | 19569 ms | 55104 ms | 57926 ms |
| `scan_context` + 局部子图 | 100% | 0% | 10667 ms | 37631 ms | 40580 ms |
| `scan_context` + 两级（粗 GICP 版） | 100% | 0% | 1024 ms | 1667 ms | 1758 ms |
| `scan_context` + 覆盖门控（精配无上限） | 100% | 0% | 1253 ms | 10269 ms | 26932 ms |
| **`scan_context` + 覆盖门控 + 精配预算 3（现状）** | **100%** | **0%** | **952 ms** | **3383 ms** | **3927 ms** |

> 基线两次是用默认线程数（20 核）跑的，最终版为可对比固定 `OMP_NUM_THREADS=4`，
> 所以上表的提速是**偏保守**的估计。

逐场景耗时（最终版，ms）：
`952 1125 1054 992 818 840 722 682 628 667 604 706 1232 687 1745 842 3927 1681 1839 3383`

结论：**p95 42.7 s（BBS 基线）/ 55.1 s（SC 基线）→ 3.38 s，召回 100%、误锁 0%**，
达到"p95 < 5 s、召回 > 95%、误锁 < 1%"的验收线。

### 用法

```bash
# 建图结束 map.pcd 旁边会有 map.pcd.scdb（saveMap 里 ScanContext 建库）
# 配置: mapping.pipeline.relocalizer: scan_context
reloc_bench map.pcd traj.tum --localizer scan_context --build-sc-db 200 --scenarios 20
```

> `--build-sc-db N` 只给"没有 .scdb 的离线地图"造一份，等价于建图时落盘的库。
> `bbs` 需要 `bbs_mount_r`（base<-lidar 安装旋转），否则按雷达系投影。

## 5. 还没做完（要真正关掉 TODO #2 还差这些）

1. **现场 20 个真实场景**（冷启动、被搬动 1–5 m、光照/堆货变化）。本次是"地图内合成扫描"，
   验证的是检索+精配的**速度与抗自相似能力**，没有覆盖"地图与现场不一致"。
2. **时序确认**：接受条件仍是单帧分数 + 次优差，还没做"连续 2–3 帧一致才接受"。
3. **描述子增量更新**：叠加层变化后 `.scdb` 需要重建（`updateEnvironment` 还没接 ScanContext 库）。
4. **失败原因码**：`localizeTopK` 只返回 0；管线日志有 `global refine weak/ambiguous`，
   但没有结构化原因枚举。
5. **110 万点生产地图复测**：子图裁剪后耗时与整图点数基本无关（目标点数被限制在 1 万），
   预计一致，但需要在现场包上复测一次。
