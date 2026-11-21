# 持久度体素地图 + 会话子图 方案（2026-11-21）

> 背景：现场出现过"明明定位错了却显示成功"（详见 `docs/ros1_deploy_runbook.md` §6.5）。
> 根因是**只靠单帧局部 GICP fitness 判断**，而堆货区每天都在变：局部子图（1.2 万点）
> 在错误位姿上也能配到 0.80~0.84，被判为定位成功且无法自行脱离。
>
> 本方案给体素加"持久度"维度，并引入"会话子图 + 全局锚定"，让
> **变化的东西不参与定位、当天几何保证不丢、全局位姿有锚**。

## 0. 决策摘要

| # | 决策 | 结论 |
|---|---|---|
| ① | 会话层怎么划 | **C：不划实体子图**。体素带 `w` + `last_seen`，"子图" = 「距我 R 内 且 最近 T 内见过」的查询 |
| ② | 跨天怎么算 | **C：连续新鲜度衰减**（无硬边界）。临时层 τ≈36 h，静态层 τ≈30 d |
| ③ | 锚定频率 | **B：每秒一次 + 质量门槛**（连续 3 帧 `static_support ≥ τ_s` 才更新）+ 恢复时事件补一次 |
| ④ | DEGRADED 怎么报 | **不扩接口**：DEGRADED 映射为 `location_status = 1`（INIT），位姿照常输出，原因只进日志 |

## 1. 数据模型

体素层是**独立 sidecar**，不动 `xzmap` v1 格式（App/其它设备不受影响）：

```c
struct Voxel {          // 0.15 m
    uint8_t  w;         // 持久度 0~255
    uint32_t last_seen; // 秒
};                      // 6 B/体素；稀疏哈希，仓库尺度 5~15 MB
```

**两个维度，不引入第三个**：删除的"连续确认"用 `k_miss` 比例表达（见 §2），
效果等价于 `miss_streak`，但只占两个字段。

挂载点（已核对）：

| 位置 | 用途 |
|---|---|
| `slam_core/map_data.h` → `MapData` | 增加可选的持久层（不进 `xzmap`，由适配层读写 sidecar） |
| `algorithms/localization/maintainer/overlay_map_maintainer.{h,cpp}` | **权重更新的落地位置**；`onLocalized(pose, scan, match, pose_confident)` 已带健康门控 |
| `slam_core/pipeline_types.h` → `Observation` | miss 射线的现成证据源（ray casting） |

## 2. 更新规则

**只在 `LOCALIZED` 且 `fitness ≥ credible_fitness` 时更新**（位姿不可信时写图 = 自我腐蚀）。

| 事件 | 操作 | 常量 |
|---|---|---|
| hit：本帧点落在体素内 | `w = min(255, w + 8)`；`last_seen = now` | `k_hit = 8` |
| miss：**射线穿过**该体素（free-space evidence） | `w = max(0, w − 16)` | `k_miss = 16` |
| FOV 外 / 超量程 / 被遮挡 | **不动** | — |
| `w == 0` | 从图中删除（= 连续 16 次 miss 才能抹掉满权重） | — |

> **必须区分"扫不到"和"没观测"**：只认射线穿过的 free-space，不认"这一帧没在这里看到点"。
> 否则机器人没去的区域、柱子后的墙会在几周内权重归零被删掉。

## 3. 读侧

```text
fresh   = (now - last_seen) < τ                 // 临时 36h / 静态 30d
w_eff   = fresh ? w : 0                         // 读时算，不做全图扫描（ARM 必须懒算）
匹配目标 = { w_eff >= W_match }                  // 只有高持久几何参与 GICP
障碍层   = { w > 0 }                            // 导航只关心"当前占据"
```

- **不改 fitness 定义**：靠"过滤目标"实现"固定物得分高、变化物得分低"，避免重标阈值链
- 新增诊断量 `static_support` = 本帧匹配上的点中 `w_eff ≥ W_match` 的占比

## 4. 会话子图 + 全局锚定

"子图"不是一个实体，而是**一次查询**：

```text
session_submap(pose) = { v | dist(v, pose) < R  &&  now - v.last_seen < T_session }
```

| 量 | 作用 | 初值 |
|---|---|---|
| `R` | 局部半径（只在这里裁点云） | 25 m |
| `T_session` | 会话新鲜度 | 24 h |
| `T_global_session` | 会话层 → 全局的位姿 | 由锚定估计 |

**锚定**：每 ~1 s，用「静态层高权重点 ↔ 会话层」做一次局部配准，得到 `T_gs`
（会话系→全局）。要求连续 3 帧 `static_support ≥ τ_s` 才接受；接受时**限速回灌**：

```text
每帧位姿修正 ≤ 2 cm / 0.5°      // 防止在堆货区出口跳变
pose_global = T_gs * pose_session
```

**进入完全变化的区域**：会话层跟着当天几何走 → 局部不丢；`T_gs` 沿用上一次 →
全局不崩；`static_support` 掉到 `τ_s` 以下 → 状态降为 DEGRADED（不谎报 LOCALIZED）。

## 5. 状态机（不扩接口）

| 状态 | 判据 | 对 App |
|---|---|---|
| `LOCALIZED` | 会话匹配健康 **且** `static_support ≥ τ_s` | `location_status = 0` |
| `DEGRADED`（新，仅内部） | 会话匹配健康，但 `static_support < τ_s` | **`location_status = 1`**（INIT），位姿照发 |
| `LOST` | 会话匹配不健康（连续 `lost_patience`） | `location_status = 2` |

日志（1 Hz）：`[quality] static_support=0.12 w_ratio=0.31 anchor_age=37s state=DEGRADED`。
**不新增话题/消息字段**（`SlamInfo` 是厂商消息，加字段会改 md5，App 会直接收不到）。

## 6. 评分策略可插拔（影子运行）

```cpp
class IMatchScorer {
  virtual Score evaluate(const MatchResult&, const PersistenceView&) = 0;
};
// Coverage          = 现状 (inliers / correspondences)
// PersistenceWeighted = Σ w_i·inlier_i / Σ w_i
// StaticOnly        = 只用 w ≥ W_match 的目标重算
// Composite         = 覆盖率 + 加权 + 一致性
```

配置 `accept.scorer` 选实现，`accept.shadow` 同时算另一个只记日志。
**换 scorer 必须配独立的阈值组**（`accept.scorer_thresholds`），不改现有 0.85/0.80/0.75 ——
这些是绑在"当前 fitness 语义"上的，混用会重演假锁。

## 7. 配置（新增段，默认不改变现有行为）

```yaml
persistence:
  enabled: true
  shadow: true                 # ★A 阶段: 只记录, 不参与决策
  voxel_size: 0.15
  hit_gain: 8
  miss_cost: 16
  tau_temp_h: 36               # 临时层(货堆)
  tau_static_d: 30             # 静态层(墙/柱)
  w_match: 128                 # 参与匹配的门限
  radius_m: 25                 # 会话子图查询半径
  session_h: 24                # 会话新鲜度
  anchor_period_s: 1.0
  anchor_min_static_support: 0.25
  anchor_max_corr_m: 0.02      # 限速回灌
  anchor_max_corr_deg: 0.5
```

阈值链不变：`min_coverage 0.35 < lost 0.75 < credible 0.80 ≤ reloc_accept 0.85`。

## 8. 分阶段实施与验收

| 阶段 | 内容 | 改动面 | 验收指标 |
|---|---|---|---|
| **A**（只记录） | sidecar 体素层 + hit/miss 更新 + 懒衰减 + 导出高权重点云 + `[quality]` 日志。**shadow=true，不参与匹配** | `overlay_map_maintainer`、`map_data.h`、io sidecar、日志 | ① 每天导出 `w_eff≥128` 的点云是否只剩立柱/墙体；② `static_support` 分布；③ 体素总量/内存实测 |
| **B** | 会话子图查询（R/T_session）+ 子图内跟踪 | `local_map/keyframe_submap` 增加第二级目标 | 堆货区整体更换后不 LOST；连续跟踪时长 |
| **C** | 锚定 `T_gs` + 限速回灌 + DEGRADED 状态 | `localization_pipeline` 状态机、`pose_publisher` 映射 | 出堆货区位姿跳变 ≤ 3 cm；`static_support` 与漂移的相关性 |
| **D** | 可插拔 scorer + 独立阈值（shadow 对比） | `acceptMatch` 上层 | 同一段数据上新旧判定差异 |

A 跑一周即可给出 B/C/D 需要的参数（R、τ、τ_s），**不需要动任何决策逻辑**。

## 9. 风险与回滚

| 风险 | 处置 |
|---|---|
| 位姿不可信时写图 → 地图自我腐蚀 | 只在 `LOCALIZED && fitness ≥ credible` 时更新 |
| 未观测区域误衰减 | 只认射线 miss；FOV/遮挡不动 |
| 锚定修正导致位姿跳变 | 限速回灌 + 只在质量门槛满足时更新 |
| DEGRADED 被当成功 | 映射为 `location_status=1`，日志带原因 |
| 层数据写坏 | sidecar 单独文件 + 保存前一版，可整层回滚（不动 `xzmap`） |

## 10. 明确不做

- **区域掩码**：需要人维护，只作为临时补丁，不做进产品
- **改厂商 `SlamInfo`**：会破坏 md5，App 直接失效
- **逐点短暂性概率**：每点/每体素跨天概率模型，ARM 上不划算，且治不了"匹配歧义"
- **替代反光板**：本方案与反光板是**并行**关系；纯货堆区没有不变几何时，权重机制会让定位
  诚实失败（LOST），而不是给出错误位姿 —— 要"能定位"仍需反光柱/立柱提供锚

## 11. 下一步

A 阶段（shadow）落地顺序：

1. `VoxelLayer`（hash voxel + 懒衰减）—— 独立类，先单测
2. `OverlayMapMaintainer::onLocalized` 里接 hit/miss 更新（用现成的 `pose_confident` 门控）
3. sidecar 读写（`map.voxel`）+ 导出高权重点云（离线看效果）
4. `[quality]` 1 Hz 日志（`static_support` / `w_ratio` / 体素数）
5. 影子跑一周，再决定 B/C 的参数
