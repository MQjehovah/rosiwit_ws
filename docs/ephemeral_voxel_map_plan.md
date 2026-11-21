# 可信度地图 + 滚动子图 定位方案（2026-11-21，v2）

> 背景：现场出现过"明明定位错了却显示成功"（`docs/ros1_deploy_runbook.md` §6.5）。
> 根因：只靠单帧局部 GICP fitness，而堆货区每天在变 —— 局部子图（1.2 万点）在错误位姿
> 上也能配到 0.80~0.84，被判成定位成功且无法自行脱离。
>
> v2 修正了两处设计错误：**不做时间衰减**（存在性只由观测证据决定）、
> **子图不是会话物件**（一个、以机器人为中心、一直在）。

## 0. 决策摘要

| # | 决策 | 结论 |
|---|---|---|
| ① | 子图形态 | **一个**机器人中心滚动子图（odom 系，半径 R，覆盖式更新） |
| ② | 新鲜度/跨天 | **不做时间衰减**。存在性只由 hit/miss 证据决定 |
| ③ | 锚定 | `map→odom` 只在「高可信几何可见」时更新，限速回灌；否则冻结 |
| ④ | DEGRADED 怎么报 | **不扩接口**：`location_status = 1`（INIT），位姿照发 |

## 0.5 与现有地图更新（`map_maintainer=overlay`）的关系

**本方案不是新造一套"地图更新"，而是给现有 overlay 做三处升级。**
`config/default.yaml` 里本来就是：

```yaml
map_maintainer: overlay    # 工厂: 非破坏性叠加(吸收半静态堆货)
```

设备日志确认在跑：`[LocPipeline] ... map_maintainer=overlay`。

### 已经有的（直接复用）

| 现有实现 | 对应本方案 |
|---|---|
| `base` 冻结 + overlay 叠加 | 静态底图 / 动态层 |
| `onLocalized(..., pose_confident)` | 只在位姿可信时写图 |
| `rayCarve(pose, scan_body)` | **射线 miss 证据**（不是"没看到就衰减"） |
| `min_confirm` | 变化确认去抖 |
| `delta_min`（>0.3 m 才沉淀） | 只吸收新结构 |
| `override_base` | 处理"被移除 / 被替换" |
| `persist_hits` | 天然的瞬态过滤（≈ 临时货物可信度低） |
| `max_voxels` / `revision()` / `popOverlayDelta` / `save` | 容量上限 / 缓存失效 / 增量落盘 / 持久化 |
| `decay_frames`（默认 0） | **不使用**：存在性只由证据决定 |

### 本方案真正新增的（3 条）

| 新增 | 现有做不到的原因 |
|---|---|
| ① **`w` 暴露出来做锚定门控** | `persist_hits` 的计数只用于过滤瞬态，没有"哪些几何可当锚"的概念 |
| ② **滚动子图（odom 系）做跟踪目标** | 现在配准目标 = 全局局部 + overlay，**全在 map 系**；`map→odom` 一错，overlay 跟着错 |
| ③ **`map→odom` 门控 / 冻结 + DEGRADED** | 现在每帧直接由 GICP 算，没有"锚冻结"和降级语义 |

### 一个必须堵住的漏洞

现有 `pose_confident` 是**基于 fitness** 的（≥ 阈值）。本次现场假锁的 fitness 是 0.80~0.84 ——
**`pose_confident` 为真**，于是 overlay 会把错误位姿下的几何写进 map 系地图，等于**强化错误**。

因此：

- 写图判据从"`fitness ≥ 阈值`"升级为"`fitness ≥ 阈值` **且** `static_support ≥ τ_s`"
- 这也是本方案引入 `w` 的根本原因：fitness 只回答"配得多好"，`static_support` 才回答
  "我是**靠什么**配上的"

## 1. 两个结构，职责严格分开

| 结构 | 内容 | 谁用 | 更新 |
|---|---|---|---|
| **全局图** | `{w}`（+ 诊断用 `last_seen`） | **只用于锚定**：挑高 `w` 几何算 `map→odom` | 证据驱动（hit/miss） |
| **子图** | odom 系下机器人 ±R 的点/体素缓冲 | **只用于跟踪**：局部 GICP 目标 | 覆盖式，每帧 |

关键点：

- **`w` 只在全局图里存在**，语义是"这块几何能不能当锚"，不是"这块几何新不新"
- **子图里没有 `w`**：它的内容永远是"当前看到的样子"，所以不需要置信度
- 子图**不需要** `session_id`、不需要"建/关"状态 —— 它一直在，走哪跟哪

这正是我们现有的坐标系结构：

```
pose_global = T_map_odom × pose_odom
                   ▲              ▲
                   │              └── LIO 里程计（已有，局部连续）
                   └── 全局匹配产物（已有，但要加门控）
```

## 2. 全局图的置信度 `w`

```c
struct Voxel {
    uint8_t  w;          // 可信度 0~255
    uint32_t last_seen;  // 仅诊断/事后分析, **不参与任何决策**
};
```

| 事件 | 操作 |
|---|---|
| hit：本帧点落在体素内，**且位姿健康** | `w = min(255, w + 8)`；`last_seen = now` |
| miss：**射线穿过**该体素（free-space evidence） | `w = max(0, w − 8)` |
| FOV 外 / 超量程 / 被遮挡 | **不动** |
| `w == 0` | 删除（满权重 → 删除 = 32 次连续 miss ≈ 3 s） |

**只在 `LOCALIZED` 或 `DEGRADED` 且局部匹配健康时更新**；`LOST` 期间只读不写
（位姿不可信时写图 = 自我腐蚀）。

> **为什么没有时间衰减**：`w` 衡量的是"观测一致性"，时间不是证据。
> 三天没去的区域保留原样是**正确**的（没有证据说它变了）；搬走的货靠射线 miss 删。
> 高 `w` 的旧货堆在搬走后的几秒内仍存在，但**不影响跟踪**（见 §3：跟踪用子图）。

> **`w` 分不出"临时"**：一堆货连续观测 100 次，`w` 同样到 255。这不影响正确性 ——
> 它在的时候用它匹配是对的；它搬走时靠子图接管 + miss 掉权。
> 若将来要"临时物即使在场也排后"，再加 `flip`（出现/消失次数），属可选增强。

## 3. 滚动子图（唯一，机器人中心）

```cpp
struct RollingSubmap {
    VoxelLayer voxels;          // odom 系, 覆盖式
    Vec3       center;          // 上次更新时的机器人位置
};
```

每帧更新（**不需要位姿可信**，因为它只记录"我看到了什么"）：

```
本帧点落到的体素      → 写入占据
射线穿过但没点的体素  → 清空
超出 R 的体素         → 丢弃
```

`R ≈ 25 m`：内存恒定（稀疏体素，几万点），走哪跟哪。

**跟踪**：当前帧 ↔ 子图（odom 系）做局部 GICP → `pose_odom`。

> 为什么它能"1 帧跟上变化"：子图是**覆盖式**的，不是积分。
> 货堆搬走的下一帧，射线穿过原位置 → 子图里就是空的，不需要等任何权重下降。

## 4. 锚定：`map→odom` 的门控

每帧算 `static_support` = 当前匹配上的点中，落在**全局高 w 体素**里的占比。

```
static_support ≥ τ_s   → 允许更新 map→odom（限速：每次 ≤2 cm / 0.5°）
static_support <  τ_s   → 冻结 map→odom（沿用最后有效值），anchor_age += dt
```

**大幅修正不静默应用**：若一次需要的修正 > 0.5 m，视为"重定位事件"而非"平滑拉回" ——
记录日志，保持 DEGRADED，等操作员/全局重定位确认。避免在堆货区出口把位姿硬拽过去。

## 5. 状态机（只看"锚"和"局部匹配"）

| 状态 | 判据 | 输出 |
|---|---|---|
| `LOCALIZED` | 局部匹配健康 **且** 锚在更新（`static_support ≥ τ_s`） | `location_status = 0` |
| `DEGRADED` | 局部匹配健康，但锚冻结（`anchor_age` 增长） | **`location_status = 1`**，位姿照发 |
| `LOST` | 局部匹配不健康（连续 `lost_patience`） | `location_status = 2` |

"突然进入堆满货的区域" = **子图继续跟 + 锚冻结 + DEGRADED**；
"出来看到柱子" = 锚恢复更新 → `LOCALIZED`。全程没有"建子图/关子图"这个动作。

日志（1 Hz）：
```
[quality] static_support=0.12 anchor_age=37s submap_pts=18432 state=DEGRADED
```

## 6. 配置

```yaml
persistence:
  enabled: true
  shadow: true                 # A 阶段: 只记录 w, 不参与锚定门控
  voxel_size: 0.15
  hit_gain: 8
  miss_cost: 8
  w_anchor: 128                # 可当锚的最低可信度
  submap_radius_m: 25          # 滚动子图半径

anchor:
  period_s: 1.0                # 锚定尝试周期
  min_static_support: 0.25     # 允许更新 map→odom 的门槛
  max_corr_m: 0.02             # 单次限速
  max_corr_deg: 0.5
  relocalize_jump_m: 0.5       # 超过此修正 -> 视为重定位事件, 不静默应用
```

阈值链保持不变：`min_coverage 0.35 < lost 0.75 < credible 0.80 ≤ reloc_accept 0.85`。

## 7. 分阶段实施与验收

| 阶段 | 内容 | 改动面 | 验收 |
|---|---|---|---|
| **A**（只记录） | **扩展 overlay 的体素**：hit/miss 维护 `w`（复用现成的 `rayCarve`），暴露 `static_support`，加 `[quality]` 日志，导出 `w≥128` 点云；`shadow=true` 不参与锚定门控 | `overlay_map_maintainer.{h,cpp}`（原有体素表加一列 `w`）、sidecar 读写 | ①导出的高 w 点云是否只剩立柱/墙；②`static_support` 分布；③体素总量/内存实测 |
| **B** | 跟踪目标切到子图（odom 系），保留全局匹配只做锚 | `localization_pipeline` 的匹配入口 | 堆货区整体更换后不 LOST；连续跟踪时长 |
| **C** | 锚定门控 + 限速回灌 + DEGRADED 状态 | `map→odom` 更新处、`pose_publisher` 状态映射 | 出口处位姿跳变 ≤3 cm；`static_support` 与漂移的相关性 |
| **D** | 可选：`flip` 字段、评分策略可插拔（影子对比） | `acceptMatch` 上层 | 同一段数据新旧判定差异 |

A 跑一周即可定量出 `τ_s`、`R`、`w_anchor` 的取值。

## 8. 代码落点（已核对）

| 位置 | 用途 |
|---|---|
| `algorithms/localization/maintainer/overlay_map_maintainer.{h,cpp}` | **主战场**：体素表加 `w`；复用 `rayCarve` 做 miss；`onLocalized(..., pose_confident)` 是现成的写入门控（需再加 `static_support` 条件） |
| `slam_core/map_data.h` → `MapData` | 挂 sidecar 持久层（不动 `xzmap` v1 格式） |
| `slam_core/pipeline_types.h` → `Observation` | miss 射线的现成证据源 |
| 新增 `algorithms/localization/local_map/rolling_submap.{h,cpp}` | 机器人中心子图（覆盖式） |
| `localization_pipeline` 匹配入口 | 目标从"全局裁剪"改为"子图" |
| `pose_publisher.cpp` / `sendTransform` / `broadcastMapOdom` | `map→odom` 门控与冻结（冻结逻辑已存在） |

## 9. 风险与回滚

| 风险 | 处置 |
|---|---|
| 位姿不可信时写 `w` | 只在匹配健康时更新；`LOST` 期间只读 |
| **假锁被当成"位姿可信"写进 overlay** | 写入条件加 `static_support ≥ τ_s`；只靠 fitness 不够（现场 0.80~0.84 的假锁会通过） |
| 未观测区域被误删 | 只认射线 miss；FOV/遮挡不动 |
| 锚冻结期间漂移累积 | `anchor_age` 暴露 + DEGRADED；超过累计位移/时长上限如实降级 |
| 出口大幅修正 | >0.5 m 视为重定位事件，不静默拉回 |
| 层数据写坏 | sidecar 独立文件 + 保留上一版，可整层回滚（不动 `xzmap`） |

## 10. 明确不做（含原因）

| 不做 | 原因 |
|---|---|
| 时间衰减 / `fresh` | "没看到就过期"违背证据原则；高 w 旧货堆不影响跟踪 |
| 区域掩码 | 需要人维护，且货堆边界每天都在动 |
| 区域级变更检测 | 子图覆盖式更新已经能 1 帧跟上，不需要额外仲裁 |
| `session_id` / 子图建关状态 | 子图是滚动的，不存在"会话" |
| 改厂商 `SlamInfo` 加字段 | 改 md5，App 直接收不到消息 |
| 替代反光板 | 并行关系。纯货堆区没有高 w 几何时，锚只能冻结（DEGRADED），要"能锚"仍需立柱/反光柱 |

## 11. 一句话总结

> **全局图回答"哪儿可以当锚"（`w`）；子图回答"我现在周围是什么"（覆盖式、odom 系）；
> `map→odom` 是两者之间的唯一交接点 —— 看得见锚就修正，看不见就冻结并如实降级。**
