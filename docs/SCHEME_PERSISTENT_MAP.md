# 持久度底图方案（Persistent Map）—— 总纲 v1

> 2026-11-25 整合定稿。整合来源：`ephemeral_voxel_map_plan.md`(v2 滚动子图+锚定)、
> `anchor_submap_design.md`(车底贴地/桥接/zip-up)、`LOCALIZATION_ANCHOR_2026-11-25.md`
> (方向可观测性实验)、ELite (ICRA 2025, dongjae0107/ELite) 论文与代码。
> **后续围绕本文档沟通与迭代**；各来源文档保留作背景与实验记录。

---

## 0. 要解决的问题（现场实证）

| 问题 | 实证 |
|---|---|
| 底图某区域完全变化（堆货/装修）后位姿漂走且修不回来 | 错 27 m 时 fitness 仍 0.96（删点后自匹配）|
| 评分失真：删点/双层冗余让错位姿也高分 | 关 dynamic_filter 后 0.96→0.12，vis 167→4421 |
| 底图小错永远修不了 | `delta_min` 门控：墙挪 0.5 m 这类"底图小错"无法修正 |
| 慢走的人留下轨迹簇 | `novel_min_cluster` 二值门控挡不住 |
| 子图锚偏 → 冻结链条 → 整体丢失 | LOCALIZATION_ANCHOR §2（可复现，是结果不是原因） |
| 初值 z=0 → 机器人"在地面下" ~1.2 m | 车底与 z=0 未对齐（传感器高 0.984 m） |

## 1. 核心思想（来自 ELite，映射到我们的架构）

把世界概率化为**两阶段持久度（ephemerality）**，代替"静态/动态"二元分类：

```
w (0~255) = 体素的持久度证据
  高 w (≥128)  → 已确认的静态结构，可当匹配目标/锚
  低 w (>0)    → 新出现、未确认的结构（观察中）
  w = 0 / 不存在 → 无证据（自由空间或从未见过）

局部瞬态 (ELite ε_l)：单次会话内看它是动的（行驶的车）
全局瞬态 (ELite ε_g)：跨会话看它是暂态的（反复停同一位置的车 vs 永久建筑）
→ w 的增减速率按"计票单位"(frame/view/session/day) 控制，天然表达两个时间尺度
```

## 2. 三张图与一条状态机

```
map.pcd            原始底图备份（稠密，如 6cm）。只读，永不修改（回滚随时可行）。
map.overlay.pcd    工作图 = 底图(降采样 0.3) + overlay，**每个点 intensity = w**。
                   **匹配目标的数据源**；w==0 的点不写盘（跨会话删除生效）。
    每 1h 且几何/权重有变化 → 后台全量落盘（无变化不写盘）。只有这一张工作图，
    没有增量日志文件；旧版 `map.overlay.map` / `map.pcd.overlay.pcd` 启动时自动清理。
子图 (session)     机器人中心滚动子图（odom 系，覆盖式更新）。
    定位成功即开建、一直在建；只在"工作图锚不住"时作为匹配目标顶上。

状态机（判据 = fitness_w 与 static_support，见 §4）：
  LOCALIZED : fitness_w ≥ 0.75 且 static_support ≥ τ_s   → 目标 = 工作图
  DEGRADED  : fitness_w ≥ 0.75 但 static_support < τ_s   → 目标 = 子图(桥接), 锚冻结
  LOST      : fitness_w < 0.75 连续 lost_patience 帧      → 重定位
```

## 3. w 账本（唯一权威规则）

| 事件 | 操作 | 说明 |
|---|---|---|
| 扫描端点命中体素（位姿可信） | `w = min(255, w+8)`；刷新 last_seen | 持久结构累积证据 |
| 射线穿过体素（自由空间） | `w = max(0, w−8)`；连续 `miss_confirm` 次确认 | 搬走的货：射线穿过 → 衰减 → 删除 |
| 扫描点离最近体素 > `delta`(0.3 m) | novel 候选：按计票单位累加票 + 目标度 γ | 新结构（新墙/新货架） |
| 候选票 ≥ `persist_hits` 且簇 ≥ `min_cluster` | **升格为体素**（w = 初值 8） | 未确认态，继续累积 |
| FOV 外 / 超量程 / 被遮挡 | **不动** | 没有证据不衰减（无时间衰减） |

**目标度 γ**（ELite 式 8）：`γ = 局部密度^(1/3)`，归一化。γ 高的簇（家具/货架）
计票快、优先升格；γ 低的稀疏簇（人）慢慢衰减——解决"慢走的人留下轨迹簇"。

**比 ELite 更进一步**（它的 gap）：底图体素**也带 w**（初始 128）。这样
"墙挪了 0.5 m"= 旧位置 w 衰减 + 新位置 novel 升格，**底图小错自然修正**；
而 ELite 的底图冻结、只收远离底图的点，修不了底图小错。

## 4. 匹配与锚定（两个用途，一套数据）

```
匹配目标 = 工作图中位姿附近 R 内的体素
    可选过滤: w ≥ w_match_min (低 w 瞬态不参与) —— ELite 式 4 的离散化
锚定判据 static_support = 本帧内点中落在 w≥128 体素的占比
    ≥ τ_s → 允许 map→odom 更新（限速 ≤2cm/0.5°）→ LOCALIZED
    < τ_s → 冻结 map→odom（沿用最后有效值）→ DEGRADED（位姿照发）
    连续 lost_patience 帧 fitness_w < 0.75 → LOST → 重定位
大幅修正 > 0.5 m → 按"重定位事件"处理，不静默拉回
```

- `dynamic_filter` 保持关闭：删点会美化评分；过滤改由 **w 阈值**承担
  （瞬态体素 w 低，天然不进高分匹配）。
- 走廊（沿向平移不可观测）：static_support 仍会虚高 → 按 LOCALIZATION_ANCHOR
  实测，靠 `max_keyframes` 加宽（24→40）+ odom 打滑判据兜底，**不强切子图**。

## 5. 与 ELite 的对照

| ELite | 我们的对应 | 状态 |
|---|---|---|
| 局部 ephemeral ε_l（会话内） | 子图（覆盖式，无 w） | ✅ 已有 |
| 持久底图 + 全局 ephemeral ε_g（跨会话） | **单一工作图**：0.3 底图与 overlay 同一账本，点+`intensity=w` | ✅ 2026-11-25 落地 |
| 底图小错由证据修正（地图不是冻结的） | 底图体素 w=128 → 射线穿过递减到 0 → 目标剔除 + 不落盘 | ✅ 2026-11-25 落地 |
| 式 5 距离衰减 hit/miss | 定长 ±8（`persistency_hit_gain`） | ⏳ 后续移植（P1） |
| 式 8 目标度 γ | `min_cluster` 二值 | ⏳ 后续移植（P2/T6） |
| 式 4 ε_g 加权 GICP | `target_w_min` 过滤（离散化） | ⏳ 后续移植（P4） |
| 会话批量合并（五分类） | 逐帧在线 + 每 1h 全量落盘 | 在线是我们的硬需求 |
| 双向 zip-up 会话对齐 | 桥接重锚 zip-up（单点） | ⏳ 段级扩展（P5） |
| —（无实时定位回路） | cov_base/static_support 锚定 + 子图桥接 | 我们独有 ✅ |

**对齐说明**：ELite 的"持久地图不是冻结的，而是带可信度、由 hit/miss 证据持续修正"这一点，
现在体现在我们的单一工作图上——底图不再是一份只读的死图，而是和 overlay 同处一个 w 账本，
墙挪了/拆了会被自由空间证据自然吸收。剩余差距是**证据的加权方式**（P1/P2）和
**地图与配准的耦合方式**（P4/P5），这些都是可后置移植的策略，不需要再改文件模型。

## 6. 落地状态与路线

### 已落地（0.2.16，设备在跑）
- 车底贴地：初值/输出按 `T(base_footprint←lidar)` 互逆换算（z=0=车底）
- 工作图 `map.overlay.pcd`（0.3 底图+overlay，点+intensity=w，单文件）：加载/1h 全量落盘/无变化不写；旧版残留启动时自动清理
- `dynamic_filter: false`：评分诚实
- overlay 计票/雕刻机制（rayCarve/observeHits/collectNovel/decay）

### 待办（**以本文档为准**，多路并行以此收敛；2026-11-25 决议）

| # | 事项 | 状态 | 说明 |
|---|---|---|---|
| T0 | 0.2.18 装机验证四步链路 | 已构建 | 贴地/锚定/桥接/zip-up（§2 状态机） |
| T1 | **反光板绝对约束** | 待开发 | 优先级最高（用户 2026-11-25 指定）。`onAbsolute` 入口已就绪；缺：强度阈值聚类检测节点 + 反光板地图配置 + 现场部署。完全变化场景的工业标准解 |
| T2 | **overlay w 加权匹配**（固定物体权重更大，忽略动态） | 待开发 | overlay 计票已能区分固定/动态（固定结构 w 累积、动态衰减）；匹配目标按 w 过滤/加权即可——低 w 瞬态不参与，高 w 固定结构主导匹配。ELite 式 4 的离散化 |
| T3 | **子图桥接 + 周期重锚 + zip-up** | 0.2.18 已实现，待验证 | 弱锚切子图（不重建）；每 50 帧对 overlay 探测；恢复后 zip-up 修正子图漂移（§4） |
| T4 | `anchorOk` → `static_support`/`cov_base` 口径统一 | 待开发 | 去自引用残余（对含子图目标算的锚质量不可信） |
| T5 | 长走廊实测 + `max_keyframes` A/B（24 vs 40） | 现场半天 | cov_base 虚高风险；实测后再定 |
| T6 | ELite P1/P3（距离加权贝叶斯 hit/miss、overlap 分支） | 二期 | 底图小错可修（墙挪 0.5 m） |

### 决议（2026-11-25，**以本文档为准**）

1. **现场配置收敛**：另一条工作流将现场改为 `map_maintainer: none`、
   `session_map.enabled: false`——与本方案冲突。**决议：现场按本方案收敛**
   （`map_maintainer: overlay` + `session_map.enabled: true`），否则计票雕刻/桥接
   均不生效，本方案只剩 w 过滤匹配一项可用。
2. **另一条工作流的交付物全部保留**：方向可观测性诊断（`evaluateObservability` +
   `[ObsProbe]`）、冻结逃逸、`map→odom` 预测位姿、写图位姿改用匹配位姿、
   `max_keyframes` A/B——它们与计票机制互补（可观测性诊断正是 P1 距离加权
   升级前的前置数据）。
3. **评分永不静默美化**：`dynamic_filter` 保持关闭；动态物干扰的解法是
   w 过滤/robust kernel，而不是删点改分。

### 决议：单一工作图文件（2026-11-25 晚，用户拍板）

**文件模型**（磁盘上只有两份相关文件，运行只读其中一份）：

```
map.pcd            原始稠密底图（如 6cm）。只读备份，永不修改，运行时不加载。
map.overlay.pcd    工作图 = 底图(0.3) + overlay，每个点 intensity = w。
                   运行时的唯一数据源。1h 全量落盘一次，无变化不写。
```

1. **首次载入即生成工作图**：`map.pcd` 降采样到 0.3，每个体素赋初值
   `base_voxel_w`（128），写入 `intensity` → 落成 `map.overlay.pcd`；
   本次会话也改用这张 0.3 图（和下次加载完全一致）。
2. **之后只加载工作图**：0.3 体素 + w 一体，不再加载稠密原图。
   （匹配本来也是 0.3：`local_map.voxel_resolution` / matcher 都是 0.3，
   其他消费方 FPFH 0.5、BBS 0.5、planar 1m 格，都在 0.3 之上。）
3. **不做增量日志**：删除 20s autosave、析构落盘、`map.pcd.overlay.pcd` 增量文件、
   `autosave_frames` 配置。代价：崩溃最多丢 1 小时学习量（几何不丢，
   `map.pcd` 永远在）。
4. **w 的持久化就是 PCD 的 intensity**：不需要额外 sidecar；`w==0` 的点落盘时
   直接不写 → 跨会话"删除"自动生效。注意 `map.pcd` 的 intensity 是反射率，
   只有工作图的 intensity 才当 w 解释（由加载路径决定）。
5. **底图网格与 overlay 同源**：落盘/首次生成都先降到 0.3，账本 ≈ 1:1 对应底图点，
   不复制第二份几何。
6. **原图更新的处理**：工作图 mtime 早于 `map.pcd`（同名地图被重新导入）→
   工作图作废，从原图重新生成。
7. **旧文件自动清理**：启动时删除旧版 `<stem>.overlay.map` 与 `map.pcd.overlay.pcd`。
8. **不改评分口径**：底图点仍参与 fitness；死亡体素（w=0）只从匹配目标剔除。
9. **体素回收时机**：删除时只记账（`m_pending_remove`），落盘时 `reclaim()`
   全量压缩一次（逐条删要扫整格点云，太贵）。
10. **待实测再定**：底图点（体素中心）与 overlay 点（扫描端点）可能落相邻体素，
    先靠"同帧先 carve 后 hit" + `override_base` 兜底；若实测残留，
    再把死亡体素膨胀到 26 邻域。`save()/load()`（overlay-only）保留为工具，主路径不用。

## 7. 与并行工作流的关系（重要）

现场 14003AO7 的配置由另一条工作流维护：`map_maintainer: none`、
`session_map.enabled: false`、`local_map: keyframe_submap`（窗口 24）。
该工作流交付了：方向可观测性诊断（`evaluateObservability`）、冻结逃逸、
`map→odom` 预测位姿、写图位姿改用匹配位姿 —— **与本方案互补不冲突**。

**决策点（需要确认）**：本方案要求现场开启
`map_maintainer: overlay` + `session_map.enabled: true`。
若现场保持关闭，则本方案只落地"w 过滤匹配目标"部分，计票雕刻部分不生效。

## 8. 参数速查（本方案相关）

| 参数 | 默认 | 说明 |
|---|---|---|
| `maintainer.persistency_unit` | session | w 计票单位（frame/view/session/day） |
| `maintainer.persistency_hit_gain` | 8 | hit 一次 +8 |
| `maintainer.persistency_miss_confirm` | 3 | 连续 3 次 miss 才衰减（去抖） |
| `maintainer.persistency_miss_action` | 0 | 0=硬删 1=递减 |
| `maintainer.persistency_session_hours` | 12 | 会话超时换新（重置计票） |
| `maintainer.persist_hits` | 3 | novel 升格票数 |
| `maintainer.novel_min_cluster` | - | 最小簇 |
| `maintainer.delta_min` | 0.3 | 离底图多远算"新"（本方案建议由 w 衰减取代） |
| `maintainer.base_voxel_w` | 128 | 底图点（体素）的 w 初值；归零才会从目标剔除 |
| `maintainer.target_w_min` | 16 | 匹配目标里 overlay 点最低 w（固定物参与，动态/未确认忽略） |
| `w_anchor`（建议新增） | 128 | 可当锚的最低 w |
| `w_match_min`（建议新增） | 0 | 参与匹配的最低 w（0=不过滤） |
| `static_support_min`（建议新增） | 0.25 | 锚更新门限（LOCALIZATION_ANCHOR §4） |
