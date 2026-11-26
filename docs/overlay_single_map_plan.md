# overlay 单一置信度地图改造方案（w-map）

> 状态：**方案定稿，尚未实施**（2026-11-24 与现场对齐）
> 触发背景：现场出现「会话子图叠图」「overlay 错位」「货架搬走后底图删不掉」三个问题。
> 本文只描述目标架构与改动清单，代码按要求暂不改动。

---

## 0. 结论先行

把「只读底图 + 可写 overlay」两层结构，改成**一张带置信度 w 的体素地图**：

```
地图 = { 体素 (x,y,z), w ∈ [0,255] }
  w = 255   底图注入时的初始值（视为"完全确认"）
  w += 8    每次 view 观测为占据（离开视野再回来才算一次）
  w -= 8    每次 view 观测为自由（射线穿过）
  w <= 0    删除该体素
匹配目标 = crop(w >= 128) + 会话子图（瞬时、不落盘）
```

- **底图只作为"转换原料"**：载入 xzmap 后按体素注入 overlay（w=255），之后不再使用底图；
- **删除就是 w 归零**，不再出现"底图上永远删不掉"；
- **虚影（w < 128）不参与匹配**，只留在文件里等确认或衰减。

---

## 1. 现状与问题

### 1.1 当前结构

| 层 | 内容 | 可写 | 参与匹配 |
|---|---|---|---|
| 底图（xzmap 载入） | 6,000,000 点 | ❌ | ✅（裁剪后） |
| overlay | 变化结构 + w | ✅（view×3 后写入） | ✅ 仅 w >= `match_w_min`(128) |
| 会话子图 | 本会话观测（世界系） | ✅（fitness >= 写入线） | ✅ 桥接态 |

### 1.2 三个已确认的根因

1. **叠图（子图内部岔开几度）**：`deskewScan(f, guess)` 用**预测位姿**做去畸变，而写图用**匹配位姿** `res.pose`，两者差 `corr_deg`（常态 0.1~3°）。每帧点云带一个旋转误差落进世界系 → 同一面墙写成两层。建图模式不会，因为去畸变与写图用同一套位姿。
2. **模式切换跳变**：目标组成变化（底图 ↔ 桥接）或种子生效时，匹配解可能一次性跳 3~5°，跳变前后的点自然岔开。
3. **单帧写入**：`persist_hits = 1`（view）时，一次低质量观测就落库；配合 0.6~0.85 的降级帧，overlay 被灌入大量错位点（实测一次 10 分钟写入 272k 点，其中 14% 在 z<0、35% 在 z>4m）。

### 1.3 同日已修复的先行项（改造时必须保留）

| 项 | 内容 | 文件 |
|---|---|---|
| 写图去畸变 | `prepareScan(f, res.pose)` 专供写图，匹配仍用 guess 版本 | `localization_pipeline.cpp` |
| 软着陆 | 模式切换/种子后 10~20 帧内单帧修正限 1°/5cm | 同上 |
| 子图刚性搬移 | 大修正时**不清空**，`T_GS' = Δ∘T_GS` 整体搬移 | `session_map.h` + pipeline |
| 虚影不参与匹配 | `maintainer.match_w_min = 128` | `overlay_map_maintainer.cpp` |
| 多次确认 | `persist_hits = 3`（view） | yaml |
| 位姿稳定才写 | `corr_m <= 0.10 m && corr_deg <= 1.0°`，coast 帧不写 | pipeline |
| 地面统一裁剪 | 用 matcher 的 z 带 + 安装外参裁一次，匹配/写图同一份点云 | pipeline `prepareScan` |
| 判丢线 | `lost_fitness 0.60`（= `accept.min_coverage`） | yaml |
| 锚定判据 | `anchor_fitness_base 0.55`、`anchor_enter_frames 5`、`anchor_return_frames 10` | yaml + pipeline |

---

## 2. 数据模型

### 2.1 w 语义

| 值 | 含义 |
|---|---|
| 255 | 底图注入（完全确认） |
| 128~254 | 高可信变化结构（参与匹配） |
| 8~127 | 已确认存在但未达"长期"标准（虚影，只存不匹配） |
| 1~7 | 首次观测候选 |
| 0 | 删除（从 cells / voxel_seen / persistency 中移除） |

写入路径：`persist_hits=3` 才第一次落库（w=8），之后每个 view +8，约 16 个 view 达到 128。

### 2.2 存储结构（沿用现有实现）

```cpp
std::unordered_map<int64_t, CloudType::Ptr> m_cells;      // 10m 分块, 供半径裁剪
std::unordered_set<int64_t> m_voxel_seen;                 // 存活体素
std::unordered_map<int64_t, PersistencyVoxel> m_persistency; // w / miss / last_key / last_frame
```

### 2.3 规模估算（待实测）

| 项 | 估算 |
|---|---|
| 600 万点 → 0.4 m 唯一体素 | 100~200 万 |
| 内存（点 12B + w 8B + 容器开销 ~40B/体素） | 100~150 MB |
| 注入耗时（aarch64，单线程） | 2~5 s（载图时后台执行） |
| 匹配裁剪 35 m 半径 | 10~20 万点（对照：39 万点时实测 2.95 s/帧） |

---

## 3. 注入算法（底图 → overlay）

注入即哈希体素化，不需要额外降采样算子：

```cpp
// OverlayMapMaintainer::setBaseMap(map_path, base)
// 前提: max_voxels 已提高; decay_frames == 0; 不再构建 m_base_kdtree / m_base_voxels
for (const auto& p : base->points) {
    if (!finite(p)) continue;
    if (p.z < z_min || p.z > z_max) continue;          // 与匹配 z 带一致
    const int64_t vk = voxelKey(floor(p.x/v), floor(p.y/v), floor(p.z/v));  // v = 0.4
    if (m_voxel_seen.count(vk)) continue;              // 同体素去重 = 降采样
    cellOf(p).push_back(p);                            // m_cells
    m_voxel_seen.insert(vk);
    m_persistency[vk] = { .w = 255 };
}
// 注入完成后: m_base_raw.reset(); 释放底图点云
```

### 必须同时处理的前提

| 项 | 现状 | 改造 |
|---|---|---|
| `max_voxels` | 400,000，超限会**整体重置 overlay** | 提高到 300 万，或改为"停止新增、不重置" |
| `decay_frames` | 0（正确） | 保持 0 |
| `m_base_kdtree` / `m_base_voxels` | 用于"离底图 > delta_min 才算新颖" | **删除**：新颖性改为"`m_voxel_seen` 里没有这个体素" |
| `override_base` / `suppressBaseByOverlay` | 双层结构下"替换"用 | 单层后**不再需要** |
| 底图内存 | 常驻 | 注入后释放 |

---

## 4. 匹配与状态机（判据不变，只换原料）

| 量 | 定义（改造后） | 用途 |
|---|---|---|
| `fitness` | 扫描点对 **overlay(w>=128) + 会话子图** 的命中率 | 采纳 / 健康 / 判丢 |
| `fitness_base` | 扫描点对 **overlay(w>=128)**（不含子图）的命中率 | 锚定 ↔ 桥接切换 |

阈值与状态机**完全沿用**：

```
accept.min_coverage      0.60   (单帧采纳)
monitor.lost_fitness     0.60   (连续 30 帧 < 此值 -> LOST)
monitor.credible_fitness 0.85   (健康 / 写图 / 有界恢复)
global.relocation_fitness 0.85  (种子/重定位)
submap.anchor_fitness_base 0.55 (低于 -> 桥接)
submap.anchor_enter_frames 5    (fitness 连续 5 帧 <0.85 才切桥接)
submap.anchor_return_frames 10  (连续 10 帧好才回锚定)
```

实现上只需把 `buildPriorTarget()` 的"底图裁剪"换成"overlay 中 w>=128 的裁剪"；`fitness_base` 的 kd 树同样建在这份子集上。会话子图仍是"瞬时未确认结构"的缓存，不落盘。

---

## 5. 写入路径（保持改造前的收敛后的规则）

1. **view 计票**：上一帧没看到、这一帧看到才 +1（静止反复看不加 w）。
2. **`persist_hits = 3`**：3 次 view 才首次落库（w=8）。
3. **位姿门控**：`coast == 0` 且 `corr_m <= 0.10 && corr_deg <= 1.0°`。
4. **点云**：必须用 `prepareScan(f, res.pose)` 的输出（去畸变位姿 = 写图位姿）。
5. **高度带**：与 matcher 同一 z 带，地面/贴地点不参与写入。
6. **匹配准入**：`match_w_min = 128`，低于它的点只存不匹配。

---

## 6. 删除路径

```
rayCarve(pose, scan) 沿传感器->端点射线步进:
    for each 途经体素:
        若该体素存活: missVoxel(vk)
missVoxel: 按 view 计一票 free -> w -= persistency_hit_gain(8); w<=0 则删除(从 cells / seen / persistency 移除)
```

- overlay 新点（w=8）→ 1 次 view 的 free 证据即删；
- **底图点（w=255）→ 需 32 次 view** 才删；
- `persistency_miss_confirm` 仍用于"同一 view 内连续帧去抖"。

### 待定项

- 底图点 32 次 view 是否太慢？备选：给"底图注入点"单独的 miss 步长（如 −32 → 8 次），或提供手动删除服务。
- 是否需要 `/reset_overlay` 服务：丢弃 overlay，从 xzmap 重新注入（误删兜底）。

---

## 7. 持久化

| 项 | 方案 |
|---|---|
| 文件 | `map.overlay.pcd`，字段 `x y z intensity`，**intensity = w** |
| 频率 | 30 分钟一次（`autosave_seconds: 1800`），另加"进程退出时保存"与 `/slam/save_overlay` 服务 |
| 体积 | 100~200 万点约 40~60 MB |
| 启动顺序 | 1) 有 `map.overlay.pcd` → 直接加载（w 从 intensity 读回），**不加载 xzmap**；2) 否则 → 载入 xzmap 注入 w=255 |
| 底图用途 | 仅作注入原料 + 误删兜底；保留只读，不参与匹配 |
| 损坏处理 | overlay 读失败 → 告警并回退到 xzmap 重新注入 |

---

## 8. 改造清单

### P1（不涉及落盘，可先验证"删除生效 / 匹配干净"）

| 文件 | 改动 |
|---|---|
| `overlay_map_maintainer.cpp/.h` | `setBaseMap` 改为注入（w=255）；删除 `m_base_kdtree`/`m_base_voxels`/`ensureBaseIndex`；`collectNovel` 新颖性改用 `m_voxel_seen`；`missVoxel` 改为 w 递减语义（底图点同样适用） |
| `localization_pipeline.cpp` | `buildPriorTarget` → overlay(w>=128) 裁剪；`fitness_base` 的 kd 树建在同一子集；删除 `suppressBaseByOverlay` 调用 |
| `localization_stage_types.h` | `MaintainerConfig`: 提高 `max_voxels` 默认；新增 `inject_voxel`（0.4）、`autosave_seconds` |
| `config/default.yaml` | `max_voxels`、注入体素、保存周期；`override_base` 标记废弃 |

### P2（地图本体可写）

| 内容 |
|---|
| pcd 带 `intensity=w` 的 save/load |
| 启动优先加载 overlay；失败回退 xzmap 注入 |
| 30 分钟自动保存 + 退出保存 + `/slam/save_overlay`、`/reset_overlay` |

### P3（标定）

| 项 | 初值 | 备注 |
|---|---|---|
| 注入体素 | 0.4 m | 与匹配体素一致可省一次降采样 |
| 匹配半径 | 35 m | 实测雷达点 p50=27.6 / p95=35.6 m |
| `max_voxels` | 3,000,000 | 需与内存实测对齐 |
| miss 步长 | −8 / view | 底图点 32 次；如需更快再调 |
| 保存周期 | 1800 s | eMMC 每天约 48 次写入 |

---

## 9. 风险与回滚

| 风险 | 缓解 |
|---|---|
| 底图点被误删 | 保留 xzmap 只读 + `/reset_overlay`；w 递减而非立即删除 |
| overlay 文件损坏 | 加载失败自动回退 xzmap 注入，保留 `.bak` |
| 内存增长 | `max_voxels` 上限 + 超限"停止新增不重置" + 监控 RSS |
| 启动变慢 | 注入 2~5 s，期间不定位（状态 INIT），与现有 reloc 库构建同类 |
| 匹配变慢 | 半径 35 m + 体素 0.4；超过 20 万点/帧则回退半径 |

回滚：恢复 `map_maintainer` 旧实现（保留 `git` 分支），并删除 `map.overlay.pcd`。

---

## 10. 待定项（实施前再确认）

1. 注入体素 0.4 还是 0.3（内存 vs 精度）。
2. 底图点 miss 步长（32 次 vs 8 次）。
3. `max_voxels` 上限与超限行为。
4. 是否保留 `keyframe_submap` 作为 `local_map`（目标来源改为 overlay 后，`local_map` 只影响 `fitness_base` 的裁剪方式）。
5. 保存是否需要"只存变化量"（增量 pcd + 底图引用），以降低写入量。

---

## 附：同类方案对照

| 方案 | 关键做法 | 与本文的对应 |
|---|---|---|
| LT-mapper / LT-memory (Kim 2022) | 两阶段：先 erase（可见性/多会话一致性），再 add（多会话确认） | 对应"w 递减删除 + persist_hits 多次确认" |
| ERASOR (2021) | 伪占据 + 视线可见性，只把 free→occupied 当变化 | 对应"射线 free 证据减 w" |
| Removert (2020) | 多会话 range-image 可见性对比 | 对应"跨 view 计票" |
| RTAB-Map lifelong | 定位模式地图冻结，仅 mapping 模式更新 | 我们用 w 门槛代替模式切换 |
| Nav2/AMCL、Cartographer 纯定位 | 不在线改图 | 反例：现场需要长期自更新 |
