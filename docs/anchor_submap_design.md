# 锚定与子图方案（底图部分变化场景下的稳定定位）

> 2026-11-23 定稿。实机：14003AO7（万集 WLR722，俯装 39.37°，离地 0.914 m）。
> 版本：0.2.16。核心提交：`05cd954` → `d9bfa01` → `f110452`（+ 修复 `ba56bfa`/`11db7d4`/`64fcef5`）。

---

## 0. 一句话

**overlay 是独立的计票/雕刻机制（每帧/view/会话），子图只负责"底图锚不住时"维持瞬时定位；
锚定判据是对底图（不含子图/overlay）的覆盖率 `cov_base`，两端（初值/输出）用同一条 TF 互逆换算。**

---

## 1. 问题（实机证据，14003AO7）

| 现象 | 实测 |
|---|---|
| 底图某区域完全被改变后，位姿漂走且**修不回来** | fitness=1.000 / cov_all=1.000 / inl=vis（自己配自己） |
| 核心位姿带 ~33° 倾角（≈雷达安装 pitch 39.37°），但发布位姿水平、点云水平 | `[planar]` nm_z=0.845, rpy=(33.2,4.1) vs 点云天花板 3.9° |
| 初值 z=0 导致机器人"在地面下" ~1.2 m | 点云 z -1.16..1.61，而地图地面 0 / 天花板 2.79 |
| 把 URDF 的 `base_link→lidar`（39.37°/89.9°/0.914 m）当 `r_il` 注入 | 点云相对机身多转 90°+倾斜（该 TF 是机械姿态，不是点云外参） |

逐段排查后确认：驱动（`roll/pitch/yaw=0`）、`ros2Pcl`、`applySelfFilter`、`processImu`
都**不做任何旋转**；点云在驱动输出时就已经是水平（地面法向 0.99°），IMU 报文则带 38.4° 的
轴偏置（`/vanjee_lidar_imu_packets` 的 linear_acceleration 与点云同 frame_id 但方向不同）。

---

## 2. 架构：三层 + 一条状态机

```
底图 (base map)      只读, 永不修改。长期定位的"锚"。
overlay (持久变化层) 独立机制, 自带策略: 每帧/view/会话在 onLocalized() 里执行
                     rayCarve(负证据清除) / observeHits(续命) / collectNovel(三道门计票)
                     → 落盘 overlay.pcd, 跨天/跨会话按 persistency 策略生效。
                     "逐步雕刻底图"实际是雕刻这一层 —— 底图永远不动。
子图 (session map)   瞬时定位用。定位成功+覆盖率高就开建、一直在建;
                     底图锚不住时参与匹配(桥接), 边建图边定位。
```

### 2.1 锚定状态机（2026-11-29 统一版）

判据 = **对底图的支撑**，只用一个口径（0.3m 体素、0.3m 内点、全量不抽样）：

- OPEN 态：匹配目标就是底图 crop，`fitness` 即底图支撑；
- BRIDGING 态：后台 `cov_base` 只对底图算（同 0.3 口径），是诚实的退出/健康判据；
  `fitness` 此时对联合目标，是自证分，只做显示。

```
CLOSED → OPEN → BRIDGING

CLOSED→OPEN:   首次 status==LOCALIZED && fitness >= credible_fitness(0.85)
                (不再要求本帧 accepted —— 开图只是开始累积)
OPEN→BRIDGING: fitness < enter_support(0.70) 连续 anchor_patience(5) 个 1Hz tick
                && 子图 >=3000 点 && status != LOST
                目标 = 底图 crop(实时剔判死点) ∪ 子图; 写入门槛 0.35 + 位姿稳
BRIDGING→OPEN: ① 周期探测重锚: 对底图目标 fitness >= relocation_fitness(0.70)
               ② cov_base >= credible(0.85) 连续 patience 个 1Hz tick
               ③ LOST: 回底图网格恢复(子图冻结, 不参与)
```

要点：

- `anchorOk`（accepted/inliers/eig/w_support）**只用于写图门槛**，不再决定桥接；
  桥接决策只用“底图支撑”一个指标 + 迟滞。
- 桥接目标**保留底图**：判死点已实时剔除，剩下的是仍有效的绝对参照；不硬切纯子图
  （纯子图 = 局部相对配对，两次探测重锚之间漂移不受约束）。
- 子图首次开图后**常驻**，不因退出桥接而关闭。
- 已删除死配置：`submap.anchor_close_patience`、`session_map.use_for_tracking`。

### 2.2 三个层次各自的"账本"

| 层 | 出生 | 死亡 | 备注 |
|---|---|---|---|
| 底图 | 建图产物 | 无 | 只读；`ensureBaseIndex` 只建 KD 树索引 |
| overlay | `collectNovel` 三道门：离底图>`delta_min`、按 `persistency_unit` 计票≥`persist_hits`、≥`novel_min_cluster` 簇 | `rayCarve`→`missVoxel`：`miss_action=0` 硬删；`=1` 软删（`w-=hit_gain`，减到 0 删） | `observeHits` 刷新存活；`decay_frames` 长期不看移除 |
| 子图 | `LOCALIZED` 且 fitness 达标时 `reset(m_pose)` | 重定位/切图 `resetSessionMap()` 时清 | `radius_m` 半径外丢弃，`voxel` 去重，`max_points` 上限 |

**底图体素不在 overlay 的账本里**：`missVoxel` 对未知 key 只清 `m_voxel_seen`，
不会删除/修改底图的任何点。"雕刻底图"是雕刻 overlay 层。

---

## 3. 初值链路（`applyInitPose`）

```
App/服务给: (x, y, z=0, yaw)   ← z=0 = 车底(base_footprint)贴地
    ↓ ① TF: T(base_footprint ← lidar)   —— 静态安装链, 不写死常量
    ↓ ② p_core = p_app * T_fp←lidar     —— 旋转+平移都换算 (互逆性保证)
    ↓ ③ setInitPose(p_core)             —— 核心从正确的 z/朝向开始匹配
```

日志：

```
[slam_algo] init pose footprint->lidar: z 0.000->0.984 (+0.984) yaw -164.8->-74.9
```

坑（都踩过）：

- `force_z_zero`（`/switch_map` 路径）在换算**之后**把 z 清零 → 白算。已改：非
  `default_map_pose` 分支不再清零。
- 不能把安装 yaw（89.9°）叠到车体航向上 —— 安装 yaw 已含在扫描自身坐标系里，
  再叠一次机器人整体转 90°。
- 不能调 `computeBaseFrames()`：那个调用会激活 `base_lidar/base_imu` 快照，
  连带开启 `applySelfFilter`（点云自滤）与位姿换算，等于把整套安装外参打开。
  高度值改用独立的 `tf_buffer_->lookupTransform("base_footprint","lidar")`。
- URDF 的 `base_link→lidar` rpy（39.37°/1.13°/89.9°）**不是点云外参**：点云在驱动输出时
  已经水平（地面法向 0.99°）。把它当 `r_il` 注入点云 = 转两次（实机多出 90°+倾斜）。

输出侧（`outputs.cpp` / `pose_publisher.cpp`）做严格互逆：

```cpp
p.pose = composeRightInv(out.pose, baseFootprintLidar());   // 雷达系 → base_footprint
```

**注意是 `baseFootprintLidar()` 不是 `baseImu()`**：核心位姿是雷达的（`r_il`=单位阵），
按 `baseImu`（底盘 imu_link，z 0.115）换算会差 0.8 m。

---

## 4. 逐帧链路（一次匹配的生命周期）

```
guess = m_prior->predict()            ← IMU 先验: 只用 gyro.z 做打滑判据, 不积分 roll/pitch
scan  = deskewScan(f, guess) → r_il 变换(单位阵, 即不转)
res   = matcher->match(scan, guess)   ← 匹配目标见 §2.1
accepted = acceptMatch(...)           ← 只门控, 不改位姿
[退化方向部分更新]                     ← 本次实机未触发 (DegeneracyProbe=0); 注意其
                                        e2.head<3>() 直接当旋转轴角用, 有平移泄漏进
                                        旋转的隐患, 待加角度上限
m_planar.apply(out_pose, scan)        ← 逐帧 z/倾角约束 (scan_plane)
m_prior->onMatch / m_fusion->onMatch / predict → out_pose
m_pose = out_pose → snapshot          ← 发布侧 composeRightInv(fp←lidar) 互逆还原
```

`[pose_cmp]` 日志（每 20 帧）把 `match / guess / out / after_planar` 的 rpy 打出来，
专门用于确认没有中途换系。

---

## 5. 遗留（都有记录，未修）

1. **overlay 计票依赖 `anchorOk(res)`**（inliers+特征值比），在桥接态是"对着含子图的
   目标"算的 —— 自引用风险仍在。若要彻底，`anchorOk` 也应改用 cov_base 口径。
2. **`[planar]` 逐帧 z 约束未生效**：核心位姿带安装 pitch（nm_z≈0.85），`scan_plane`
   每帧 `jump`。当前 z 靠初值+底图锚定撑住；若底图长期失效需要桥接长时间工作，
   需把约束也换到与位姿一致的系。
3. **退化部分更新的平移泄漏**：`e2.head<3>()` 直接当旋转轴角，特征基混合了平移分量。
   实机未触发，建议加角度上限。
4. `base_footprint` 与 `base_link` 的 7 cm：现发布为 `base_footprint` 语义（车底贴地）。

---

## 6. 参数速查

| 参数 | 默认 | 作用 |
|---|---|---|
| `session_map.enabled` | true | 子图开关（总闸） |
| `session_map.radius_m / voxel / max_points` | 25 / 0.15 / 30 万 | 子图范围/去重/容量 |
| `localization.monitor.credible_fitness` | 0.75 | cov_base/fitness 的"高"线 |
| `matcher.dynamic_dist` | 0.4 | 底图离群点删除门限（只影响优化，不影响 cov_base 分母） |
| `matcher.inlier_dist` | 0.3 | "配上"的距离判据（cov_base 用它） |
| `anchor.close_patience` | - | cov_base 达标持续 N 帧才切回底图 |
| `init_pose_frame` | lidar | 初值换算目标系（私有参数） |

## 7. 日志速查

| 日志 | 含义 |
|---|---|
| `[slam_algo] init pose footprint->lidar: z 0.000->0.984 (+0.984) yaw ...` | 初值 TF 换算结果 |
| `[quality] ... cov_base=1.000 ... session_map=1 size=...` | 底图覆盖率/锚定状态/子图规模 |
| `[anchor] switch to BASE/submap cov_base=...` | 锚定切换 |
| `[session_map] open (base anchored, building ahead)` | 子图开建 |
| `[planar] why=... up_z=... nm_z=... rpy=(...) F=... dz=...` | 逐帧平面约束诊断 |
| `[pose_cmp] match=(..) guess=(..) out=(..) / after_planar` | 位姿四点追踪（33° 定位用） |
