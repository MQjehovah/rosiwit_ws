# 扫图仪地图 → 机器人地图（xzmap / pgm）导出规范

本文记录把 `rosiwit_slam` 建出的地图导入机器人（`xz_3d_slam`，数据目录 `/opt/xzrobot`）
时必须遵守的约定。**每一条都在真机上验证过**，任何一条不满足，现象都是"界面上激光贴墙、
但定位一直报丢失"，而日志里的 `inlier` 只有 0.4~0.75（正常 > 0.89）。

## 1. 目录与数据库

```
/opt/xzrobot/maps/<uuid>/
    map.pgm            2D 栅格（0=占据 205=未知 254=自由）
    map.yaml           image 用绝对路径, resolution 0.050, occupied 0.65 / free 0.25
    map.pcd            3D 点云（世界坐标）
    map.xzmap          定位用的关键帧地图（见下）
    map.jpg / thumb.png / map_origin.*   界面预览（可后补）
```

`/opt/xzrobot/database.db` 表 `map`：

| 字段 | 说明 |
|---|---|
| `name` / `uuid` | 地图名 / 目录名（毫秒时间戳） |
| `originX/Y/Z` | 取自 `map.yaml` 的 origin（2D） |
| `resolution` | 0.05 |
| `md5` | **`md5(map.pgm)`**（与现有记录一致） |
| `floor` / `building` | 可空 |

## 2. xzmap 的五条硬约定

| # | 约定 | 代码依据 | 违反后的现象 |
|---|---|---|---|
| 1 | `key_frame_point` 是**关键帧局部坐标** | `map_updater.cpp:581` 保存前 `T⁻¹`；`nano_gicp_localization.cpp:1646` 载入时 `local2global` | 子图整体被二次变换，inlier≈0.38 |
| 2 | 角度单位是**毫弧度**（rad×1000），不是毫度 | `map_updater.cpp:603` `pose_6d.yaw(rad)*1000`；读取 `tf.yaw*0.001` 后直接给 `pcl::getTransformation`（要弧度） | 每个关键帧的点按 1/57 的角度旋转，inlier≈0.75，且**看起来仍能对上** |
| 3 | 关键帧位姿扁平：`z=0, roll=0, pitch=0`，只留 `yaw` | `mapping_conf.json: "fix_z": true, "fix_rotation": true` | 与机器人扫描的 z/姿态基准不一致 |
| 4 | 地图 z 以**地面 = 0**（具体取**地图原点附近 12 m 内的地面**，不是全图最低峰） | 真机实测：所有能用的地图原点地面都在 `+0.06…+0.16` | 整体高/低 1.5 m 以上，inlier 永远到不了 0.89 |
| 5 | **每个关键帧要带"整片"点云**（半径 8 m，自适应体素压到 ~2400 点），不能只带最近邻格子里的点 | 机器人 `kf_map_filter_num=2` 会隔一个丢一个关键帧 | 覆盖掉 49%，fitness 卡 0.75（详见 §6） |
| 6 | 关键帧间距参考 `keyframeMeterGap = 1.0 m` | `mapping_conf.json` | 影响子图密度 |

补充：`key_frame_points_2d` 可以为空数组（`fromJson` 不读它，只有 `fromJsonExpandMap` 读）。

### 4) z 锚点为什么不能用"全局最低强峰"

2026-11-19 的 `genz` 图（1252 s、1598 m、含 35 m z 漂移的长坡道）实测：

- `detectFloorZ`（z∈[-4,4] 直方图取最低 >35% 强峰）= **-3.93**，整图被抬高 3.93 m，
  原点地面变成 **+2.36**，重定位 fitness 只有 **0.68**，`th_fitness_relocalization=0.89`
  永远达不到 → 一直"定位丢失"；
- 改成"原点附近 12 m 内 z 的 2% 分位"= **-1.54**，原点地面回到 **0**，与其他地图一致。

图一旦有 z 漂移（长坡道、回环），全图统计量就一定被漂移区带偏，**只能按原点定锚**。

## 3. 定位判定（用于自查）

```
inlier(fitness_score_) = 命中点数 / 有效点数
命中条件：到地图最近点的距离 ≤ sqrt(vf_scan_res) ≈ 0.447 m
成功条件：inlier > th_fitness_relocalization = 0.89
```

自查方法：看 `/opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log`：

```
NanoGicpLocalization::Point2PlaneICP(), overlap final_fitness is 0.5x, inlier is 0.9x
```

`inlier > 0.89` 才会出现 `get_good_relocalization_ = true`，否则一直 `localization_lost!!!`。

## 4. 导出

```bash
python3 scripts/export_robot_map.py <run_dir> --kf-step 1.0 --zip map.zip
# 或（离线 C++，与在线 /save_robot_map 同一实现）
offline_map_export <dump_dir> <out_dir> --res 0.05 --kf-step 1.0 \
  --generator ray_casting --post denoise,close,dilate --close 3 --dilate 1 \
  --min-comp 30 --min-height 0.1 --max-height 2.0
```

`run_dir` 需要：`map.pcd`、`odom_bag`（含 `/lio_odom`），可选 `map.pgm` / `map.yaml`。
脚本会：**原点地面 z=0 对齐** → 按 `kf-step` 抽关键帧 → 每帧取 **8 m 半径内整片点云**
（自适应体素 ≤2400 点）→ 写**局部坐标 + 毫弧度 + 扁平位姿**的 `map.xzmap`。

导出时务必看这行自检日志（`[robot_map_export]`）：

```
kf=1492 pts=2741871 step=1m ground_z=-1.53762 (origin) pts/kf_med=1796 gap@kf_filter2 med=0.15 max=1.14m
```

| 字段 | 期望 | 不满足时 |
|---|---|---|
| `(origin)` | 必须是 origin，出现 `global` 说明原点附近点太少 | 检查 `getGlobalMap`/dump 是否完整 |
| `pts/kf_med` | **> 1000**（真机地图 1.8k–2.5k） | <400 会打印 WARN，隔帧后必出空洞 |
| `gap@kf_filter2 max` | 远小于 2×8 m | > 3 m 打印 WARN，加大半径或减小 kf-step |

## 5. PGM 注意

PGM 由 `PcdMapManager::saveGridMap` 用 `observations_`（逐关键帧点云）做 ray_casting 重建，
所以：

- `mapping.map_manager.max_observation_points` 必须大于整段轨迹的总观测点数
  （默认 2e6 ≈ 只保留最后 322 帧，**PGM 会缺段**）；400 s 建议 4e7，更长按比例加；
- 行人/瞬态过滤：`dynamic_removal + persist_thresh`（前端）、`min_hits`（栅格）、
  `min_component_cells`（后处理）；
- 墙要清晰：`save_voxel` 0.1（点间距），`resolution` 0.05，`mark_radius` 1。

> 已知现象（未修）：长坡道/漂移图里，2D 图会把**天花板**标成障碍物。因为
> `min_height/max_height` 是相对**每帧 RANSAC 地面**的高度带，而手持 Mid-360 竖直
> FOV 只有 −7°，画帧脚下 10 m 内没有地面点，2% 分位取到的是远处地面，带子被抬高。
> 自查：统计 occupied 格对应点的"离地高度"，若集中在 2–4 m 即为此问题。

## 6. 关键帧必须带"整片"点云（2026-11-19 真机踩坑）

机器人定位端 `localization_conf.json` 里 `kf_map_filter_num = 2`，`setKfMap()` 的实现是：

```cpp
for (int i = 0; i < kf_points.size(); ++i)
    if (i % kf_map_filter_num_ != 0) continue;   // 隔一个丢一个关键帧
```

之前导出用"每个点归到最近的关键帧"（Voronoi 切片），每个关键帧只拥有自己 ~1 m 的格子。
于是隔帧丢弃 = **在地图上开 1 m 宽的横沟**：

| | 旧导出（Voronoi 切片） | 新导出（8 m 半径整片点云） |
|---|---|---|
| 点/帧 | 350 | 1796 |
| 隔帧后 0.1 m 格子覆盖 | 278.8 → **141.5 m²（−49.2%）** | 无空洞（每帧覆盖 8 m） |
| 真机 fitness | 0.75~0.79（卡在 0.89 下方） | **0.960** |
| 真机 inlier | 0.88~0.93 | **0.996** |

对照：机器人自己建的地图（`站台` 146 帧/265k 点、`1` 980 帧/2.46M 点）都是
**每帧一幅完整扫描**（1.8k–2.5k 点/帧），所以它们不怕隔帧丢弃。

> 结论：导出时"每帧点数"和"每帧覆盖半径"是硬指标，不是可调参数。
> 判定阈值 0.447 m（`√vf_scan_res`）决定了空隙必须远小于 0.5 m。
