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
| 4 | 地图 z 以**地面 = 0** | 机器人建图原点在地面 | 整体高 1.4 m（手持传感器高度），inlier≈0.45 |
| 5 | 关键帧间距参考 `keyframeMeterGap = 1.0 m` | `mapping_conf.json` | 影响子图密度 |

补充：`key_frame_points_2d` 可以为空数组（`fromJson` 不读它，只有 `fromJsonExpandMap` 读）。

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
```

`run_dir` 需要：`map.pcd`、`odom_bag`（含 `/lio_odom`），可选 `map.pgm` / `map.yaml`。
脚本会：地面 z=0 对齐 → 按 `kf-step` 抽关键帧 → 写**局部坐标 + 毫弧度 + 扁平位姿**的
`map.xzmap`，并同步输出平移后的 `map.pcd`。

## 5. PGM 注意

PGM 由 `PcdMapManager::saveGridMap` 用 `observations_`（逐关键帧点云）做 ray_casting 重建，
所以：

- `mapping.map_manager.max_observation_points` 必须大于整段轨迹的总观测点数
  （默认 2e6 ≈ 只保留最后 322 帧，**PGM 会缺段**）；400 s 建议 4e7，更长按比例加；
- 行人/瞬态过滤：`dynamic_removal + persist_thresh`（前端）、`min_hits`（栅格）、
  `min_component_cells`（后处理）；
- 墙要清晰：`save_voxel` 0.1（点间距），`resolution` 0.05，`mark_radius` 1。
