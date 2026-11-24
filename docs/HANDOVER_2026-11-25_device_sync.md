# 设备 14003AO7 与本地仓库的差异（2026-11-25 02:00）

> 起因：录包验证正常，但现场反馈"子图经常建歪 → 整体定位丢失"。排查发现**设备代码比本地新**，
> 两边不是同一个版本。此文档把设备侧状态、差异和同步方式记下来，避免再对不上。

## 1. 版本对照

| 项 | 设备 14003AO7 | 本地 / origin |
|---|---|---|
| 包版本 | `rosiwit-slam-ros1 0.2.16`（11-24 21:16 编译安装） | — |
| HEAD | `f4aff06` | `107cc35` |
| 已安装 .so | `/opt/ros/melodic/lib/librosiwit_slam.so` md5 `fc640407d70f7621452fbd82f88fbbd0` | — |
| 配置 | `/opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml`（与设备仓库一致） | `ros1_ws/src/rosiwit_slam_ros1/config/default.yaml` |
| 仓库形态 | **shallow**，且 `git ls-remote origin` 超时（现场连不上 GitHub） | 正常 |

设备在我 push `107cc35`（11-24 13:32）之后继续提交，**从未上传**：

```
f4aff06  11-25 01:5x  rosiwit  低证据帧中性处理 + min_inliers_full_score 300->150   ← 原来未提交, 已补提交
95c4277  11-24 21:07  rosiwit  fix(matcher): 退化扫描打折增加内点豁免(小房间定位被误判丢)
3fb925e  11-24 20:47  rosiwit  fix(localization): 写图位姿一致性 + 子图/overlay 门控 + 一致性观测
55d7aba  11-24 18:41  rosiwit  docs(plan): overlay 单一置信度体素地图(w-map)改造方案
107cc35  11-24 13:32  jimingqing  ← 本地 HEAD / origin/master
```

> 注：`f4aff06` 是 11-24 21:11 改完但一直没提交的 4 个文件（config + pipeline.cpp/h + stage_types.h），
> 且**已包含在 21:16 编译的 0.2.16 二进制里**。2026-11-25 02:00 已在设备上补提交（工作区已干净）。

## 2. 配置差异（设备 vs 本地 107cc35）

只列影响定位/子图的，`session_map`/`overlay` 相关按现场要求已关闭，单列在末尾。

| 配置项 | 设备 | 本地 |
|---|---|---|
| `pipeline.local_map` | **keyframe_submap** | radius_submap |
| `submap.max_keyframes` | **24**（15->24） | 15 |
| `submap.seed_neighbor_check` / `_trans_m` / `_yaw_deg` / `_margin` | true / 0.5 / 5.0 / 0.03 | 无 |
| `submap.min_inliers_full_score` | **150** | 300 |
| `monitor.lost_fitness` | **0.60** | 0.75 |
| `monitor.anchor_fitness_base` / `_enter_frames` / `_return_frames` | 0.55 / 5 / 10 | 无 |
| `accept.min_coverage` | **0.6** | 0.7 |
| `accept.min_evidence_points` / `low_evidence_limit` | **150 / 300** | 无 |
| `maintainer.delta_min` | **0.5** | 0.3 |
| `maintainer.match_w_min` / `max_rmse` | 128 / 0.06 | 无 |
| `maintainer.persistency_unit` | **view** | session |
| `map_maintainer` | **none** | overlay |
| `session_map.enabled` | **false** | true |
| `session_map.radius_m` / `match_radius_m` / `voxel` | 35 / 35 / 0.3 | 25 / — / 0.2 |
| `session_map.write_fitness_bridging` / `write_max_corr_m` / `_deg` | 0.35 / 0.25 / 3.0 | 无 |

含义要点：
- 设备**把 local_map 改回 keyframe_submap**（不是 radius_submap），但把关键帧数提到 24、加了播种邻域校验；
- 设备把门限**放宽**（`min_coverage` 0.7->0.6、`lost_fitness` 0.75->0.60），并给"靠站/贴墙证据不足"加了**中性帧**处理（不累计不健康，超 300 帧仍判不健康）；
- 设备把 overlay 的写入条件加严（`match_w_min`、`max_rmse`、`delta_min 0.5`），但当前 `map_maintainer: none`、`session_map.enabled: false`，即两项自更新都关着。

## 3. 设备上的备份 / 同步素材（已生成）

```
/home/xzrobot/backup/dev_sync/0001-docs-plan-overlay-w-map.patch
/home/xzrobot/backup/dev_sync/0002-fix-localization-overlay.patch
/home/xzrobot/backup/dev_sync/0003-fix-matcher.patch
/home/xzrobot/backup/dev_sync/0004-fix-localization-min_inliers_full_score-300-150.patch
/home/xzrobot/backup/rosiwit_ws_src_20261125_0202.tgz     (源码快照, 28MB)
```

同步到本地/origin（在能连机器人网络的机器上）：

```bash
scp xzrobot@192.168.1.100:'~/backup/dev_sync/*.patch' /tmp/devsync/
cd <rosiwit_ws> && git am /tmp/devsync/*.patch && git push origin master
```

> 终端转发通道对长行会截断（实测 ~40 列），所以没有在线路里硬传 37KB 的 patch；
> 设备也连不上 GitHub（`ls-remote` 超时），只能走人工/局域网拷贝。

## 4. 结论（给现场）

1. 之前"设备 vs 本地不一致"**不是配置被改乱**，而是设备 11-24 晚上继续开发了 3 个提交 + 4 个未提交文件；
2. 那批改动已补提交（`f4aff06`），设备仓库现在干净；补丁和源码快照在 `~/backup/`；
3. 本地录包验证用的是 `radius_submap + session/overlay 开`，**不能代表设备**；
   要复现设备行为，需要按上表把配置切到设备值（或直接同步设备代码后重跑）；
4. 设备连不上 GitHub，后续同步请走局域网拷贝 + 本地 push。
