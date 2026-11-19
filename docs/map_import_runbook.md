# 扫图仪地图 → 机器人设备 导入 Runbook（交接版）

> 适用范围：把 `rosiwit_slam` 手持建图的结果导入机器人 `14004361`
> （`xz_3d_slam`，数据目录 `/opt/xzrobot`）。
> 本文所有结论都在真机上验证过；踩过的坑写在每节末尾。

---

## 0. 环境与凭证

| 项 | 值 |
|---|---|
| 设备 SN | `14004361` |
| 登录 | `xzrobot` / `xzyz2022!`（操作方式：terminal MCP：`connect_terminal` → `send_command`） |
| 设备 IP | 192.168.1.100 / 192.168.12.1 / 10.4.67.14(4G)，**连不到 192.168.31.34**，文件只能走 OSS |
| OSS | bucket `xz-server`，endpoint `oss-cn-shanghai`（AK/SK 见 `pack_*.py`；**建议轮换**） |
| 设备地图目录 | `/opt/xzrobot/maps/<uuid>/` |
| 设备业务库 | `/opt/xzrobot/database.db`（表 `map`） |
| 设备 SLAM 日志 | `/opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log`（10 MB 轮转，注意 `.1.log`） |
| 设备 App 日志 | `/opt/xzrobot/logs/xzrobot_app2/_<日期>` |
| 地图配置 | `/opt/ros/melodic/share/xz_3d_slam/conf/localization_conf.json`、`mapping_conf.json` |
| 导图工具（离线） | `~/rosiwit_ws/ros2_ws/install/rosiwit_slam_ros2/lib/rosiwit_slam_ros2/offline_map_export` |
| 导图实现（源码） | `ros2_ws/src/rosiwit_slam_ros2/src/io/robot_map_export.cpp`（与在线 `/save_robot_map` 共用） |
| 导入脚本（本机） | `C:\Users\mqjeh\AppData\Local\Temp\opencode\import_map.py`（设备侧只依赖 stdlib + `convert`） |

---

## 1. 总流程

```
建图 dump  ──► offline_map_export（或在线 /save_robot_map）──► map.pcd/map.pgm/map.yaml/map.xzmap
        ──► zip ──► OSS ──► 设备 curl ──► /tmp/xxx ──► import_map.py ──► /opt/xzrobot/maps/<uuid> + database.db
        ──► App 里选图 + 给初始位姿（或 rosservice /switch_map）
        ──► 看 xz_3d_slam.log 的 fitness_score_（目标 > 0.89）
```

## 2. 导出（关键：两条硬规则）

```bash
source /opt/ros/humble/setup.bash
source ~/rosiwit_ws/ros2_ws/install/setup.bash
offline_map_export <dump_dir> <out_dir> \
  --res 0.05 --kf-step 1.0 --generator ray_casting \
  --post denoise,close,dilate --close 3 --dilate 1 --min-comp 30 \
  --min-height 0.1 --max-height 2.0
```

`dump_dir` 需要：`global_map.pcd`、`observations.txt` + `obs/*.pcd`、`traj.csv`、`meta.txt`。

**必须看这行自检日志：**

```
[robot_map_export] .../map.xzmap kf=1492 pts=2741871 step=1m
   ground_z=-1.53762 (origin) pts/kf_med=1796 gap@kf_filter2 med=0.15 max=1.14m
```

| 字段 | 期望 | 不满足的后果 |
|---|---|---|
| `(origin)` | 必须是 origin | 出现 `global` = 原点附近没点，z 锚点会错 |
| `ground_z` | 原点地面，本图 −1.54 | 见 §2.1 |
| `pts/kf_med` | **> 1000** | <400 打 WARN，隔帧后地图出 1 m 空洞 → fitness 卡 0.75 |
| `gap@kf_filter2 max` | 远小于 2×8 m | >3 m 打 WARN，覆盖不足 |

### 2.1 z 锚点 = 地图原点附近的地面（不是全图最低峰）

- 图有 z 漂移（长坡道/回环）时，全局直方图峰值会被漂移区拉走：
  本图 `detectFloorZ` 取到 **−3.93**，整图抬高 3.93 m，原点地面变 **+2.36** → fitness 0.68，永远过不了 0.89。
- 正确做法：**第一个关键帧附近 12 m 内点云 z 的 2% 分位** → 平移到 0（本图 −1.538）。
- 设备上所有能用地图的原点地面都在 **+0.06…+0.16**（厂家 `站台`、`1`、`livox_pgm_lc3`、`AAA`）。

### 2.2 关键帧必须带"整片"点云

- 机器人 `localization_conf.json` 的 `kf_map_filter_num = 2` 是**隔一个丢一个关键帧**。
- 旧导出（每个点只归最近关键帧，≈1 m 切片）丢一半 = 地图上开 1 m 宽的沟：
  0.1 m 格子覆盖 278.8 → 141.5 m²（**−49.2%**），而匹配阈值只有 `√vf_scan_res`=0.447 m。
- 新导出：每帧取 **8 m 半径内整片点云**，自适应体素压到 2400 点 → 1796 点/帧
  （机器人自己的地图是 1.8k–2.5k 点/帧）。
- 真机效果：fitness **0.960**、inlier **0.996**。

### 2.3 xzmap 格式约定（真机逐条验证）

| # | 约定 | 依据 |
|---|---|---|
| 1 | `key_frame_point` 是**关键帧局部坐标** | `map_updater.cpp:581` 存 `T⁻¹`；`nano_gicp_localization.cpp:1646` 载入 `local2global` |
| 2 | 角度**毫弧度**（rad×1000） | `map_updater.cpp:603`；读取 `tf.yaw*0.001` 直接进 `pcl::getTransformation` |
| 3 | 位姿扁平 `z=0, roll=0, pitch=0` | `mapping_conf.json: fix_z / fix_rotation` |
| 4 | 地面 z ≈ 0（按原点定锚，见 §2.1） | 所有可用地图实测 |
| 5 | id 从 1、数值 int、`key_frame_points_2d` 可为空数组 | 设备解析器 |

自查方法：把 xzmap 的局部点按 pose 还原回世界系，和 `dump/global_map.pcd` 对比
z 中值应一致（同一 `ground_z` 平移）；`pose yaw` 与 `traj.csv` 的 `yaw*1000` 应逐帧吻合
（本图误差 0.25 mrad）。

## 3. 打包上传

```bash
# 本机/WSL：打包（zip 只放 4 个文件）
python3 -c "import zipfile,os;z=zipfile.ZipFile('m.zip','w',zipfile.ZIP_DEFLATED);[z.write(f,f) for f in ('map.pcd','map.pgm','map.yaml','map.xzmap')];z.close()"
# 上传 OSS 并拿 24h 预签名 GET URL（脚本已写好在 %TEMP%\opencode\pack_*.py）
python3 /mnt/c/Users/mqjeh/AppData/Local/Temp/opencode/pack_newcode.py
```

## 4. 设备侧下载 + 导入

```bash
# 1) 下载校验（必须与源 md5 一致）
cd /tmp && rm -rf genz && mkdir genz && curl -sSL -o genz.zip '<预签名URL>' \
  && md5sum genz.zip && unzip -oq genz.zip -d genz && ls -l genz

# 2) 写一个导入脚本（本机 import_map.py 用 base64 传过去，设备无需 numpy）
base64 -d > /tmp/import_map.py <<< '<base64>'

# 3) 导入（uuid = 毫秒时间戳，name = 地图名）
python3 /tmp/import_map.py /tmp/genz 1795057481684 genz_noclamp_1119
# -> IMPORTED (47, 'genz_noclamp_1119', '1795057481684', -208.436, -510.812, 0.05, '5b2883...')
```

`import_map.py` 做的事：
1. `cp map.pgm/map.pcd/map.xzmap` → `/opt/xzrobot/maps/<uuid>/`
2. 重写 `map.yaml`：`image: <绝对路径>/map.pgm`，`origin` 取自源 yaml，`occupied 0.65 / free 0.25`
3. `convert` 生成 `map.jpg`、`thumb.png`、`map_origin.jpg/pgm`
4. `md5 = md5(map.pgm)`，插 `database.db` 表 `map`
   （`name, uuid, originX/Y/Z, resolution, md5, floor, building`；floor/building 可空）

## 5. 切图 + 验证

```bash
# 方式 A：App 里选图 + 点位姿（推荐，用户操作）
# 方式 B：命令行（注意 map_dir 必须是 .../map.yaml，传目录会报 yaml-cpp: bad file）
source /opt/ros/melodic/setup.bash
rosservice call /switch_map "{map_name: '<name>', map_dir: '/opt/xzrobot/maps/<uuid>/map.yaml',
  map_resolution: 0.05, map_origin: [ox, oy, 0.0], map_negate: false,
  map_occupied_thresh: 0.65, map_free_thresh: 0.25,
  initial_pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: X, y: Y, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: QZ, w: QW}}, covariance: [0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}}}"
```

**替换文件后必须重新 switch（节点内存里还是旧图）**，日志里要有新的
`setKfMap(), kf_map_filter_num_ is 2, kf_points.size() is <N>`。

### 判定标准（`nano_gicp_localization.cpp`）

```
fitness = 命中点数 / 落在 [fitness_floor_cut=0, fitness_roof_cut=5.0] 的地图系 z 内的点数
命中条件：到最近地图点距离 ≤ √vf_scan_res ≈ 0.447 m
nr == 0  -> fitness 直接返回 0.0（全部被 z 带切掉）
放行：fitness > th_fitness_relocalization = 0.89 -> get_good_relocalization_ = true
丢失：fitness_count > 100 || !get_good_relocalization_  -> localization_lost = true
```

日志过滤：

```bash
grep -a 'fitness_score_ is' /opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log | tail -5
grep -a 'overlap final_fitness is' ...     # 这一行里的 inlier 是另一套指标（局部子图）
grep -ac 'localization_lost' ...
grep -a 'NO ENCOUGN' ...                   # 出现 = 扫描点被 z 带切光
```

正常参考值：**fitness ≥ 0.95、inlier ≥ 0.99**；0.68–0.79 说明导出有问题，不是环境问题。

## 6. 已知限制（本图）

`F:\scan_20261117_065219` 这条 1252 s / 1599 m 的图有 **~34 m 纯平移 z 漂移**
（`ground(x,y) − traj_z = −1.59 ± 0.29`，整条路径上传感器离地高度几乎不变）。

| 地面 |z| | 可用路径 | 占比 |
|---|---|---|
| ≤3 m | 437 m | 23.5% |
| ≤5 m | 587 m | 36.7% |

因为 fitness 的 z 带是**世界系固定 [0,5]**，地面掉到 −30 m 的地方线上点全被切掉 → fitness=0 → 丢失。

两条路（未做，待定）：

- **A 裁图**：只保留地面 |z| ≤ 3 m 的关键帧（可用 437 m）。
- **B 拉平**：`z' = z − plane(x,y)`（plane 用轨迹或地面场），地面处处=0 → 1600 m 全可用；
  2D 的 pgm 不受影响（它的带子是相对每帧地面的），xzmap 需要重导一次。

## 7. 当前状态（2026-11-19 15:5x）

| 项 | 值 |
|---|---|
| 地图 | name `genz_noclamp_1119`，uuid `1795057481684`，DB id=47 |
| 设备上的 xzmap | md5 `c1ae629c241dc5f8e4994fbf04229a33`（931 帧 / 206 万点 / 63 MB，验证通过版） |
| map.pgm md5 | `5b2883fcb18858f1729647ca19490228` |
| 实测（用户开一圈） | fitness 中位 **0.973**、p90 0.978、max 0.998；`lost` 仅 2 次瞬时（原位姿 z 始终 −2.2…+0.26） |
| 代码 | 已提交 `5efbf97`（关键帧整片点云 + 原点 z 锚 + 自检日志）；**git push 因网络失败，待重试** |
| 未决 | §6 的漂移处理方案；2D 图天花板被当障碍（见下） |

### 2D 图天花板问题（未修）

`min_height/max_height` 是**相对每帧 RANSAC 地面**的高度带，但手持 Mid-360 竖直 FOV 窄
（实测仰角 p5…p95 = −20°…+63°，25° 倾斜已生效），少数帧脚下没有地面点（如 frame 0 最远只到 10.2 m），
2% 分位取到远处地面 → 带子抬高 → 天花板进障碍层。

修法：把栅格的地面改成**按位置的地面场**（与定 z 用同一套），重跑 PGM（约 10 s）。

## 8. 临时脚本清单（本机 `%TEMP%\opencode\`）

| 脚本 | 用途 |
|---|---|
| `import_map.py` | 设备侧导入（建目录/写 yaml/生成 jpg/插库） |
| `pack_*.py` | 打包 + 上传 OSS + 出预签名 URL |
| `build_scan_xzmap.py` | 从 `dump/obs` 用真实逐帧点云重建 xzmap（扫描式关键帧） |
| `ground_field.py` / `align_ground.py` / `mk_h095.py` | 地面场、z 平移试验（历史） |
| `check_holes.py` / `check_ceiling2.py` / `probe_fov.py` | 空洞、天花板、FOV 诊断 |
| `audit_map.py` | xzmap/PGM/yaml 一致性审计（含 2D↔3D 坐标系检查） |
| `do_commit.sh` | 提交脚本（可删） |
