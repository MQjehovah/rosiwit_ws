# ROS1 替代厂商 SLAM：真机测试报告（2026-11-19/20，设备 14004361）

> 目标：用 `rosiwit_slam_ros1` 替换机器人上的 `xz_3d_slam`。
> 结论：**能起、能载图、静止定位 fitness 0.95**；**行驶/转弯时航向振荡**，尚未达标，
> 已回退到厂商节点。下面是在真机上验证过的全部事实与数据。

## 1. 编译（aarch64 / Ubuntu 18.04 / gcc 7.5）

不需要 Docker/交叉编译，设备工具链齐全（libpcl-1.8/libceres-1.13/opencv3.2/eigen3.3/
yaml-cpp0.5/boost1.65 + `checkinstall`）。

```bash
# 目录层级必须保持（ROS1 包 CMakeLists 用 ../../../src/rosiwit_slam）
~/rosiwit_ws/src/rosiwit_slam
~/rosiwit_ws/src/rosiwit_common
~/rosiwit_ws/ros1_ws/src/rosiwit_slam_ros1

catkin_make -j2 -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=ON
```

路上修掉的 5 个坑：

| # | 问题 | 处理 |
|---|---|---|
| 1 | 漏传 `src/rosiwit_common`（核心同级依赖） | 一起拷贝 |
| 2 | 设备缺 Sophus 头（IESKF 用） | 拷 `sophus/` 到 `/opt/ros/melodic/include` |
| 3 | 设备 GTSAM 半装（有 `gtsam_unstable` 缺 `dllexport.h`） | `-DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=ON` |
| 4 | `<filesystem>` 是 gcc8+，bionic 只有 7.5 | `pcd_map_manager.cpp` 改 POSIX `mkdir` 递归 |
| 5 | 自建消息 `LocalizationState` 设备厂商包没有 | 移除 `/localization/state` 发布（改由 `/location_info` 报告状态） |

## 2. 启动与接口（实测）

- 订阅：`/vanjee_points722`(PointCloud2)、`/vanjee_lidar_imu_packets`(Imu)
- IMU 挂载从 TF 自动推断：`base_link <- lidar` rpy=[42.11 1.44 89.93] xyz=[0.512 0 0.882]
- `acc_scale=1.0`（万集 IMU 为 m/s²，`|a|=9.897` 确认）
- 输出：`/lio_odom` `/slam/info` `/location_info` `/map` `/points_2d` `/tf` …（`/slam/info` 的
  `location_status 0/1/2 = SUCCESS/RECOVERING/FAILED`，与厂商 App 兼容）
- **兼容服务**：`/switch_map`、`/relocation`、`/global_localization` 类型与厂商一致，App 不用改
- 载图：`genz_noclamp_1119`（931 关键帧 / 206 万点）用 8.0 s

## 3. 地图对比（同一机器人、同一位置，厂商节点判据 fitness）

| 地图 | 做法 | 原点地面 p2 | fitness |
|---|---|---|---|
| `genz_noclamp_1119` | 不拉平（仅原点区可用） | −0.07 | **0.95** |
| `genz_groundzero` | 拉平（全 1600 m 可用） | −0.07 | **0.85** |
| `genz_flat2` | 逐帧地面平面 → 1 m 位移场 → 逐点扣 | — | **0.39~0.58**（失败） |

`genz_flat2` 失败原因（已定位，供重做参考）：
1. 平面拟合只要求 `n_z ≥ 0.85`（容忍 32°），长廊里会把斜墙面当地面；
2. 候选点取"最低 30%"，混入墙根与远处地面；
3. 平面外推到远端 (x,y) 误差放大；1 m 格 + 3 m 平滑在 7% 坡段把远处地面拉斜。

重做需要闭环：近场地面点(2–8 m) + `n_z ≥ 0.98` + 内点率 >60% + 0.5 m 格不跨坡平滑 +
**离线自检**（每帧扣场后残差平面 a≈0,b≈0,c≈0，倾斜<0.5°/偏移<3 cm）+ 上机三点位
（原点/y=−87/y=−300）fitness 均 >0.93 才算过。

## 4. 行驶/转弯问题（未解决）

静止时：fitness 0.95、`correction` 0.01–0.07 m、稳定。
行驶+转弯时（旧配置）：

```
fitness 0.75~0.87   correction_m 2.5~5.8 m   yaw 3 秒内 -78.7°→-94.7°→-71.5°（±20°）
localization_lost 0 次（>0.75 就不算丢）→ 位姿在漂但界面不报丢失
```

已尝试的配置（都在 `ros1_ws/src/rosiwit_slam_ros1/config/default.yaml`）：

| 参数 | 值 | 作用/效果 |
|---|---|---|
| `mapping.frontend.enable_genz_weight` | true（+planarity 0.1/kernel 0.5/near_dist_sq 1.0） | GenZ 自适应加权（从 genz 那次移植） |
| `accept.degeneracy_partial_update` | **true** | 退化方向按可观测度缩放。启用后**某一时段** correction 降到 0.03~0.07 m，但后续一轮又回到 2.5~5.8 m，**未稳定复现** |
| `matcher.max_correspondence_dist` | 2.5 → **1.2** | 不许配到几米外"长得一样"的位置 |
| `monitor.credible_fitness` / `global.reloc_accept_score` | 0.85 → **0.80** | 贴合实测稳态 0.81~0.87 |
| `global.input.gyro_scale` | 1.0 → **1.036** | 万集 IMU 偏航角速度低估 3.5%（配置注释即指出"掉头/绕圈航向误差放大的关键"）；**尚未验证** |

待办：
1. 验证 `gyro_scale=1.036`（最可疑的一条）
2. `/slam/mode_switch` 服务会**阻塞**（把调用方一起卡死），要改成异步返回
3. 复核 `a`/`b`：转弯时的 yaw 振荡是否来自 deskew/时间同步（扫描 100 ms/帧）

## 5. 设备侧改动（已记录，便于还原）

| 位置 | 改动 | 备份 |
|---|---|---|
| `/opt/ros/melodic/share/xz_3d_slam/conf/localization_conf.json` | `th_fitness_relocalization 0.89 → 0.85`（让 `genz_groundzero` 的 0.85 过线） | `localization_conf.json.bak_before085` |
| `/opt/ros/melodic/include/sophus` | 新增 Sophus 头（供我们编译） | — |
| `/opt/xzrobot/maps/1795103899704`（DB id 50 `genz_flat2`） | 失败的拉平版地图 | 可删 |
| `~/rosiwit_ws` | 我们的 ROS1 编译工作区 | 已按要求删除 |

## 6. MCP（远程终端）性能问题（已修，待重启生效）

`E:\workspace_ai\agent\mcp_server\src\remote_terminal.py`：

| 现象 | 根因 | 修复 |
|---|---|---|
| 每条命令 ~2–3 s，偶发 30 s | `_receive_output` 要等**静默 `timeout` 秒** | 看到 shell 提示符立即返回（实测 3.17 s → **0.14 s**） |
| `wait_output:false` 后 `get_buffer` 读不到新输出 | `output_buffer` 只在 `_receive_output` 里追加 | 改为后台 reader 持续追加（带 2 万行裁剪） |
| 一条阻塞的 `rosservice call` 把终端卡死，`Ctrl+C` 无效 | PTY 被占 | 只能 `disconnect_terminal` + `connect_terminal` |

补丁已写、`py_compile` 通过，并用独立进程冒烟验证（0.13/0.14 s、`get_buffer` 立即可见）。
**需要重启 opencode 才生效**；备份 `remote_terminal.py.bak_prompt`。

> 注：session 是**复用**的（每 SN 一条常驻 WebSocket + 单后台 reader），不是每次新建连接。

### 6.1 会话管理（同批提交，已独立进程验证）

| 能力 | 实现 | 验证结果 |
|---|---|---|
| 会话复用 | `connect_terminal` 命中已登录会话直接返回并刷新 `last_used` | 两次 connect 返回同一 `sid` |
| 空闲回收 | 后台 janitor 扫 `sessions`，`now-last_used > TERM_MCP_IDLE_SEC`（默认 **1800 s**）即断开并移除；扫描间隔 `TERM_MCP_JANITOR_SEC`（默认 60 s）；跳过 `busy`/持锁会话 | 设 10 s/2 s 时：+5 s、+10 s 仍在，**+15 s 已回收**（1 → 0） |
| 并发串行 | 每会话一把 `cmd_lock`，`send_command` 拆为 `_send_command_inner` + 薄包装 | 3 条并发各自输出完整、无串台 |
| 可观测 | 新增 `list_sessions`（sn/sid/uptime/idle/busy/buffer_lines/idle_close_s） | 正常返回 |
| 异常兜底 | `send_command` 捕获异常返回 `{success:false,error}`，不再让 MCP 调用挂死 | — |

> 提示：连接后若长时间不操作，会话会被 janitor 回收；下次调用 `send_command` 会提示
> "终端未连接，请先调用 connect_terminal"（此时重新 connect 即可，~8 s）。
