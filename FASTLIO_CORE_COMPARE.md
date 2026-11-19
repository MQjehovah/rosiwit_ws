# rosiwit_slam fast_lio2 核心 vs 官方 FAST-LIO2 / 朋友版

## 结论
- 默认参数/核心更新方程已对齐官方；0–400s 与官方轨迹逐点对比，**xy 偏差全程 ≤0.15m**，
  z 最大 1.33m（官方同段本身也漂，first |z|>1m at 312s，never >2m），max |v| 1.77 m/s。
- rosiwit 的所有自加功能都保留为开关，默认关闭 = 官方语义。
- GenZ-ICP 是朋友版额外的前端加权，默认关；打开后与朋友版一致（含 1.0 m² 近邻门限）。

## 朋友版
`FAST_LIO_ROS2` = 官方 FAST-LIO2 ROS2 + GenZ-ICP + 轮速/NHC。
- GenZ：5 邻域协方差 → planarity；平面点 point-to-plane(1行)，非平面点 point-to-point(3行)，
  Geman-McClure + sqrt(alpha) 混合；近邻平方距离门限 1.0 m²。
- 轮速/NHC 是底盘机器人专用，手持扫图仪不适用。
- 其余核心与官方一致。

## A. 保留的开关（默认关 = 官方）
| 开关 | 默认 | 打开后 |
|---|---|---|
| physical_clamp_enable | false | predict/update 物理限幅 |
| update_gate_enable | false | 整帧更新拒绝 |
| max_state_velocity | 0 | 速度/单帧步长限幅 |
| robust_kernel | none | tukey/cauchy/huber |
| min_effective_points | 1 | 有效点阈值 |
| imu_static_init | false | 仅低角速度样本初始化 |
| acc_scale_normalize | false | 9.81/\|a_mean\| 归一 |
| imu_gap_reset | false | IMU 空洞重置 |
| imu_max_prop_dt | 0 | 传播 dt 截断 |
| init_rot_cov | 1.0 | 初始姿态协方差 |
| **degenerate_skip** | **false** | 有效点过少/占比过低时跳过本帧更新 |
| **degenerate_min_effective_points / degenerate_ratio** | **0 / 0** | skip 的点数/占比判据 |
| **degenerate_gain** | **0** | (0,1] 弱方向自适应降权；0 = 只检测 |
| degenerate_eigen_ratio | 0.02 | 退化诊断阈值（检测日志始终开） |
| enable_genz_weight + genz_* | false | 朋友版 GenZ（门限 1.0 m²） |
| ground_follow_z + ground_* | false | 地面 z/vz 跟随 |
| flat_z_max | 0 | 硬 z 边界 |
| flat_z_sigma + flat_z_min/max_band | 0 | 软 z 死区 |
| dynamic_removal / free_space_carving | false | 动态点/自由空间 |
| odom_* / zupt_* / map_insert_* | off/0 | 底盘/静止/入图限制 |
| estimate_imu_lidar_extrinsic | false | 在线外参 |

## B. 本次补上的两个开关（之前未开关化）
| 开关 | 默认 | 官方语义 | 旧行为 |
|---|---|---|---|
| `drop_on_imu_gap` | **false** | 等 IMU，不丢帧 | true = 丢掉 IMU 覆盖不足的雷达帧 |
| `official_p_update` | **true** | 无有效点时仍按 H/b 更新 P | false = 直接 return，保留 predict 的 P |

## C. 本次删除的实验功能（已确认无用/死代码）
- `floor_z_sigma` / `floor_z_p_floor` / `floor_level_angle_deg` / `floor_rate_thresh` / `floor_step_thresh`
- `terrain_z_sigma` / `terrain_z_p_floor`
- `ground_normal_tilt_sigma` / `gate_deg` / `p_floor`
- `gravity_tilt_sigma` / `tau` / `p_floor` / `static_gate` / `gyro_max`
- `flat_vz_sigma`
- 随功能删除的前端成员：`m_floor_*`、`m_mean_acc_*`、`m_acc_lp*`、`m_ground_n_body`、`m_ground_h_body`。

> 退化检测/降权**没有删除**：`degenerate_eigen_ratio` 的检测与 `degen probe` 日志常开；
> `degenerate_skip` / `degenerate_gain` 恢复为开关，默认 false/0（官方行为）。

## D. 参数已对齐官方
na/ng=0.05，nba/nbg=0.001，lidar_cov_inv=5000，cube_len=10000，
lidar_max_range=60，scan=0.15，map=0.3，filter=3，near=5，max_iter=5，
acc_scale=9.81，收敛阈值 0.00015 m / 0.01°。

## 验证方法
`fastlio_official.yaml`（全开关关、官方参数）跑 0–400s，与官方 0–400 `odom_q.csv`
逐点对比。最终清理版 `core_official_clean_0_400`：4065 帧 / 407s，**dxy ≤0.15m**，
first |z|>1m at 312s，never |z|>2m，max |v| 1.77 m/s。
（退化检测日志正常：`ratio=0.0093 thr=0.02 n_weak=1`，仅上报、不改 H/b。）
