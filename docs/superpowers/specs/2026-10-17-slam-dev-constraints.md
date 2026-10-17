# 开发约束与规定（建图 / 定位）

> 这份文档是**硬约束**，不是建议。新增代码、改配置、联调排查前先读一遍。
> 来源：2026-10-17 ROS1 实机联调（14 个缺陷的复盘），每一条都对应一次真实故障。

---

## 1. 同一功能只能有一处实现（最高优先级）

实机代价：同一个"初始位姿换算"逻辑写了两次（`/initialpose` 一份、`/switch_map`+`/relocation` 一份），
只改了其中一处，导致"切定位后角标还是差 90°"，多花了两轮实机。

规则：

1. **入口唯一**：初始位姿只有一个入口 `SlamEngine::applyInitPose(in, zero_if_insane, force_z_zero)`。
   `/initialpose`、`/switch_map`、`/relocation` 全部委托给它，差异只能体现在**参数**上。
2. **查一次、缓存**：TF/安装角只在 `SlamEngine::lidarToBase()` 里查一次并缓存；
   100 Hz 回调里**禁止**做会阻塞的查询（见第 5 条）。
3. **同一功能出现第二种实现时，先删旧的再合新的**，不允许"两份并存、各自演进"。

## 2. 坐标系约定（唯一权威表）

| 量 | 坐标系 | 说明 |
|---|---|---|
| 核心内部位姿 `SlamOutput::pose` | **雷达/IMU 刚体系** | `r_il/t_il` 描述它与雷达的关系（本机两者同 frame → identity） |
| 核心输出 `world_cloud` | map 系 | 已含位姿，**不需要**再做安装角换算 |
| `curr_localization_pose` / `slam_info.robot_pose` / `map→odom` TF | **base_link 系** | 经 `r_lb_` 换算后发布 |
| 上层下发的初始位姿 | **base_link 系** | 进核心前换算成雷达系 |
| 地图文件（`.xzmap` / PCD） | 核心内部（雷达系） | 与内部位姿一致 |
| 2D 栅格 `map.pgm/.yaml` | map 系，`origin` = 栅格左下角 | 写盘时首行=最大 y；**读取时必须解析 origin** |
| 2D 栅格高度带 | **相对地面**：`z - ground_z ∈ [grid_min_height, grid_max_height]` | `ground_z ≈ -t_lb.z()`（本机 −0.898）；**不要用绝对 z** |

**换算只允许出现在两个地方**：
- 输入侧：`SlamEngine::applyInitPose()`（base → 雷达）
- 输出侧：`LegacyXz3dSlamInterface::publishCurrPose()`（雷达 → base）

任何其它位置出现安装角换算，都属于违规复制。

## 3. 核心层与 ROS 薄层分离

1. `src/rosiwit_slam`（核心）**不允许**出现 ROS 头文件 / 类型 / 参数读取 —— 它同时被 ROS1/ROS2 复用。
   需要开关就加**成员变量 + 配置项**，不要 `ros::param`。
2. ROS 相关的一切（topic、service、TF、参数）只存在于 `ros1_ws/rosiwit_slam_ros1`（薄层）。
3. 薄层只做三件事：**取数据 → 调核心 → 发布结果**。算法决策不下沉到薄层。
   （反例：薄层曾自己实现一套栅格生成，与核心不一致 → 实时 `/map` 有地面噪点、存盘 pgm 干净。）

## 4. 参数：可调项必须集中标注

1. 所有可调参数集中在 `ros1_ws/src/rosiwit_slam_ros1/config/default.yaml`，**同名参数只允许出现一次**。
2. 关键参数必须写 3 行注释：**当前值 / 实测依据 / 调大调小的后果**。
3. 改参数必须在提交信息里写"为什么"和"实测数据"。禁止裸调（示例：把 `planar.mode` 设成 `none`
   必须写明"雷达俯装 41.67°，约束拉平导致覆盖率 0.43 → 关闭后 0.99"）。

## 5. 并发与实时性（100 Hz 回调内禁止阻塞）

| 禁止 | 后果（实测） |
|---|---|
| 在定时器回调里 `lookupTransform` 且失败后重试 | 每次最多阻塞 3 s，100 Hz 定时器被拖死，`/slam/info` 与位姿全部停发 |
| `std::thread::join()` 等待后端线程 | 后端在 `optimize()` 时服务线程被挂死，表现为"切图卡住"（复现两次） |
| 每帧重建大 KD-tree / 全图体素 | 定位输出掉到 1.4 Hz |

替代方案：一次查询 + 缓存；`detach` + STOP 任务；缓存与门限（见 `matcher` 参数注释）。

## 6. 日志分级（现场排查的前提）

- **启动类/一次性**日志：保留并用 `std::cout << … << std::endl`（roslaunch 管道下 ROS_INFO 会延迟落盘）。
- **每帧/每秒**日志：必须可关（开关默认安静）或强节流；否则既刷屏又拖慢现场判断。
- **诊断探针**命名统一：`[LocProbe]`（定位）、`[PubProbe]`（发布链路）、`[FLIO2]`（前端外参）。
  探针的作用是**区分"两个界面表现相同、原因不同"的故障**，例如：
  - `[PubProbe]` 有心跳但 `have_new_pose=0` → 核心没产出位姿
  - `[PubProbe]` 没有心跳 → 发布定时器被阻塞

## 7. 定位/建图联调的经验规则

1. **先确认模式**：FREE 模式下按旧实现**不发布**任何位姿/点云。
   "什么都没有"的第一嫌疑是模式没切，不是算法坏。（重启后 `slam_init_type: free` → 回 FREE）
2. **覆盖率 `fitness` 是 `cs.coverage`（0~1，越大越好）**，不是 gicp 的误差分数。
   覆盖率低 = 点云与地图系统性错位，不是"抖动"。
3. **`planar` 约束假设车体水平**。若核心 body 是雷达系且雷达有安装角（本机 pitch 41.67°），
   必须关闭或改到 base_link 系做，否则覆盖率会被压到 ~0.43。
4. **位姿会跑出地图**：一旦局部子图为空，运动先验仍在积分 → 漂到几十米后再也救不回来。
   必须有逸出保护（回退到最近被接受的位姿 + reset 先验），并向上层报"丢失"。
5. **匹配成功 ≠ 被采纳**：`acceptMatch()` 有门限（`min_coverage` 等），
   拒绝时必须能说明原因（`[LocProbe] REJECT`），否则现场无法判断。

## 8. ROS1 / ROS2 共用核心的改动守则

`src/rosiwit_slam` 的改动会同时影响 ROS2 节点。改这些文件后必须：

1. 本机跑核心单测（`catkin_make run_tests_rosiwit_slam_ros1`，当前 54 个全绿）；
2. 在 ROS2 环境跑 `colcon build` + `colcon test`；
3. ROS2 端各跑一次建图/定位冒烟，重点看：
   - 建图：`[PcdMapManager] Rebuilt map … matched k/k`、`anchor trans` 在 cm~m 级
   - 定位：`matching_degree` 正常、输出频率 ≥10 Hz
4. **高风险文件**（改完必须端到端验证，不只是编译）：
   `ceres_backend.cpp`（后端残差）、`pcd_map_manager.cpp`（地图重建/栅格）、
   `localization_pipeline.cpp`（接受门限）、`slam_pipeline.cpp`（模式分发）

## 9. 提交规范

1. 一个缺陷一个提交，提交信息写清：**现象 → 根因（含实测数字）→ 改动 → 验证**；
2. 涉及实机验证的，把关键日志片段贴进提交信息（如 `fitness 0.434 → 0.992`）；
3. 禁止把调试探针与功能改动混在同一个提交里（便于回滚）。

## 10. 安装角与轴系定案（2026-10-17 三项独立实测，不再讨论）

**结论：URDF 原始 rpy(41.67, 1.82, 89.89) 就是真实安装；点云/IMU/URDF 三者轴系一致，不存在任何"驱动 90° 旋转"。`mount_extra_*` 默认必须全 0。**

证据链（任何一条被质疑时重测对应项）：

| # | 测量 | 结果 |
|---|---|---|
| 1 | IMU 静止重力（`/vanjee_lidar_imu_packets`，静止均值） | `[-0.02, 0.66, 0.75]`，与 R_origᵀ·ẑ 预测 0° 偏差；Rx90 家族预测 ~89° 偏差 |
| 2 | 原始点云地面层（`/vanjee_points722` 单帧，按候选矩阵投影后找 10cm 薄层） | R_orig：地面 z=-0.93m（=安装高度 0.898m✓），25.1% 点在层内；Rx90·R_orig：5.8% 无地面层 |
| 3 | 地图世界系水平度（解析 `map.xzmap` 的 `kf_frame_poses`，位姿平面拟合） | DDD 倾斜 1.0°、FFF 1.8°、CCC 1.2°（健康）；**旧图 AAA 倾斜 59.8°、关键帧 z 漂到 +69m（废图，已删）** |

历史教训：当年"Rx(90) 让 BBS 0.598→0.9958"的实验是在 AAA（世界系斜 60° 的废图）上做的——
Rx90 补偿的是**坏地图的倾斜**，被误当成"驱动轴系约定"写进了代码（f1c4266，已在 2919834 撤销）。
**凡是从实验得出"常量修正"，必须先证明实验所用的地图/数据本身是健康的。**

附：旧实现（xz_3d_slam 源码）在**入口处**把点云变换到车体系（`Ry(45°)·Rz(90°)`+t(0.52,0,0.916)，XYZ 序），
核心直接输出车体位姿；本移植反其道（核心吃原始点云、发布时经 TF 换算）。两者数学等价，但**换算位置只能有一处**（见第 2 条）。

## 11. 参数服务器残留陷阱（rosrun 私参不随节点退出清除）

实测踩坑：`rosrun ... _mount_extra_yaw_deg:=90` 在节点退出后**仍留在参数服务器**，
下次不带该参启动时会静默生效，导致"代码默认值"与"实际生效值"不一致（本次 yaw=90 残留实锤）。

规则：
1. 任何带 `_param:=` 的实验性启动，结束后必须 `rosparam delete /xz_3d_slam_node/<param>`；
2. 排查"行为与代码不符"时，第一件事 `rosparam list | grep xz_3d_slam_node` 对比预期；
3. 节点启动日志必须打印关键参数的**生效值**（如 `[compat] effective ...`），不能只打印来源。

## 12. 地图健康度快检（建图/定位排查前置步骤）

用 `map.xzmap` 的 `kf_frame_poses`（×1000 量化）做平面拟合：法向偏离 z 轴 >5° 即废图
（正常 1~2°）。py2 脚本要点：`# -*- coding: utf-8 -*-` 头 + ASCII 输出 + 位姿正则要含
`key_frame_id` 前缀。**换地图调试前先跑这一步**——在废图上得出的任何结论（包括上面的 Rx90 教训）都不可信。

## 13. 未完成事项（截至 2026-10-17）

| 项 | 状态 |
|---|---|
| ROS2 `colcon build` + 端到端冒烟 | **未执行**（本机仅装 melodic） |
| 薄层 `buildGridFromPoints` 死代码清理 | ✅ 已删除 `1f287fd`，测试改为覆盖 `setGridSource` `61d9248` |
| `setGlobalGridMap()`（无人调用） | ✅ 已删除 `1f287fd` |
| `[LocProbe]`/`[PubProbe]` 探针降频 | ✅ 5s / 10s 一条 `1f287fd`；如需完全静默再引入成员开关 |
| 重启后恢复上次模式（可选） | 用户明确暂不需要 |
| 万集驱动 `ros_frame_id: lidar` 改在安装目录 | 重装/OTA 会覆盖，需纳入配置管理 |
| `mount_extra_*` 归零 + 参数残留清除 + 实机复验 | ✅ `2919834`；fpfh 0.994 / matching 0.98 / fitness 0.983（DDD） |
| AAA 废图删除（目录+DB 行） | ✅ 2026-10-17；DB 备份 `database.db.bak_before_del_aaa` |
| BBB 地图（仅 1 关键帧、0.3MB，基本为空） | 待用户决定是否删除 |
| BBS 车体系投影：同一矩阵同时作用于地图+扫描，在"地图已水平、扫描带安装角"时非最优（共轭抵消后仅 z-band/2D 搜索受影响） | 低优先（默认重定位走 FPFH，3D 且与安装无关） |
| 快速旋转丢失定位根因 | 需旋转时录 rosbag 复现 |
| FFF 地图实机定位验证 | 未做（与 DDD 同管线，预期一致） |
| 三张原厂旧图（京东mall/jdmall/5楼，2026-01/05 建造）能否被本移植读取 | 未测；旧格式 `kf_frame_poses` 解析为 0，需确认 readXzMap 兼容性后再给用户用 |

## 14. 2026-10-17 晚事故：节点崩溃 + 接口卡死（已修复 132b57c）

**现象**：① /relocation（指定初始位姿重定位）接口很慢；② 定位突然完全无输出。

**根因一（崩溃）**：错误位姿下 dynamic_filter 把扫描滤到 16 点 → PCL 1.8 GICP
computeCovariances（要求点数 ≥ k_correspondences=20）**段错误**，进程直接消失
（apport/journal 均无记录，但单测 	est_gicp_matcher_guard 用 4×4 点云确定性复现
exit 139，铁证）。修复：GicpMatcher 最小点数守卫（matcher.min_match_points，默认 50）。

**根因二（接口卡死）**：节点是单线程 os::spin()，所有回调串行。丢失后冷启动一帧内
最多 = bounded recovery 27 GICP + fpfh top_k=5 × 两级精配 200 GICP ≈ 上千次 GICP，
单帧回调分钟级 → /relocation 等服务排队数十秒、输出停更；且 weak-reject 路径**漏设冷却**，
下一帧立刻再来全量风暴。修复：候选按分数降序 + 达标即停（0.95/0.97）+ 3s 墙钟预算
（精配与 bounded recovery 双双生效）+ weak-reject 补冷却 + etry_interval 30→45 帧。

**实测**：/relocation 正常位姿 RTT 0.00s、输出恢复 0.09s；/global_localization
40s→13~15s（fitness 0.996）；图外位姿重定位不再崩溃。

**遗留（按优先级）**：
1. 图外位姿 /relocation 最坏仍可排队 ~10s（fpfh 特征计算本身未预算、单线程 spinner 架构性限制）。正确方向：核心内部用独立工作线程跑重定位，onLidar 只置标志——改动大，未做。
2. /switch_map 慢：LocalizationPipeline::setMap 把 46MB xzmap **解析两遍**（L300 kfs_fb + L334 kfs，手写 char 级 JSON 解析器），同步阻塞服务线程——待单次解析+共享数据。
3. 显示点云含非墙体点（疑似地面泄漏）：scan_f_map 过滤带 z-ground ∈ (0, 2.0) 下界
   含 0 会放行贴地点；or_curr_scan_point 完全无 z 过滤且坐标系标注为 map 实为驱动系——待与
   E:\workspace_slam\3d_slam 旧实现核对语义后修。
4. PCL 

## 14. 2026-10-17 晚事故：节点崩溃 + 接口卡死（已修复 `132b57c`）

**现象**：① /relocation（指定初始位姿重定位）接口很慢；② 定位突然完全无输出。

**根因一（崩溃）**：错误位姿下 `dynamic_filter` 把扫描滤到 16 点 → PCL 1.8 GICP
`computeCovariances`（要求点数 >= k_correspondences=20）**段错误**，进程直接消失
（apport/journal 均无记录，但单测 `test_gicp_matcher_guard` 用 4x4 点云确定性复现
exit 139，铁证）。修复：`GicpMatcher` 最小点数守卫（`matcher.min_match_points`，默认 50）。

**根因二（接口卡死）**：节点是单线程 `ros::spin()`，所有回调串行。丢失后冷启动一帧内
最多 = bounded recovery 27 GICP + fpfh top_k=5 x 两级精配 200 GICP ≈ 上千次 GICP，
单帧回调分钟级 → /relocation 等服务排队数十秒、输出停更；且 weak-reject 路径**漏设冷却**，
下一帧立刻再来全量风暴。修复：候选按分数降序 + 达标即停（0.95/0.97）+ 3s 墙钟预算
（精配与 bounded recovery 双双生效）+ weak-reject 补冷却 + `retry_interval` 30→45 帧。

**实测**：/relocation 正常位姿 RTT 0.00s、输出恢复 0.09s；/global_localization
40s→13~15s（fitness 0.996）；图外位姿重定位不再崩溃。

**遗留（按优先级）**：
1. 图外位姿 /relocation 最坏仍可排队 ~10s（fpfh 特征计算本身未预算、单线程 spinner 架构性限制）。正确方向：核心内部用独立工作线程跑重定位，`onLidar` 只置标志——改动大，未做。
2. /switch_map 慢：`LocalizationPipeline::setMap` 把 46MB xzmap **解析两遍**（L300 kfs_fb + L334 kfs，手写 char 级 JSON 解析器），同步阻塞服务线程——待单次解析+共享数据。
3. 显示点云含非墙体点（疑似地面泄漏）：`scan_f_map` 过滤带 `z-ground ∈ (0, 2.0)` 下界含 0 会放行贴地点；`or_curr_scan_point` 完全无 z 过滤且坐标系标注为 map 实为驱动系——待与 E:\workspace_slam\3d_slam 旧实现核对语义后修。
4. PCL "Failed to find match for field normal_x" 告警刷屏（~4 行/帧，cosmetic），源头待查。
