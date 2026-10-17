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

## 10. 未完成事项（截至 2026-10-17）

| 项 | 状态 |
|---|---|
| ROS2 `colcon build` + 端到端冒烟 | **未执行**（本机仅装 melodic） |
| 薄层 `buildGridFromPoints` 死代码清理 | ✅ 已删除 `1f287fd`，测试改为覆盖 `setGridSource` `61d9248` |
| `setGlobalGridMap()`（无人调用） | ✅ 已删除 `1f287fd` |
| `[LocProbe]`/`[PubProbe]` 探针降频 | ✅ 5s / 10s 一条 `1f287fd`；如需完全静默再引入成员开关 |
| 重启后恢复上次模式（可选） | 用户明确暂不需要 |
| 万集驱动 `ros_frame_id: lidar` 改在安装目录 | 重装/OTA 会覆盖，需纳入配置管理 |
