# rosiwit_perception_ros1

前置深度相机 3D 语义目标（位置 + 速度）的 ROS1 薄节点。

- 算法核心在 [`src/rosiwit_perception/object_semantics`](../../../src/rosiwit_perception/object_semantics)（ROS 无关，独立单测）
- 类别可配置：默认 `person`，新增 `car`/`bicycle` 等只改 `classes` 参数
- 实施方案：[`docs/superpowers/plans/2026-11-01-person-3d-semantics.md`](../../../docs/superpowers/plans/2026-11-01-person-3d-semantics.md)

## 构建

```bash
cd ros1_ws
source /opt/ros/melodic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=rosiwit_perception_ros1
```

## 运行

```bash
roslaunch rosiwit_perception_ros1 object_semantics.launch
# 无 NPU 时用外部 2D 检测做 bring-up
roslaunch rosiwit_perception_ros1 object_semantics.launch detector:=external
```

| 话题 | 说明 |
|---|---|
| `/perception/objects` | `SemanticObjectArray`（全部类别） |
| `/perception/persons` | 仅 `person` 子集（导航/旧接口） |
| `/perception/objects/markers` | RViz 球 + 速度箭头 + 文本 |
| `/perception/objects/image` | 标注图 |
| `/perception/objects/status` | JSON：FPS/耗时/深度有效率 |
| `/perception/objects/log` | JSONL：每目标每帧一行 |
| `/perception/objects/detections_2d` | `detector=external` 时的输入（自带 `class_id`） |

## 接入 sim_slam_nav.launch

`roslaunch rosiwit_starter_ros1 sim_slam_nav.launch` 默认会：

- 给仿真车加前视彩色/深度相机（`/camera_01/...`，装在实机 `camera_01_link` 位置）
- 在车前方生成 3 个行人占位模型（`person_1..3`）
- 跑 `sim_person_detector.py`：按 Gazebo 真值位姿投影出 2D 框
- 跑本包的 `object_semantics_node`（`detector=external`），输出 `/perception/objects`

推荐用开阔世界看效果（默认 house.world 在 (0,0) 没地面）：

```bash
roslaunch rosiwit_starter_ros1 sim_slam_nav.launch \
  world_name:=$(rospack find rosiwit_simulation_ros1)/world/perception.world
```

不需要感知时加 `enable_perception:=false`。

## 闭环仿真验收

不需要 Gazebo/真实相机：`sim_scene_node` 用已知几何渲染彩色、深度、相机内参、TF、odom 和真值框，
跑**真实的** `object_semantics_node`，再由 `sim_eval_node` 对真值统计位置/速度/ID/虚假轨迹。

```bash
bash src/rosiwit_perception_ros1/scripts/run_sim_test.sh
```

跑两个场景（`persons` 4 人互不遮挡、`car` 单车），全部 PASS 返回 0；日志在
`/tmp/object_semantics_sim_<scenario>.log`，逐帧 CSV 在
`/tmp/object_semantics_sim_<scenario>.csv`，报告在
`/tmp/object_semantics_sim_report.txt`。

当前基线（20 s、10 Hz、深度噪声 + 10% 空洞、检测框抖动 ±1.5 px）：

| 场景 | 匹配率 | ID 切换 | 位置 RMSE | 速度 RMSE |
|---|---|---|---|---|
| persons | 100% | 0 | 0.032 m | 0.017 m/s |
| car | 100% | 0 | 0.005 m | 0.018 m/s |

仿真的边界：

- 检测框由真值投影生成，**不验证**检测器 AP、RKNN、真实深度噪声与标定误差；
- 多人同框遮挡的帧标记为不可见并跳过（MVP 不支持拆分），不算漏检；
- 真值目标中心 vs 深度前表面：`classes[].surface_offset` 必须按类别配置
  （人约半身厚 0.15 m，车辆按车长），否则位置会系统性偏向相机一侧。
