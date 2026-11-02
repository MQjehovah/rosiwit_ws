# 3D 语义目标核心（ROS 无关）

实施方案：[`docs/superpowers/plans/2026-11-01-person-3d-semantics.md`](../../../docs/superpowers/plans/2026-11-01-person-3d-semantics.md)

本目录是**算法核心**，不依赖 ROS；ROS1 薄节点在
[`ros1_ws/src/rosiwit_perception_ros1`](../../../ros1_ws/src/rosiwit_perception_ros1)。

类别不写死：`class_id` / `class_name` 从检测框一路带到跟踪输出，`person` 只是
配置里的第一个类别；新增 `car`/`bicycle` 等只需在节点的 `classes` 参数里追加。

```
include/object_semantics/
  object_types.h            数据结构与参数（类别/观测/跟踪/机器人状态）
  depth_associator.h        2D 框 → 深度 ROI → 稳健深度 → 反投影 + 协方差
  multi_object_tracker.h    world 系 2D 恒速 KF + 匈牙利关联 + 生命周期 + 双速度
  detector_postprocess.h    YOLO 输出解码 + 类别筛选 + 按类别 NMS + letterbox
src/                        上述实现
test/                       gtest（像素映射/深度统计/反投影/协方差/跟踪/速度/解码/NMS）
```

## 构建与测试

```bash
cd src/rosiwit_perception/object_semantics
cmake -S . -B build && cmake --build build -j8
ctest --test-dir build --output-on-failure
```

## 关键约定

- **坐标系**：位置输出在 `base_link`；跟踪内部在 `world`(odom) 系做 KF，
  因此"静止的目标"世界速度为 0，`velocity_base` 是相对机器人的接近速度。
- **外参**：`base_link <- camera_optical`（x 前/y 左/z 上 ← x 右/y 下/z 前），
  节点按彩色时间戳从 TF 查询注入，核心不假设任何安装角度。
- **深度**：`16UC1`、ROI 内取中位数；有效率 < `min_valid_ratio` 时
  `depth_valid=false` 并回退到地面投影（大协方差、低置信）。
- **尺寸先验**：用框顶射线在 median 深度处估计目标顶部高度，超出该类别的
  `min_height`/`max_height` 只降置信不丢点（避免蹲坐/儿童/远处车辆被直接丢弃）。
- **类别**：关联时要求 `class_id` 相同；NMS 按类别分组，不同类别的重叠框互不抑制。
- **状态**：`tentative → confirmed`（命中 3 帧）→ `coasting`（≤0.5 s）→ 删除；
  `velocity_valid` 仅在 confirmed + 深度有效 + 年龄 ≥0.3 s 时为真。

## 尚未实现（板端 / 后续）

- `detector=rknn` 的板端绑定：需要 `rknn_api.h` + `librknnrt` + 量化后的 `.rknn`
  模型才能验证；本机只有 ROS2，ROS1 包与 RKNN 分支均未编译验证。
- 多人同框的**拆分**（深度直方图双峰/连通域）：当前只置 `ambiguous` 标记。
