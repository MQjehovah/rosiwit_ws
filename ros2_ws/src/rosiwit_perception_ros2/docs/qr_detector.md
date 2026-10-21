# 二维码识别与视觉对接（QR Detector）

相机订阅 + 多二维码检测/解码 + 3D 位姿 + 多帧确认 + TF map 位姿 +
视觉伺服靠近 action。

## 结构

| 层 | 文件 | 职责 |
|---|---|---|
| 核心（无 ROS） | `src/rosiwit_perception/rosiwit_perception/qr_detector.py` | OpenCV 多码检测、角点排序、solvePnP、伺服控制律 |
| ROS 2 薄层 | `ros2_ws/src/rosiwit_perception_ros2/rosiwit_perception_ros2/qr_detector_node.py` | 订阅、多帧确认、TF、JSON/消息发布、action |
| 消息 | `rosiwit_perception_msgs/msg/QrCodeArray.msg` | 结构化结果 |
| 动作 | `rosiwit_perception_msgs/action/ApproachQr.action` | 靠近/对接 |

## 运行

```bash
ros2 launch rosiwit_perception_ros2 perception.launch.py
# 或单独启动
ros2 launch rosiwit_perception_ros2 qr_detector.launch.py
```

换成单目 USB 相机：

```bash
ros2 launch rosiwit_perception_ros2 qr_detector.launch.py \
    image_topic:=/camera/image_raw \
    camera_info_topic:=/camera/camera_info
```

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera_front/camera/image_raw` | `sensor_msgs/Image` | 可参数覆盖 |
| 订阅 | `/camera_front/camera/camera_info` | `sensor_msgs/CameraInfo` | 收到后才做位姿 |
| 发布 | `/qr/detections` | `std_msgs/String` (JSON) | 兼容旧接口 |
| 发布 | `/qr/codes` | `rosiwit_perception_msgs/QrCodeArray` | 正式接口，含 map 位姿 |
| 发布 | `/qr/debug_image` | `sensor_msgs/Image` | 标注图 |
| Action | `/qr/approach` | `rosiwit_perception_msgs/ApproachQr` | 视觉伺服靠近 |

## 多帧确认

同一文本连续命中 `confirm_hits` 次才输出；超过 `confirm_max_age` 未再出现就
移除。用于过滤单帧误检。默认 `2` 次 / `0.5s`。

## 位姿与 TF

- 核心输出 `tvec/rvec` 在**相机光学系**（x 右、y 下、z 前）。
- 节点根据 `CameraInfo.header.frame_id` 判断：
  - 名称含 `optical` → 直接使用；
  - 否则视为 camera body/link 系（x 前、y 左、z 上），先做光学系→body 转换。
- 开启 `use_tf` 时，用 TF 把位姿变换到 `target_frame`（默认 `map`）；
  TF 不可用时自动退回相机系，不会整条链路失效。
- `QrCode.pose.header.frame_id` 会明确写出实际坐标系。
- 二维码正对相机时姿态为单位四元数 `(0,0,0,1)`。

`qr_size` 必须与真实码边长一致，否则距离/位置等比例偏差。

## JSON 格式（`/qr/detections`）

```json
{
  "header": {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": "camera_link"},
  "count": 1,
  "detections": [{
    "text": "SHELF-A-01",
    "corners": [[541.0, 261.0], [740.0, 261.0],
                [740.0, 460.0], [541.0, 460.0]],
    "center": [640.5, 360.5],
    "pose": {
      "frame_id": "camera_front_optical",
      "position": {"x": 0.0, "y": 0.0, "z": 0.25},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "distance": 0.25
    },
    "pose_target": {
      "frame_id": "map",
      "position": {"x": 1.0, "y": 2.0, "z": 0.25},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }]
}
```

## 视觉靠近 action

```bash
ros2 action send_goal /qr/approach \
  rosiwit_perception_msgs/action/ApproachQr \
  "{text: 'DOCK-01', target_distance: 0.25, timeout: 30.0}" --feedback
```

控制律（核心 `compute_approach_cmd`）：

- `linear.x = k_linear * (distance - target_distance)`，方位偏差大时先转不前进；
- `angular.z = -k_angular * atan2(x, z)`（二维码在右 → 右转）；
- 距离/方位进入容差 → `SUCCEEDED`；
- 丢失超过 `approach_loss_timeout` 或超时 → `ABORTED`；
- 取消/结束时一定发布零速 `Twist`。

注意：action 直接发布 `/cmd_vel`，只能在导航空闲时用于充电桩/工位精对接。

## 主要参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `qr_size` | `0.10` | 二维码物理边长（米），`<=0` 关闭位姿 |
| `confirm_hits` | `2` | 连续命中次数 |
| `confirm_max_age` | `0.5` | 检测结果保留时间 |
| `target_frame` | `map` | TF 目标坐标系 |
| `use_tf` | `true` | 是否做 TF 变换 |
| `max_rate_hz` | `10` | 检测限频 |
| `min_size_px` | `24` | 最小码边长 |
| `approach_max_linear` | `0.15` | 对接最大线速度 |
| `approach_max_angular` | `0.5` | 对接最大角速度 |
| `approach_distance_tol` | `0.03` | 距离容差 |
| `approach_bearing_tol` | `0.03` | 方位容差 |
| `approach_loss_timeout` | `0.8` | 目标丢失超时 |

## 测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/rosiwit_perception \
    python3 -m pytest src/rosiwit_perception/test/test_qr_detector.py -q
```
