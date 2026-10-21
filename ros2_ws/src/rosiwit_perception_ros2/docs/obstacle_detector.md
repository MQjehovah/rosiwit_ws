# 相机低矮障碍检测

LiDAR 对纸箱、椅子腿、线缆等低矮/悬空目标不敏感。`obstacle_detector` 用前视
单目相机做地面颜色分割，把障碍物的**底部接触点**投到地面平面，输出
`base_link` 下的 `PointCloud2`。

## 流程

1. 取画面底部左右两侧作为地面颜色样本（Lab 中值）；
2. 颜色距离超过阈值的像素视为非地面；
3. 形态学滤波 + 连通域，过滤小区域；
4. 取每个连通域底部接触点，用 `undistortPoints` + 射线-地面平面求交；
5. 过滤距离/横向范围和相机后方目标；
6. 发布点云 + 最近距离停障信号。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera_front/camera/image_raw` | `sensor_msgs/Image` | 可参数覆盖 |
| 订阅 | `/camera_front/camera/camera_info` | `sensor_msgs/CameraInfo` | 内参/畸变 |
| 发布 | `/perception/obstacles` | `sensor_msgs/PointCloud2` | `base_link`，z 为 `obstacle_height` |
| 发布 | `/perception/obstacle_stop` | `std_msgs/Bool` | 最近障碍 < `stop_distance` |

## 接入导航

点云可直接接到导航安全层或体素层：

```yaml
rosiwit_navigation_node:
  ros__parameters:
    safety_pointcloud_topic: /perception/obstacles
    safety_enabled: true
    safety_min_height: 0.05
    safety_max_height: 1.0
```

或：

```yaml
    voxel_cloud_topic: /perception/obstacles
```

注意安全层只订阅一个点云话题；若需要 LiDAR + 相机同时参与，需先做点云合并。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `camera_name` | `front` | 使用 `config.py` 中的哪路相机 |
| `ground_z` | `-0.11` | 地面相对 `base_link` 的高度 |
| `obstacle_height` | `0.10` | 输出点云 z（base_link） |
| `stop_distance` | `0.35` | 停障距离 |
| `max_distance` | `3.0` | 最远检测距离 |
| `max_lateral` | `1.5` | 横向范围 |
| `min_area_ratio` | `0.0015` | 最小连通域面积占比 |
| `color_threshold` | `35` | Lab 颜色距离阈值 |
| `roi_top_ratio` | `0.30` | 忽略画面上方 30% |
| `use_tf_extrinsics` | `false` | 是否用 TF 外参（默认用 `config.py`） |

## 调参建议

- 地面颜色不均/阴影：提高 `color_threshold`，或先做光照归一化；
- 花纹地毯：容易把图案当障碍，建议改用语义分割/深度模型；
- 误检太多：提高 `min_area_ratio` / `color_threshold`，限制 `roi_top_ratio`；
- 漏检深色小物体：降低 `min_area_ratio`，但可能引入噪声。
