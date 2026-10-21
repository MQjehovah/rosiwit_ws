# 地面纹理地图持久化与重定位

`ground_texture_mapper` 把 AVM 鸟瞰图按位姿拼成全局地面纹理，并支持：

- 保存/加载纹理地图（`.npz`，原子写入）
- 用当前 BEV 在已加载地图中做局部重定位
- 发布纹理精化后的位姿

## 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/ground_texture_mapper/save_texture_map` | `rosiwit_perception_msgs/SaveTextureMap` | `path` 空则用配置里的 `save_path` |
| `/ground_texture_mapper/load_texture_map` | `rosiwit_perception_msgs/LoadTextureMap` | 加载后替换当前地图 |
| `/ground_texture_mapper/relocalize` | `rosiwit_perception_msgs/Relocalize` | 输入先验位姿，返回修正位姿 + score |

```bash
ros2 service call /ground_texture_mapper/save_texture_map \
  rosiwit_perception_msgs/srv/SaveTextureMap \
  "{path: '/home/robot/maps/office_texture.npz'}"

ros2 service call /ground_texture_mapper/load_texture_map \
  rosiwit_perception_msgs/srv/LoadTextureMap \
  "{path: '/home/robot/maps/office_texture.npz'}"

ros2 service call /ground_texture_mapper/relocalize \
  rosiwit_perception_msgs/srv/Relocalize \
  "{prior_pose: {header: {frame_id: 'odom'},
                 pose: {position: {x: 1.0, y: 2.0},
                        orientation: {w: 1.0}}},
    search_radius: 0.5, search_angle: 0.3}"
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/ground_texture_map` | `sensor_msgs/Image` | 全局纹理图 |
| `/ground_texture_grid` | `nav_msgs/OccupancyGrid` | RViz 显示 |
| `/ground_texture_pose` | `geometry_msgs/PoseStamped` | 纹理匹配精化后的位姿（`map_frame`，默认 `odom`） |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `save_path` | `~/.ros/rosiwit_ground_texture.npz` | 默认保存路径 |
| `auto_load` | `false` | 启动时自动加载 `save_path` |
| `map_frame` | `odom` | 输出位姿/地图坐标系 |
| `relocalize_search_radius` | `0.5` | 默认位置搜索半径（米） |
| `relocalize_search_angle` | `0.3` | 默认角度搜索范围（弧度，±） |

## 原理

直接拿当前 BEV 的中心 patch 去匹配会失败：BEV 的 x 轴朝上，而纹理画布的 y
轴朝下，两者相差 90°。实现中先在**当前 yaw 下 warp 整帧 BEV**，再从 warp
结果里取模板，与画布搜索区域做 `TM_CCOEFF_NORMED`，遍历若干 yaw 偏移取最
高分。这个修正同时让原有的 `refine_pose` 变得可用。

## 限制

- 属于局部重定位：默认 ±0.5m / ±17°，适合修正里程计漂移，不是全局 kidnapping
  恢复；
- 地图分辨率 5mm/px，纹理重复区域（纯色地砖）匹配分数可能偏低；
- 匹配分数阈值默认 0.35，可通过服务返回的 `score` 二次判断。
