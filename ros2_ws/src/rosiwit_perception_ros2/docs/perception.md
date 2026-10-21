# rosiwit_perception 感知子系统

## 1. 架构

```
rosiwit_perception (纯 Python 核心，无 ROS 依赖)
  ├── birdseye.py         AVM BEV remap / 拼接
  ├── ground_texture.py   地面纹理地图 + 持久化 + 重定位
  ├── qr_detector.py      二维码检测/解码 + solvePnP + 视觉伺服控制律
  └── obstacle_detector.py 单目地面障碍检测（地面平面投影）

rosiwit_perception_ros2 (ROS 2 薄层，模块彼此独立)
  ├── avm_node.py
  ├── ground_texture_mapper.py
  ├── qr_detector_node.py
  ├── obstacle_detector_node.py
  ├── perception_manager.py
  └── module.py            PerceptionModule 基类：心跳/计数/健康状态

rosiwit_perception_msgs
  ├── msg/QrCodeArray.msg
  ├── srv/SaveTextureMap.srv / LoadTextureMap.srv / Relocalize.srv
  └── action/ApproachQr.action
```

每个感知模块都是独立进程、独立可执行文件。`perception_manager` 只订阅心跳，
不转发数据，因此单个模块崩溃不会阻塞其他模块。

## 2. 一键启动

```bash
source /opt/ros/humble/setup.bash
cd ~/rosiwit_ws/ros2_ws
colcon build --packages-select rosiwit_perception_msgs rosiwit_perception_ros2
source install/setup.bash

# 全部模块
ros2 launch rosiwit_perception_ros2 perception.launch.py

# 只启 AVM + 管理器
ros2 launch rosiwit_perception_ros2 perception.launch.py \
    enable_ground_texture:=false enable_qr:=false enable_obstacles:=false

# 无相机，用合成图验证 AVM
ros2 launch rosiwit_perception_ros2 perception.launch.py use_test_pattern:=true
```

启动参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `params_file` | `config/perception.yaml` | 统一参数文件 |
| `qr_params_file` | `config/qr_detector.yaml` | QR 独立参数（覆盖统一配置） |
| `enable_avm` | `true` | AVM |
| `enable_ground_texture` | `true` | 地面纹理 |
| `enable_qr` | `true` | 二维码 |
| `enable_obstacles` | `true` | 相机低矮障碍 |
| `enable_manager` | `true` | 健康管理器 |
| `use_test_pattern` | `false` | AVM 合成测试图 |

## 3. 模块接口

| 模块 | 输入 | 输出 |
|---|---|---|
| `avm_node` | 四路 `/camera_*/camera/image_raw` + `camera_info` | `/avm/birdseye`、`/avm/debug_*` |
| `ground_texture_mapper` | `/avm/birdseye`、`/lio_odom` | `/ground_texture_map`、`/ground_texture_grid`、`/ground_texture_pose` |
| `qr_detector` | `/camera_front/camera/image_raw` + `camera_info` | `/qr/detections`(JSON)、`/qr/codes`(QrCodeArray)、`/qr/debug_image`、`/qr/approach` action |
| `obstacle_detector` | 前视相机 | `/perception/obstacles`(PointCloud2)、`/perception/obstacle_stop`(Bool) |
| `perception_manager` | `/perception/heartbeat` | `/perception/status`、`/perception/healthy`、`/diagnostics` |

### 与导航对接

相机障碍点云是 `base_link` 下的 `PointCloud2`，可直接接到导航安全层/体素层：

```yaml
rosiwit_navigation_node:
  ros__parameters:
    safety_pointcloud_topic: /perception/obstacles
    # 或
    voxel_cloud_topic: /perception/obstacles
```

注意：安全层只订阅一个点云话题。若同时需要 LiDAR 和相机，需要在上游合并
点云，或让相机点云接到体素层、LiDAR 保留在安全层。

二维码对接使用 `/qr/approach` action，直接发布 `/cmd_vel`，**只能在导航空闲
时使用**（充电桩/工位精对接）。

## 4. 可观测性

每个模块 1Hz 发布 JSON 心跳到 `/perception/heartbeat`，manager 汇总：

```bash
ros2 topic echo /perception/status
ros2 topic echo /perception/healthy
ros2 service call /perception_manager/get_status std_srvs/srv/Trigger
ros2 topic echo /diagnostics
```

`/perception/status` 示例：

```json
{
  "healthy": true,
  "modules": {
    "avm": {"state": "OK", "age": 0.3, "fps": 14.8,
             "frames": 1234, "dropped": 2, "last_frame_age": 0.07,
             "cameras": 4, "degraded": false},
    "qr_detector": {"state": "OK", "age": 0.2, "fps": 9.9,
                    "frames": 812, "dropped": 0,
                    "raw": 1, "confirmed": 1},
    "obstacle_detector": {"state": "OK", "age": 0.4, "fps": 9.8,
                          "frames": 800, "dropped": 0, "obstacles": 2}
  }
}
```

manager 判定规则：
- 心跳超过 `timeout`（默认 3s）未更新 → `STALE` / ERROR；
- 从没收到过 → `MISSING` / ERROR；
- 超过 `0.7 * timeout` → `WARN`；
- 全部 `OK` 才认为整体 healthy。

## 5. 新增感知模块

1. 核心算法放 `src/rosiwit_perception/rosiwit_perception/`，只依赖 numpy/cv2；
2. ROS 节点继承 `PerceptionModule`：

```python
from rosiwit_perception_ros2.module import PerceptionModule

class MyNode(PerceptionModule):
    def __init__(self):
        super().__init__('my_node', module_name='my_module')

    def on_result(self):
        self.note_frame(foo=1)   # 成功处理一帧
        # self.note_drop('reason') # 失败/丢弃
```

3. 在 `config/perception.yaml` 的 `perception_manager.modules` 里加入模块名；
4. 在 `launch/perception.launch.py` 增加一个 Node（带 `enable_*` 条件）；
5. 在 `setup.py` 注册可执行文件 + wrapper。

## 6. 测试

```bash
# 核心算法（无需 ROS）
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/rosiwit_perception \
    python3 -m pytest src/rosiwit_perception/test -q

# ROS 2 图像转换
source /opt/ros/humble/setup.bash
cd ros2_ws/src/rosiwit_perception_ros2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest test -q
```

## 7. 当前边界

- AVM 已支持每路独立内参/畸变，TF 外参开关默认关闭（需现场确认 TF 树）；
- 地面纹理定位是局部模板匹配，适合 ±0.5m/±17° 内的漂移修正，不是全局重定位；
- 障碍检测是“地面颜色 + 底部接触点”的轻量方案，花纹地毯/强反光地面需调参；
- 人/宠物检测、悬崖/负障碍、1D 条码需要额外的模型或传感器选型，接口已可按
  上述模块化方式接入。
