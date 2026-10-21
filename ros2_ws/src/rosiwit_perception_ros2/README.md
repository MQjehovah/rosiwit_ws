# rosiwit_perception_ros2

模块化感知 ROS 2 薄层：AVM 鸟瞰、地面纹理地图/重定位、二维码识别/视觉对接、
相机低矮障碍检测，以及统一健康管理。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
cd ~/rosiwit_ws/ros2_ws
colcon build --packages-select rosiwit_perception_msgs rosiwit_perception_ros2
source install/setup.bash
ros2 launch rosiwit_perception_ros2 perception.launch.py
```

无相机验证 AVM：

```bash
ros2 launch rosiwit_perception_ros2 perception.launch.py use_test_pattern:=true
```

查看健康状态：

```bash
ros2 topic echo /perception/status
ros2 topic echo /diagnostics
ros2 service call /perception_manager/get_status std_srvs/srv/Trigger
```

## 文档

- [统一架构 / 启动 / 观测性](docs/perception.md)
- [二维码识别与视觉对接](docs/qr_detector.md)
- [地面纹理持久化与重定位](docs/ground_texture.md)
- [相机低矮障碍检测](docs/obstacle_detector.md)

## 目录

```
rosiwit_perception_ros2/
  avm_node.py                 四路 AVM（每路独立内参/畸变，可降级）
  ground_texture_mapper.py    纹理拼接 + 保存/加载/重定位服务
  qr_detector_node.py         二维码 + 多帧确认 + TF + approach action
  obstacle_detector_node.py   相机低矮障碍 -> PointCloud2
  perception_manager.py       心跳汇总 -> status/diagnostics
  module.py                   PerceptionModule 基类
  image_codec.py              Image <-> numpy（不依赖 cv_bridge）
```

## 测试

```bash
# 核心算法
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=../../../src/rosiwit_perception \
    python3 -m pytest ../../../src/rosiwit_perception/test -q

# ROS 2 图像转换
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest test -q
```
