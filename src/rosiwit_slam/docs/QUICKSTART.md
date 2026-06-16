# rosiwit_slam 快速启动指南

## 🚀 30秒快速启动

```bash
# 1. 设置环境
cd /home/jmq/agent/workspace/projects/rosiwit_ws/src/rosiwit_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. 启动节点（通过 launch 文件）
ros2 launch rosiwit_slam fast_lio2.launch.py

# 3. 或直接运行节点
ros2 run rosiwit_slam rosiwit_slam --ros-args -p use_sim_time:=true
```

## ✅ 项目状态

- **编译**: ✅ 成功 (可执行文件 1.8MB)
- **运行**: ✅ 正常 (节点可启动)
- **接口**: ✅ 完整 (所有话题和服务正常)

## 📋 ROS2接口

| 类型 | 接口 | 用途 |
|------|------|------|
| **订阅** | `/velodyne_points` | PointCloud2 点云 (10Hz) |
| **订阅** | `/imu` | IMU数据 (100Hz) |
| **发布** | `/odom_estimated` | 里程计估计 |
| **发布** | `/path_estimated` | 轨迹路径 |
| **发布** | `/cloud_map` | 点云地图 |
| **服务** | `/save_map` | 保存地图 |
| **服务** | `/save_pcd` | 保存PCD文件 |

## ⚠️ 关键要求

### 1. 点云格式（重要！）

必须包含以下字段（Livox格式）：
```
- x, y, z (FLOAT32) - 位置
- intensity (FLOAT32) - 强度
- normal_x, normal_y, normal_z (FLOAT32) - 法向量
- curvature (FLOAT32) - 曲率
```

### 2. 仿真时间

测试时必须：
- 节点启动参数: `--ros-args -p use_sim_time:=true`
- 测试脚本发布: `/clock` 话题

### 3. QoS配置

使用 `RELIABLE` 可靠性策略：
```python
qos = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE
)
```

## 🧪 测试脚本

| 脚本 | 用途 | 推荐度 |
|------|------|--------|
| `test_mapping_auto.sh` | 自动化测试（推荐） | ⭐⭐⭐ |
| `test_mapping_comprehensive.py` | 综合建图测试 | ⭐⭐⭐ |
| `test_livox.py` | Livox格式测试 | ⭐⭐ |
| `diagnose.sh` | 诊断脚本 | ⭐⭐ |

## 📊 测试输出示例

```
[INFO] [rosiwit_slam]: Parameters loaded: lidar_topic=/velodyne_points, imu_topic=/imu
[INFO] [rosiwit_slam]: Subscribers created
[INFO] [rosiwit_slam]: Publishers created
[INFO] [rosiwit_slam]: Services created
[INFO] [rosiwit_slam]: rosiwit_slam Node initialized successfully!
[INFO] [comprehensive_mapping_test]: LiDAR sent: 1, points: 1100, time: 0.10s
[INFO] [comprehensive_mapping_test]: ✅ 收到里程计 #1: pos=[0.12, 0.05, 0.00]
[INFO] [comprehensive_mapping_test]: ✅ 收到路径 #1, poses: 10
[INFO] [comprehensive_mapping_test]: ✅ 收到地图 #1, points: 1100
```

## 🔧 常见问题

### Q1: 节点启动但没有输出？

**A:** 检查以下几点：
1. 是否提供了正确格式的点云数据
2. 仿真时间是否同步（/clock话题）
3. QoS设置是否匹配

### Q2: 如何使用真实数据？

**A:** 使用rosbag播放：
```bash
ros2 bag play your_data.bag --clock
```

### Q3: 如何保存地图？

**A:** 调用服务：
```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

## 📚 更多文档

- [USAGE_GUIDE.md](USAGE_GUIDE.md) - 详细使用指南
- [PROJECT_REPORT.md](PROJECT_REPORT.md) - 项目完成报告
- [TEST_REPORT.md](TEST_REPORT.md) - 测试报告
- [docs/architecture.md](docs/architecture.md) - 架构设计

## 🎯 下一步

1. **测试**: 运行 `./test_mapping_auto.sh` 验证功能
2. **可视化**: 使用 `rviz2` 查看地图和轨迹
3. **真实数据**: 使用Livox雷达数据测试

---
**最后更新**: 2026-04-24
**项目路径**: `/home/jmq/agent/workspace/project/fast_lio2_slam`