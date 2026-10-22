# rosiwit_simulation_ros1

ROS1 (catkin) Gazebo simulation package for the rosiwit differential-drive
robot, aligned with `rosiwit_simulation_ros2`.

## 与 ros2 包的对齐关系

| 功能 | rosiwit_simulation_ros2 | rosiwit_simulation_ros1 |
|---|---|---|
| 启动 | `launch/simulator_gazebo.launch.py` | `launch/simulator_gazebo.launch` |
| 世界 | `world/{house,large,simple}.world` | 相同（复制） |
| 机器人模型 | `mbot_with_lidar3d_gazebo.xacro` | 相同结构，ROS1 Gazebo 插件 |
| 默认出生点 | house.world `(0, 0, 0.1)` yaw 0 | 相同 |
| 3D 雷达 | `libgazebo_ros_ray_sensor.so` (ROS2) | `libgazebo_ros_velodyne_laser.so` (`velodyne_gazebo_plugins`) |
| IMU | `libgazebo_ros_imu_sensor.so` (ROS2) | `libgazebo_ros_imu.so` |
| 差速驱动 | `libgazebo_ros_diff_drive.so` (ROS2) | 同名插件（camelCase 参数） |
| 模型/网格 | 包内自带 | 软链接到 `ros2_ws/.../rosiwit_simulation_ros2/{models,meshes}` |

> `models/` 和 `meshes/` 采用相对软链接，避免在 git 中重复约 100MB 资源。
> 若需要把本包独立分发，请把它们替换成实体目录。

## 依赖

- ROS1 Noetic
- `gazebo_ros`, `gazebo_plugins`
- `xacro`, `robot_state_publisher`, `joint_state_publisher`

## 编译

```bash
cd ~/rosiwit_ws/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行

```bash
# 默认 house.world + VLP-16 + IMU，机器人出生在 (0,0) 地面
roslaunch rosiwit_simulation_ros1 simulator_gazebo.launch

# 指定世界/出生点
roslaunch rosiwit_simulation_ros1 simulator_gazebo.launch \
  world_name:=$(rospack find rosiwit_simulation_ros1)/world/large.world \
  spawn_y:=-18 spawn_yaw:=1.57

# 无 GUI
roslaunch rosiwit_simulation_ros1 simulator_gazebo.launch gui:=false
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度输入 |
| `/odom` | `nav_msgs/Odometry` | 轮式里程计 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 3D 雷达点云 |
| `/imu` | `sensor_msgs/Imu` | IMU |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 坐标变换 |
| `/scan` | `sensor_msgs/LaserScan` | 2D 雷达模型（`mbot_with_laser_gazebo.xacro`） |

## 建图测试脚本

```bash
rosrun rosiwit_simulation_ros1 sim_drive.py _mode:=figure8 _duration:=120
```

## 说明

- ROS1 与 ROS2 的 Gazebo 插件参数命名不同，因此 `urdf/xacro` 下保留了
  两套插件配置；几何结构、传感器布局和世界文件保持一致。
- 相机/鱼眼/深度相机变体目前以 ROS2 包为准；本包先覆盖默认的
  3D 雷达 + IMU 组合，以及 2D 雷达组合。
