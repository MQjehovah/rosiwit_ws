# 快速开始教程

本教程帮助你快速上手 Diffbot Navigation 导航系统。

---

## 目录

- [环境准备](#环境准备)
- [安装](#安装)
- [配置](#配置)
- [运行](#运行)
- [基本使用](#基本使用)
- [示例场景](#示例场景)
- [常见问题](#常见问题)
- [进阶主题](#进阶主题)

---

## 环境准备

### 系统要求

| 项目 | 要求 |
|------|------|
| **操作系统** | Ubuntu 22.04 LTS |
| **ROS 版本** | ROS2 Humble Hawksbill |
| **内存** | 至少 4 GB |
| **存储** | 至少 2 GB 可用空间 |

### 安装 ROS2 Humble

```bash
# 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装编译工具
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 安装 Navigation2

```bash
# 安装 Nav2 及相关包
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-simple-commander \
                 ros-humble-slam-toolbox \
                 ros-humble-robot-localization \
                 ros-humble-dwb-plugins \
                 ros-humble-dwb-critics
```

---

## 安装

### 方法一：从源码编译（推荐）

```bash
# 创建工作空间
mkdir -p ~/diffbot_navigation_ws/src
cd ~/diffbot_navigation_ws/src

# 克隆仓库
git clone https://github.com/ai-dev-team/diffbot_navigation.git

# 安装依赖
cd ~/diffbot_navigation_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

### 方法二：使用 Docker

```bash
# 拉取镜像
docker pull ai-dev-team/diffbot-navigation:latest

# 运行容器
docker run -it --rm \
    --name diffbot-nav \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    ai-dev-team/diffbot-navigation:latest
```

---

## 配置

### 1. 机器人参数配置

编辑 `config/navigation_params.yaml`：

```yaml
# 机器人尺寸
robot_radius: 0.22  # 机器人外接圆半径 (m)

# 运动学参数
wheel_separation: 0.34  # 轮间距 (m)
wheel_radius: 0.05     # 轮半径 (m)

# 速度限制
max_velocity_x: 0.5      # 最大前进速度 (m/s)
max_velocity_theta: 1.0  # 最大旋转速度 (rad/s)
min_velocity_x: -0.3     # 最大后退速度 (m/s)

# 加速度限制
max_accel_x: 0.5        # 最大线加速度 (m/s²)
max_accel_theta: 1.0    # 最大角加速度 (rad/s²)
```

### 2. 控制器参数配置

编辑 `config/controller_params.yaml`：

```yaml
# Pure Pursuit 参数
lookahead_distance: 0.6       # 前视距离 (m)
min_lookahead_distance: 0.3   # 最小前视距离 (m)
max_lookahead_distance: 1.0   # 最大前视距离 (m)
lookahead_gain: 0.5           # 前视距离增益

# PID 参数
k_p_linear: 1.0    # 线速度比例增益
k_i_linear: 0.01   # 线速度积分增益
k_d_linear: 0.1    # 线速度微分增益
k_p_angular: 2.0   # 角速度比例增益
k_i_angular: 0.01  # 角速度积分增益
k_d_angular: 0.2   # 角速度微分增益

# 目标容差
xy_goal_tolerance: 0.1   # 位置容差 (m)
yaw_goal_tolerance: 0.1  # 角度容差 (rad)
```

### 3. 避障参数配置

编辑 `config/obstacle_params.yaml`：

```yaml
# 避障模式
avoidance_mode: "hybrid"  # dwa, potential_field, hybrid

# 检测范围
detection_range: 2.0           # 检测范围 (m)
safe_distance: 0.5            # 安全距离 (m)
min_obstacle_distance: 0.2    # 最小障碍物距离 (m)

# DWA 参数
max_velocity_x: 0.5           # 采样最大线速度
max_velocity_theta: 1.0       # 采样最大角速度
sim_time: 1.5                 # 模拟时间 (s)
sim_granularity: 0.025        # 模拟粒度 (s)

# 势场参数
k_att: 1.0      # 引力增益
k_rep: 0.8      # 斥力增益
d_0: 1.0        # 斥力作用范围 (m)
```

### 4. 窄道参数配置

编辑 `config/obstacle_params.yaml` (narrow_passage 部分)：

```yaml
narrow_passage:
  # 机器人参数
  robot_width: 0.44    # 机器人宽度 (m)
  robot_length: 0.5    # 机器人长度 (m)
  
  # 窄道检测
  min_passage_width: 0.6  # 最小通道宽度 (m)
  detection_range: 3.0    # 检测范围 (m)
  safety_margin: 0.08     # 安全余量 (m)
  
  # 通行速度
  passage_velocity: 0.15       # 通行速度 (m/s)
  max_passage_velocity: 0.2    # 最大通行速度 (m/s)
  
  # 控制增益
  angle_correction_gain: 2.0     # 角度修正增益
  lateral_correction_gain: 1.5   # 横向修正增益
```

---

## 运行

### 启动导航节点

```bash
# 加载环境
source ~/diffbot_navigation_ws/install/setup.bash

# 启动导航
ros2 launch diffbot_navigation navigation.launch.py
```

### 带参数启动

```bash
# 指定地图
ros2 launch diffbot_navigation navigation.launch.py \
    map:=/path/to/map.yaml

# 指定配置文件
ros2 launch diffbot_navigation navigation.launch.py \
    params_file:=/path/to/params.yaml

# 使用仿真
ros2 launch diffbot_navigation navigation.launch.py \
    use_sim_time:=True
```

### 启动 RViz 可视化

```bash
# 在新终端中启动 RViz
ros2 launch diffbot_navigation rviz.launch.py
```

---

## 基本使用

### 1. 发送导航目标

#### 方法一：使用 RViz

1. 点击工具栏的 "2D Goal Pose" 按钮
2. 在地图上点击并拖动，设置目标位置和方向
3. 机器人开始导航

#### 方法二：使用命令行

```bash
# 发送导航目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}" \
    --once
```

#### 方法三：使用 Python 脚本

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    # 等待导航系统就绪
    navigator.waitUntilNav2Active()
    
    # 设置目标
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 2.0
    goal.pose.position.y = 1.0
    goal.pose.orientation.w = 1.0
    
    # 开始导航
    navigator.goToPose(goal)
    
    # 等待完成
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f'距离目标: {feedback.distance_remaining:.2f} m')
    
    result = navigator.getResult()
    if result:
        print('导航成功！')
    else:
        print('导航失败')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. 控制导航

```bash
# 暂停导航
ros2 service call /diffbot_navigation/pause std_srvs/srv/Empty

# 恢复导航
ros2 service call /diffbot_navigation/resume std_srvs/srv/Empty

# 停止导航
ros2 service call /diffbot_navigation/stop std_srvs/srv/Empty

# 清除代价地图
ros2 service call /diffbot_navigation/clear_costmap nav2_msgs/srv/ClearCostmapAroundRobot
```

### 3. 监控状态

```bash
# 查看导航状态
ros2 topic echo /diffbot_navigation/feedback

# 查看当前路径
ros2 topic echo /diffbot_navigation/path

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看障碍物
ros2 topic echo /diffbot_navigation/obstacles
```

---

## 示例场景

### 场景一：单点导航

```bash
# 启动导航
ros2 launch diffbot_navigation navigation.launch.py

# 在另一个终端，发送目标
ros2 run diffbot_navigation simple_navigator --ros-args \
    -p goal_x:=3.0 \
    -p goal_y:=2.0 \
    -p goal_theta:=1.57
```

### 场景二：避障导航

```bash
# 启动导航
ros2 launch diffbot_navigation navigation.launch.py

# 启动动态障碍物模拟器（如有）
ros2 run obstacle_simulator dynamic_obstacle

# 发送目标，系统将自动避障
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}" \
    --once
```

### 场景三：窄道通行

```bash
# 确保窄道参数已配置
# robot_width < min_passage_width < detection_range

# 启动导航
ros2 launch diffbot_navigation navigation.launch.py

# 发送穿过窄道的目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" \
    --once

# 监控窄道状态
ros2 topic echo /diffbot_navigation/narrow_passage_state
```

### 场景四：多点导航

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # 定义多个目标点
    goals = [
        (1.0, 0.0, 0.0),      # (x, y, yaw)
        (2.0, 1.0, 1.57),
        (1.0, 2.0, 3.14),
        (0.0, 1.0, -1.57),
        (0.0, 0.0, 0.0),
    ]
    
    for i, (x, y, yaw) in enumerate(goals):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        
        print(f'前往目标 {i+1}/{len(goals)}: ({x}, {y})')
        navigator.goToPose(goal)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            print(f'  剩余距离: {feedback.distance_remaining:.2f} m')
        
        if navigator.getResult():
            print(f'  目标 {i+1} 到达')
        else:
            print(f'  目标 {i+1} 失败')
            break
    
    print('导航完成！')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 常见问题

### Q1: 导航一直失败？

**可能原因：**
1. 地图未加载
2. 定位丢失
3. 起点或目标点在障碍物中

**解决方法：**
```bash
# 检查地图
ros2 topic echo /map --once

# 检查定位
ros2 topic echo /amcl_pose --once

# 清除代价地图
ros2 service call /clear_costmap nav2_msgs/srv/ClearCostmapAroundRobot
```

### Q2: 机器人不走直线？

**可能原因：**
1. PID 参数不当
2. 前视距离设置过小
3. 里程计漂移

**解决方法：**
```yaml
# 调整 controller_params.yaml
lookahead_distance: 0.8  # 增大前视距离
k_p_angular: 2.5         # 增大角速度增益
```

### Q3: 避障不灵敏？

**可能原因：**
1. 障碍物检测范围太小
2. 安全距离设置过大

**解决方法：**
```yaml
# 调整 obstacle_params.yaml
detection_range: 3.0    # 增大检测范围
safe_distance: 0.3      # 减小安全距离
```

### Q4: 窄道无法通过？

**可能原因：**
1. 机器人宽度配置错误
2. 安全余量过大
3. 通行速度过快

**解决方法：**
```yaml
# 调整窄道参数
robot_width: 0.44          # 确保正确
safety_margin: 0.05        # 减小余量
passage_velocity: 0.1      # 降低速度
```

### Q5: 编译错误？

```bash
# 清理并重新编译
cd ~/diffbot_navigation_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 进阶主题

### 自定义规划器

```cpp
// 自定义全局规划器
#include "nav2_core/global_planner.hpp"

class MyPlanner : public nav2_core::GlobalPlanner {
public:
    void configure(...) override {
        // 初始化配置
    }
    
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override {
        // 实现自定义规划算法
    }
};

// 注册插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_package::MyPlanner, nav2_core::GlobalPlanner)
```

### 自定义控制器

```cpp
// 自定义控制器
#include "nav2_core/controller.hpp"

class MyController : public nav2_core::Controller {
public:
    void configure(...) override {
        // 初始化
    }
    
    geometry_msgs::msg::Twist computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity) override {
        // 实现自定义控制算法
    }
    
    void setPlan(const nav_msgs::msg::Path& path) override {
        // 设置路径
    }
};
```

### 调试技巧

```bash
# 查看节点信息
ros2 node list
ros2 node info /diffbot_navigation

# 查看话题频率
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# 查看参数
ros2 param list /diffbot_navigation
ros2 param get /diffbot_navigation max_velocity_x

# 动态修改参数
ros2 param set /diffbot_navigation max_velocity_x 0.8

# 录制数据
ros2 bag record -a -o navigation_test

# 回放数据
ros2 bag play navigation_test
```

### 性能优化

```yaml
# 减少控制频率可降低 CPU 使用率
controller_frequency: 10.0  # 从 20 Hz 降到 10 Hz

# 优化代价地图更新频率
costmap_update_frequency: 2.0  # 从 5 Hz 降到 2 Hz

# 减少轨迹采样数量
trajectory_samples: 20  # 从 40 降到 20
```

---

## 参考资料

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://navigation.ros.org/)
- [TEB Local Planner](https://wiki.ros.org/teb_local_planner)
- [Pure Pursuit 算法](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)

---

**文档版本**: 0.1.0  
**最后更新**: 2026-04-25  
**维护者**: AI Development Team