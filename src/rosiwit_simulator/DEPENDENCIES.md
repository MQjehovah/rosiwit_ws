# ============================================================
# rosiwit_simulator - Dependencies (ROS2 Humble)
# ============================================================
# Complete dependency list for the simulator package
# Last updated: 2026-05-05
# Migration: ROS1 Noetic → ROS2 Humble
# ============================================================

## System Requirements

| Requirement | Version | Notes |
|---|---|---|
| Operating System | Ubuntu 22.04 LTS (Jammy) | WSL2 supported |
| ROS Distribution | ROS2 Humble Hawksbill | `ros:humble-ros-base-jammy` |
| Gazebo | 11.x | GPU ray sensor support for 3D LiDAR |
| CMake | ≥ 3.8 | ament_cmake minimum |
| Python | ≥ 3.10 | Ubuntu 22.04 default |

## Build Dependencies (buildtool_depend)

| Package | ROS2 Package | Notes |
|---|---|---|
| ament_cmake | `ament_cmake` | ROS2 build system (replaces catkin) |

## Runtime Dependencies (exec_depend)

| Package | ROS2 Package | Purpose |
|---|---|---|
| Gazebo | `ros-humble-gazebo-ros` | Gazebo 11 simulation bridge |
| Gazebo PKGs | `ros-humble-gazebo-ros-pkgs` | Gazebo ROS2 integration |
| sensor_msgs | `ros-humble-sensor-msgs` | LaserScan, PointCloud2, IMU |
| geometry_msgs | `ros-humble-geometry-msgs` | Twist, Pose, etc. |
| nav_msgs | `ros-humble-nav-msgs` | OccupancyGrid, Odometry |
| tf2 | `ros-humble-tf2` | Transform library |
| tf2_ros | `ros-humble-tf2-ros` | Transform broadcaster/listener |
| xacro | `ros-humble-xacro` | URDF/Xacro macro processor |
| robot_state_publisher | `ros-humble-robot-state-publisher` | Publish robot state to TF |
| joint_state_publisher | `ros-humble-joint-state-publisher` | Publish joint states |
| RViz2 | `ros-humble-rviz2` | 3D visualization |
| launch | `ros-humble-launch` | ROS2 launch system |
| launch_ros | `ros-humble-launch-ros` | ROS2 launch ROS integration |

## Navigation Dependencies (Optional)

| Package | ROS2 Package | Purpose |
|---|---|---|
| Nav2 Map Server | `ros-humble-nav2-map-server` | Map loading/saving |
| Nav2 AMCL | `ros-humble-nav2-amcl` | Adaptive Monte Carlo Localization |
| Nav2 Lifecycle | `ros-humble-nav2-lifecycle-manager` | Nav2 node lifecycle |
| Nav2 Bringup | `ros-humble-nav2-bringup` | Navigation launch files |

## SLAM Dependencies (External - rosiwit_slam)

| Package | ROS2 Package | Purpose |
|---|---|---|
| FAST-LIO2 | `fast_lio2_slam` | 3D LiDAR SLAM (ROS2 native) |
| pcl_conversions | `ros-humble-pcl-conversions` | PointCloud conversions |
| pcl_ros | `ros-humble-pcl-ros` | PCL ROS2 integration |

## Docker Base Image

| Image | Tag | Size | Notes |
|---|---|---|---|
| ros | `humble-ros-base-jammy` | ~800MB | Official ROS2 Humble base |

## Python Dependencies

| Package | Version | Purpose |
|---|---|---|
| launch | (via apt) | ROS2 Python launch API |
| launch_ros | (via apt) | ROS2 launch node wrappers |
| os | stdlib | File path operations |
| pathlib | stdlib | Path handling |

## Migration Notes (ROS1 → ROS2)

| ROS1 Package | → | ROS2 Package | Change |
|---|---|---|---|
| `roslaunch` | → | `launch` + `launch_ros` | XML → Python launch |
| `catkin` | → | `ament_cmake` | Build system |
| `roscpp`/`rospy` | → | `rclcpp`/`rclpy` | Client library |
| `rosout` | → | `/rosout` | Topic namespace |
| `tf` | → | `tf2` | Transform library |
| `gazebo_ros` | → | `ros-humble-gazebo-ros` | Gazebo bridge |
| `ros1_bridge` | → | (removed) | Not needed for native ROS2 |

## Installation Commands

### Native (Ubuntu 22.04)
```bash
# ROS2 Humble base
sudo apt install ros-humble-ros-base

# Core dependencies
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs \
    ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs \
    ros-humble-tf2-ros ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
    ros-humble-rviz2 ros-humble-launch ros-humble-launch-ros

# Navigation (optional)
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager ros-humble-nav2-bringup

# Colcon build tools
sudo apt install python3-colcon-common-extensions
```

### Docker
```bash
docker pull ros:humble-ros-base-jammy
cd docker && ./build.sh
```
