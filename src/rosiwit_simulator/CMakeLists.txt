cmake_minimum_required(VERSION 3.8)
project(rosiwit_simulator)

# ROS2 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 安装所有资源目录到 share/${PROJECT_NAME}
install(DIRECTORY
  launch
  config
  map
  models
  meshes
  world
  urdf
  rviz
  images
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本
install(PROGRAMS
  launch/auto_drive_figure8.py
  launch/simulated_sensor.py
  scripts/sim_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
