cmake_minimum_required(VERSION 3.8)
project(rosiwit_simulation_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  map
  models
  meshes
  world
  urdf
  rviz
  images
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_drive.py
  scripts/generate_large_loop_world.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
