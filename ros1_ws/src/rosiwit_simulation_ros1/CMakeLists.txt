cmake_minimum_required(VERSION 3.0.2)
project(rosiwit_simulation_ros1)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  launch
  urdf
  world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# models/ and meshes/ are symlinks into the ROS2 simulator package so that the
# ~100 MB of shared Gazebo assets are not duplicated in git. Resolve them at
# configure time so install-space builds contain real directories.
get_filename_component(_models_real "${CMAKE_CURRENT_SOURCE_DIR}/models" REALPATH)
get_filename_component(_meshes_real "${CMAKE_CURRENT_SOURCE_DIR}/meshes" REALPATH)
install(DIRECTORY ${_models_real}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
install(DIRECTORY ${_meshes_real}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

catkin_install_python(PROGRAMS
  scripts/sim_drive.py
  scripts/generate_large_loop_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
