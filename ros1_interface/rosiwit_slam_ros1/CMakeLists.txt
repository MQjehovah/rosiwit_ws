# ============================================================================
# rosiwit_slam_ros1 — ROS1 薄接口包 (与 rosiwit_slam_ros2 等价)
#   仅包含 ROS1 节点与 srv; 算法核心由 ../../src/rosiwit_slam 内联构建 (源码共享)。
# ============================================================================
cmake_minimum_required(VERSION 3.0.2)
project(rosiwit_slam_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
    tf tf2 tf2_ros pcl_conversions pcl_ros visualization_msgs
    message_generation)

add_service_files(FILES SaveMap.srv LoadMap.srv SaveGridMap.srv SetSlamMode.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
                   tf tf2 tf2_ros pcl_conversions visualization_msgs message_runtime)

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../src/rosiwit_slam
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_slam_core)

# ==================== ROS1 节点 (薄) ====================
add_executable(rosiwit_slam_ros1_node src/rosiwit_slam_ros1_node.cpp)
add_dependencies(rosiwit_slam_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(rosiwit_slam_ros1_node PRIVATE
    include ${catkin_INCLUDE_DIRS})
target_compile_definitions(rosiwit_slam_ros1_node PRIVATE -pipe -pthread)
target_link_libraries(rosiwit_slam_ros1_node
    ${catkin_LIBRARIES}
    rosiwit_slam)                      # 核心库 (含 PCL/OpenCV/GTSAM/Ceres/Sophus 等传递依赖)

install(TARGETS rosiwit_slam_ros1_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
