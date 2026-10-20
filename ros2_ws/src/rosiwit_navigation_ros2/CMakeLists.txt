# ============================================================================
# rosiwit_navigation_ros2 — ROS2 薄接口包
#   仅包含 ROS2 生命周期节点与话题桥接; 算法核心由
#   ../../../src/rosiwit_navigation 内联构建 (源码共享)。
#   与 rosiwit_navigation_ros1 等价。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(rosiwit_navigation_ros2 VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_navigation
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_navigation_core)

# ==================== ROS2 节点 (薄) ====================
add_executable(rosiwit_navigation_node
    src/main.cpp
    src/ros_interface/navigation_node.cpp
    src/ros_interface/ros_utils.cpp)
target_include_directories(rosiwit_navigation_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rosiwit_navigation_node PRIVATE -pipe -pthread)
ament_target_dependencies(rosiwit_navigation_node
    rclcpp rclcpp_lifecycle geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
    visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(rosiwit_navigation_node
    rosiwit_navigation)                # 核心库 (算法/规划器/控制器)
add_dependencies(rosiwit_navigation_node rosiwit_navigation)

# ==================== 安装 ====================
install(TARGETS rosiwit_navigation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ==================== 导出 ====================
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp rclcpp_lifecycle geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
    visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs)

ament_package()
