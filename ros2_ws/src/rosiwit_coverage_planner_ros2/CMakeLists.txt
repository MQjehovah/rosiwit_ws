# ============================================================================
# rosiwit_coverage_planner_ros2 — ROS2 薄接口包
#   仅包含 ROS2 节点与话题/服务桥接; 算法核心由
#   ../../../src/rosiwit_coverage_planner 内联构建 (源码共享)。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(rosiwit_coverage_planner_ros2 VERSION 1.0.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_coverage_planner
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_coverage_planner_core)

# ==================== ROS2 节点 (薄) ====================
add_library(coverage_planner_component SHARED
    src/coverage_planner_node.cpp)
target_include_directories(coverage_planner_component PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(coverage_planner_component
    rclcpp rclcpp_components nav_msgs geometry_msgs std_srvs)
target_link_libraries(coverage_planner_component
    rosiwit_coverage_planner)          # 核心库 (含 OpenCV 传递依赖)
rclcpp_components_register_nodes(coverage_planner_component
    "coverage_planner::CoveragePlannerNode")

add_executable(coverage_planner_node
    src/main.cpp)
target_include_directories(coverage_planner_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(coverage_planner_node
    rclcpp rclcpp_components nav_msgs geometry_msgs)
target_link_libraries(coverage_planner_node
    coverage_planner_component)

# ==================== 安装 ====================
install(TARGETS
    coverage_planner_component
    coverage_planner_node
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ==================== 导出 ====================
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components nav_msgs geometry_msgs std_srvs)

ament_package()
