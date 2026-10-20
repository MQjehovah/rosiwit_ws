# ============================================================================
# rosiwit_slam_ros2 — ROS2 薄接口包
#   仅包含 ROS2 节点与 srv; 算法核心由 ../rosiwit_slam 内联构建 (源码共享)。
#   与 rosiwit_slam_ros1 等价。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(rosiwit_slam_ros2 VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==================== 消息和服务生成 ====================
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SaveMap.srv"
    "srv/LoadMap.srv"
    "srv/SaveGridMap.srv"
    "srv/SetSlamMode.srv"
    DEPENDENCIES builtin_interfaces
)

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_slam
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_slam_core)

# ==================== ROS2 节点 (薄) ====================
set(NODE_NAME slam_node)
add_executable(${NODE_NAME}
    src/main.cpp
    src/ros_interface/ros_utils.cpp
    src/ros_interface/slam_node.cpp)
set_target_properties(${NODE_NAME} PROPERTIES OUTPUT_NAME "rosiwit_slam_ros2")
target_include_directories(${NODE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(${NODE_NAME} PRIVATE BOOST_ALL_NO_LIB)
target_precompile_headers(${NODE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/pch.hpp)

ament_target_dependencies(${NODE_NAME}
    rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros std_srvs
    pcl_conversions visualization_msgs)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${NODE_NAME}
    "${typesupport_target}"
    rosiwit_slam)                      # 核心库 (含 PCL/OpenCV/GTSAM/Ceres/Sophus 等传递依赖)

# ==================== 安装 ====================
install(TARGETS ${NODE_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

# ==================== 测试 ====================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_frame_utils test/test_frame_utils.cpp)
    if(TARGET test_frame_utils)
        target_include_directories(test_frame_utils PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
        target_link_libraries(test_frame_utils rosiwit_slam)
    endif()
endif()

# ==================== 导出 ====================
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros std_srvs pcl_conversions visualization_msgs)

ament_package()
