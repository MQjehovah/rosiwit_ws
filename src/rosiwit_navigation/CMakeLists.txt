# ============================================================
# rosiwit_navigation - 导航系统
# 架构分层: ros_interface → nav_core → algorithms
# ============================================================

cmake_minimum_required(VERSION 3.14)
project(rosiwit_navigation)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 编译器警告
# ============================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
    -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual
    -Wconversion -Wsign-conversion -Wnull-dereference -Wdouble-promotion
    -Wformat=2 -Wmisleading-indentation)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# ============================================================
# 依赖
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTest QUIET)

# Nav2 可选
find_package(nav2_core QUIET)
find_package(nav2_util QUIET)
find_package(nav2_costmap_2d QUIET)
find_package(nav2_msgs QUIET)

# ============================================================
# Include
# ============================================================
include_directories(include src
  src/algorithms/global_planner
  src/algorithms/local_planner
  src/algorithms/safe_passage
  src/algorithms/support)

# ============================================================
# 算法层 Libraries
# ============================================================

# 全局规划器 (A*, NavFn, 路径平滑)
add_library(nav_planners SHARED
  src/algorithms/global_planner/astar_planner.cpp
  src/algorithms/global_planner/path_smoother.cpp
  src/algorithms/global_planner/navfn_planner.cpp
)
target_include_directories(nav_planners PUBLIC include)
ament_target_dependencies(nav_planners
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# 局部控制器 (PurePursuit — 无 Nav2 依赖)
add_library(nav_controllers SHARED
  src/algorithms/local_planner/pure_pursuit_controller.cpp
)
target_include_directories(nav_controllers PUBLIC include)
ament_target_dependencies(nav_controllers
  rclcpp geometry_msgs nav_msgs)

# 支撑模块 (速度限制)
add_library(nav_tracking SHARED
  src/algorithms/support/velocity_limiter.cpp
)
target_include_directories(nav_tracking PUBLIC include)
ament_target_dependencies(nav_tracking rclcpp geometry_msgs)

# 安全通行 — 窄道检测
add_library(nav_narrow_passage SHARED
  src/algorithms/safe_passage/narrow_passage_detector.cpp
)
target_include_directories(nav_narrow_passage PUBLIC include)
ament_target_dependencies(nav_narrow_passage
  rclcpp sensor_msgs geometry_msgs nav_msgs)

# 局部避障 — 障碍物检测
add_library(nav_obstacle_detection SHARED
  src/algorithms/local_planner/obstacle_detector.cpp
)
target_include_directories(nav_obstacle_detection PUBLIC include)
ament_target_dependencies(nav_obstacle_detection
  rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs nav_msgs tf2_ros nav2_costmap_2d)

# 导航主库 (路径规划 + 轨迹生成 + 避障规划 + 窄道规划)
add_library(nav_navigation_lib SHARED
  src/algorithms/support/path_planner.cpp
  src/algorithms/support/trajectory_generator.cpp
  src/algorithms/local_planner/obstacle_avoidance_planner.cpp
  src/algorithms/safe_passage/narrow_passage_planner.cpp
)
target_include_directories(nav_navigation_lib PUBLIC include)
target_link_libraries(nav_navigation_lib
  nav_planners nav_narrow_passage nav_obstacle_detection)
ament_target_dependencies(nav_navigation_lib
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
  nav2_util nav2_costmap_2d)

# ============================================================
# nav_core 层 — 工厂
# ============================================================
add_library(nav_core SHARED
  src/nav_core/navigation_factory.cpp
)
target_include_directories(nav_core PUBLIC include)
target_link_libraries(nav_core nav_planners nav_controllers)
ament_target_dependencies(nav_core rclcpp)

# ============================================================
# ros_interface 层 — ROS2 生命周期节点
# ============================================================
add_library(nav_ros_interface SHARED
  src/ros_interface/navigation_node.cpp
  src/ros_interface/ros_utils.cpp
)
target_include_directories(nav_ros_interface PUBLIC include)
target_link_libraries(nav_ros_interface
  nav_navigation_lib nav_tracking nav_narrow_passage
  nav_obstacle_detection nav_core)
ament_target_dependencies(nav_ros_interface
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  sensor_msgs tf2 tf2_ros tf2_geometry_msgs nav2_util)

# ============================================================
# 可执行文件
# ============================================================
add_executable(rosiwit_navigation_node
  src/main.cpp)
target_link_libraries(rosiwit_navigation_node nav_ros_interface)
ament_target_dependencies(rosiwit_navigation_node rclcpp rclcpp_lifecycle)

# Nav2 依赖的控制器 (可选)
if(nav2_core_FOUND)
  add_library(nav_diff_drive SHARED
    src/algorithms/local_planner/diff_drive_controller.cpp
  )
  target_include_directories(nav_diff_drive PUBLIC include)
  target_link_libraries(nav_diff_drive nav_tracking)
  ament_target_dependencies(nav_diff_drive
    rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
    tf2_ros tf2_geometry_msgs nav2_core nav2_costmap_2d)

  install(TARGETS nav_diff_drive
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  find_package(ament_lint_auto QUIET)

  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_narrow_passage_detector test/test_narrow_passage_detector.cpp)
    target_link_libraries(test_narrow_passage_detector nav_narrow_passage GTest::GTest GTest::Main)
    ament_target_dependencies(test_narrow_passage_detector rclcpp sensor_msgs geometry_msgs)

    ament_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
    target_link_libraries(test_trajectory_generator nav_navigation_lib GTest::GTest GTest::Main)
    ament_target_dependencies(test_trajectory_generator rclcpp geometry_msgs nav_msgs)

    ament_add_gtest(test_velocity_limiter test/test_velocity_limiter.cpp)
    target_link_libraries(test_velocity_limiter nav_tracking GTest::GTest GTest::Main)
    ament_target_dependencies(test_velocity_limiter rclcpp geometry_msgs)

    ament_add_gtest(test_narrow_passage test/test_narrow_passage.cpp)
    target_link_libraries(test_narrow_passage
      nav_narrow_passage nav_navigation_lib GTest::GTest GTest::Main)
    ament_target_dependencies(test_narrow_passage rclcpp geometry_msgs nav_msgs)
  endif()

  if(COMMAND ament_lint_auto_find_test_dependencies)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# ============================================================
# 安装
# ============================================================
install(TARGETS
  nav_planners nav_controllers nav_tracking nav_narrow_passage
  nav_obstacle_detection nav_navigation_lib nav_core nav_ros_interface
  rosiwit_navigation_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
