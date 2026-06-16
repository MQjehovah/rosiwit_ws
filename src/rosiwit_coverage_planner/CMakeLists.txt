cmake_minimum_required(VERSION 3.8)
project(ros2_coverage_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

# Library: Coverage Utilities
add_library(coverage_utils SHARED
  src/coverage_utils.cpp
)
ament_target_dependencies(coverage_utils
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(coverage_utils ${OpenCV_LIBS})

# Library: Planner Context
add_library(planner_context SHARED
  src/planner_context.cpp
)
ament_target_dependencies(planner_context
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(planner_context
  coverage_utils
  zigzag_planner
  spiral_planner
)

# Library: Map Preprocessor (P0 Optimization)
add_library(map_preprocessor SHARED
  src/map_preprocessor.cpp
)
ament_target_dependencies(map_preprocessor
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(map_preprocessor ${OpenCV_LIBS})

# Library: Scan Direction Optimizer (P0 Optimization)
add_library(scan_direction_optimizer SHARED
  src/scan_direction_optimizer.cpp
)
ament_target_dependencies(scan_direction_optimizer
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(scan_direction_optimizer ${OpenCV_LIBS})

# Library: Zone Decomposer (P0 Optimization)
add_library(zone_decomposer SHARED
  src/zone_decomposer.cpp
)
ament_target_dependencies(zone_decomposer
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(zone_decomposer
  coverage_utils
  ${OpenCV_LIBS}
)

# Library: Turn Optimizer (P0 Optimization)
add_library(turn_optimizer SHARED
  src/turn_optimizer.cpp
)
ament_target_dependencies(turn_optimizer
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(turn_optimizer ${OpenCV_LIBS})

# Library: Zigzag Planner
add_library(zigzag_planner SHARED
  src/zigzag_planner.cpp
)
ament_target_dependencies(zigzag_planner
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(zigzag_planner
  coverage_utils
  map_preprocessor
  scan_direction_optimizer
  zone_decomposer
  turn_optimizer
  ${OpenCV_LIBS}
)

# Library: Spiral Planner
add_library(spiral_planner SHARED
  src/spiral_planner.cpp
)
ament_target_dependencies(spiral_planner
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(spiral_planner
  coverage_utils
  turn_optimizer
  zigzag_planner
  ${OpenCV_LIBS}
)

# Library: Coverage Planner Node (Component)
add_library(coverage_planner_component SHARED
  src/coverage_planner_node.cpp
)
ament_target_dependencies(coverage_planner_component
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(coverage_planner_component
  coverage_utils
  planner_context
  zigzag_planner
  spiral_planner
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(coverage_planner_component "coverage_planner::CoveragePlannerNode")

# Executable: Coverage Planner Node (main)
add_executable(coverage_planner_node
  src/main.cpp
)
ament_target_dependencies(coverage_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
)
target_link_libraries(coverage_planner_node
  coverage_planner_component
)

# Install targets
install(TARGETS
  coverage_utils
  planner_context
  map_preprocessor
  scan_direction_optimizer
  zone_decomposer
  turn_optimizer
  zigzag_planner
  spiral_planner
  coverage_planner_component
  coverage_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install test maps (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_maps")
  install(DIRECTORY test_maps/
    DESTINATION share/${PROJECT_NAME}/test_maps
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest testing support
  find_package(ament_cmake_gtest REQUIRED)

  # ===========================================
  # Unit Tests for ZoneDecomposer (P0 Module)
  # ===========================================
  ament_add_gtest(test_zone_decomposer
    test/test_zone_decomposer.cpp
  )
  target_link_libraries(test_zone_decomposer
    zone_decomposer
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_zone_decomposer
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Unit Tests for TurnOptimizer (P0 Module)
  # ===========================================
  ament_add_gtest(test_turn_optimizer
    test/test_turn_optimizer.cpp
  )
  target_link_libraries(test_turn_optimizer
    turn_optimizer
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_turn_optimizer
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Acceptance Tests (验收标准测试)
  # ===========================================
  ament_add_gtest(test_acceptance
    test/test_acceptance.cpp
  )
  target_link_libraries(test_acceptance
    zone_decomposer
    turn_optimizer
    coverage_utils
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_acceptance
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Unit Tests for CoverageUtils
  # ===========================================
  ament_add_gtest(test_coverage_utils
    test/test_coverage_utils.cpp
  )
  target_link_libraries(test_coverage_utils
    coverage_utils
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_coverage_utils
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ===========================================
  # Unit Tests for ZigzagPlanner
  # ===========================================
  ament_add_gtest(test_zigzag_planner
    test/test_zigzag_planner.cpp
  )
  target_link_libraries(test_zigzag_planner
    zigzag_planner
    coverage_utils
    zone_decomposer
    turn_optimizer
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_zigzag_planner
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Unit Tests for SpiralPlanner
  # ===========================================
  ament_add_gtest(test_spiral_planner
    test/test_spiral_planner.cpp
  )
  target_link_libraries(test_spiral_planner
    spiral_planner
    coverage_utils
    turn_optimizer
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_spiral_planner
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Integration Tests
  # ===========================================
  ament_add_gtest(test_integration
    test/test_integration.cpp
  )
  target_link_libraries(test_integration
    zigzag_planner
    spiral_planner
    coverage_utils
    zone_decomposer
    turn_optimizer
    planner_context
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_integration
    rclcpp
    nav_msgs
    geometry_msgs
  )

  # ===========================================
  # Performance Tests
  # ===========================================
  ament_add_gtest(test_performance
    test/test_performance.cpp
  )
  target_link_libraries(test_performance
    zone_decomposer
    turn_optimizer
    zigzag_planner
    spiral_planner
    coverage_utils
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(test_performance
    rclcpp
    nav_msgs
    geometry_msgs
  )
endif()

ament_package()