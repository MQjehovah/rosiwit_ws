# ============================================================
# Diffbot Navigation - 简化 CMakeLists.txt
# 仅包含核心模块和基础测试
# ============================================================

cmake_minimum_required(VERSION 3.14)
project(rosiwit_navigation)

# 所有目标启用位置无关代码（共享库必需）
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ============================================================
# 编译器警告标志 (所有构建类型)
# ============================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wmisleading-indentation
  )
endif()

# MSVC 警告
if(MSVC)
  add_compile_options(/W4 /w14640)
endif()

# ============================================================
# 构建类型特定优化
# ============================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  # Release: 性能优先 + 安全加固
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -O2 -DNDEBUG
      -D_FORTIFY_SOURCE=2          # CVE-5 修复: glibc 缓冲区溢出检测
      -fstack-protector-strong     # 栈溢出保护
      -fstack-clash-protection     # 栈冲突保护
      -fPIC                        # 位置无关代码（共享库必须用 -fPIC 而非 -fPIE）
    )
    # 链接时加固
    add_link_options(
      -Wl,-z,relro
      -Wl,-z,now
      -Wl,-z,noexecstack
    )
  endif()
  message(STATUS "Building in Release mode with -O2 -DNDEBUG + security hardening")

elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  # RelWithDebInfo: 性能 + 调试符号 + 安全加固
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -O2 -g
      -D_FORTIFY_SOURCE=2
      -fstack-protector-strong
      -fPIC                        # 位置无关代码（共享库必须用 -fPIC 而非 -fPIE）
    )
    add_link_options(-Wl,-z,relro -Wl,-z,now)
  endif()
  message(STATUS "Building in RelWithDebInfo mode with -O2 -g + hardening")

elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Debug: 无优化 + Sanitizers (CVE-6 修复)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O0 -g)

    # AddressSanitizer: 检测内存错误 (越界、use-after-free、double-free)
    option(ENABLE_ASAN "Enable AddressSanitizer" ON)
    if(ENABLE_ASAN AND NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
      add_link_options(-fsanitize=address)
      message(STATUS "Debug: AddressSanitizer enabled")
    endif()

    # UndefinedBehaviorSanitizer: 检测未定义行为
    option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" ON)
    if(ENABLE_UBSAN)
      add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
      add_link_options(-fsanitize=undefined)
      message(STATUS "Debug: UndefinedBehaviorSanitizer enabled")
    endif()
  endif()
  message(STATUS "Building in Debug mode with -O0 -g + sanitizers")
endif()

# ============================================================
# 依赖项
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(GTest REQUIRED)

# Nav2 可选依赖
find_package(nav2_core QUIET)
find_package(nav2_util QUIET)
find_package(nav2_costmap_2d QUIET)
message(STATUS "DEBUG: nav2_core_DIR = '${nav2_core_DIR}'")
if(NOT "${nav2_core_DIR}" STREQUAL "")
  message(STATUS "Nav2 core found - will build Nav2-dependent targets")
else()
  message(STATUS "Nav2 core NOT found - skipping Nav2-dependent targets")
endif()

# ============================================================
# 窄道检测器库
# ============================================================
add_library(diffbot_narrow_passage_lib SHARED
  src/narrow_passage/narrow_passage_detector.cpp
)
target_include_directories(diffbot_narrow_passage_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diffbot_narrow_passage_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# ============================================================
# 规划器库
# ============================================================
add_library(diffbot_planners_lib SHARED
  src/planners/navfn_planner.cpp
  src/planners/astar_planner.cpp
)
target_include_directories(diffbot_planners_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diffbot_planners_lib
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# ============================================================
# 控制器库
# ============================================================
add_library(diffbot_controllers_lib SHARED
  src/controllers/pure_pursuit_controller.cpp
)
target_include_directories(diffbot_controllers_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diffbot_controllers_lib
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# ============================================================
# 导航库——轨迹生成、路径规划、避障等
# ============================================================
add_library(diffbot_navigation_lib SHARED
  src/navigation/trajectory_generator.cpp
  src/navigation/path_planner.cpp
  src/navigation/smooth_navigation.cpp
  src/navigation/smooth_navigation_main.cpp
  src/obstacle_avoidance/obstacle_detector.cpp
  src/obstacle_avoidance/obstacle_avoidance_planner.cpp
  src/narrow_passage/narrow_passage_planner.cpp
)
target_include_directories(diffbot_navigation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(diffbot_navigation_lib
  diffbot_narrow_passage_lib
  diffbot_planners_lib
)
ament_target_dependencies(diffbot_navigation_lib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  nav2_costmap_2d
)

# ============================================================
# 控制器工具库——速度限制器、差速驱动控制器
# （注意：PIDController 在 pid_controller.hpp 中为 header-only，
#  diff_drive_controller.cpp 依赖 nav2_core，单独编译）
# ============================================================
add_library(diffbot_controller_utils_lib SHARED
  src/controller/velocity_limiter.cpp
)
target_include_directories(diffbot_controller_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diffbot_controller_utils_lib
  rclcpp
  geometry_msgs
)

# diff_drive_controller 依赖 nav2_core，仅在 Nav2 可用时编译
if(NOT "${nav2_core_DIR}" STREQUAL "")
  add_library(diffbot_controller_lib SHARED
    src/controller/diff_drive_controller.cpp
  )
  target_include_directories(diffbot_controller_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(diffbot_controller_lib
    diffbot_controller_utils_lib
  )
  ament_target_dependencies(diffbot_controller_lib
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    nav2_core
    nav2_costmap_2d
  )
endif()

# ============================================================
# 单元测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # 窄道检测器测试
  ament_add_gtest(test_narrow_passage_detector test/test_narrow_passage_detector.cpp)
  target_link_libraries(test_narrow_passage_detector
    diffbot_narrow_passage_lib
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_narrow_passage_detector
    rclcpp
    sensor_msgs
    geometry_msgs
  )

  # 差速控制器基础测试（不依赖 Nav2）
  ament_add_gtest(test_diff_drive_controller test/test_diff_drive_controller.cpp)
  target_link_libraries(test_diff_drive_controller
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_diff_drive_controller
    rclcpp
    geometry_msgs
    nav_msgs
  )

  # 轨迹生成器测试
  ament_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
  target_link_libraries(test_trajectory_generator
    diffbot_navigation_lib
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_trajectory_generator
    rclcpp
    geometry_msgs
    nav_msgs
  )

  # 速度限制器测试
  ament_add_gtest(test_velocity_limiter test/test_velocity_limiter.cpp)
  target_link_libraries(test_velocity_limiter
    diffbot_controller_utils_lib
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_velocity_limiter
    rclcpp
    geometry_msgs
  )

  # ==========================================================
  # 障碍物检测器测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_obstacle_detector test/test_obstacle_detector.cpp)
  # target_link_libraries(test_obstacle_detector
  #   diffbot_navigation_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_obstacle_detector
  #   rclcpp
  #   geometry_msgs
  #   sensor_msgs
  # )

  # ==========================================================
  # 窄通道测试
  # ==========================================================
  ament_add_gtest(test_narrow_passage test/test_narrow_passage.cpp)
  target_link_libraries(test_narrow_passage
    diffbot_narrow_passage_lib
    diffbot_navigation_lib
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_narrow_passage
    rclcpp
    geometry_msgs
    nav_msgs
  )

  # ==========================================================
  # 避障测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_obstacle_avoidance test/test_obstacle_avoidance.cpp)
  # target_link_libraries(test_obstacle_avoidance
  #   diffbot_navigation_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_obstacle_avoidance
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs
  # )

  # ==========================================================
  # 性能测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_performance test/test_performance.cpp)
  # target_link_libraries(test_performance
  #   diffbot_navigation_lib
  #   diffbot_planners_lib
  #   diffbot_controller_utils_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_performance
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs

  # ==========================================================
  # 单点导航测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_single_point_navigation test/test_single_point_navigation.cpp)
  # target_link_libraries(test_single_point_navigation
  #   diffbot_navigation_lib
  #   diffbot_controller_utils_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_single_point_navigation
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs

  # ==========================================================
  # 轨迹生成器边缘测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_trajectory_generator_edge test/test_trajectory_generator_edge.cpp)
  # target_link_libraries(test_trajectory_generator_edge
  #   diffbot_navigation_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_trajectory_generator_edge
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs

  # ==========================================================
  # A* 规划器测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  # target_link_libraries(test_astar_planner
  #   diffbot_planners_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_astar_planner
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs

  # ==========================================================
  # A* 规划器性能测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_astar_planner_perf test/test_astar_planner_perf.cpp)
  # target_link_libraries(test_astar_planner_perf
  #   diffbot_planners_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_astar_planner_perf
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs

  # ==========================================================
  # 窄通道增强测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_narrow_passage_enhanced test/test_narrow_passage_enhanced.cpp)
  # target_link_libraries(test_narrow_passage_enhanced
  #   diffbot_narrow_passage_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_narrow_passage_enhanced
  #   rclcpp
  #   sensor_msgs
  #   geometry_msgs
  # )

  # ==========================================================
  # PID 控制器测试 (TODO: API 不匹配，需要重写测试)
  # ==========================================================
  # ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  # target_link_libraries(test_pid_controller
  #   diffbot_controller_utils_lib
  #   GTest::GTest
  #   GTest::Main
  # )
  # ament_target_dependencies(test_pid_controller
  #   rclcpp
  #   geometry_msgs
  #   nav_msgs
  # )

  # ==========================================================
  # 代码规范检查
  # ==========================================================
  if(COMMAND ament_lint_auto_find_test_dependencies)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# ============================================================
# 安装
# ============================================================
install(TARGETS
  diffbot_narrow_passage_lib
  diffbot_planners_lib
  diffbot_controllers_lib
  diffbot_navigation_lib
  diffbot_controller_utils_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET diffbot_controller_lib)
  install(TARGETS
    diffbot_controller_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
