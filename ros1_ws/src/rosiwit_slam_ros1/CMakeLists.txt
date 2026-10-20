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
    message_generation
    xzrobot_ros_dependency)

add_service_files(FILES SaveMap.srv LoadMap.srv SaveGridMap.srv SetSlamMode.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# 生成 map.jpg / thumb.png 要用 OpenCV (核心已依赖它; 这里显式声明, 供薄层直接用)
find_package(OpenCV REQUIRED)
# GTSAM/Ceres 的 imported target 会引用 Boost::timer；CMake 3.10 环境需要显式创建该 target，
# 否则在 51.19 这类机器上 add_library 会报 "Boost::timer not found"。
find_package(Boost REQUIRED COMPONENTS timer)

# 本包是纯接口层(仅一个可执行 + 一个内部静态库), 不对外导出头文件, 因此不声明 INCLUDE_DIRS
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
                   tf tf2 tf2_ros pcl_conversions visualization_msgs message_runtime
                   xzrobot_ros_dependency)

# ==================== Ceres 兼容垫片 (薄层专属, 不改核心) ====================
# Ubuntu 18.04 / melodic 的 libceres-dev 是 1.13.0: 其 CeresConfig.cmake 只定义了
# 无命名空间的 IMPORTED 目标 `ceres` 与变量 CERES_INCLUDE_DIRS/CERES_LIBRARIES,
# 而带命名空间的 `Ceres::ceres` 自 Ceres 1.14 才有。核心 CMakeLists 用的是 Ceres::ceres,
# 所以这里在 add_subdirectory 之前补一个等价目标。
# Ceres >= 1.14 时 NOT TARGET 为假 -> 整段自动跳过, 无副作用。
find_package(Ceres QUIET)
if(Ceres_FOUND AND NOT TARGET Ceres::ceres)
    # Ceres 1.13 的 CeresConfig.cmake 只导出无命名空间的 IMPORTED 目标 `ceres`,
    # 且它不是 GLOBAL 的(因此不能直接 add_library(... ALIAS ...))。
    # 这里补一个 GLOBAL 的 INTERFACE 目标桥接过去:
    #   INTERFACE_LINK_LIBRARIES 里的 `ceres` 会解析到上面那个导入目标
    #   (解析不到时退化为 -lceres, /usr/lib/libceres.so 存在, 同样可用)。
    add_library(Ceres::ceres INTERFACE IMPORTED GLOBAL)
    set_target_properties(Ceres::ceres PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${CERES_LIBRARIES}")
    message(STATUS "rosiwit_slam_ros1: shimmed Ceres::ceres (Ceres ${CERES_VERSION}, CERES_LIBRARIES='${CERES_LIBRARIES}')")
endif()

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_slam
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_slam_core)

# ==================== 接口层静态库 (便于 gtest 直接链接) ====================
set(ROSIWIT_ROS1_LIB_SOURCES
    src/config/config.cpp
    src/interface/realtime_grid.cpp
    src/interface/internal.cpp
    src/monitor/lost_monitor.cpp
    src/engine/slam_engine.cpp
    src/interface/outputs.cpp
    src/interface/convert.cpp
    src/interface/assembly.cpp
    src/interface/cloud_publisher.cpp
    src/interface/pose_publisher.cpp
    src/interface/warning.cpp
    src/interface/services.cpp
    src/io/xzmap_reader.cpp
    src/io/xzmap_writer.cpp
    src/io/map_adapter.cpp
    # T6 追加 src/legacy_subscribers.cpp 等
)
add_library(rosiwit_slam_ros1_lib ${ROSIWIT_ROS1_LIB_SOURCES})
target_include_directories(rosiwit_slam_ros1_lib PUBLIC
    src src/config src/engine src/interface src/monitor src/io
    ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(rosiwit_slam_ros1_lib PRIVATE -pipe -pthread)
target_link_libraries(rosiwit_slam_ros1_lib PUBLIC ${catkin_LIBRARIES} rosiwit_slam ${OpenCV_LIBS})
add_dependencies(rosiwit_slam_ros1_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS rosiwit_slam_ros1_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ==================== ROS1 节点 (薄) ====================
add_executable(rosiwit_slam_ros1_node src/main.cpp)
target_compile_options(rosiwit_slam_ros1_node PRIVATE -pipe -pthread)
target_link_libraries(rosiwit_slam_ros1_node rosiwit_slam_ros1_lib)
add_dependencies(rosiwit_slam_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 历史地图兼容工具: 从 map.xzmap 重新生成带 unknown 的 PGM
add_executable(repair_map_grid src/tools/repair_map_grid.cpp)
target_link_libraries(repair_map_grid rosiwit_slam_ros1_lib)
add_dependencies(repair_map_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS repair_map_grid RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(map_cleanup src/tools/map_cleanup.cpp)
target_link_libraries(map_cleanup rosiwit_slam_ros1_lib)
add_dependencies(map_cleanup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS map_cleanup RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ==================== 测试 ====================
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_config test/test_config.cpp)
    if(TARGET test_config)
        target_link_libraries(test_config rosiwit_slam_ros1_lib)
        # fixture 路径以宏传入, 不依赖测试进程的 CWD
        target_compile_definitions(test_config PRIVATE
            LEGACY_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()

    catkin_add_gtest(test_realtime_grid test/test_realtime_grid.cpp)
    if(TARGET test_realtime_grid)
        target_link_libraries(test_realtime_grid rosiwit_slam_ros1_lib)
        # 本包 config 目录以宏传入, 不依赖测试进程的 CWD
        target_compile_definitions(test_realtime_grid PRIVATE
            ROSIWIT_ROS1_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    endif()

    catkin_add_gtest(test_lost_monitor test/test_lost_monitor.cpp)
    if(TARGET test_lost_monitor)
        target_link_libraries(test_lost_monitor rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_calibration_controller ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_slam/test/test_calibration_controller.cpp)
    if(TARGET test_calibration_controller)
        target_link_libraries(test_calibration_controller rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_online_calibrator ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_slam/test/test_online_calibrator.cpp)
    if(TARGET test_online_calibrator)
        target_link_libraries(test_online_calibrator rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_convert test/test_convert.cpp)
    if(TARGET test_convert)
        target_link_libraries(test_convert rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_gicp_matcher_guard test/test_gicp_matcher_guard.cpp)
    if(TARGET test_gicp_matcher_guard)
        target_link_libraries(test_gicp_matcher_guard rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_reloc_disambig test/test_reloc_disambig.cpp)
    if(TARGET test_reloc_disambig)
        target_link_libraries(test_reloc_disambig rosiwit_slam_ros1_lib)
    endif()

    catkin_add_gtest(test_ekf_prior test/test_ekf_prior.cpp)
    if(TARGET test_ekf_prior)
        target_link_libraries(test_ekf_prior rosiwit_slam_ros1_lib)
    endif()
endif()

install(TARGETS rosiwit_slam_ros1_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 校验脚本与它的 golden 放同一个目录, 这样脚本里用 "$(dirname $0)" 就能找到 golden
install(PROGRAMS scripts/check_legacy_interface.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
install(FILES scripts/legacy_interface.golden
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
install(PROGRAMS scripts/smoke_legacy.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
install(PROGRAMS scripts/test_localization_roundtrip.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
# 合成数据发布器装在 bin, 这样 `rosrun rosiwit_slam_ros1 synthetic_publisher.py` 在
# devel 与 install 两种布局下都能找到
install(PROGRAMS test/synthetic_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
