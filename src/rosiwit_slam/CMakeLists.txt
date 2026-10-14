cmake_minimum_required(VERSION 3.16)
project(rosiwit_slam VERSION 1.0.0 LANGUAGES C CXX)

# ==================== 编译设置 ====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# ==================== Unity构建加速 (CMake ≥ 3.16) ====================
# 将多个源文件合并为少数编译单元(batch)，显著减少头文件重复解析开销
# 注意: 需显式列出源文件或用 GLOB_RECURSE "src/**/*.cpp"
# set(CMAKE_UNITY_BUILD ON)
# set(CMAKE_UNITY_BUILD_BATCH_SIZE 8)

# ==================== 编译器加速标志 ====================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    # -pipe: 使用管道而非临时文件，减少磁盘I/O
    # -pthread: ikd_Tree 使用 pthread 互斥锁
    string(APPEND CMAKE_CXX_FLAGS " -pipe -pthread")
    string(APPEND CMAKE_C_FLAGS " -pthread")
endif()

# ==================== ccache (可选, 缓存未变更的 TU) ====================
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" CACHE STRING "" FORCE)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" CACHE STRING "" FORCE)
    # PCH 兼容: 允许 pch_defines/time_macros 变更时不刷新缓存
    set(ENV{CCACHE_SLOPPINESS} "pch_defines,time_macros")
    message(STATUS "ccache enabled: ${CCACHE_PROGRAM}")
else()
    message(STATUS "ccache not found, install with: sudo apt install ccache")
endif()

# ==================== ROS2依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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

# ==================== 第三方库依赖 ====================
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration segmentation sample_consensus features)

# Sophus - 系统库 (ros-humble-sophus)
find_package(Sophus REQUIRED)
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# FAST-LIO2 lidar_processor 多线程并行 (匹配 FAST-LIO2_ROS2)
find_package(OpenMP QUIET)
if(OpenMP_FOUND)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
    string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")
endif()

# Ceres - 用于后端位姿图优化
find_package(Ceres REQUIRED)

# GTSAM - 尝试查找，用于后端位姿图优化
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
    message(STATUS "GTSAM found: enabling ISAM2 backend optimization")
    add_compile_definitions(USE_GTSAM)
else()
    message(STATUS "GTSAM not found: using simplified pose graph optimization")
endif()

# octomap - 2D 栅格第三方对照策略 (octomap_grid_map)
find_package(octomap QUIET)
if(octomap_FOUND)
    message(STATUS "octomap found: enabling 'octomap' grid strategy")
    add_compile_definitions(HAVE_OCTOMAP)
else()
    message(STATUS "octomap not found: 'octomap' grid strategy disabled")
endif()

# 尝试查找yaml-cpp
find_package(yaml-cpp QUIET)

# OpenCV - 2D 栅格后处理管道 (形态学/细化/直线)
find_package(OpenCV REQUIRED)

# ==================== 编译选项 ====================
if(NOT yaml-cpp_FOUND)
    message(WARNING "yaml-cpp not found, configuration file loading disabled")
    add_compile_definitions(YAML_CPP_DISABLED)
endif()

# PCL不使用Boost (在 NODE_NAME 定义之后设置)
# target_compile_definitions(${NODE_NAME} PRIVATE BOOST_ALL_NO_LIB) # moved below

# ==================== 包含目录 ====================
include_directories(
    include
    src
    src/algorithms/fast_lio2
    src/algorithms/fast_lio2/map_builder
    src/algorithms
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

# ROS2消息包含目录
include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 添加各个ROS包的include路径
foreach(dep rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros std_srvs pcl_conversions visualization_msgs)
    if(${dep}_INCLUDE_DIRS)
        include_directories(${${dep}_INCLUDE_DIRS})
    endif()
endforeach()

if(yaml-cpp_FOUND)
    include_directories(${yaml-cpp_INCLUDE_DIRS})
endif()

if(Ceres_FOUND)
    include_directories(${CERES_INCLUDE_DIRS})
endif()

if(GTSAM_FOUND)
    include_directories(${GTSAM_INCLUDE_DIRS})
endif()

if(octomap_FOUND)
    include_directories(${OCTOMAP_INCLUDE_DIRS})
endif()

# ==================== 源文件 (显式列表, 避免 GLOB_RECURSE 遗漏新文件) ====================
# 2D 栅格生成策略 (接口 slam_core/i_grid_map_generator.h)
set(GRID_MAP_SOURCES
    src/algorithms/grid_map/height_projection_grid_map.cpp
    src/algorithms/grid_map/ray_casting_grid_map.cpp
    src/algorithms/grid_map/occupancy_logodds.cpp
    src/algorithms/grid_map/grid_map_io.cpp
    src/algorithms/grid_map/grid_map_factory.cpp
    src/algorithms/grid_map/grid_map_postprocess.cpp
    src/algorithms/grid_map/grid_map_metrics.cpp
    src/algorithms/grid_map/ground_segmentation.cpp
)

# octomap 策略仅在找到 octomap 时编入 (策略内部由 HAVE_OCTOMAP 条件注册)
if(octomap_FOUND)
    list(APPEND GRID_MAP_SOURCES src/algorithms/grid_map/octomap_grid_map.cpp)
endif()

# 可编排定位流水线 (接口 slam_core/i_scan_matcher.h 等): 各阶段可插拔策略
set(LOCALIZATION_SOURCES
    src/algorithms/localization/localization_utils.cpp
    src/algorithms/localization/registration_metrics.cpp
    src/algorithms/localization/localization_factory.cpp
    src/algorithms/localization/localization_pipeline.cpp
    src/algorithms/localization/matcher/gicp_matcher.cpp
    src/algorithms/localization/matcher/pp_icp_matcher.cpp
    src/algorithms/localization/matcher/p2p_icp_matcher.cpp
    src/algorithms/localization/matcher/vgicp_matcher.cpp
    src/algorithms/localization/local_map/global_map_provider.cpp
    src/algorithms/localization/local_map/radius_submap_provider.cpp
    src/algorithms/localization/local_map/keyframe_submap_provider.cpp
    src/algorithms/localization/motion_prior/constant_velocity_prior.cpp
    src/algorithms/localization/motion_prior/imu_motion_prior.cpp
    src/algorithms/localization/monitor/threshold_monitor.cpp
    src/algorithms/localization/monitor/windowed_monitor.cpp
    src/algorithms/localization/maintainer/overlay_map_maintainer.cpp
    src/algorithms/localization/global/scan_context_global_localizer.cpp
    src/algorithms/localization/global/bbs_global_localizer.cpp
    src/algorithms/localization/global/fpfh_ransac_global_localizer.cpp
)

set(SOURCES
    src/main.cpp
    src/ros_interface/ros_utils.cpp
    src/ros_interface/slam_node.cpp
    src/slam_core/slam_base.cpp
    src/slam_core/slam_factory.cpp
    src/slam_core/slam_pipeline.cpp
    src/io/xzmap_writer.cpp
    src/io/xzmap_reader.cpp
    src/algorithms/fast_lio2/fast_lio2_frontend.cpp
    src/algorithms/fast_lio2/map_builder/commons.cpp
    src/algorithms/fast_lio2/map_builder/ieskf.cpp
    src/algorithms/fast_lio2/map_builder/imu_processor.cpp
    src/algorithms/fast_lio2/map_builder/lidar_processor.cpp
    src/algorithms/fast_lio2/map_builder/ikd_Tree.cpp
    src/algorithms/fast_lio2/map_builder/map_builder.cpp
    src/algorithms/ceres_backend/ceres_backend.cpp
    src/algorithms/gicp_localization/gicp_localization.cpp
    src/algorithms/pcd_map_manager/pcd_map_manager.cpp
    ${GRID_MAP_SOURCES}
    ${LOCALIZATION_SOURCES}
    # 新增算法 (2026-07)
    src/algorithms/gtsam_backend/gtsam_isam2_backend.cpp
    src/algorithms/scan_context/scan_context.cpp
    src/algorithms/scan_context/scan_context_lc.cpp
    src/algorithms/scan_context/scan_context_localization.cpp
    src/algorithms/point_lio/point_lio_frontend.cpp
)

# ==================== 可执行目标 ====================
# 使用独立的 NODE_NAME 避免与 rosidl_generate_interfaces(${PROJECT_NAME} ...) 目标冲突
set(NODE_NAME slam_node)
add_executable(${NODE_NAME} ${SOURCES})
set_target_properties(${NODE_NAME} PROPERTIES OUTPUT_NAME "rosiwit_slam")

# PCL不使用Boost (仅对当前目标生效, 避免影响第三方依赖)
target_compile_definitions(${NODE_NAME} PRIVATE BOOST_ALL_NO_LIB)

# ==================== 预编译头 (PCH) ====================
# PCH 让编译器将 pch.hpp 中的重型头文件 (Eigen, PCL, STL) 预编译一次，
# 所有 TU 共享该缓存，大幅减少头文件重复解析开销。
# 注意：不包含 <rclcpp/rclcpp.hpp> 以避免旧版 GCC 的 std::invoke_result bug。
set(PCH_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/pch.hpp)
target_precompile_headers(${NODE_NAME} PRIVATE ${PCH_HEADER})

ament_target_dependencies(${NODE_NAME}
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    std_srvs
    pcl_conversions
    visualization_msgs
)

# 链接生成的 Service 消息类型
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${NODE_NAME} "${typesupport_target}")

# 链接库
target_link_libraries(${NODE_NAME}
    ${PCL_LIBRARIES}
)

if(octomap_FOUND)
    target_link_libraries(${NODE_NAME} ${OCTOMAP_LIBRARIES})
endif()

target_link_libraries(${NODE_NAME} ${OpenCV_LIBS})

# Sophus - 仅在找到时链接
if(Sophus_FOUND)
    target_link_libraries(${NODE_NAME} Sophus::Sophus)
endif()

if(yaml-cpp_FOUND)
    target_link_libraries(${NODE_NAME} yaml-cpp)
endif()

if(Ceres_FOUND)
    target_link_libraries(${NODE_NAME} Ceres::ceres)
endif()

if(GTSAM_FOUND)
    target_link_libraries(${NODE_NAME} gtsam)
endif()

# ==================== 编译定义 ====================
target_compile_definitions(${NODE_NAME} PRIVATE
    EIGEN_MPL2_ONLY
)

# ==================== 离线栅格对比工具 ====================
# 读取 PCD, 用所有已注册策略各生成一张 PGM/YAML, 便于横向对比
add_executable(grid_map_tool tools/grid_map_tool.cpp ${GRID_MAP_SOURCES})
target_include_directories(grid_map_tool PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
if(octomap_FOUND)
    target_link_libraries(grid_map_tool ${OCTOMAP_LIBRARIES})
endif()
target_link_libraries(grid_map_tool ${PCL_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS grid_map_tool DESTINATION lib/${PROJECT_NAME})

# 离线后处理/评估工具
add_executable(grid_map_post tools/grid_map_post.cpp
    src/algorithms/grid_map/grid_map_io.cpp
    src/algorithms/grid_map/grid_map_postprocess.cpp
    src/algorithms/grid_map/grid_map_metrics.cpp)
target_include_directories(grid_map_post PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(grid_map_post ${OpenCV_LIBS})
install(TARGETS grid_map_post DESTINATION lib/${PROJECT_NAME})

# ==================== 离线定位配准评估工具 ====================
# 对同一地图/扫描依次运行各已注册配准器, 输出误差/覆盖率/残差/耗时对比
add_executable(localization_eval tools/localization_eval.cpp
    ${LOCALIZATION_SOURCES}
    src/io/xzmap_reader.cpp
    src/algorithms/scan_context/scan_context.cpp)
target_include_directories(localization_eval PRIVATE
    include src ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(localization_eval ${PCL_LIBRARIES})
if(yaml-cpp_FOUND)
    target_link_libraries(localization_eval yaml-cpp)
endif()
install(TARGETS localization_eval DESTINATION lib/${PROJECT_NAME})

# ==================== 安装 ====================
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

# ==================== 测试 ====================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Sophus QUIET)

    # ---- 依赖 Sophus 的测试 (FastLio2Frontend 使用 Sophus::SO3d) ----
    if(Sophus_FOUND)

        # SlamFactory 单元测试
        ament_add_gtest(test_slam_factory
            test/test_slam_factory.cpp
            src/slam_core/slam_pipeline.cpp
            src/slam_core/slam_factory.cpp
            src/slam_core/slam_base.cpp
            src/io/xzmap_writer.cpp
            src/io/xzmap_reader.cpp
            src/algorithms/fast_lio2/fast_lio2_frontend.cpp
            src/algorithms/fast_lio2/map_builder/commons.cpp
            src/algorithms/fast_lio2/map_builder/ieskf.cpp
            src/algorithms/fast_lio2/map_builder/imu_processor.cpp
            src/algorithms/fast_lio2/map_builder/lidar_processor.cpp
            src/algorithms/fast_lio2/map_builder/ikd_Tree.cpp
            src/algorithms/fast_lio2/map_builder/map_builder.cpp)
        target_include_directories(test_slam_factory PRIVATE
            include src src/algorithms src/algorithms/fast_lio2 src/algorithms/fast_lio2/map_builder
            ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
        target_link_libraries(test_slam_factory ${PCL_LIBRARIES} Sophus::Sophus)
        if(yaml-cpp_FOUND)
            target_link_libraries(test_slam_factory yaml-cpp)
        endif()
        if(octomap_FOUND)
            target_link_libraries(test_slam_factory ${OCTOMAP_LIBRARIES})
        endif()
        target_link_libraries(test_slam_factory ${OpenCV_LIBS})
        if(Ceres_FOUND)
            target_sources(test_slam_factory PRIVATE src/algorithms/ceres_backend/ceres_backend.cpp)
            target_link_libraries(test_slam_factory Ceres::ceres)
        endif()
        target_sources(test_slam_factory PRIVATE
            src/algorithms/gicp_localization/gicp_localization.cpp
            src/algorithms/pcd_map_manager/pcd_map_manager.cpp
            ${GRID_MAP_SOURCES}
            ${LOCALIZATION_SOURCES}
            src/algorithms/scan_context/scan_context.cpp
            src/algorithms/scan_context/scan_context_lc.cpp
            src/algorithms/scan_context/scan_context_localization.cpp
            src/algorithms/point_lio/point_lio_frontend.cpp)
        if(GTSAM_FOUND)
            target_sources(test_slam_factory PRIVATE src/algorithms/gtsam_backend/gtsam_isam2_backend.cpp)
            target_link_libraries(test_slam_factory gtsam)
        endif()
        install(TARGETS test_slam_factory DESTINATION lib/${PROJECT_NAME})

        # SlamPipeline orchestrator test
        ament_add_gtest(test_slam_pipeline
            test/test_slam_pipeline.cpp
            src/slam_core/slam_pipeline.cpp
            src/slam_core/slam_base.cpp
            src/slam_core/slam_factory.cpp
            src/io/xzmap_writer.cpp
            src/io/xzmap_reader.cpp
            src/algorithms/fast_lio2/fast_lio2_frontend.cpp
            src/algorithms/fast_lio2/map_builder/commons.cpp
            src/algorithms/fast_lio2/map_builder/ieskf.cpp
            src/algorithms/fast_lio2/map_builder/imu_processor.cpp
            src/algorithms/fast_lio2/map_builder/lidar_processor.cpp
            src/algorithms/fast_lio2/map_builder/ikd_Tree.cpp
            src/algorithms/fast_lio2/map_builder/map_builder.cpp)
        target_include_directories(test_slam_pipeline PRIVATE
            include src src/algorithms src/algorithms/fast_lio2 src/algorithms/fast_lio2/map_builder
            ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
        target_link_libraries(test_slam_pipeline ${PCL_LIBRARIES} Sophus::Sophus)
        if(yaml-cpp_FOUND)
            target_link_libraries(test_slam_pipeline yaml-cpp)
        endif()
        if(octomap_FOUND)
            target_link_libraries(test_slam_pipeline ${OCTOMAP_LIBRARIES})
        endif()
        target_link_libraries(test_slam_pipeline ${OpenCV_LIBS})
        if(Ceres_FOUND)
            target_sources(test_slam_pipeline PRIVATE src/algorithms/ceres_backend/ceres_backend.cpp)
            target_link_libraries(test_slam_pipeline Ceres::ceres)
        endif()
        target_sources(test_slam_pipeline PRIVATE
            src/algorithms/gicp_localization/gicp_localization.cpp
            src/algorithms/pcd_map_manager/pcd_map_manager.cpp
            ${GRID_MAP_SOURCES}
            ${LOCALIZATION_SOURCES}
            src/algorithms/scan_context/scan_context.cpp
            src/algorithms/scan_context/scan_context_lc.cpp
            src/algorithms/scan_context/scan_context_localization.cpp
            src/algorithms/point_lio/point_lio_frontend.cpp)
        if(GTSAM_FOUND)
            target_sources(test_slam_pipeline PRIVATE src/algorithms/gtsam_backend/gtsam_isam2_backend.cpp)
            target_link_libraries(test_slam_pipeline gtsam)
        endif()
        install(TARGETS test_slam_pipeline DESTINATION lib/${PROJECT_NAME})

    else()
        message(STATUS "Sophus not found — skipping test_slam_factory and test_slam_pipeline (need Sophus for SO3d)")
    endif()

    # ---- 不依赖 Sophus 的测试 (可无条件运行) ----

    # SlamBase 同步基类单元测试
    ament_add_gtest(test_slam_base_sync test/test_slam_base_sync.cpp src/slam_core/slam_base.cpp)
    target_include_directories(test_slam_base_sync PRIVATE
        include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_slam_base_sync ${PCL_LIBRARIES})
    install(TARGETS test_slam_base_sync DESTINATION lib/${PROJECT_NAME})

    # CeresBackend 位姿图优化单元测试
    ament_add_gtest(test_ceres_backend
        test/test_ceres_backend.cpp
        src/algorithms/ceres_backend/ceres_backend.cpp)
    target_include_directories(test_ceres_backend PRIVATE
        include ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
    target_link_libraries(test_ceres_backend Ceres::ceres)
    if(yaml-cpp_FOUND)
        target_include_directories(test_ceres_backend PRIVATE ${yaml-cpp_INCLUDE_DIRS})
        target_link_libraries(test_ceres_backend yaml-cpp)
    endif()
    install(TARGETS test_ceres_backend DESTINATION lib/${PROJECT_NAME})

    # GICPLocalization 单元测试
    ament_add_gtest(test_gicp_localization
        test/test_gicp_localization.cpp
        src/algorithms/gicp_localization/gicp_localization.cpp)
    target_include_directories(test_gicp_localization PRIVATE
        include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_gicp_localization ${PCL_LIBRARIES})
    if(yaml-cpp_FOUND)
        target_link_libraries(test_gicp_localization yaml-cpp)
    endif()
    install(TARGETS test_gicp_localization DESTINATION lib/${PROJECT_NAME})

    # PcdMapManager 单元测试
    ament_add_gtest(test_pcd_map_manager
        test/test_pcd_map_manager.cpp
        src/algorithms/pcd_map_manager/pcd_map_manager.cpp
        ${GRID_MAP_SOURCES})
    target_include_directories(test_pcd_map_manager PRIVATE
        include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_pcd_map_manager ${PCL_LIBRARIES})
    if(yaml-cpp_FOUND)
        target_link_libraries(test_pcd_map_manager yaml-cpp)
    endif()
    if(octomap_FOUND)
        target_link_libraries(test_pcd_map_manager ${OCTOMAP_LIBRARIES})
    endif()
    target_link_libraries(test_pcd_map_manager ${OpenCV_LIBS})
    install(TARGETS test_pcd_map_manager DESTINATION lib/${PROJECT_NAME})

    # 2D 栅格生成策略契约测试 (参数化遍历所有策略)
    ament_add_gtest(test_grid_map_generator
        test/test_grid_map_generator.cpp
        ${GRID_MAP_SOURCES})
    target_include_directories(test_grid_map_generator PRIVATE
        include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_grid_map_generator ${PCL_LIBRARIES})
    if(octomap_FOUND)
        target_link_libraries(test_grid_map_generator ${OCTOMAP_LIBRARIES})
    endif()
    target_link_libraries(test_grid_map_generator ${OpenCV_LIBS})
    install(TARGETS test_grid_map_generator DESTINATION lib/${PROJECT_NAME})

    # 定位流水线各阶段契约测试 (参数化遍历所有配准器)
    ament_add_gtest(test_localization_pipeline
        test/test_localization_pipeline.cpp
        ${LOCALIZATION_SOURCES}
        src/io/xzmap_writer.cpp
        src/io/xzmap_reader.cpp
        src/algorithms/scan_context/scan_context.cpp)
    target_include_directories(test_localization_pipeline PRIVATE
        include src ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_localization_pipeline ${PCL_LIBRARIES})
    if(yaml-cpp_FOUND)
        target_link_libraries(test_localization_pipeline yaml-cpp)
    endif()
    install(TARGETS test_localization_pipeline DESTINATION lib/${PROJECT_NAME})

    # 注释保留: 已移除的旧测试说明 (test_scan_context_lc 已随 simple_scan_context 删除)
    # 2. IEKF/iKD-Tree 测试已移除 ...
    # 3. 点云滤波器/IMU处理器测试已移除 ...
    # MapManager测试已移除 ...
endif()

# ==================== 导出 ====================
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    std_srvs
    pcl_conversions
    visualization_msgs
)

# ==================== ament包 ====================
ament_package()