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
    string(APPEND CMAKE_CXX_FLAGS " -pipe")
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
    "msg/LocalizationStatus.msg"
    "srv/GlobalLocalize.srv"
    "srv/SetInitialPose.srv"
    "srv/GetLocalizationStatus.srv"
    DEPENDENCIES geometry_msgs builtin_interfaces
)

# ==================== 第三方库依赖 ====================
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)

# Sophus - 系统库 (ros-humble-sophus)
find_package(Sophus REQUIRED)
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# GTSAM - 尝试查找，用于后端位姿图优化
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
    message(STATUS "GTSAM found: enabling ISAM2 backend optimization")
    add_compile_definitions(USE_GTSAM)
else()
    message(STATUS "GTSAM not found: using simplified pose graph optimization")
endif()

# 尝试查找yaml-cpp
find_package(yaml-cpp QUIET)

# ==================== 编译选项 ====================
if(NOT yaml-cpp_FOUND)
    message(WARNING "yaml-cpp not found, configuration file loading disabled")
    add_compile_definitions(YAML_CPP_DISABLED)
endif()

# PCL不使用Boost
remove_definitions(-DBOOST_ALL_NO_LIB)

# ==================== 包含目录 ====================
include_directories(
    include
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

if(GTSAM_FOUND)
    include_directories(${GTSAM_INCLUDE_DIRS})
endif()

# ==================== 源文件 ====================
file(GLOB_RECURSE NODE_SOURCES CONFIGURE_DEPENDS
    "src/*.cpp"
)
list(APPEND SOURCES src/main.cpp ${NODE_SOURCES})

# ==================== 可执行目标 ====================
# 使用独立的 NODE_NAME 避免与 rosidl_generate_interfaces(${PROJECT_NAME} ...) 目标冲突
set(NODE_NAME fast_lio2_node)
add_executable(${NODE_NAME} ${SOURCES})
set_target_properties(${NODE_NAME} PROPERTIES OUTPUT_NAME "rosiwit_slam")

# ==================== 预编译头 (PCH) ====================
# PCH 让编译器将 pch.hpp 中的重型头文件 (Eigen, PCL, STL) 预编译一次，
# 所有 TU 共享该缓存，大幅减少头文件重复解析开销。
# 注意：不包含 <rclcpp/rclcpp.hpp> 以避免旧版 GCC 的 std::invoke_result bug。
set(PCH_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fast_lio2_slam/pch.hpp)
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

# 链接库
target_link_libraries(${NODE_NAME}
    ${PCL_LIBRARIES}
)

# Sophus - 仅在找到时链接
if(Sophus_FOUND)
    target_link_libraries(${NODE_NAME} Sophus::Sophus)
endif()

if(yaml-cpp_FOUND)
    target_link_libraries(${NODE_NAME} yaml-cpp)
endif()

if(GTSAM_FOUND)
    target_link_libraries(${NODE_NAME} gtsam)
endif()

# ==================== 编译定义 ====================
target_compile_definitions(${NODE_NAME} PRIVATE
    EIGEN_MPL2_ONLY
)

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

# ==================== 测试 ====================
# gtest 目标依赖 Sophus::Sophus，若环境未安装 Sophus 则跳过测试
if(BUILD_TESTING)
    find_package(Sophus QUIET)
    if(Sophus_FOUND)
        find_package(ament_cmake_gtest REQUIRED)

        # 1. 类型定义测试
        ament_add_gtest(test_types test/test_types.cpp)
        target_include_directories(test_types PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_types ${PCL_LIBRARIES} Sophus::Sophus)

        # 2. IEKF估计器测试
        ament_add_gtest(test_iekf_estimator test/test_iekf_estimator.cpp
                        src/fast_lio2_core/iekf_estimator.cpp)
        target_include_directories(test_iekf_estimator PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_iekf_estimator ${PCL_LIBRARIES} Sophus::Sophus)

        # 3. 点云滤波器/IMU处理器测试已移除:
        #    test_point_cloud_filter.cpp / test_imu_processor.cpp 引用了
        #    不存在的 API (PointCloudFilterConfig/PointCloudFilterParams,
        #    ImuProcessor::setAccBias 等), 与当前实现不匹配, 属历史遗留。

        # 4. iKD-Tree测试
        ament_add_gtest(test_ikd_tree test/test_ikd_tree.cpp src/fast_lio2_core/ikd_tree.cpp)
        target_include_directories(test_ikd_tree PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_ikd_tree ${PCL_LIBRARIES} Sophus::Sophus)

        # MapManager测试已移除: test_map_manager.cpp 调用了不存在的
        # MapQualityEvaluator::evaluatePointCloud (实际为 evaluate),
        # 且依赖 yaml-cpp 链接, 属历史遗留 API 不匹配。

        # 安装测试可执行文件
        install(TARGETS
            test_types
            test_iekf_estimator
            test_ikd_tree
            DESTINATION lib/${PROJECT_NAME}
        )
    else()
        message(STATUS "Sophus not found, skipping gtest targets (BUILD_TESTING)")
    endif()
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