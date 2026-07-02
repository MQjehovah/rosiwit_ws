cmake_minimum_required(VERSION 3.8)
project(rosiwit_slam VERSION 1.0.0 LANGUAGES C CXX)

# ==================== 编译设置 ====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# ==================== ccache (可选, 缓存未变更的 TU) ====================
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
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

# Sophus - 尝试查找，找不到则使用header-only版本
find_package(Sophus QUIET)
if(NOT Sophus_FOUND)
    message(STATUS "Sophus not found, using header-only version from Eigen")
    add_compile_definitions(SOPHUS_USE_EIGEN)
endif()

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

# 预编译头: 稳定的重型头文件一次编译, 多个 TU 共享
# 注意: 加 PCH 后 std::invoke_result 在某些 TU 报 "template argument 1 is invalid",
# 可能是 PCH 与 rclcpp 模板交互有 bug。先禁用 PCH, 主优化靠 ccache + 多 TU。
# target_precompile_headers(${NODE_NAME} PRIVATE
#     <rclcpp/rclcpp.hpp>
#     <sensor_msgs/msg/point_cloud2.hpp>
#     <sensor_msgs/msg/imu.hpp>
#     <nav_msgs/msg/odometry.hpp>
#     <pcl/point_cloud.h>
#     <pcl/point_types.h>
#     <pcl/filters/voxel_grid.h>
#     <Eigen/Dense>
#     <Eigen/Geometry>
#     <vector>
#     <memory>
#     <mutex>
# )

ament_target_dependencies(${NODE_NAME}
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    std_srvs
    pcl_conversions
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
    target_link_libraries(${NODE_NAME} ${GTSAM_LIBRARIES})
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
        ament_add_gtest(test_iekf_estimator test/test_iekf_estimator.cpp)
        target_include_directories(test_iekf_estimator PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_iekf_estimator ${PCL_LIBRARIES} Sophus::Sophus)

        # 3. 点云滤波器测试
        ament_add_gtest(test_point_cloud_filter test/test_point_cloud_filter.cpp)
        target_include_directories(test_point_cloud_filter PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_point_cloud_filter ${PCL_LIBRARIES} Sophus::Sophus)

        # 4. IMU处理器测试
        ament_add_gtest(test_imu_processor test/test_imu_processor.cpp)
        target_include_directories(test_imu_processor PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_imu_processor ${PCL_LIBRARIES} Sophus::Sophus)

        # 5. iKD-Tree测试
        ament_add_gtest(test_ikd_tree test/test_ikd_tree.cpp)
        target_include_directories(test_ikd_tree PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_ikd_tree ${PCL_LIBRARIES} Sophus::Sophus)

        # 6. MapManager测试
        ament_add_gtest(test_map_manager test/map_manager/test_map_manager.cpp)
        target_include_directories(test_map_manager PRIVATE
            include
            ${EIGEN3_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${Sophus_INCLUDE_DIRS}
        )
        target_link_libraries(test_map_manager ${PCL_LIBRARIES} Sophus::Sophus)

        # 安装测试可执行文件
        install(TARGETS
            test_types
            test_iekf_estimator
            test_point_cloud_filter
            test_imu_processor
            test_ikd_tree
            test_map_manager
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