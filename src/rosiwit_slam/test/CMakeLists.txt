cmake_minimum_required(VERSION 3.8)
project(fast_lio2_slam_standalone_test VERSION 1.0.1 LANGUAGES C CXX)

# ==================== 编译设置 ====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# ==================== 第三方库依赖 ====================
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

# ==================== GTest ====================
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# ==================== 包含目录 ====================
include_directories(
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# ==================== Sophus Header-Only 支持 ====================
# 检查系统中是否有Sophus
find_package(Sophus QUIET)
if(Sophus_FOUND)
    include_directories(${Sophus_INCLUDE_DIRS})
    message(STATUS "Found Sophus package: ${Sophus_INCLUDE_DIRS}")
else()
    # 尝试多种可能的Sophus位置
    set(SOPHUS_SEARCH_PATHS
        /usr/include
        /usr/local/include
        /tmp/Sophus
        ${PROJECT_SOURCE_DIR}/../../Sophus
        ${PROJECT_SOURCE_DIR}/../third_party/Sophus
    )

    foreach(search_path ${SOPHUS_SEARCH_PATHS})
        if(EXISTS "${search_path}/sophus/se3.hpp")
            # 注意：include目录应该是search_path，因为代码中使用 #include <sophus/se3.hpp>
            include_directories(${search_path})
            message(STATUS "Found Sophus headers at: ${search_path}")
            set(SOPHUS_FOUND TRUE)
            break()
        endif()
    endforeach()

    if(NOT SOPHUS_FOUND)
        message(WARNING "Sophus headers not found. Download with: git clone https://github.com/strasdat/Sophus.git /tmp/Sophus")
    endif()
endif()

# ==================== 测试目标 ====================

# 测试1: MapManager类型测试（简化版，不依赖完整项目头文件）
add_executable(test_map_manager map_manager/test_types_only.cpp)
target_include_directories(test_map_manager PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_map_manager
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_map_manager Sophus::Sophus)
endif()

target_compile_definitions(test_map_manager PRIVATE EIGEN_MPL2_ONLY)

# 测试2: Types类型定义测试
add_executable(test_types test_types.cpp)
target_include_directories(test_types PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_types
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_types Sophus::Sophus)
endif()

target_compile_definitions(test_types PRIVATE EIGEN_MPL2_ONLY)

# 测试3: IMU处理器测试（修复版）
add_executable(test_imu_processor test_imu_processor_fixed.cpp)
target_include_directories(test_imu_processor PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_imu_processor
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_imu_processor Sophus::Sophus)
endif()

target_compile_definitions(test_imu_processor PRIVATE EIGEN_MPL2_ONLY)

# 测试4: 点云滤波测试
add_executable(test_point_cloud_filter test_point_cloud_filter.cpp)
target_include_directories(test_point_cloud_filter PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_point_cloud_filter
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_point_cloud_filter Sophus::Sophus)
endif()

target_compile_definitions(test_point_cloud_filter PRIVATE EIGEN_MPL2_ONLY)

# 测试5: IKD-Tree测试
add_executable(test_ikd_tree test_ikd_tree.cpp)
target_include_directories(test_ikd_tree PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_ikd_tree
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_ikd_tree Sophus::Sophus)
endif()

target_compile_definitions(test_ikd_tree PRIVATE EIGEN_MPL2_ONLY)

# 测试6: IEKF估计器测试
add_executable(test_iekf_estimator test_iekf_estimator.cpp)
target_include_directories(test_iekf_estimator PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(test_iekf_estimator
    ${PCL_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
)
if(Sophus_FOUND)
    target_link_libraries(test_iekf_estimator Sophus::Sophus)
endif()

target_compile_definitions(test_iekf_estimator PRIVATE EIGEN_MPL2_ONLY)

# ==================== 测试命令 ====================
enable_testing()

add_test(NAME test_map_manager COMMAND test_map_manager)
add_test(NAME test_types COMMAND test_types)
add_test(NAME test_imu_processor COMMAND test_imu_processor)
add_test(NAME test_point_cloud_filter COMMAND test_point_cloud_filter)
add_test(NAME test_ikd_tree COMMAND test_ikd_tree)
add_test(NAME test_iekf_estimator COMMAND test_iekf_estimator)