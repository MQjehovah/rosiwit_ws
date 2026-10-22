# ============================================================================
# rosiwit_slam — 算法核心 (ROS 无关)
#   独立开发/测试/验证:  cd src/rosiwit_slam && cmake -S . -B build && cmake --build build && ctest --test-dir build
#   ROS 接口包(rosiwit_slam_ros1 / rosiwit_slam_ros2) 以 add_subdirectory 内联复用本库, 保持薄接口且等价。
#   本目录含 COLCON_IGNORE, 不被 colcon 单独构建 (由接口包内联构建)。
# ============================================================================
# 3.10 而不是 3.16: 本仓库的 ROS1 目标机 (Ubuntu 18.04 / melodic, 如机器人 192.168.51.19)
# 自带 cmake 只有 3.10.2, 而核心并没有用到 3.10 之后才有的 CMake 功能 —— 已在实机上
# 把本值降到 3.10 后全量编译通过 (2026-10-16, aarch64)。3.16 那个限制会让目标机必须
# 自行升级 cmake (实测 apt/github/cmake.org 在机器人上都走不通), 纯属无谓门槛。
cmake_minimum_required(VERSION 3.10)
project(rosiwit_slam VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 独立构建(顶层)时为 ON: 额外构建测试与离线工具; 被 ROS 包 add_subdirectory 时为 OFF
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(ROSIWIT_SLAM_STANDALONE ON)
else()
    set(ROSIWIT_SLAM_STANDALONE OFF)
endif()

# ---- 依赖 ----
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration segmentation sample_consensus features)
find_package(OpenCV REQUIRED)
find_package(Sophus QUIET)
if(NOT Sophus_FOUND)
    # ROS1 Melodic ships Sophus as headers only (no SophusConfig.cmake).
    find_path(SOPHUS_INCLUDE_DIR sophus/so3.hpp
        HINTS /opt/ros/$ENV{ROS_DISTRO}/include /usr/local/include /usr/include)
    if(SOPHUS_INCLUDE_DIR)
        set(Sophus_FOUND TRUE)
        set(Sophus_INCLUDE_DIRS ${SOPHUS_INCLUDE_DIR})
    endif()
endif()
find_package(Ceres QUIET)
find_package(GTSAM QUIET)
find_package(octomap QUIET)
find_package(yaml-cpp QUIET)
find_package(OpenMP QUIET)

# ---- 源文件 ----
file(GLOB_RECURSE CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
if(NOT octomap_FOUND)
    list(FILTER CORE_SOURCES EXCLUDE REGEX "octomap_grid_map\\.cpp$")
endif()

# ---- 核心库 ----
add_library(rosiwit_slam ${CORE_SOURCES})
target_include_directories(rosiwit_slam PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/fast_lio2
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/fast_lio2/map_builder
    ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_definitions(rosiwit_slam PUBLIC EIGEN_MPL2_ONLY)
target_compile_options(rosiwit_slam PRIVATE -pipe -pthread)
target_link_libraries(rosiwit_slam PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS})

if(Sophus_FOUND)
    target_compile_definitions(rosiwit_slam PRIVATE SOPHUS_USE_BASIC_LOGGING)
    target_include_directories(rosiwit_slam PRIVATE ${Sophus_INCLUDE_DIRS})
    if(TARGET Sophus::Sophus)
        target_link_libraries(rosiwit_slam PUBLIC Sophus::Sophus)
    endif()
endif()
if(yaml-cpp_FOUND)
    target_include_directories(rosiwit_slam PRIVATE ${yaml-cpp_INCLUDE_DIRS})
    target_link_libraries(rosiwit_slam PUBLIC yaml-cpp)
else()
    target_compile_definitions(rosiwit_slam PRIVATE YAML_CPP_DISABLED)
endif()
if(Ceres_FOUND)
    target_include_directories(rosiwit_slam PRIVATE ${CERES_INCLUDE_DIRS})
    target_link_libraries(rosiwit_slam PUBLIC Ceres::ceres)
endif()
if(GTSAM_FOUND)
    target_compile_definitions(rosiwit_slam PRIVATE USE_GTSAM)
    target_include_directories(rosiwit_slam PRIVATE ${GTSAM_INCLUDE_DIRS})
    find_library(GTSAM_UNSTABLE_LIBRARY NAMES gtsam_unstable
        HINTS "${GTSAM_INCLUDE_DIR}/../lib/${CMAKE_LIBRARY_ARCHITECTURE}" "${GTSAM_INCLUDE_DIR}/../lib")
    if(GTSAM_UNSTABLE_LIBRARY)
        target_link_libraries(rosiwit_slam PUBLIC gtsam ${GTSAM_UNSTABLE_LIBRARY})
    else()
        target_link_libraries(rosiwit_slam PUBLIC gtsam)
    endif()
endif()
if(octomap_FOUND)
    target_compile_definitions(rosiwit_slam PRIVATE HAVE_OCTOMAP)
    target_include_directories(rosiwit_slam PRIVATE ${OCTOMAP_INCLUDE_DIRS})
    target_link_libraries(rosiwit_slam PUBLIC ${OCTOMAP_LIBRARIES})
endif()
if(OpenMP_FOUND)
    target_compile_definitions(rosiwit_slam PRIVATE MP_EN MP_PROC_NUM=2)
    target_link_libraries(rosiwit_slam PUBLIC OpenMP::OpenMP_CXX)
endif()

# ---- 安装 (供 add_subdirectory 之外的可选安装; ROS 包用 add_subdirectory 复用, 不依赖导出) ----
install(TARGETS rosiwit_slam
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(ROSIWIT_SLAM_STANDALONE)
    # ---- 离线工具 ----
    foreach(tool grid_map_tool grid_map_post localization_eval traj_eval)
        add_executable(${tool} tools/${tool}.cpp)
        target_link_libraries(${tool} PRIVATE rosiwit_slam)
    endforeach()

    # ---- 测试 (算法独立验证) ----
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
        foreach(ts ${TEST_SOURCES})
            get_filename_component(tname ${ts} NAME_WE)
            add_executable(${tname} ${ts})
            target_link_libraries(${tname} PRIVATE rosiwit_slam GTest::gtest GTest::gtest_main)
            add_test(NAME ${tname} COMMAND ${tname})
        endforeach()
        message(STATUS "rosiwit_slam standalone: ${ROSIWIT_SLAM_STANDALONE}, tests enabled")
    else()
        message(STATUS "GTest not found — core tests disabled")
    endif()
endif()
