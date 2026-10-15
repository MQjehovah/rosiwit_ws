# ============================================================================
# rosiwit_slam — 算法核心 (ROS 无关)
#   独立开发/测试/验证:  cd src/rosiwit_slam && cmake -S . -B build && cmake --build build && ctest --test-dir build
#   ROS 接口包(rosiwit_slam_ros1 / rosiwit_slam_ros2) 以 add_subdirectory 内联复用本库, 保持薄接口且等价。
#   本目录含 COLCON_IGNORE, 不被 colcon 单独构建 (由接口包内联构建)。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
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
    target_link_libraries(rosiwit_slam PUBLIC Sophus::Sophus)
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
    target_link_libraries(rosiwit_slam PUBLIC gtsam)
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
    foreach(tool grid_map_tool grid_map_post localization_eval map_cleanup traj_eval)
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
