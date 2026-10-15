# ============================================================================
# rosiwit_coverage_planner — 覆盖路径规划算法核心 (ROS 无关)
#   独立开发/测试/验证:  cd src/rosiwit_coverage_planner && cmake -S . -B build && cmake --build build && ctest --test-dir build
#   ROS 接口包 (rosiwit_coverage_planner_ros2) 以 add_subdirectory 内联复用本库。
#   本目录含 COLCON_IGNORE, 不被 colcon 单独构建 (由接口包内联构建)。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(rosiwit_coverage_planner VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(ROSIWIT_COVERAGE_STANDALONE ON)
else()
    set(ROSIWIT_COVERAGE_STANDALONE OFF)
endif()

# ---- 依赖 ----
find_package(OpenCV REQUIRED)

# ---- 源文件 ----
file(GLOB_RECURSE CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# ---- 核心库 ----
add_library(rosiwit_coverage_planner ${CORE_SOURCES})
set_target_properties(rosiwit_coverage_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rosiwit_coverage_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rosiwit_coverage_planner PUBLIC ${OpenCV_LIBS})
target_compile_options(rosiwit_coverage_planner PRIVATE -pipe)

# ---- 安装 ----
install(TARGETS rosiwit_coverage_planner
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(ROSIWIT_COVERAGE_STANDALONE)
    # ---- 测试 (算法独立验证) ----
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
        foreach(ts ${TEST_SOURCES})
            get_filename_component(tname ${ts} NAME_WE)
            add_executable(${tname} ${ts})
            target_link_libraries(${tname} PRIVATE rosiwit_coverage_planner
                GTest::gtest GTest::gtest_main)
            add_test(NAME ${tname} COMMAND ${tname})
        endforeach()
        message(STATUS "rosiwit_coverage_planner standalone: ${ROSIWIT_COVERAGE_STANDALONE}, tests enabled")
    else()
        message(STATUS "GTest not found — core tests disabled")
    endif()
endif()
