# ============================================================================
# rosiwit_navigation — 导航算法核心 (ROS 无关)
#   独立开发/测试/验证:  cd src/rosiwit_navigation && cmake -S . -B build && cmake --build build && ctest --test-dir build
#   ROS 接口包 (rosiwit_navigation_ros1 / rosiwit_navigation_ros2) 以
#   add_subdirectory 内联复用本库, 保持薄接口且等价。
#   本目录含 COLCON_IGNORE, 不被 colcon 单独构建 (由接口包内联构建)。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(rosiwit_navigation VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 独立构建(顶层)时为 ON: 额外构建核心单元测试; 被 ROS 包 add_subdirectory 时为 OFF
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(ROSIWIT_NAVIGATION_STANDALONE ON)
else()
    set(ROSIWIT_NAVIGATION_STANDALONE OFF)
endif()

# ---- 源文件 ----
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# ---- 核心库 ----
add_library(rosiwit_navigation ${CORE_SOURCES})
set_target_properties(rosiwit_navigation PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rosiwit_navigation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/global_planner
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/local_planner
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/support)
target_compile_options(rosiwit_navigation PRIVATE -pipe)

# ---- 安装 (供 add_subdirectory 之外的可选安装) ----
install(TARGETS rosiwit_navigation
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(ROSIWIT_NAVIGATION_STANDALONE)
    # ---- 测试 (算法独立验证) ----
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
        foreach(ts ${TEST_SOURCES})
            get_filename_component(tname ${ts} NAME_WE)
            add_executable(${tname} ${ts})
            target_link_libraries(${tname} PRIVATE rosiwit_navigation GTest::gtest GTest::gtest_main)
            add_test(NAME ${tname} COMMAND ${tname})
        endforeach()
        message(STATUS "rosiwit_navigation standalone: ${ROSIWIT_NAVIGATION_STANDALONE}, tests enabled")
    else()
        message(STATUS "GTest not found — core tests disabled")
    endif()
endif()
