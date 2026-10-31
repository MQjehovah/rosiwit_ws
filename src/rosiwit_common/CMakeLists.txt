# ============================================================================
# rosiwit_common — 通用并发基础层 (ROS 无关, 无第三方依赖)
#   独立构建/测试:  cd src/rosiwit_common && cmake -S . -B build && cmake --build build && ctest --test-dir build
#   被 ROS 包 add_subdirectory 内联复用 (与 src/rosiwit_navigation / src/rosiwit_slam 一致)
# ============================================================================
cmake_minimum_required(VERSION 3.10)
project(rosiwit_common VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(ROSIWIT_COMMON_STANDALONE ON)
else()
    set(ROSIWIT_COMMON_STANDALONE OFF)
endif()

file(GLOB_RECURSE COMMON_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(rosiwit_common ${COMMON_SOURCES})
target_include_directories(rosiwit_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rosiwit_common PRIVATE -Wall -Wextra -O2)
set_target_properties(rosiwit_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
find_package(Threads REQUIRED)
target_link_libraries(rosiwit_common PUBLIC Threads::Threads)

# ROS 接口包内联构建时必须安装，否则运行期找不到 librosiwit_common.so。
install(TARGETS rosiwit_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

if(ROSIWIT_COMMON_STANDALONE)
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
        foreach(ts ${TEST_SOURCES})
            get_filename_component(tname ${ts} NAME_WE)
            add_executable(${tname} ${ts})
            target_link_libraries(${tname} PRIVATE rosiwit_common GTest::gtest GTest::gtest_main)
            add_test(NAME ${tname} COMMAND ${tname})
        endforeach()
        message(STATUS "rosiwit_common standalone: tests enabled (${TEST_SOURCES})")
    else()
        message(STATUS "GTest not found — rosiwit_common tests disabled (可在 ROS 包内用 catkin gtest 跑)")
    endif()
endif()
