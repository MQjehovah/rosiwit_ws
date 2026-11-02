# 3D 语义目标核心（ROS 无关）
#
# 独立构建/测试:
#   cd src/rosiwit_perception/object_semantics
#   cmake -S . -B build && cmake --build build -j8 && ctest --test-dir build
#
# ROS1/ROS2 薄节点通过 add_subdirectory 复用本库, 不反向依赖 ROS。
cmake_minimum_required(VERSION 3.10)
project(rosiwit_object_semantics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(OBJECT_SEMANTICS_STANDALONE ON)
else()
    set(OBJECT_SEMANTICS_STANDALONE OFF)
endif()

find_package(Eigen3 REQUIRED)

add_library(object_semantics
    src/depth_associator.cpp
    src/multi_object_tracker.cpp
    src/detector_postprocess.cpp)
set_target_properties(object_semantics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(object_semantics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(TARGET Eigen3::Eigen)
    target_link_libraries(object_semantics PUBLIC Eigen3::Eigen)
else()
    # 老版本 Eigen (如 ROS1 melodic / Eigen 3.3.4) 没有 imported target
    target_include_directories(object_semantics PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

install(TARGETS object_semantics ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(OBJECT_SEMANTICS_STANDALONE)
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
        foreach(ts ${TEST_SOURCES})
            get_filename_component(tname ${ts} NAME_WE)
            add_executable(${tname} ${ts})
            target_link_libraries(${tname} PRIVATE object_semantics GTest::gtest GTest::gtest_main)
            add_test(NAME ${tname} COMMAND ${tname})
        endforeach()
        message(STATUS "object_semantics standalone: tests enabled (${TEST_SOURCES})")
    else()
        message(STATUS "GTest not found - core tests disabled")
    endif()
endif()
