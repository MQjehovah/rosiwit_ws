# ============================================================================
# rosiwit_navigation_ros1 — ROS1 薄接口包 (与 rosiwit_navigation_ros2 等价)
#   仅包含 ROS1 节点; 算法核心由 ../../../src/rosiwit_navigation 内联构建 (源码共享)。
# ============================================================================
cmake_minimum_required(VERSION 3.0.2)
project(rosiwit_navigation_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp nav_msgs geometry_msgs sensor_msgs std_msgs
    tf2 tf2_ros tf2_geometry_msgs std_srvs visualization_msgs
    actionlib actionlib_msgs xzrobot_ros_dependency)

catkin_package(
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs std_msgs
                   tf2 tf2_ros tf2_geometry_msgs std_srvs visualization_msgs
                   actionlib actionlib_msgs xzrobot_ros_dependency)

# ==================== 算法核心 (源码共享, 独立开发, 此处内联构建) ====================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_navigation
                 ${CMAKE_CURRENT_BINARY_DIR}/rosiwit_navigation_core)

# ==================== ROS1 节点 (薄) ====================
# 接口层源文件:
#   movebase_protocol.cpp  — robot_movebase action 报文解析 (纯 C++, 可单测)
#   movebase_interface.cpp — action 服务端 + 旧话题适配 (ROS 侧)
# 行为状态机不在这里, 在算法核心 core::ActionTask。
add_executable(rosiwit_navigation_ros1_node
    src/rosiwit_navigation_ros1_node.cpp
    src/movebase_protocol.cpp
    src/movebase_interface.cpp)
# 源码内联的算法核心头文件必须优先于 /opt/ros 里可能存在的旧安装头文件,
# 否则 add_subdirectory 用的是新库, 头文件却是旧的.
target_include_directories(rosiwit_navigation_ros1_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_navigation/include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(rosiwit_navigation_ros1_node PRIVATE -pipe -pthread)
target_link_libraries(rosiwit_navigation_ros1_node
    ${catkin_LIBRARIES}
    rosiwit_navigation)                # 核心库 (规划器/控制器/路径平滑/任务状态机)
# movebase_interface.h 直接 include 了 xzrobot_ros_dependency 生成的头文件
# (action: NaviCmdActionAction.h). 没有下面这行时, 并行构建 (-j) 不保证
# "先由 xzrobot_ros_dependency 生成头文件、再编译本节点", 干净构建会偶发
# fatal error: xzrobot_ros_dependency/NaviCmdActionAction.h: No such file or directory
add_dependencies(rosiwit_navigation_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS rosiwit_navigation_ros1_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# ==================== 接口协议单测 (纯 C++, 无需 ROS 运行时) ====================
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_movebase_protocol
        test/test_movebase_protocol.cpp
        src/movebase_protocol.cpp)
    if(TARGET test_movebase_protocol)
        target_include_directories(test_movebase_protocol PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_navigation/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(test_movebase_protocol rosiwit_navigation)
    endif()
endif()
