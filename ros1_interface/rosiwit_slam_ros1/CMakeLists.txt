cmake_minimum_required(VERSION 3.0.2)
project(rosiwit_slam_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
    tf tf2 tf2_ros pcl_conversions pcl_ros visualization_msgs
    message_generation)

add_service_files(FILES SaveMap.srv LoadMap.srv SaveGridMap.srv SetSlamMode.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration segmentation sample_consensus features)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres QUIET)
find_package(GTSAM QUIET)
find_package(Sophus QUIET)
find_package(octomap QUIET)
find_package(yaml-cpp QUIET)
find_package(OpenMP QUIET)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
                   tf tf2 tf2_ros pcl_conversions visualization_msgs message_runtime
)

# ---- 复用 ROS 无关的算法核心 (roswit_slam 包) ----
set(SLAM_PKG "${CMAKE_CURRENT_SOURCE_DIR}/../../src/rosiwit_slam")

include_directories(
    include
    ${SLAM_PKG}/include
    ${SLAM_PKG}/src
    ${SLAM_PKG}/src/algorithms
    ${SLAM_PKG}/src/algorithms/fast_lio2
    ${SLAM_PKG}/src/algorithms/fast_lio2/map_builder
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 核心源码 (排除 ROS2 专有: ros_interface/ + main.cpp), 与 ROS2 共享同一份代码
file(GLOB_RECURSE CORE_SOURCES "${SLAM_PKG}/src/*.cpp")
list(FILTER CORE_SOURCES EXCLUDE REGEX "/ros_interface/")
list(FILTER CORE_SOURCES EXCLUDE REGEX "/main\\.cpp$")

add_executable(rosiwit_slam_ros1_node
    src/rosiwit_slam_ros1_node.cpp
    ${CORE_SOURCES})
add_dependencies(rosiwit_slam_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- 编译定义 (与 ROS2 CMake 对齐) ----
target_compile_definitions(rosiwit_slam_ros1_node PRIVATE EIGEN_MPL2_ONLY -pipe -pthread)
if(GTSAM_FOUND)
    target_compile_definitions(rosiwit_slam_ros1_node PRIVATE USE_GTSAM)
    target_include_directories(rosiwit_slam_ros1_node PRIVATE ${GTSAM_INCLUDE_DIRS})
endif()
if(octomap_FOUND)
    target_compile_definitions(rosiwit_slam_ros1_node PRIVATE HAVE_OCTOMAP)
    target_include_directories(rosiwit_slam_ros1_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
endif()
if(Sophus_FOUND)
    target_compile_definitions(rosiwit_slam_ros1_node PRIVATE SOPHUS_USE_BASIC_LOGGING)
    target_include_directories(rosiwit_slam_ros1_node PRIVATE ${Sophus_INCLUDE_DIRS})
endif()
if(NOT yaml-cpp_FOUND)
    target_compile_definitions(rosiwit_slam_ros1_node PRIVATE YAML_CPP_DISABLED)
endif()
if(OpenMP_FOUND)
    target_compile_definitions(rosiwit_slam_ros1_node PRIVATE MP_EN MP_PROC_NUM=2)
    target_link_libraries(rosiwit_slam_ros1_node OpenMP::OpenMP_CXX)
endif()

target_link_libraries(rosiwit_slam_ros1_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
if(yaml-cpp_FOUND)
    target_include_directories(rosiwit_slam_ros1_node PRIVATE ${yaml-cpp_INCLUDE_DIRS})
    target_link_libraries(rosiwit_slam_ros1_node yaml-cpp)
endif()
if(Ceres_FOUND)
    target_include_directories(rosiwit_slam_ros1_node PRIVATE ${CERES_INCLUDE_DIRS})
    target_link_libraries(rosiwit_slam_ros1_node ${CERES_LIBRARIES})
endif()
if(GTSAM_FOUND)
    target_link_libraries(rosiwit_slam_ros1_node gtsam)
endif()
if(octomap_FOUND)
    target_link_libraries(rosiwit_slam_ros1_node ${OCTOMAP_LIBRARIES})
endif()
if(Sophus_FOUND)
    target_link_libraries(rosiwit_slam_ros1_node Sophus::Sophus)
endif()

install(TARGETS rosiwit_slam_ros1_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
