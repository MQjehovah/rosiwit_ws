cmake_minimum_required(VERSION 3.14)
project(rosiwit_navigation)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ROS_INCS ${rclcpp_INCLUDE_DIRS} ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
set(ROS_LIBS ${rclcpp_LIBRARIES} ${rclcpp_lifecycle_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

include_directories(include src
  src/algorithms/global_planner
  src/algorithms/local_planner
  src/algorithms/support)

add_library(nav_planners SHARED
  src/algorithms/global_planner/astar_planner.cpp
  src/algorithms/support/path_smoother.cpp
  src/algorithms/global_planner/navfn_planner.cpp)

add_library(nav_controllers SHARED
  src/algorithms/local_planner/pure_pursuit_controller.cpp
  src/algorithms/local_planner/diff_drive_controller.cpp
  src/algorithms/local_planner/dwa_controller.cpp
  src/algorithms/local_planner/teb_controller.cpp)

add_library(nav_tracking SHARED
  src/algorithms/support/velocity_limiter.cpp)

add_library(nav_navigation_lib SHARED
  src/algorithms/support/trajectory_generator.cpp)

add_library(nav_core SHARED src/nav_core/navigation_factory.cpp)
target_link_libraries(nav_core nav_planners nav_controllers)

add_library(nav_ros_interface SHARED
  src/ros_interface/navigation_node.cpp src/ros_interface/ros_utils.cpp)
target_link_libraries(nav_ros_interface
  nav_navigation_lib nav_tracking nav_core ${ROS_LIBS})
target_include_directories(nav_ros_interface PRIVATE ${ROS_INCS})

add_executable(rosiwit_navigation_node src/main.cpp)
target_link_libraries(rosiwit_navigation_node nav_ros_interface)
target_include_directories(rosiwit_navigation_node PRIVATE ${ROS_INCS})

install(TARGETS nav_planners nav_controllers nav_tracking nav_navigation_lib
  nav_core nav_ros_interface rosiwit_navigation_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
