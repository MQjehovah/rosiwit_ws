# 3D 语义目标 ROS1 薄节点
#
# 算法核心在 src/rosiwit_perception/object_semantics（ROS 无关，独立单测）；
# 本包只做: 订阅图像/相机信息/TF/odom -> 检测(可选外部输入) -> 深度关联 -> 跟踪 -> 发布。
#
# 类别由 classes 参数给出（默认 person），新增类别只改配置。
#
# 独立构建核心单测:
#   cd src/rosiwit_perception/object_semantics && cmake -S . -B build && cmake --build build && ctest --test-dir build
cmake_minimum_required(VERSION 3.0.2)
project(rosiwit_perception_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs
  tf cv_bridge message_generation)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# RKNN 是板端专有运行时: 有 rknn_api.h 时才编进去, 否则节点启动时明确报错退出
find_path(RKNN_INCLUDE_DIR rknn_api.h PATHS /usr/include /usr/local/include)
find_library(RKNN_LIBRARY rknnrt PATHS /usr/lib /usr/local/lib)

add_message_files(FILES
  SemanticObject.msg
  SemanticObjectArray.msg
  ObjectDetection2D.msg
  ObjectDetection2DArray.msg
  SimTarget.msg
  SimGroundTruth.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs tf
                 cv_bridge
  DEPENDS Eigen3)

# ---- 算法核心（源码共享） ----
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rosiwit_perception/object_semantics
                 ${CMAKE_CURRENT_BINARY_DIR}/object_semantics_core)

add_executable(object_semantics_node
  src/object_semantics_node.cpp)
target_include_directories(object_semantics_node PRIVATE
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(object_semantics_node
  object_semantics ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(object_semantics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(RKNN_INCLUDE_DIR AND RKNN_LIBRARY)
  target_compile_definitions(object_semantics_node PRIVATE HAVE_RKNN=1)
  target_include_directories(object_semantics_node PRIVATE ${RKNN_INCLUDE_DIR})
  target_link_libraries(object_semantics_node ${RKNN_LIBRARY})
  message(STATUS "object_semantics: RKNN found -> detector enabled")
else()
  message(STATUS "object_semantics: RKNN not found -> only external detections / dry-run")
endif()

# ---- 离线闭环仿真（场景真值 -> 节点 -> 评测），不参与产品链路 ----
add_executable(sim_scene_node sim/sim_scene_node.cpp)
target_include_directories(sim_scene_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(sim_scene_node ${catkin_LIBRARIES})
add_dependencies(sim_scene_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sim_eval_node sim/sim_eval_node.cpp)
target_include_directories(sim_eval_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(sim_eval_node ${catkin_LIBRARIES})
add_dependencies(sim_eval_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS object_semantics_node sim_scene_node sim_eval_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/run_sim_test.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
