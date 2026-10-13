// include/algorithms/grid_map/grid_map_io.h
#pragma once
#include <string>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 将栅格地图导出为 PGM + YAML (兼容 ROS map_server)
bool saveGridMapPGM(const GridInfo& grid,
                    const std::string& pgm_path,
                    const std::string& yaml_path,
                    const std::string& frame_id = "map");

/// 从 PGM + YAML 载入栅格地图
bool loadGridMapPGM(const std::string& pgm_path,
                    const std::string& yaml_path,
                    GridInfo& grid);

} // namespace rosiwit_slam
