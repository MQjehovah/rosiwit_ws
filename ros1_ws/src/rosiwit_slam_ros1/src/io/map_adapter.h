#pragma once
// ROS1 私有地图格式 (xzmap) <-> 核心中立 MapData 的适配层。
// 核心不再读取/写入 xzmap。
#include <string>
#include "slam_core/map_data.h"

namespace rosiwit_slam_ros1 {
namespace map_adapter {

/// path 可以是 <dir>、<dir>/map 或 <dir>/map.xzmap。
bool loadMapData(const std::string& path, rosiwit_slam::MapData& out);
bool saveMapData(const rosiwit_slam::MapData& data, const std::string& path);

}  // namespace map_adapter
}  // namespace rosiwit_slam_ros1
