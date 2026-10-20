#pragma once
// 内部完整地图文档（核心层）：
//   - 点云空间索引用现有 IMapStore / VoxelMapStore
//   - 关键帧、栅格、元数据、逐帧观测一起组成完整地图
// 该结构不包含任何 xzmap/ROS1 私有格式信息。
#include <memory>
#include <string>
#include <vector>

#include "slam_core/i_grid_map_generator.h"
#include "slam_core/i_map_store.h"
#include "slam_core/pipeline_types.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

struct MapMeta {
    std::string map_id;
    std::string frame_id = "map";
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    uint64_t revision = 0;
};

struct MapData {
    MapMeta meta;
    std::shared_ptr<IMapStore> points;       // VoxelMapStore / PcdMapStore
    std::vector<KeyFrame> keyframes;         // 定位 keyframe_submap 所需
    GridInfo grid;                           // 2D 占据栅格，可为空
    std::vector<Observation> observations;   // ray casting / 建图证据，可为空
};

}  // namespace rosiwit_slam
