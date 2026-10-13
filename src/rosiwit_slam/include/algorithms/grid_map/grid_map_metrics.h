// include/algorithms/grid_map/grid_map_metrics.h
#pragma once
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 2D 栅格质量指标 (用于量化"好看": 墙体连通 / 噪点 / 自由区连通 / 墙厚)
struct GridMetrics {
    long occupied = 0;
    long free = 0;
    long unknown = 0;
    int  occupied_components = 0;   // 占据连通域数量(越少越整)
    long largest_component = 0;     // 最大占据连通域面积(越大越连续)
    int  noise_components = 0;      // 面积 < noise_threshold 的占据簇(噪点)
    long noise_cells = 0;
    int  free_components = 0;       // 自由连通域数量(越少越连通)
    double mean_thickness = 0.0;    // 平均墙厚(格) = 占据格数 / 骨架格数
    double continuity = 0.0;        // largest_component / occupied
};

GridMetrics computeGridMetrics(const GridInfo& grid, int connectivity = 8,
                               int noise_threshold = 20);

} // namespace rosiwit_slam
