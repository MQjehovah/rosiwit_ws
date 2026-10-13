// include/algorithms/grid_map/grid_map_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_grid_map_generator.h"

namespace rosiwit_slam {

/// 栅格地图生成策略工厂: 按名称创建 IGridMapGenerator.
class GridMapFactory {
public:
    static std::unique_ptr<IGridMapGenerator> create(const std::string& name);
    static std::vector<std::string> listNames();
};

} // namespace rosiwit_slam
