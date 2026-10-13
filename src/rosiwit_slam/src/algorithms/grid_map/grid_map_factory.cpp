// src/algorithms/grid_map/grid_map_factory.cpp
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/height_projection_grid_map.h"
#include "algorithms/grid_map/ray_casting_grid_map.h"

namespace rosiwit_slam {

std::unique_ptr<IGridMapGenerator> GridMapFactory::create(const std::string& name) {
    if (name == "height_projection") return std::make_unique<HeightProjectionGridMap>();
    if (name == "ray_casting")       return std::make_unique<RayCastingGridMap>();
    return nullptr;
}

std::vector<std::string> GridMapFactory::listNames() {
    return { "height_projection", "ray_casting" };
}

} // namespace rosiwit_slam
