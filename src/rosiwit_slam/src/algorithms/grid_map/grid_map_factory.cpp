// src/algorithms/grid_map/grid_map_factory.cpp
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/height_projection_grid_map.h"
#include "algorithms/grid_map/ray_casting_grid_map.h"
#include "algorithms/grid_map/occupancy_logodds.h"
#ifdef HAVE_OCTOMAP
#include "algorithms/grid_map/octomap_grid_map.h"
#endif

namespace rosiwit_slam {

std::unique_ptr<IGridMapGenerator> GridMapFactory::create(const std::string& name) {
    if (name == "height_projection") return std::make_unique<HeightProjectionGridMap>();
    if (name == "ray_casting")       return std::make_unique<RayCastingGridMap>();
    if (name == "occupancy_logodds") return std::make_unique<LogOddsGridMap>();
#ifdef HAVE_OCTOMAP
    if (name == "octomap")           return std::make_unique<OctomapGridMap>();
#endif
    return nullptr;
}

std::vector<std::string> GridMapFactory::listNames() {
    std::vector<std::string> names = { "height_projection", "ray_casting", "occupancy_logodds" };
#ifdef HAVE_OCTOMAP
    names.push_back("octomap");
#endif
    return names;
}

} // namespace rosiwit_slam
