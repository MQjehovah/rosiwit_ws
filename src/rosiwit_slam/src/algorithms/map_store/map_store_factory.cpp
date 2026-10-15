// src/algorithms/map_store/map_store_factory.cpp
#include "algorithms/map_store/map_store_factory.h"
#include "algorithms/map_store/pcd_map_store.h"
#include "algorithms/map_store/voxel_map_store.h"

namespace rosiwit_slam {

std::unique_ptr<IMapStore> MapStoreFactory::create(const std::string& name, double leaf) {
    if (name == "pcd")   return std::make_unique<PcdMapStore>(leaf);
    if (name == "voxel") return std::make_unique<VoxelMapStore>(leaf);
    return nullptr;
}

std::vector<std::string> MapStoreFactory::listNames() { return {"pcd", "voxel"}; }

} // namespace rosiwit_slam
