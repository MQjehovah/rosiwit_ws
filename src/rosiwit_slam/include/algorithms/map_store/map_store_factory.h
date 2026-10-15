// include/algorithms/map_store/map_store_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 地图存储工厂: 按名创建 IMapStore (与算法正交, 可配置切换).
class MapStoreFactory {
public:
    static std::unique_ptr<IMapStore> create(const std::string& name, double leaf = 0.3);
    static std::vector<std::string> listNames();
};

} // namespace rosiwit_slam
