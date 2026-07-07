// include/slam_core/slam_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/i_frontend.h"
#include "slam_core/i_backend.h"
#include "slam_core/i_localization.h"
#include "slam_core/i_map_manager.h"
#include "slam_core/i_loop_closure.h"

namespace rosiwit_slam {

class SlamFactory {
public:
    static std::unique_ptr<ISlamAlgorithm> create(const std::string& name);
    static std::vector<std::string> listNames();
    static std::unique_ptr<IFrontend> createFrontend(const std::string& name);
    static std::unique_ptr<IBackend> createBackend(const std::string& name);
    static std::unique_ptr<ILocalization> createLocalization(const std::string& name);
    static std::unique_ptr<IMapManager> createMapManager(const std::string& name);
    static std::unique_ptr<ILoopClosure> createLoopClosure(const std::string& name);
};

} // namespace rosiwit_slam
