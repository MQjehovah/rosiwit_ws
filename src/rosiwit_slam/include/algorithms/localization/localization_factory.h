// include/algorithms/localization/localization_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_scan_matcher.h"
#include "slam_core/i_local_map_provider.h"
#include "slam_core/i_motion_prior.h"
#include "slam_core/i_global_localizer.h"
#include "slam_core/i_localization_monitor.h"
#include "slam_core/i_map_maintainer.h"
#include "slam_core/i_pose_fusion.h"

namespace rosiwit_slam {

/// 定位各阶段策略工厂: 与建图侧 GridMapFactory 对应, 按名创建可插拔算法.
class LocalizationFactory {
public:
    static std::unique_ptr<IScanMatcher>         createMatcher(const std::string& name);
    static std::unique_ptr<ILocalMapProvider>    createLocalMapProvider(const std::string& name);
    static std::unique_ptr<IMotionPrior>         createMotionPrior(const std::string& name);
    static std::unique_ptr<IGlobalLocalizer>     createGlobalLocalizer(const std::string& name);
    static std::unique_ptr<ILocalizationMonitor> createMonitor(const std::string& name);
    static std::unique_ptr<IMapMaintainer>       createMapMaintainer(const std::string& name);
    static std::unique_ptr<IPoseFusion>          createPoseFusion(const std::string& name);

    static std::vector<std::string> listMatchers();
    static std::vector<std::string> listLocalMapProviders();
    static std::vector<std::string> listMotionPriors();
    static std::vector<std::string> listGlobalLocalizers();
    static std::vector<std::string> listMonitors();
    static std::vector<std::string> listMapMaintainers();
    static std::vector<std::string> listPoseFusions();
};

} // namespace rosiwit_slam
