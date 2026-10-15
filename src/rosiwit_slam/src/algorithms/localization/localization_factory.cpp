// src/algorithms/localization/localization_factory.cpp
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/matcher/gicp_matcher.h"
#include "algorithms/localization/matcher/pp_icp_matcher.h"
#include "algorithms/localization/matcher/p2p_icp_matcher.h"
#include "algorithms/localization/matcher/vgicp_matcher.h"
#include "algorithms/localization/matcher/ndt2d_matcher.h"
#include "algorithms/localization/local_map/global_map_provider.h"
#include "algorithms/localization/local_map/radius_submap_provider.h"
#include "algorithms/localization/local_map/keyframe_submap_provider.h"
#include "algorithms/localization/motion_prior/constant_velocity_prior.h"
#include "algorithms/localization/motion_prior/imu_motion_prior.h"
#include "algorithms/localization/global/scan_context_global_localizer.h"
#include "algorithms/localization/global/bbs_global_localizer.h"
#include "algorithms/localization/global/fpfh_ransac_global_localizer.h"
#include "algorithms/localization/monitor/threshold_monitor.h"
#include "algorithms/localization/monitor/windowed_monitor.h"
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"

namespace rosiwit_slam {

std::unique_ptr<IScanMatcher> LocalizationFactory::createMatcher(const std::string& name) {
    if (name == "gicp")    return std::make_unique<GicpMatcher>();
    if (name == "pp_icp")  return std::make_unique<PpIcpMatcher>();
    if (name == "p2p_icp") return std::make_unique<P2pIcpMatcher>();
    if (name == "vgicp")   return std::make_unique<VgicpMatcher>();
    if (name == "ndt2d")   return std::make_unique<Ndt2dMatcher>();
    return nullptr;
}

std::unique_ptr<ILocalMapProvider> LocalizationFactory::createLocalMapProvider(const std::string& name) {
    if (name == "global")          return std::make_unique<GlobalMapProvider>();
    if (name == "radius_submap")   return std::make_unique<RadiusSubmapProvider>();
    if (name == "keyframe_submap") return std::make_unique<KeyframeSubmapProvider>();
    return nullptr;
}

std::unique_ptr<IMotionPrior> LocalizationFactory::createMotionPrior(const std::string& name) {
    if (name == "constant_velocity") return std::make_unique<ConstantVelocityPrior>();
    if (name == "imu")               return std::make_unique<ImuMotionPrior>();
    return nullptr;
}

std::unique_ptr<IGlobalLocalizer> LocalizationFactory::createGlobalLocalizer(const std::string& name) {
    if (name == "none")         return std::make_unique<NoGlobalLocalizer>();
    if (name == "scan_context") return std::make_unique<ScanContextGlobalLocalizer>();
    if (name == "bbs")          return std::make_unique<BbsGlobalLocalizer>();
    if (name == "fpfh_ransac")  return std::make_unique<FpfhRansacGlobalLocalizer>();
    return nullptr;
}

std::unique_ptr<ILocalizationMonitor> LocalizationFactory::createMonitor(const std::string& name) {
    if (name == "threshold") return std::make_unique<ThresholdMonitor>();
    if (name == "windowed")  return std::make_unique<WindowedMonitor>();
    return nullptr;
}

std::unique_ptr<IMapMaintainer> LocalizationFactory::createMapMaintainer(const std::string& name) {
    if (name == "overlay") return std::make_unique<OverlayMapMaintainer>();
    return nullptr;
}

std::vector<std::string> LocalizationFactory::listMatchers()          { return {"gicp", "pp_icp", "p2p_icp", "vgicp", "ndt2d"}; }
std::vector<std::string> LocalizationFactory::listLocalMapProviders() { return {"global", "radius_submap", "keyframe_submap"}; }
std::vector<std::string> LocalizationFactory::listMotionPriors()      { return {"constant_velocity", "imu"}; }
std::vector<std::string> LocalizationFactory::listGlobalLocalizers()  { return {"none", "scan_context", "bbs", "fpfh_ransac"}; }
std::vector<std::string> LocalizationFactory::listMonitors()          { return {"threshold", "windowed"}; }
std::vector<std::string> LocalizationFactory::listMapMaintainers()    { return {"overlay"}; }

} // namespace rosiwit_slam
