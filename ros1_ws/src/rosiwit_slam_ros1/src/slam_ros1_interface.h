// ============================================================================
// SlamRos1Interface —— ROS1 唯一对外接口
//
// 外部只挂这一个对象:
//   - LegacyXz3dSlamInterface: 老 xz_3d_slam 话题/服务/TF (兼容基线)
//   - RosiwitInterface:       仅保留不冲突的新输出 (lio_odom/lio_path/cloud_map/
//                             grid_map/body_cloud/world_cloud), 冲突项全部抑制
//
// 不再有 input_profile 两套接口同时对外; 老接口优先.
// ============================================================================
#pragma once

#include <memory>

#include "legacy_xz3dslam_interface.h"
#include "rosiwit_interface.h"
#include "slam_engine.h"

namespace rosiwit_slam_ros1 {

class SlamRos1Interface {
public:
    SlamRos1Interface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine)
        : legacy_(new LegacyXz3dSlamInterface(nh, pnh, engine)),
          outputs_(new RosiwitInterface(nh, pnh, engine, /*suppress_conflicts=*/true)) {}

private:
    std::unique_ptr<LegacyXz3dSlamInterface> legacy_;
    std::unique_ptr<RosiwitInterface> outputs_;
};

} // namespace rosiwit_slam_ros1
