// src/algorithms/localization/localization_utils.cpp
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace rosiwit_slam {

CloudType::Ptr downsampleCloud(const CloudType::Ptr& in, double resolution) {
    CloudType::Ptr out(new CloudType());
    if (!in || in->empty()) return out;
    if (resolution <= 0.0) { *out = *in; return out; }
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(static_cast<float>(resolution),
                   static_cast<float>(resolution),
                   static_cast<float>(resolution));
    vg.filter(*out);
    return out;
}

CloudType::Ptr filterFinite(const CloudType::Ptr& in) {
    CloudType::Ptr out(new CloudType());
    if (!in) return out;
    out->reserve(in->size());
    for (const auto& p : in->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
            out->push_back(p);
    }
    return out;
}

double yawFromRot(const M3D& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

void poseDelta(const PoseStamped& a, const PoseStamped& b, V3D& dt, double& angle_rad) {
    const M3D dR = b.rot.transpose() * a.rot;
    dt = a.trans - b.trans;
    Eigen::AngleAxisd aa(dR);
    angle_rad = aa.angle();
}

void poseInnovation(const PoseStamped& match, const PoseStamped& pred,
                    Eigen::Matrix<double, 6, 1>& e) {
    const M3D dR = pred.rot.transpose() * match.rot;
    Eigen::AngleAxisd aa(dR);
    V3D axis = aa.axis();
    if (!axis.allFinite()) axis = V3D::Zero();
    e.head<3>() = axis * aa.angle();
    e.tail<3>() = match.trans - pred.trans;
}

void suppressBaseByOverlay(CloudType::Ptr& cloud, const CloudType::Ptr& overlay,
                           double radius) {
    if (!cloud || cloud->empty() || !overlay || overlay->empty() || radius <= 0) return;
    pcl::KdTreeFLANN<PointType> kd;
    kd.setInputCloud(overlay);
    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size());
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    const double r2 = radius * radius;
    for (const auto& p : cloud->points) {
        if (kd.nearestKSearch(p, 1, idx, sq) <= 0 || sq[0] > r2) out->push_back(p);
    }
    cloud = out;
}

} // namespace rosiwit_slam
