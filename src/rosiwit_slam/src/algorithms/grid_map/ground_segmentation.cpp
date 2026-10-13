// src/algorithms/grid_map/ground_segmentation.cpp
#include "algorithms/grid_map/ground_segmentation.h"
#include <algorithm>
#include <cmath>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

namespace rosiwit_slam {

GroundModel fitGroundPercentile(const std::vector<double>& zs, const GridMapConfig& cfg) {
    GroundModel g;
    g.n = Eigen::Vector3f::UnitZ();
    if (zs.empty()) {
        g.d = 0.0f;
        g.valid = false;
        return g;
    }
    std::vector<double> s = zs;
    std::sort(s.begin(), s.end());
    size_t i = static_cast<size_t>(0.02 * (s.size() - 1));
    if (i >= s.size()) i = s.size() - 1;
    g.d = -static_cast<float>(s[i]);
    g.valid = true;
    return g;
}

GroundModel fitGround(const CloudType::Ptr& cloud, const GridMapConfig& cfg) {
    GroundModel g;
    g.n = Eigen::Vector3f::UnitZ();

    // 手动指定地面高度
    if (!std::isnan(cfg.ground_z)) {
        g.d = -static_cast<float>(cfg.ground_z);
        g.valid = true;
        return g;
    }
    if (!cloud || cloud->empty()) return g;

    // 收集 z 供分位数兜底
    std::vector<double> zs;
    zs.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) zs.push_back(p.z);
    }

    if (cfg.ground_method == "ransac" && zs.size() >= 50) {
        try {
            // 先以 z 低分位数(2%)作为地面高度先验(该值对本数据稳定取到地面),
            // 只在该先验附近的薄层内做 RANSAC, 以恢复地面倾角; 避免上视雷达选中天花板.
            GroundModel prior = fitGroundPercentile(zs, cfg);
            const double z_prior = -prior.d;                 // n=Z, d=-z
            const double slab = 0.35;                         // 薄层半厚(m)

            CloudType::Ptr low(new CloudType());
            low->reserve(cloud->size() / 4 + 1);
            for (const auto& p : cloud->points) {
                if (!std::isfinite(p.z)) continue;
                if (p.z <= z_prior + slab) low->push_back(p);
            }
            if (low->size() >= 30) {
                pcl::SACSegmentation<PointType> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(cfg.ransac_dist_thr);
                seg.setMaxIterations(cfg.ransac_max_iter > 0 ? cfg.ransac_max_iter : 200);
                seg.setInputCloud(low);
                pcl::ModelCoefficients coeffs;
                pcl::PointIndices inliers;
                seg.segment(inliers, coeffs);
                if (coeffs.values.size() == 4 && !inliers.indices.empty()) {
                    Eigen::Vector3f n(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
                    float nrm = n.norm();
                    if (nrm > 1e-6f) {
                        n /= nrm;
                        float d = coeffs.values[3] / nrm;
                        if (n.z() < 0.0f) { n = -n; d = -d; }
                        if (n.z() >= 0.8f) {
                            g.n = n;
                            g.d = d;
                            g.valid = true;
                            return g;
                        }
                    }
                }
            }
            return prior;   // 薄层内无有效平面 → 用水平先验
        } catch (const std::exception&) {
            // 回退分位数
        }
    }
    return fitGroundPercentile(zs, cfg);
}

} // namespace rosiwit_slam
