// src/slam_core/trajectory_metrics.cpp
#include "slam_core/trajectory_metrics.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

TrajMetrics evaluateTrajectory(const std::vector<PoseStamped>& est,
                               const std::vector<PoseStamped>& gt) {
    TrajMetrics m;
    const int n = static_cast<int>(std::min(est.size(), gt.size()));
    if (n == 0) return m;
    m.n = n;

    double sum_ate = 0.0;
    for (int i = 0; i < n; ++i) sum_ate += (est[i].trans - gt[i].trans).norm();
    m.ate_mean = sum_ate / n;
    double sq = 0.0;
    for (int i = 0; i < n; ++i) {
        const double d = (est[i].trans - gt[i].trans).norm();
        sq += d * d;
    }
    m.ate_rmse = std::sqrt(sq / n);

    if (n >= 2) {
        double tr = 0.0, ro = 0.0;
        for (int i = 1; i < n; ++i) {
            const M3D dR_est = est[i - 1].rot.transpose() * est[i].rot;
            const V3D dt_est = est[i - 1].rot.transpose() * (est[i].trans - est[i - 1].trans);
            const M3D dR_gt = gt[i - 1].rot.transpose() * gt[i].rot;
            const V3D dt_gt = gt[i - 1].rot.transpose() * (gt[i].trans - gt[i - 1].trans);
            // 相对误差 E = gt_rel^{-1} * est_rel
            const M3D dR = dR_gt.transpose() * dR_est;
            const V3D dt = dR_gt.transpose() * (dt_est - dt_gt);
            tr += dt.squaredNorm();
            const double ang = Eigen::AngleAxisd(dR).angle();
            ro += ang * ang;
        }
        m.rpe_trans_rmse = std::sqrt(tr / (n - 1));
        m.rpe_rot_rmse_deg = std::sqrt(ro / (n - 1)) * 57.2957795;
    }
    return m;
}

} // namespace rosiwit_slam
