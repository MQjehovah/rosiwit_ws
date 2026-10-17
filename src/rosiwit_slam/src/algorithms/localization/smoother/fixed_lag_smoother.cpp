// src/algorithms/localization/smoother/fixed_lag_smoother.cpp
#include "algorithms/localization/smoother/fixed_lag_smoother.h"
#include <cmath>

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

namespace rosiwit_slam {

namespace {
gtsam::Pose3 toPose(const PoseStamped& p) {
    return gtsam::Pose3(gtsam::Rot3(p.rot), gtsam::Point3(p.trans.x(), p.trans.y(), p.trans.z()));
}
PoseStamped fromPose(const gtsam::Pose3& g, double t) {
    PoseStamped p;
    p.time = t;
    p.rot = g.rotation().matrix();
    p.trans = g.translation();
    return p;
}
std::unique_ptr<gtsam::IncrementalFixedLagSmoother> g_smoother;
} // namespace

void FixedLagSmoother::reset(const PoseStamped& init) {
    g_smoother = std::make_unique<gtsam::IncrementalFixedLagSmoother>(m_cfg.lag_sec);
    gtsam::NonlinearFactorGraph nfg;
    gtsam::Values nv;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    using gtsam::symbol_shorthand::X;
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    nfg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), toPose(init), prior_noise);
    nv.insert(X(0), toPose(init));
    ts[X(0)] = init.time;
    g_smoother->update(nfg, nv, ts);
    m_idx = 0;
    m_last_pred = init;
    m_last_est = init;
    m_has_last = true;
}

void FixedLagSmoother::addFrame(double time, const PoseStamped& pred, bool has_match,
                                const PoseStamped& match, const Eigen::Matrix<double, 6, 6>& match_cov) {
    if (!g_smoother || !m_has_last) { m_last_est = has_match ? match : pred; return; }
    using gtsam::symbol_shorthand::X;
    const long i = m_idx + 1;
    gtsam::NonlinearFactorGraph nfg;
    gtsam::Values nv;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;

    // 运动先验 between: prev_pred^{-1} ∘ pred
    const gtsam::Pose3 dp = toPose(m_last_pred).between(toPose(pred));
    gtsam::Vector6 mv;
    mv << m_cfg.motion_rot_sigma * m_cfg.motion_rot_sigma, m_cfg.motion_rot_sigma * m_cfg.motion_rot_sigma,
          m_cfg.motion_rot_sigma * m_cfg.motion_rot_sigma, m_cfg.motion_trans_sigma * m_cfg.motion_trans_sigma,
          m_cfg.motion_trans_sigma * m_cfg.motion_trans_sigma, m_cfg.motion_trans_sigma * m_cfg.motion_trans_sigma;
    nfg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), dp,
                                                           gtsam::noiseModel::Diagonal::Variances(mv));
    nv.insert(X(i), toPose(pred));

    // 扫描匹配 unary (带协方差)
    if (has_match) {
        Eigen::Matrix<double, 6, 6> C = match_cov;
        C += Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;  // 保证正定
        // GTSAM 噪声序为 (rot, trans), 与我们的 cov 一致
        auto noise = gtsam::noiseModel::Gaussian::Covariance(C);
        nfg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(i), toPose(match), noise);
    }
    ts[X(i)] = time;
    try {
        g_smoother->update(nfg, nv, ts);
        gtsam::Values est = g_smoother->calculateEstimate();
        if (est.exists(X(i))) {
            m_last_est = fromPose(est.at<gtsam::Pose3>(X(i)), time);
        }
    } catch (const std::exception&) {
        m_last_est = has_match ? match : pred;
    }
    m_last_pred = pred;
    m_idx = i;
    m_has_last = true;
}

bool FixedLagSmoother::getPose(PoseStamped& out, Eigen::Matrix<double, 6, 6>* cov) {
    if (!g_smoother || !m_has_last) return false;
    out = m_last_est;
    if (cov) *cov = m_last_cov;
    return true;
}

} // namespace rosiwit_slam

#else  // !USE_GTSAM: 直通降级
namespace rosiwit_slam {
void FixedLagSmoother::reset(const PoseStamped& init) { m_last_est = init; m_has_last = true; }
void FixedLagSmoother::addFrame(double time, const PoseStamped& pred, bool has_match,
                                const PoseStamped& match, const Eigen::Matrix<double, 6, 6>&) {
    m_last_est = has_match ? match : pred;
    m_last_est.time = time;
    m_has_last = true;
}
bool FixedLagSmoother::getPose(PoseStamped& out, Eigen::Matrix<double, 6, 6>* cov) {
    if (!m_has_last) return false;
    out = m_last_est;
    if (cov) *cov = m_last_cov;
    return true;
}
} // namespace rosiwit_slam
#endif
