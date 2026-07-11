// src/algorithms/gtsam_backend/gtsam_isam2_backend.cpp
#ifdef USE_GTSAM
#include "algorithms/gtsam_backend/gtsam_isam2_backend.h"
#include <iostream>
#include <fstream>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

bool GtsamIsam2Backend::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[GtsamIsam2Backend] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        // 仍初始化, 使用默认参数
    } else {
#ifndef YAML_CPP_DISABLED
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            auto backend = config["backend"];
            if (backend) {
                m_odom_trans_noise = backend["odom_trans_noise"].as<double>(m_odom_trans_noise);
                m_odom_rot_noise  = backend["odom_rot_noise"].as<double>(m_odom_rot_noise);
                m_lc_trans_noise  = backend["lc_trans_noise"].as<double>(m_lc_trans_noise);
                m_lc_rot_noise    = backend["lc_rot_noise"].as<double>(m_lc_rot_noise);
                m_isam2_relin_thresh = backend["isam2_relin_thresh"].as<int>(m_isam2_relin_thresh);
                m_enable_partial_relin = backend["enable_partial_relin"].as<bool>(m_enable_partial_relin);
            }
        } catch (const std::exception& e) {
            std::cerr << "[GtsamIsam2Backend] Config parse error: " << e.what() << std::endl;
            return false;
        }
#endif
    }

    m_graph = std::make_unique<gtsam::NonlinearFactorGraph>();
    m_initial_estimates = std::make_unique<gtsam::Values>();

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = m_isam2_relin_thresh;
    params.enablePartialRelinearizationCheck = m_enable_partial_relin;
    m_isam2 = std::make_unique<gtsam::ISAM2>(params);

    m_inited = true;
    std::cout << "[GtsamIsam2Backend] Initialized (relin_thresh=" << m_isam2_relin_thresh
              << ", partial_relin=" << m_enable_partial_relin << ")" << std::endl;
    return true;
}

void GtsamIsam2Backend::addKeyFrame(const KeyFrame& kf) {
    if (!m_inited) return;

    if (m_kf_index.count(kf.id)) {
        // 更新已有的关键帧
        KeyFrameNode& node = m_keyframes[m_kf_index[kf.id]];
        node.pose = kf.pose;
        node.timestamp = kf.timestamp;
        return;
    }

    KeyFrameNode node;
    node.id = kf.id;
    node.pose = kf.pose;
    node.timestamp = kf.timestamp;
    m_kf_index[kf.id] = m_keyframes.size();
    m_keyframes.push_back(node);
}

void GtsamIsam2Backend::addConstraints(const std::vector<Constraint>& constraints) {
    m_pending_constraints.insert(m_pending_constraints.end(),
                                  constraints.begin(), constraints.end());
}

gtsam::Pose3 GtsamIsam2Backend::toGtsamPose(const PoseStamped& p) const {
    gtsam::Rot3 R(p.rot(0,0), p.rot(0,1), p.rot(0,2),
                  p.rot(1,0), p.rot(1,1), p.rot(1,2),
                  p.rot(2,0), p.rot(2,1), p.rot(2,2));
    gtsam::Point3 t(p.trans.x(), p.trans.y(), p.trans.z());
    return gtsam::Pose3(R, t);
}

PoseStamped GtsamIsam2Backend::fromGtsamPose(const gtsam::Pose3& gp, double time) const {
    PoseStamped p;
    p.time = time;
    const gtsam::Matrix3& R = gp.rotation().matrix();
    p.rot << R(0,0), R(0,1), R(0,2),
             R(1,0), R(1,1), R(1,2),
             R(2,0), R(2,1), R(2,2);
    p.trans.x() = gp.x(); p.trans.y() = gp.y(); p.trans.z() = gp.z();
    return p;
}

bool GtsamIsam2Backend::optimize() {
    if (!m_inited || m_keyframes.empty()) return false;

    size_t N = m_keyframes.size();
    if (N <= m_last_opt_size) {
        // 无新帧, 但仍处理回环约束
        if (m_pending_constraints.empty()) return false;
    }

    using gtsam::symbol_shorthand::X;  // X(0), X(1), ...

    // === 1. 添加新关键帧的初始估计 ===
    for (size_t i = m_last_opt_size; i < N; ++i) {
        if (i == 0) {
            // 首帧: 先验因子 (固定初始位姿)
            gtsam::Matrix66 prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
            m_graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                X(0), toGtsamPose(m_keyframes[0].pose), prior_noise);
        } else {
            // 与上一帧的里程计约束
            const auto& prev = m_keyframes[i-1].pose;
            const auto& curr = m_keyframes[i].pose;

            // 相对位姿: T_prev_curr = T_prev^{-1} * T_curr
            gtsam::Pose3 dPose = toGtsamPose(prev).between(toGtsamPose(curr));

            gtsam::Vector6 odom_noise_vec;
            odom_noise_vec << m_odom_trans_noise, m_odom_trans_noise, m_odom_trans_noise,
                              m_odom_rot_noise, m_odom_rot_noise, m_odom_rot_noise;
            auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(odom_noise_vec);

            m_graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(i-1), X(i), dPose, odom_noise);
        }

        // 初始值
        m_initial_estimates->insert(X(i), toGtsamPose(m_keyframes[i].pose));
    }

    // === 2. 添加回环约束 ===
    for (const auto& c : m_pending_constraints) {
        auto from_it = m_kf_index.find(c.from_kf);
        auto to_it = m_kf_index.find(c.to_kf);
        if (from_it == m_kf_index.end() || to_it == m_kf_index.end()) continue;
        if (from_it->second == to_it->second) continue;

        gtsam::Pose3 rel_pose(
            gtsam::Rot3(c.relative_pose.rot),
            gtsam::Point3(c.relative_pose.trans.x(),
                          c.relative_pose.trans.y(),
                          c.relative_pose.trans.z()));

        gtsam::Vector6 lc_noise_vec;
        lc_noise_vec << m_lc_trans_noise, m_lc_trans_noise, m_lc_trans_noise,
                        m_lc_rot_noise, m_lc_rot_noise, m_lc_rot_noise;
        // 置信度加权
        double weight = 1.0 / std::max(c.cov, 0.01);
        auto lc_noise = gtsam::noiseModel::Diagonal::Sigmas(lc_noise_vec * weight);

        m_graph->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(from_it->second), X(to_it->second), rel_pose, lc_noise);
    }
    m_pending_constraints.clear();

    // === 3. iSAM2 增量更新 ===
    try {
        m_isam2->update(*m_graph, *m_initial_estimates);
        m_isam2->update();  // 第二次 update 触发重线性化

        gtsam::Values result = m_isam2->calculateEstimate();

        // 更新关键帧位姿
        for (size_t i = 0; i < N; ++i) {
            if (result.exists(X(i))) {
                gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(i));
                m_keyframes[i].pose = fromGtsamPose(pose, m_keyframes[i].pose.time);
                m_keyframes[i].is_optimized = true;
            }
        }

        m_last_opt_size = N;

        // 清空已处理的图 (iSAM2 已内部维护)
        m_graph->resize(0);
        m_initial_estimates->clear();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[GtsamIsam2Backend] Optimization failed: " << e.what() << std::endl;
        return false;
    }
}

bool GtsamIsam2Backend::getUpdatedPoses(std::vector<PoseStamped>& poses) {
    poses.clear();
    for (const auto& kf : m_keyframes) {
        poses.push_back(kf.pose);
    }
    return !poses.empty();
}

} // namespace rosiwit_slam
#endif // USE_GTSAM
