// src/algorithms/fast_livo2/fast_livo2_frontend.cpp
#include "algorithms/fast_livo2/fast_livo2_frontend.h"

#include <cmath>
#include <iostream>

#include "algorithms/fast_livo2/livo_mapper.h"
#include "algorithms/fast_livo2/parse_config.h"

namespace rosiwit_slam {

FastLivo2Frontend::FastLivo2Frontend() = default;
FastLivo2Frontend::~FastLivo2Frontend() = default;

bool FastLivo2Frontend::init(const std::string& config_path) {
    std::string err;
    if (!parseFastLivo2Config(config_path, m_config, err)) {
        std::cerr << "[FastLivo2Frontend] init failed: " << err << std::endl;
        return false;
    }
    m_mapper.reset(new LivoMapper());
    if (!m_mapper->init(m_config)) {
        std::cerr << "[FastLivo2Frontend] mapper init failed" << std::endl;
        m_mapper.reset();
        return false;
    }
    m_inited = true;
    return true;
}

void FastLivo2Frontend::onImu(const IMUSample& s) {
    if (m_mapper) m_mapper->addImu(s);
}

void FastLivo2Frontend::onImage(const ImageFrame& f) {
    if (!m_mapper) return;
    m_mapper->addImage(f);
    // 图像到达后再推进一次：LIVO 的 LIO/VIO 切分需要图像时间，
    // 只等下一帧 LiDAR 会多等一个扫描周期甚至卡住。
    std::lock_guard<std::mutex> lock(m_process_mtx);
    m_mapper->process();
}

void FastLivo2Frontend::onLidar(const LidarFrame& f) {
    if (!m_mapper) return;
    std::lock_guard<std::mutex> lock(m_process_mtx);
    m_mapper->addLidar(f);
    m_mapper->process();
}

bool FastLivo2Frontend::getOdometry(PoseStamped& out) {
    if (!m_mapper || !m_mapper->hasPose()) return false;
    out = m_mapper->pose();
    return true;
}

bool FastLivo2Frontend::getClouds(CloudType::Ptr& body, CloudType::Ptr& world) {
    if (!m_mapper) return false;
    body = m_mapper->bodyCloud();
    world = m_mapper->worldCloud();
    return body != nullptr || world != nullptr;
}

bool FastLivo2Frontend::getRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out) {
    if (!m_mapper) return false;
    out = m_mapper->rgbCloud();
    return out != nullptr;
}

bool FastLivo2Frontend::shouldMakeKeyframe(const PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(m_mtx);
    if (!m_has_last_keyframe) {
        m_last_keyframe_pose = pose;
        m_has_last_keyframe = true;
        return true;
    }
    const double dt = (pose.trans - m_last_keyframe_pose.trans).norm();
    const Eigen::Matrix3d dR = m_last_keyframe_pose.rot.transpose() * pose.rot;
    const double dtheta = Eigen::AngleAxisd(dR).angle();
    if (dt > m_config.keyframe_trans_thresh || dtheta > m_config.keyframe_rot_thresh) {
        m_last_keyframe_pose = pose;
        return true;
    }
    return false;
}

bool FastLivo2Frontend::getKeyFrame(KeyFrame& out) {
    if (!m_mapper || !m_mapper->hasPose()) return false;
    const PoseStamped pose = m_mapper->pose();
    if (!shouldMakeKeyframe(pose)) return false;

    out.pose = pose;
    out.cloud = m_mapper->bodyCloud();
    out.timestamp = static_cast<int64_t>(pose.time * 1e9);
    std::lock_guard<std::mutex> lock(m_mtx);
    out.id = std::to_string(++m_keyframe_counter);
    return true;
}

bool FastLivo2Frontend::getGlobalMap(PointVec& /*out*/) {
    // 累计地图由 SlamPipeline 的 map_manager 负责；前端只提供当前帧点云。
    return false;
}

}  // namespace rosiwit_slam
