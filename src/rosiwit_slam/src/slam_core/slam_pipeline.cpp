#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool SlamPipeline::init(const std::string& config_path) {
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (...) {
        return false;
    }

    m_pipeline_name = cfg["pipeline"].as<std::string>("custom_pipeline");

    auto modules = cfg["modules"];
    if (modules) {
        std::string name;
        name = modules["frontend"].as<std::string>("");
        if (!name.empty()) {
            m_frontend = SlamFactory::createFrontend(name);
            if (m_frontend && !m_frontend->init(config_path)) {
                m_frontend.reset();
            }
        }
    }

    return m_frontend != nullptr;
}

bool SlamPipeline::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_frontend) return false;

    for (const auto& imu : pkg.imus) {
        m_frontend->onImu(imu);
    }
    m_frontend->onLidar(pkg.frame);

    PoseStamped odom;
    if (!m_frontend->getOdometry(odom)) return false;

    KeyFrame kf;
    if (m_frontend->getKeyFrame(kf)) {
        ++m_frame_count;
        if (m_backend) m_backend->addKeyFrame(kf);
        if (m_loop) m_loop->addKeyFrame(kf);
        if (m_map_mgr) m_map_mgr->addSubMap(kf);
    }

    if (m_backend && m_frame_count > 0 && m_frame_count % 50 == 0) {
        if (m_loop) {
            PoseStamped rel_pose;
            double cov = 1.0;
            if (m_loop->detect(rel_pose, cov)) {
                m_backend->addConstraints({});
            }
        }
        m_backend->optimize();
    }

    out.pose = odom;
    m_frontend->getClouds(out.body_cloud, out.world_cloud);
    out.state = SlamState::RUNNING;
    out.has_new_pose = true;
    return true;
}

bool SlamPipeline::getGlobalMap(PointVec& out) {
    if (m_frontend) return m_frontend->getGlobalMap(out);
    if (m_map_mgr) return m_map_mgr->getGlobalMap(out);
    return false;
}

SlamState SlamPipeline::state() const {
    return SlamState::RUNNING;
}

} // namespace rosiwit_slam
