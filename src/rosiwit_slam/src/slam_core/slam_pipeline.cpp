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

    std::string mode = cfg["pipeline_mode"].as<std::string>("mapping");
    m_is_localization_mode = (mode == "localization");

    auto modules = cfg["modules"];
    if (!modules) return false;

    // Create modules based on config
    std::string name;
    name = modules["frontend"].as<std::string>("");
    if (!name.empty()) {
        m_frontend = SlamFactory::createFrontend(name);
        if (m_frontend && !m_frontend->init(config_path)) {
            m_frontend.reset();
        }
    }

    name = modules["backend"].as<std::string>("");
    if (!name.empty()) {
        m_backend = SlamFactory::createBackend(name);
        if (m_backend && !m_backend->init(config_path)) {
            m_backend.reset();
        }
    }

    name = modules["loop_closure"].as<std::string>("");
    if (!name.empty()) {
        m_loop = SlamFactory::createLoopClosure(name);
        if (m_loop && !m_loop->init(config_path)) {
            m_loop.reset();
        }
    }

    name = modules["localization"].as<std::string>("");
    if (!name.empty()) {
        m_localization = SlamFactory::createLocalization(name);
        if (m_localization && !m_localization->init(config_path)) {
            m_localization.reset();
        }
    }

    name = modules["map_manager"].as<std::string>("");
    if (!name.empty()) {
        m_map_mgr = SlamFactory::createMapManager(name);
        if (m_map_mgr && !m_map_mgr->init(config_path)) {
            m_map_mgr.reset();
        }
    }

    // Localization mode: load map and set initial pose
    if (m_is_localization_mode) {
        if (!m_localization || !m_map_mgr) {
            return false;  // 定位模式需要定位模块和地图管理
        }
        std::string map_name = cfg["localization_map"].as<std::string>("map.pcd");
        if (m_map_mgr->loadMap(map_name)) {
            m_localization->setMap(map_name);
            // 如果配置了初始位姿
            if (cfg["init_pose"]) {
                PoseStamped init;
                auto p = cfg["init_pose"];
                init.trans.x() = p["x"].as<double>(0.0);
                init.trans.y() = p["y"].as<double>(0.0);
                init.trans.z() = p["z"].as<double>(0.0);
                m_localization->setInitPose(init);
            }
            m_loc_initialized = true;
        }
        return m_localization != nullptr;
    }

    // Mapping mode: at least need frontend
    return m_frontend != nullptr;
}

bool SlamPipeline::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    if (m_is_localization_mode) {
        return handleLocalization(pkg, out);
    }
    return handleMapping(pkg, out);
}

bool SlamPipeline::handleMapping(const SyncPackage& pkg, SlamOutput& out) {
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

bool SlamPipeline::handleLocalization(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_localization) return false;

    for (const auto& imu : pkg.imus) {
        m_localization->onImu(imu);
    }
    m_localization->onLidar(pkg.frame);

    if (m_localization->getStatus() == ILocalization::LOCALIZED) {
        m_localization->getPose(out.pose);
        out.state = SlamState::RUNNING;
        out.has_new_pose = true;
        return true;
    }
    return false;
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
