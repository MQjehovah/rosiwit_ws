#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <pcl/common/transforms.h>

namespace rosiwit_slam {

SlamPipeline::~SlamPipeline() {
    if (m_backend_running) {
        pushBackendTask(BackendTask::STOP);
        if (m_backend_thread.joinable()) m_backend_thread.join();
    }
}

bool SlamPipeline::init(const std::string& config_path) {
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (...) {
        return false;
    }

    m_pipeline_name = cfg["pipeline"].as<std::string>("custom_pipeline");

    std::string mode_str = cfg["pipeline_mode"].as<std::string>("idle");
    if (mode_str == "mapping")          m_mode = PipelineMode::MAPPING;
    else if (mode_str == "localization") m_mode = PipelineMode::LOCALIZATION;
    else                                m_mode = PipelineMode::IDLE;

    auto modules = cfg["modules"];
    if (!modules) return true;

    std::string name;

    auto createFrontend = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_frontend = SlamFactory::createFrontend(name);
        if (m_frontend && !m_frontend->init(config_path)) m_frontend.reset();
    };
    auto createBackend = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_backend = SlamFactory::createBackend(name);
        if (m_backend && !m_backend->init(config_path)) m_backend.reset();
        if (m_backend) {
            m_backend_running = true;
            m_backend_thread = std::thread(&SlamPipeline::backendThreadLoop, this);
        }
    };
    auto createLoopClosure = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_loop = SlamFactory::createLoopClosure(name);
        if (m_loop && !m_loop->init(config_path)) m_loop.reset();
    };
    auto createLocalization = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_localization = SlamFactory::createLocalization(name);
        if (m_localization && !m_localization->init(config_path)) m_localization.reset();
    };
    auto createMapManager = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_map_mgr = SlamFactory::createMapManager(name);
        if (m_map_mgr && !m_map_mgr->init(config_path)) m_map_mgr.reset();
    };

    createFrontend("frontend");
    createBackend("backend");
    createLoopClosure("loop_closure");
    createLocalization("localization");
    createMapManager("map_manager");

    if (m_mode == PipelineMode::LOCALIZATION && m_map_mgr && m_localization) {
        std::string map_name = cfg["localization_map"].as<std::string>("");
        if (!map_name.empty()) {
            PoseStamped init; init.trans = V3D::Zero(); init.rot = M3D::Identity();
            if (cfg["init_pose"]) {
                auto p = cfg["init_pose"];
                init.trans.x() = p["x"].as<double>(0.0);
                init.trans.y() = p["y"].as<double>(0.0);
                init.trans.z() = p["z"].as<double>(0.0);
            }
            loadMapForLocalization(map_name, init);
        }
    }
    return true;
}

void SlamPipeline::setMode(PipelineMode mode) { m_mode = mode; }

bool SlamPipeline::loadMapForLocalization(const std::string& path, const PoseStamped& init_pose) {
    if (!m_map_mgr || !m_map_mgr->loadMap(path)) return false;
    if (m_localization) { m_localization->setMap(path); m_localization->setInitPose(init_pose); }
    return true;
}

void SlamPipeline::onImu(const IMUSample& s) {
    auto mode = m_mode.load();
    if (mode == PipelineMode::IDLE) return;
    if (mode == PipelineMode::LOCALIZATION) {
        if (m_localization) m_localization->onImu(s);
        if (m_frontend) m_frontend->onImu(s);
    } else {
        SlamBase::onImu(s);
    }
}

void SlamPipeline::onLidar(const LidarFrame& f) {
    auto mode = m_mode.load();
    if (mode == PipelineMode::IDLE) return;
    if (mode == PipelineMode::LOCALIZATION) {
        if (m_localization) {
            m_localization->onLidar(f);
            if (m_localization->getStatus() == ILocalization::LOCALIZED) {
                SlamOutput out;
                m_localization->getPose(out.pose);
                out.state = SlamState::RUNNING;
                out.has_new_pose = true;
                if (f.cloud && !f.cloud->empty()) {
                    CloudType::Ptr world_scan(new CloudType());
                    Eigen::Affine3d T = Eigen::Affine3d::Identity();
                    T.translate(out.pose.trans);
                    T.rotate(out.pose.rot);
                    pcl::transformPointCloud(*f.cloud, *world_scan, T);
                    out.world_cloud = world_scan;
                    out.body_cloud = f.cloud;
                }
                emitOutput(out);
            }
        }
        if (m_frontend) m_frontend->onLidar(f);
    } else {
        SlamBase::onLidar(f);
    }
}

bool SlamPipeline::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    return handleMapping(pkg, out);
}

// ==================== 后端线程 ====================

void SlamPipeline::pushBackendTask(BackendTask task, KeyFrame kf) {
    std::lock_guard<std::mutex> lock(m_queue_mutex);
    m_backend_queue.push({task, std::move(kf)});
    m_queue_cv.notify_one();
}

void SlamPipeline::backendThreadLoop() {
    while (true) {
        BackendItem item;
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            m_queue_cv.wait(lock, [this] { return !m_backend_queue.empty(); });
            item = std::move(m_backend_queue.front());
            m_backend_queue.pop();
        }
        if (item.task == BackendTask::STOP) break;

        if (item.task == BackendTask::ADD_KF && m_backend) {
            m_backend->addKeyFrame(item.kf);
        }
        if (item.task == BackendTask::OPTIMIZE && m_backend) {
            m_backend->optimize();
        }
    }
}

// ==================== 建图管线 ====================

bool SlamPipeline::handleMapping(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_frontend) return false;
    for (const auto& imu : pkg.imus) m_frontend->onImu(imu);
    m_frontend->onLidar(pkg.frame);

    PoseStamped odom;
    if (!m_frontend->getOdometry(odom)) return false;

    KeyFrame kf;
    if (m_frontend->getKeyFrame(kf)) {
        ++m_frame_count;
        // 后端: 异步入队, 不阻塞前端
        if (m_backend) pushBackendTask(BackendTask::ADD_KF, kf);
        // 回环检测 + 地图管理: 同步执行 (轻量)
        if (m_loop) m_loop->addKeyFrame(kf);
        if (m_map_mgr) m_map_mgr->addSubMap(kf);

        // 回环检测: 每 30 帧触发一次
        if (m_loop && m_frame_count % 30 == 0) {
            std::cout << "[Pipeline] LC check at kf_" << m_frame_count << std::endl;
            PoseStamped rel_pose;
            double cov = 0;
            std::string matched_id;
            if (m_loop->detect(rel_pose, cov, matched_id)) {
                Constraint c;
                c.from_kf = matched_id;
                c.to_kf = kf.id;
                c.relative_pose = rel_pose;
                c.cov = cov;
                if (m_backend) {
                    m_backend->addConstraints({c});
                    pushBackendTask(BackendTask::OPTIMIZE);
                }
            }
        }
    }

    // 定期触发后端优化 (每 50 关键帧)
    if (m_backend && m_frame_count > 0 && m_frame_count % 50 == 0) {
        pushBackendTask(BackendTask::OPTIMIZE);
    }

    out.pose = odom;
    m_frontend->getClouds(out.body_cloud, out.world_cloud);
    out.state = SlamState::RUNNING;
    out.has_new_pose = true;
    return true;
}

bool SlamPipeline::getGlobalMap(PointVec& out) {
    if (m_mode == PipelineMode::LOCALIZATION && m_map_mgr) return m_map_mgr->getGlobalMap(out);
    if (m_frontend) return m_frontend->getGlobalMap(out);
    if (m_map_mgr) return m_map_mgr->getGlobalMap(out);
    return false;
}

SlamState SlamPipeline::state() const {
    if (m_mode == PipelineMode::IDLE) return SlamState::READY;
    return SlamState::RUNNING;
}

bool SlamPipeline::saveMap(const std::string& path) {
    if (!m_map_mgr) return false;
    bool ok = m_map_mgr->saveMap(path);
    if (ok && m_loop) {
        m_loop->saveDatabase(path + ".scdb");
    }
    return ok;
}

bool SlamPipeline::loadMap(const std::string& path) {
    if (!m_map_mgr) return false;
    PoseStamped init;
    init.trans = V3D::Zero();
    init.rot = M3D::Identity();
    if (!loadMapForLocalization(path, init)) return false;
    m_map_mgr->generateGridMap(0.05);
    return true;
}

bool SlamPipeline::saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution) {
    if (!m_map_mgr) return false;
    return m_map_mgr->saveGridMap(pgm_path, yaml_path, resolution);
}

bool SlamPipeline::setPipelineMode(const std::string& mode) {
    if (mode == "mapping")         { setMode(PipelineMode::MAPPING); return true; }
    else if (mode == "localization") { setMode(PipelineMode::LOCALIZATION); return true; }
    else if (mode == "idle")         { setMode(PipelineMode::IDLE); return true; }
    return false;
}

std::string SlamPipeline::getPipelineMode() const {
    switch (m_mode.load()) {
        case PipelineMode::MAPPING:       return "mapping";
        case PipelineMode::LOCALIZATION:  return "localization";
        default:                          return "idle";
    }
}

bool SlamPipeline::setInitPose(const PoseStamped& pose) {
    if (!m_localization) return false;
    m_localization->setInitPose(pose);
    return true;
}

GridInfo SlamPipeline::getGridInfo() const {
    if (!m_map_mgr) return {};
    return m_map_mgr->getGridInfo();
}

} // namespace rosiwit_slam
