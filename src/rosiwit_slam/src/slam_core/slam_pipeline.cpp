#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"
#include "algorithms/map_store/voxel_map_store.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <pcl/common/transforms.h>

namespace rosiwit_slam {

namespace {
// SE3 组合/求逆: poseMul(a,b) = 先 b 后 a
PoseStamped poseMul(const PoseStamped& a, const PoseStamped& b) {
    PoseStamped c;
    c.time = b.time;
    c.rot = a.rot * b.rot;
    c.trans = a.rot * b.trans + a.trans;
    return c;
}
PoseStamped poseInv(const PoseStamped& p) {
    PoseStamped q;
    q.time = p.time;
    q.rot = p.rot.transpose();
    q.trans = -q.rot * p.trans;
    return q;
}
}  // namespace

// ==================== 后端 worker ====================

BackendWorker::BackendWorker(std::shared_ptr<IBackend> backend,
                             std::shared_ptr<IMapManager> map_mgr,
                             std::shared_ptr<ILoopClosure> loop,
                             std::shared_ptr<AnchorState> anchor)
    : m_backend(std::move(backend)),
      m_map_mgr(std::move(map_mgr)),
      m_loop(std::move(loop)),
      m_anchor(std::move(anchor)) {}

void BackendWorker::start() {
    if (m_thread.joinable()) return;
    auto self = shared_from_this();
    m_thread = std::thread([self] { self->run(); });
    m_loop_thread = std::thread([self] { self->runLoop(); });
}

void BackendWorker::push(BackendTask task, const KeyFrame& kf) {
    // 回环检测走独立队列/线程, 不进入主后端队列.
    if (task == BackendTask::LOOP_CLOSURE) {
        std::lock_guard<std::mutex> lk(m_loop_mu);
        if (m_loop_stop) return;
        constexpr size_t kMaxLoopQueue = 200;
        if (m_loop_queue.size() >= kMaxLoopQueue) m_loop_queue.pop_front();
        m_loop_queue.push_back(kf);
        m_loop_cv.notify_one();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_stop) return;
        // OPTIMIZE 合并成一个 pending 标志, 不占队列, 也不会被 queue full 丢掉.
        if (task == BackendTask::OPTIMIZE) {
            m_optimize_requested = true;
            m_cv.notify_one();
            return;
        }
        constexpr size_t kMaxQueue = 500;
        if (m_queue.size() >= kMaxQueue) {
            // 回环任务已独立队列; 主队列只保留 ADD_KF/MAP_SUBMAP.
            m_queue.pop_front();
            std::cerr << "[BackendWorker] queue full; dropped oldest task" << std::endl;
        }
        m_queue.push_back({task, kf});
    }
    m_cv.notify_one();
}

void BackendWorker::stopAsync() {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_stop = true;
    }
    {
        std::lock_guard<std::mutex> lk(m_loop_mu);
        m_loop_stop = true;
        m_loop_queue.clear();
    }
    m_cv.notify_all();
    m_loop_cv.notify_all();
    // 不 join: 后端可能正卡在一次 optimize() 里, join 会把调用线程(ROS 服务线程)挂死。
    // worker 只持有自己的 shared 状态, 线程结束后自然释放。
    if (m_thread.joinable()) m_thread.detach();
    if (m_loop_thread.joinable()) m_loop_thread.detach();
}

void BackendWorker::optimizeAndPublishAnchor() {
    if (m_backend && m_backend->optimize()) {
        std::vector<PoseStamped> corrected;
        if (m_map_mgr && m_backend->getUpdatedPoses(corrected) && !corrected.empty()) {
            m_map_mgr->onBackendOptimized(corrected);
            PoseStamped anchor;
            if (m_map_mgr->getCorrectionAnchor(anchor) && m_anchor) {
                std::lock_guard<std::mutex> lk(m_anchor->mu);
                m_anchor->anchor = anchor;
                m_anchor->have = true;
            }
        }
    }
}

void BackendWorker::run() {
    while (true) {
        BackendItem item;
        bool have_item = false;
        {
            std::unique_lock<std::mutex> lk(m_mu);
            m_cv.wait(lk, [this] {
                return m_stop || m_optimize_requested || !m_queue.empty();
            });
            if (m_stop) break;
            if (!m_queue.empty()) {
                item = std::move(m_queue.front());
                m_queue.pop_front();
                have_item = true;
            }
        }
        if (have_item) {
            if (item.task == BackendTask::ADD_KF && m_backend) {
                m_backend->addKeyFrame(item.kf);
            }
            if (item.task == BackendTask::MAP_SUBMAP && m_map_mgr) {
                m_map_mgr->addSubMap(item.kf);
            }
        }

        // 队列排空后执行合并的优化请求. 若新任务已经到达则先继续消费,
        // 保证 ADD_KF/MAP_SUBMAP 不因一次长时间 Ceres 优化而积压。
        bool optimize_now = false;
        {
            std::lock_guard<std::mutex> lk(m_mu);
            if (m_optimize_requested && m_queue.empty()) {
                m_optimize_requested = false;
                optimize_now = true;
            }
        }
        if (optimize_now) {
            std::cout << "[BackendWorker] optimize (coalesced request)" << std::endl;
            optimizeAndPublishAnchor();
        }
    }
}

void BackendWorker::requestOptimize() {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_stop) return;
        m_optimize_requested = true;
    }
    m_cv.notify_one();
}

void BackendWorker::runLoop() {
    while (true) {
        KeyFrame kf;
        {
            std::unique_lock<std::mutex> lk(m_loop_mu);
            m_loop_cv.wait(lk, [this] {
                return m_loop_stop || !m_loop_queue.empty();
            });
            if (m_loop_stop) break;
            if (m_loop_queue.empty()) continue;
            kf = std::move(m_loop_queue.front());
            m_loop_queue.pop_front();
        }
        if (!m_loop) continue;

        // ScanContext + local-submap GICP 只在这个线程里跑.
        m_loop->addKeyFrame(kf);
        if (++m_loop_lc_keyframes % 30 != 0) continue;

        PoseStamped rel_pose;
        double cov = 0.0;
        std::string matched_id;
        if (!m_loop->detect(rel_pose, cov, matched_id) || !m_backend) continue;

        Constraint c;
        c.from_kf = matched_id;
        c.to_kf = kf.id;
        c.relative_pose = rel_pose;
        c.cov = cov;
        m_backend->addConstraints({c});
        // 只请求优化, 由主 backend worker 串行执行 Ceres/map rebuild.
        requestOptimize();
    }
}

SlamPipeline::~SlamPipeline() {
    if (m_backend_worker) m_backend_worker->stopAsync();
    {
        std::lock_guard<std::mutex> lk(m_loc_mu);
        m_loc_stop = true;
    }
    m_loc_cv.notify_all();
    if (m_loc_worker.joinable()) m_loc_worker.join();
}

bool SlamPipeline::init(const std::string& config_path) {
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (...) {
        return false;
    }

    // 雷达 -> IMU 外参 (ROS1 层在 override YAML 顶层注入)。定位模式输出
    // body_cloud/world_cloud 时必须先把扫描转到 IMU 系, 否则倾斜安装的雷达
    // 会把点云/地图整体转歪 (实机 43 度倾斜安装就是这种情况)。
    if (cfg["r_il"] && cfg["r_il"].IsSequence() && cfg["r_il"].size() == 9) {
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R(r, c) = cfg["r_il"][r * 3 + c].as<double>();
            }
        }
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        if (cfg["t_il"] && cfg["t_il"].IsSequence() && cfg["t_il"].size() == 3) {
            t << cfg["t_il"][0].as<double>(), cfg["t_il"][1].as<double>(),
                 cfg["t_il"][2].as<double>();
        }
        m_lidar_to_imu.setIdentity();
        m_lidar_to_imu.block<3, 3>(0, 0) = R.cast<float>();
        m_lidar_to_imu.block<3, 1>(0, 3) = t.cast<float>();
        m_lidar_to_imu_valid = true;
    }

    const YAML::Node glob = cfg["global"];
    auto g = [&](const char* key) -> YAML::Node {
        if (glob && glob[key]) return glob[key];
        return cfg[key];
    };
    m_pipeline_name = g("pipeline").as<std::string>("custom_pipeline");

    std::string mode_str = g("pipeline_mode").as<std::string>("idle");
    if (mode_str == "mapping")          m_mode = PipelineMode::MAPPING;
    else if (mode_str == "localization") m_mode = PipelineMode::LOCALIZATION;
    else                                m_mode = PipelineMode::IDLE;

    auto modules = (glob && glob["modules"]) ? glob["modules"] : cfg["modules"];
    if (!modules) return true;

    std::string name;
    std::vector<std::string> module_errors;
    auto module_failed = [&](const std::string& key, const std::string& module_name) {
        module_errors.push_back(key + "=" + module_name);
        std::cerr << "[SlamPipeline] module '" << key << "' = '" << module_name
                  << "' unavailable or init failed" << std::endl;
    };

    auto createFrontend = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_frontend = SlamFactory::createFrontend(name);
        if (!m_frontend || !m_frontend->init(config_path)) {
            m_frontend.reset();
            module_failed(key, name);
        }
    };
    auto createBackend = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_backend = std::shared_ptr<IBackend>(SlamFactory::createBackend(name));
        if (!m_backend || !m_backend->init(config_path)) {
            m_backend.reset();
            module_failed(key, name);
            return;
        }
        m_anchor = std::make_shared<AnchorState>();
        m_backend_worker = std::make_shared<BackendWorker>(m_backend, m_map_mgr, m_loop, m_anchor);
        m_backend_worker->start();
    };
    auto createLoopClosure = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        auto loop = SlamFactory::createLoopClosure(name);
        if (!loop || !loop->init(config_path)) {
            module_failed(key, name);
            return;
        }
        m_loop = std::shared_ptr<ILoopClosure>(std::move(loop));
    };
    auto createLocalization = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_localization = SlamFactory::createLocalization(name);
        if (!m_localization || !m_localization->init(config_path)) {
            m_localization.reset();
            module_failed(key, name);
        }
    };
    auto createMapManager = [&](const std::string& key) {
        name = modules[key].as<std::string>("");
        if (name.empty()) return;
        m_map_mgr = std::shared_ptr<IMapManager>(SlamFactory::createMapManager(name));
        if (!m_map_mgr || !m_map_mgr->init(config_path)) {
            m_map_mgr.reset();
            module_failed(key, name);
        }
    };

    // map_manager 先建: 后端 worker 需要共享它.
    createMapManager("map_manager");
    createFrontend("frontend");
    createLoopClosure("loop_closure");
    createLocalization("localization");
    createBackend("backend");

    if (!module_errors.empty()) {
        std::cerr << "[SlamPipeline] init failed: configured module(s) could not be created:";
        for (const auto& e : module_errors) std::cerr << " " << e;
        std::cerr << std::endl;
        return false;
    }
    if (m_mode == PipelineMode::MAPPING && !m_frontend) {
        std::cerr << "[SlamPipeline] init failed: mapping mode requires modules.frontend" << std::endl;
        return false;
    }
    if (m_mode == PipelineMode::LOCALIZATION && (!m_localization || !m_map_mgr)) {
        std::cerr << "[SlamPipeline] init failed: localization mode requires "
                     "modules.localization and modules.map_manager" << std::endl;
        return false;
    }

    // 视觉松耦合: 读 vision 节点, 启用时创建 VO 因子源
    if (cfg["vision"] && cfg["vision"]["enabled"].as<bool>(false)) {
        m_visual_cfg.enabled       = true;
        m_visual_cfg.assoc_dt      = cfg["vision"]["assoc_dt"].as<double>(m_visual_cfg.assoc_dt);
        m_visual_cfg.vo_trans_noise = cfg["vision"]["vo_trans_noise"].as<double>(m_visual_cfg.vo_trans_noise);
        m_visual_cfg.vo_rot_noise  = cfg["vision"]["vo_rot_noise"].as<double>(m_visual_cfg.vo_rot_noise);
        m_visual = std::make_unique<VisualConstraintSource>();
        m_visual->configure(m_visual_cfg);
        std::cout << "[SlamPipeline] vision loose-coupling enabled" << std::endl;
    }

    if (m_mode == PipelineMode::LOCALIZATION && m_map_mgr && m_localization) {
        std::string map_name = cfg["localization_map"].as<std::string>("");
        if (map_name.empty()) {
            std::cerr << "[SlamPipeline] localization mode requires localization_map" << std::endl;
            return false;
        }
        if (!map_name.empty()) {
            PoseStamped init; init.trans = V3D::Zero(); init.rot = M3D::Identity();
            init.time = -1.0;   // 无 init_pose 配置时走冷启动全局重定位
            if (cfg["init_pose"]) {
                auto p = cfg["init_pose"];
                init.time = 0.0;
                init.trans.x() = p["x"].as<double>(0.0);
                init.trans.y() = p["y"].as<double>(0.0);
                init.trans.z() = p["z"].as<double>(0.0);
            }
            loadMapForLocalization(map_name, init);
        }
    }
    {
        std::lock_guard<std::mutex> lk(m_loc_mu);
        m_loc_stop = false;
    }
    if (!m_loc_worker.joinable()) m_loc_worker = std::thread([this] { localizationWorker(); });
    return true;
}

void SlamPipeline::setMode(PipelineMode mode) {
    m_mode = mode;
    if (mode != PipelineMode::LOCALIZATION) {
        std::lock_guard<std::mutex> lk(m_loc_mu);
        m_loc_buf.clear();
    }
    m_loc_cv.notify_all();
}

void SlamPipeline::onVisualOdometry(double time, const PoseStamped& vo_pose) {
    if (m_visual) m_visual->addVisualOdometry(time, vo_pose);
}

void SlamPipeline::onOdom(const PoseStamped& odom) {
    if (m_mode.load() == PipelineMode::LOCALIZATION && m_localization) m_localization->onOdom(odom);
}

bool SlamPipeline::getHighRatePose(double t, PoseStamped& out) {
    if (m_mode.load() != PipelineMode::LOCALIZATION || !m_localization) return false;
    return m_localization->predictAt(t, out);
}

bool SlamPipeline::getPoseCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) {
    if (m_mode.load() != PipelineMode::LOCALIZATION || !m_localization) return false;
    return m_localization->getPoseWithCov(out, cov);
}

void SlamPipeline::onAbsolute(const PoseStamped& pose, const Eigen::Matrix<double, 6, 6>& cov) {
    if (m_mode.load() == PipelineMode::LOCALIZATION && m_localization) m_localization->onAbsolute(pose, cov);
}

void SlamPipeline::onVisualLoop(const std::string& from_id, const std::string& to_id,
                                const PoseStamped& rel, double cov) {
    if (!m_backend) return;
    m_backend->addConstraints({VisualConstraintSource::makeLoopConstraint(from_id, to_id, rel, cov)});
    pushBackendTask(BackendTask::OPTIMIZE);
}

bool SlamPipeline::loadMapForLocalization(const std::string& path, const PoseStamped& init_pose) {
    if (!m_localization) return false;
    // 核心只接受 PCD/中立地图数据；xzmap 由 ROS1 适配层转成 MapData 后注入。
    std::ifstream pcd_probe(path);
    if (!pcd_probe.good()) {
        std::cout << "[SlamPipeline] no PCD map at " << path << std::endl;
        return false;
    }
    if (m_map_mgr && !m_map_mgr->loadMap(path)) {
        std::cout << "[SlamPipeline] PCD load failed: " << path << std::endl;
        return false;
    }
    m_localization->setMap(path);
    m_localization_map_loaded = true;
    // init_pose.time < 0 = "无初始位姿": 不 setInitPose, 走冷启动全局重定位。
    // (2026-10-17 实测: 这里曾无条件用伪造的 identity 位姿初始化 -> /switch_map
    //  不带 initpose 时从原点僵尸跟踪, fitness 0.74 既不判丢也不恢复)
    if (init_pose.time >= 0.0) {
        m_localization->setInitPose(init_pose);
    } else {
        std::cout << "[SlamPipeline] no init pose given: cold-start global relocalization"
                  << std::endl;
    }
    return true;
}

void SlamPipeline::onImu(const IMUSample& s) {
    auto mode = m_mode.load();
    if (mode == PipelineMode::IDLE) return;
    if (mode == PipelineMode::LOCALIZATION) {
        // 定位模式不喂建图前端: 前端只在 onLidar 里消费 IMU 缓冲, 而定位模式不喂 lidar,
        // 继续喂 IMU 会让 FastLio2Frontend 的 m_imu_buffer 无界增长 (且计算完全无用).
        if (m_localization) m_localization->onImu(s);
    } else {
        SlamBase::onImu(s);
    }
}

void SlamPipeline::onLidar(const LidarFrame& f) {
    auto mode = m_mode.load();
    if (mode == PipelineMode::IDLE) return;
    if (mode == PipelineMode::LOCALIZATION) {
        // 只入队: GICP/重定位不能占用 sensor callback, 否则 IMU 会在 DDS/队列里
        // 积压, 匹配结束后再被积分一遍, 造成重复积分/过冲.
        {
            std::lock_guard<std::mutex> lk(m_loc_mu);
            if (m_loc_buf.size() >= 2) m_loc_buf.pop_front();
            m_loc_buf.push_back(f);
        }
        m_loc_cv.notify_one();
        return;
    }
    SlamBase::onLidar(f);
}

void SlamPipeline::localizationWorker() {
    while (true) {
        LidarFrame f;
        {
            std::unique_lock<std::mutex> lk(m_loc_mu);
            m_loc_cv.wait(lk, [this] {
                return m_loc_stop || (m_mode.load() == PipelineMode::LOCALIZATION &&
                                      !m_loc_buf.empty());
            });
            if (m_loc_stop) break;
            if (m_mode.load() != PipelineMode::LOCALIZATION || !m_localization) {
                m_loc_buf.clear();
                continue;
            }
            f = std::move(m_loc_buf.front());
            m_loc_buf.pop_front();
        }
        // 重定位/GICP 在 worker 线程执行，ROS spinner 仍能处理服务、TF、slam/info。
        handleLocalization(f);
    }
}

void SlamPipeline::handleLocalization(const LidarFrame& f) {
    if (!m_localization) return;
    m_localization->onLidar(f);

    const ILocalization::Status st = m_localization->getStatus();
    const bool pose_usable = (st == ILocalization::LOCALIZED ||
                              st == ILocalization::INIT);

    // INIT 仍然有可用的末位姿，按 RUNNING 输出；是否可信由核心
    // localizationStatus() 单独给出，ROS 层不再根据 state 二次猜健康状态。
    if (pose_usable) {
        SlamOutput out;
        m_localization->getPose(out.pose);
        out.state = SlamState::RUNNING;
        out.has_new_pose = true;
        if (f.cloud && !f.cloud->empty()) {
            CloudType::Ptr body_scan = f.cloud;
            if (m_lidar_to_imu_valid) {
                body_scan.reset(new CloudType());
                pcl::transformPointCloud(*f.cloud, *body_scan, m_lidar_to_imu);
            }
            CloudType::Ptr world_scan(new CloudType());
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.translate(out.pose.trans);
            T.rotate(out.pose.rot);
            pcl::transformPointCloud(*body_scan, *world_scan, T);
            out.world_cloud = world_scan;
            out.body_cloud = body_scan;
        }
        emitOutput(out);
        return;
    }

    // INIT/LOST: 仍输出最后有效位姿用于界面保持位置，避免“完全没数据”。
    SlamOutput loose;
    if (m_localization->getPose(loose.pose)) {
        loose.state = SlamState::LOST;
        loose.has_new_pose = true;
        if (f.cloud && !f.cloud->empty()) {
            CloudType::Ptr body_scan = f.cloud;
            if (m_lidar_to_imu_valid) {
                body_scan.reset(new CloudType());
                pcl::transformPointCloud(*f.cloud, *body_scan, m_lidar_to_imu);
            }
            CloudType::Ptr world_scan(new CloudType());
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.translate(loose.pose.trans);
            T.rotate(loose.pose.rot);
            pcl::transformPointCloud(*body_scan, *world_scan, T);
            loose.world_cloud = world_scan;
            loose.body_cloud = body_scan;
        }
        emitOutput(loose);
    }
}

bool SlamPipeline::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    // 模式切换后, SlamBase 里可能还残留上一种模式已同步的包;
    // 定位模式绝不能继续喂建图前端.
    if (m_mode.load() != PipelineMode::MAPPING) return false;
    return handleMapping(pkg, out);
}

// ==================== 后端任务入口 ====================

void SlamPipeline::pushBackendTask(BackendTask task, const KeyFrame& kf) {
    if (m_backend_worker) m_backend_worker->push(task, kf);
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
        // 地图管理 + ScanContext/ICP 全部移到 backend worker:
        // 前端 work 组只负责传感器/前端/TF, 不能被地图锁或 ICP 拖住.
        if (m_map_mgr) pushBackendTask(BackendTask::MAP_SUBMAP, kf);
        // 视觉松耦合: 关键帧注册 + 相邻关键帧 VO between 因子入图
        if (m_visual) {
            m_visual->addKeyframe(kf.id, kf.timestamp);
            std::vector<Constraint> vcs = m_visual->popBetweenConstraints();
            if (!vcs.empty() && m_backend) m_backend->addConstraints(vcs);
        }

        // 回环检测 + ICP 在 backend worker 中异步执行, 每 30 个关键帧检测一次.
        if (m_loop) pushBackendTask(BackendTask::LOOP_CLOSURE, kf);
    }

    // 定期触发后端优化 (每 50 关键帧)
    if (m_backend && m_frame_count > 0 && m_frame_count % 50 == 0) {
        pushBackendTask(BackendTask::OPTIMIZE);
    }

    out.body_cloud = out.world_cloud = nullptr;
    m_frontend->getClouds(out.body_cloud, out.world_cloud);

    // 实时回环修正: 后端优化后, 把输出位姿/轨迹与 world_cloud 平移到修正坐标系(A∘R),
    // 使 odom/tf/path 与重建后的地图一致, 回环瞬间可看到"拉回"。
    PoseStamped anchor;
    bool have_anchor = false;
    if (m_anchor) {
        std::lock_guard<std::mutex> lock(m_anchor->mu);
        if (m_anchor->have) { anchor = m_anchor->anchor; have_anchor = true; }
    }
    out.pose = poseMul(anchor, odom);
    out.pose.vel = anchor.rot * odom.vel;
    out.odom_pose = odom;
    out.map_T_odom = anchor;
    out.has_odom_pose = true;
    if (have_anchor && out.world_cloud && !out.world_cloud->empty()) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.rotate(anchor.rot);
        T.translate(anchor.trans);
        CloudType::Ptr w(new CloudType());
        pcl::transformPointCloud(*out.world_cloud, *w, T);
        out.world_cloud = w;
    }
    out.state = SlamState::RUNNING;
    out.has_new_pose = true;
    return true;
}

bool SlamPipeline::getGlobalMap(PointVec& out) {
    if (m_mode == PipelineMode::LOCALIZATION && m_map_mgr) return m_map_mgr->getGlobalMap(out);
    // mapping: 优先发布后端修正版全局地图(map_manager 关键帧重建), 使回环修正实时可见
    if (m_mode == PipelineMode::MAPPING && m_map_mgr) {
        PointVec m;
        if (m_map_mgr->getGlobalMap(m) && !m.empty()) {
            out = std::move(m);
            return true;
        }
    }
    if (m_frontend) return m_frontend->getGlobalMap(out);
    if (m_map_mgr) return m_map_mgr->getGlobalMap(out);
    return false;
}

SlamState SlamPipeline::state() const {
    if (m_mode == PipelineMode::IDLE) return SlamState::READY;
    return SlamState::RUNNING;
}

bool SlamPipeline::loadLocalizationMapData(const MapData& data) {
    if (!m_localization) return false;
    m_localization->setMapData(data);
    m_localization_map_loaded = true;
    std::cout << "[SlamPipeline] localization MapData loaded: "
              << data.meta.map_id << " kfs=" << data.keyframes.size() << std::endl;
    return true;
}

bool SlamPipeline::exportMapData(MapData& out) {
    if (!m_map_mgr) return false;
    PointVec pts;
    if (!m_map_mgr->getGlobalMap(pts) || pts.empty()) return false;
    CloudType::Ptr cloud(new CloudType());
    cloud->reserve(pts.size());
    for (const auto& p : pts) cloud->push_back(p);
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    auto store = std::make_shared<VoxelMapStore>(0.3);
    store->buildFromCloud(cloud);
    out.points = store;
    if (m_loop) m_loop->getKeyframes(out.keyframes);
    out.grid = m_map_mgr->getGridInfo();
    out.meta.map_id = "";
    out.meta.frame_id = "map";
    out.meta.resolution = out.grid.resolution;
    out.meta.origin_x = out.grid.origin_x;
    out.meta.origin_y = out.grid.origin_y;
    out.meta.revision = m_map_mgr->getRevision();
    return true;
}

void SlamPipeline::cancelMapLoad() {
    if (m_localization) m_localization->cancelSetMap();
}

bool SlamPipeline::saveMap(const std::string& path) {
    if (!m_map_mgr) return false;
    bool ok = m_map_mgr->saveMap(path);
    if (ok && m_loop) m_loop->saveDatabase(path + ".scdb");
    return ok;
}

bool SlamPipeline::loadMap(const std::string& path) {
    if (!m_map_mgr) return false;
    // 无初始位姿 (time<0 标记): 加载后由冷启动全局重定位接管, 不再伪造 identity 起点
    PoseStamped init;
    init.trans = V3D::Zero();
    init.rot = M3D::Identity();
    init.time = -1.0;
    if (!loadMapForLocalization(path, init)) return false;
    m_map_mgr->generateGridMap(0.05);
    return true;
}

bool SlamPipeline::appendMap(const std::string& path) {
    if (!m_map_mgr) return false;
    if (!m_map_mgr->appendMap(path)) return false;
    m_map_mgr->generateGridMap(0.05);
    return true;
}

bool SlamPipeline::resetForNewMapping(const std::string& config_path) {
    // 旧 xz_3d_slam 每次切建图都 DELETE_PTR(mapping_ptr_) 再 new 一个, 所以上一轮绝不残留;
    // 我们是单核心实例, 只能显式重建内部模块达到同样效果。
    //
    // 0) 停定位 worker，避免 reset 模块时 worker 仍持有 m_localization。
    {
        std::lock_guard<std::mutex> lk(m_loc_mu);
        m_loc_stop = true;
    }
    m_loc_cv.notify_all();
    if (m_loc_worker.joinable()) m_loc_worker.join();
    //
    // 1) 停后端 worker: stopAsync() 只置停止并 detach, 不 join (后端可能正卡在 optimize).
    //    worker 只持有自己的 shared_ptr<IBackend/IMapManager/AnchorState>, 不会访问本对象,
    //    所以旧线程晚于新实例退出也不会串数据/悬空。
    if (m_backend_worker) {
        m_backend_worker->stopAsync();
        m_backend_worker.reset();
        std::cout << "[SlamPipeline] backend worker stopped (detached, no join)" << std::endl;
    }

    // 2) 丢掉所有模块 + 锚点状态 (= "删掉建图实例")
    m_frontend.reset();
    m_backend.reset();
    m_loop.reset();
    m_localization.reset();
    m_map_mgr.reset();
    m_visual.reset();
    m_anchor.reset();

    // 先置 IDLE 再 init: 否则 init() 里那段 "m_mode == LOCALIZATION 就自动载图" 会误触发
    m_mode = PipelineMode::IDLE;
    m_localization_map_loaded = false;
    m_frame_count = 0;

    // 3) 重建 (与启动时同一条路径)
    const bool ok = init(config_path);
    std::cout << "[SlamPipeline] resetForNewMapping: modules rebuilt (" << (ok ? "ok" : "FAILED")
              << ")" << std::endl;
    return ok;
}

bool SlamPipeline::saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution) {
    if (!m_map_mgr) return false;
    return m_map_mgr->saveGridMap(pgm_path, yaml_path, resolution);
}

void SlamPipeline::requestRelocalization() {
    if (m_localization) m_localization->requestRelocalization();
}

bool SlamPipeline::getRealtimeGrid(double resolution, rosiwit_slam::GridInfo& out) {
    // **必须走 generateGridMap**, 它内部用的是 buildFromObservations(逐帧观测) ——
    // 和 saveGridMap 完全同一条路径, 有逐帧地面处理。薄层以前是自己拿合并后的全局点云
    // 调 gen->build(), 没有地面处理 -> 实时 /map 有地面噪点, 而存下来的 pgm 干净。
    if (!m_map_mgr) return false;
    if (!m_map_mgr->generateGridMap(resolution)) return false;
    out = m_map_mgr->getGridInfo();
    return out.width > 0 && out.height > 0;
}

bool SlamPipeline::setGridMapGenerator(const std::string& name) {
    if (!m_map_mgr) return false;
    return m_map_mgr->setGridMapGenerator(name);
}

bool SlamPipeline::setPipelineMode(const std::string& mode) {
    if (mode == "mapping") {
        if (!m_frontend) { std::cerr << "[SlamPipeline] mapping unavailable: frontend missing" << std::endl; return false; }
        setMode(PipelineMode::MAPPING);
        return true;
    }
    if (mode == "localization") {
        if (!m_localization) { std::cerr << "[SlamPipeline] localization unavailable: module missing" << std::endl; return false; }
        if (!m_localization_map_loaded.load()) {
            std::cerr << "[SlamPipeline] localization requires load_map first" << std::endl;
            return false;
        }
        setMode(PipelineMode::LOCALIZATION);
        return true;
    }
    if (mode == "idle") { setMode(PipelineMode::IDLE); return true; }
    return false;
}

std::string SlamPipeline::getPipelineMode() const {
    switch (m_mode.load()) {
        case PipelineMode::MAPPING:       return "mapping";
        case PipelineMode::LOCALIZATION:  return "localization";
        default:                          return "idle";
    }
}

ILocalization::Status SlamPipeline::localizationStatus() const {
    if (!m_localization) return ILocalization::LOST;
    return m_localization->getStatus();
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

uint64_t SlamPipeline::getRevision() const {
    if (!m_map_mgr) return 0;
    return m_map_mgr->getRevision();
}

double SlamPipeline::getMatchingDegree() const {
    // 定位匹配度: 透传定位流水线的 confidence() (= m_last_match.fitness, 覆盖率 [0,1]).
    // 旧 xz_3d_slam 的 slam/info.matching_degree = localiza.getFitness(), 实机实测为 0.83 这种真值
    // (不是 0) —— 所以这里必须给出真值, 否则监控端看到的匹配度永远是 0.
    return m_localization ? m_localization->confidence() : 0.0;
}

} // namespace rosiwit_slam
