#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"
#include "io/xzmap_writer.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
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
    // 老格式 (xz_3d_slam) 的地图只有 <base>.xzmap + pgm/yaml, **没有 PCD** ——
    // 所以这里不能把"PCD 加载失败"当成致命错误 (旧实现 localiza_ptr_->run("<dir>/map")
    // 也只看它自己那份 xzmap)。定位入口 setMap() 会在没有 PCD 时从关键帧重建基准点云。
    if (m_map_mgr && !m_map_mgr->loadMap(path)) {
        std::cout << "[SlamPipeline] no PCD at " << path
                  << " (legacy xzmap-only map); localization map is loaded from <base>.xzmap"
                  << std::endl;
    }
    m_localization->setMap(path);
    m_localization->setInitPose(init_pose);
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
        // ⚠ **定位模式下不再喂建图前端**: 之前这里无条件调 m_frontend->onLidar(f),
        // 导致 FAST-LIO 前端与定位流水线**同时**吃同一份数据 —— 实机表现为
        // "NO Effective Points!" 刷屏(165+ 次)、ikd-tree 线程空转(KD_TREE::multi_thread_rebuild)、
        // CPU 被白白吃掉, 而定位结果反而发不出来。现在严格二选一。
        // (核心库不依赖 ROS, 所以这里不做参数化; 若将来要"前端热备"再加成员开关)
        (void)0;
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
            if (m_backend->optimize()) {
                // 后端反馈: 把修正后的关键帧位姿交给地图管理器重建全局地图,
                // 让回环优化真正作用到保存/发布的地图上(修正拐点叠图)。
                std::vector<PoseStamped> corrected;
                if (m_map_mgr && m_backend->getUpdatedPoses(corrected) && !corrected.empty()) {
                    m_map_mgr->onBackendOptimized(corrected);
                    // 取回前沿锚点, 供主线程把之后输出(odom/path/world_cloud)实时转到修正系
                    PoseStamped anchor;
                    if (m_map_mgr->getCorrectionAnchor(anchor)) {
                        std::lock_guard<std::mutex> lock(m_anchor_mutex);
                        m_out_anchor_ = anchor;
                        m_have_anchor_ = true;
                    }
                }
            }
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

        // 视觉松耦合: 关键帧注册 + 相邻关键帧 VO between 因子入图
        if (m_visual) {
            m_visual->addKeyframe(kf.id, kf.timestamp);
            std::vector<Constraint> vcs = m_visual->popBetweenConstraints();
            if (!vcs.empty() && m_backend) m_backend->addConstraints(vcs);
        }

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

    out.body_cloud = out.world_cloud = nullptr;
    m_frontend->getClouds(out.body_cloud, out.world_cloud);

    // 实时回环修正: 后端优化后, 把输出位姿/轨迹与 world_cloud 平移到修正坐标系(A∘R),
    // 使 odom/tf/path 与重建后的地图一致, 回环瞬间可看到"拉回"。
    PoseStamped anchor;
    bool have_anchor = false;
    {
        std::lock_guard<std::mutex> lock(m_anchor_mutex);
        if (m_have_anchor_) { anchor = m_out_anchor_; have_anchor = true; }
    }
    out.pose = poseMul(anchor, odom);
    out.pose.vel = anchor.rot * odom.vel;
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

bool SlamPipeline::saveMap(const std::string& path) {
    if (!m_map_mgr) return false;
    bool ok = m_map_mgr->saveMap(path);
    if (ok && m_loop) {
        m_loop->saveDatabase(path + ".scdb");
        exportXzMap(path);
    }
    return ok;
}

// xzmap 导出: 关键帧(body系点云) + 回环优化后位姿, 整理为 io 模块的中性结构后落盘。
// 本函数不感知设备格式的任何细节 (键名/量化/坐标系约定都在 io/xzmap_writer 内)。
bool SlamPipeline::exportXzMap(const std::string& pcd_path) {
    if (!m_loop) return false;

    std::vector<KeyFrame> kfs;
    if (!m_loop->getKeyframes(kfs) || kfs.empty()) {
        std::cerr << "[SlamPipeline] xzmap export: no keyframes" << std::endl;
        return false;
    }

    // 回环优化位姿按序回填 (backend 与 loop 关键帧数量一致时生效)
    std::vector<PoseStamped> refined;
    if (m_backend && m_backend->getUpdatedPoses(refined)
            && refined.size() == kfs.size()) {
        for (size_t i = 0; i < kfs.size(); ++i) kfs[i].pose = refined[i];
    } else {
        std::cerr << "[SlamPipeline] xzmap export: optimized poses unavailable ("
                  << refined.size() << "/" << kfs.size()
                  << "), fallback to frontend poses" << std::endl;
    }

    std::vector<io::XzMapKeyframe> xz_kfs;
    xz_kfs.reserve(kfs.size());
    for (auto& kf : kfs) {
        io::XzMapKeyframe xk;
        const M3D& r = kf.pose.rot;
        // ZYX 欧拉角提取 (R = Rz(yaw)*Ry(pitch)*Rx(roll)), 值域 [-pi, pi]
        xk.pose.roll  = std::atan2(r(2, 1), r(2, 2));
        xk.pose.pitch = std::atan2(-r(2, 0), std::sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2)));
        xk.pose.yaw   = std::atan2(r(1, 0), r(0, 0));
        xk.pose.x = kf.pose.trans.x();
        xk.pose.y = kf.pose.trans.y();
        xk.pose.z = kf.pose.trans.z();
        if (kf.cloud) {
            xk.points.reserve(kf.cloud->size());
            for (const auto& pt : kf.cloud->points) {
                xk.points.emplace_back(pt.x, pt.y, pt.z);
            }
        }
        xz_kfs.push_back(std::move(xk));
    }

    // map.pcd -> 同目录同名 .xzmap
    std::string base = pcd_path;
    const auto slash = base.find_last_of('/');
    const std::string dir = (slash != std::string::npos) ? base.substr(0, slash + 1) : "";
    base = (slash != std::string::npos) ? base.substr(slash + 1) : base;
    const auto dot = base.rfind('.');
    const std::string stem = (dot != std::string::npos) ? base.substr(0, dot) : base;
    const std::string xz_path = dir + stem + ".xzmap";

    long long total = 0;
    if (io::writeXzMap(xz_path, stem, xz_kfs, &total)) {
        std::cout << "[SlamPipeline] xzmap saved: " << xz_path
                  << " (kf=" << xz_kfs.size() << ", pts=" << total << ")" << std::endl;
        return true;
    }
    std::cerr << "[SlamPipeline] xzmap save failed: " << xz_path << std::endl;
    return false;
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

bool SlamPipeline::resetForNewMapping(const std::string& config_path) {
    // 旧 xz_3d_slam 每次切建图都 DELETE_PTR(mapping_ptr_) 再 new 一个, 所以上一轮绝不残留;
    // 我们是单核心实例, 只能显式重建内部模块达到同样效果。
    //
    // 1) 停后端线程 —— **必须用核心自己的停止协议**: 后端线程的 wait 谓词只看"队列非空"
    //    (backendThreadLoop), 唯一退出路径是取出一个 BackendTask::STOP。
    //    实测教训: 只置 m_backend_running=false + notify_all **叫不醒**它, join 会永久阻塞,
    //    整个 ROS 线程卡死在 /slam/mode_switch 里(节点假死: /slam/info 停发)。
    if (m_backend_thread.joinable()) {
        pushBackendTask(BackendTask::STOP);
        // ⚠ **不能 join**: 后端可能正卡在一次 optimize() 里(或定位匹配占满 CPU), join 会把
        // 调用线程(ROS 服务线程)一起挂住 -> 现场表现"切图/切模式卡住"。实测复现过两次。
        // STOP 入队后线程会在本轮结束时自行退出; 它持有的是自己那份模块 shared_ptr,
        // 与随后重建的新实例互不影响。
        m_backend_thread.detach();
        std::cout << "[SlamPipeline] backend thread detached (STOP queued, no join)"
                  << std::endl;
    }
    m_backend_running = false;

    // 2) 丢掉所有模块 + 队列/锚点状态 (= "删掉建图实例")
    m_frontend.reset();
    m_backend.reset();
    m_loop.reset();
    m_localization.reset();
    m_map_mgr.reset();
    m_visual.reset();
    {
        std::lock_guard<std::mutex> lk(m_queue_mutex);
        std::queue<BackendItem> empty;
        std::swap(m_backend_queue, empty);
    }
    {
        std::lock_guard<std::mutex> lk(m_anchor_mutex);
        m_out_anchor_ = PoseStamped();
        m_have_anchor_ = false;
    }
    // 先置 IDLE 再 init: 否则 init() 里那段 "m_mode == LOCALIZATION 就自动载图" 会误触发
    m_mode = PipelineMode::IDLE;
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

double SlamPipeline::getMatchingDegree() const {
    // 定位匹配度: 透传定位流水线的 confidence() (= m_last_match.fitness, 覆盖率 [0,1]).
    // 旧 xz_3d_slam 的 slam/info.matching_degree = localiza.getFitness(), 实机实测为 0.83 这种真值
    // (不是 0) —— 所以这里必须给出真值, 否则监控端看到的匹配度永远是 0.
    return m_localization ? m_localization->confidence() : 0.0;
}

} // namespace rosiwit_slam
