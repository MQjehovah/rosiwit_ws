// src/algorithms/localization/global/scan_context_global_localizer.cpp
#include "algorithms/localization/global/scan_context_global_localizer.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>

namespace rosiwit_slam {

void ScanContextGlobalLocalizer::configure(const GlobalLocalizeConfig& cfg) {
    m_cfg = cfg;
    ScanContext::Config sc_cfg;
    sc_cfg.num_rings = cfg.sc_num_rings;
    sc_cfg.num_sectors = cfg.sc_num_sectors;
    sc_cfg.max_radius = cfg.sc_max_radius;
    sc_cfg.min_radius = cfg.sc_min_radius;
    sc_cfg.z_cap = cfg.sc_z_cap;      // 高度截断: 让"同地点"的查询/关键帧描述子可比
    m_sc = std::make_shared<ScanContext>(sc_cfg);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m_level_r(r, c) = cfg.sc_level_r[r * 3 + c];

    m_gicp.setMaximumIterations(cfg.icp_max_iter);
    m_gicp.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp.setTransformationEpsilon(1e-8);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);
    // 粗配准: 少迭代 + 松对应距离, 只用来淘汰错误候选
    m_gicp_coarse.setMaximumIterations(12);
    m_gicp_coarse.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp_coarse.setTransformationEpsilon(1e-6);
    m_gicp_coarse.setEuclideanFitnessEpsilon(1e-5);
}

bool ScanContextGlobalLocalizer::setMap(const std::string& map_path,
                                        const CloudType::Ptr& global_map) {
    m_ready = false;
    m_db.clear();

    const std::string db_path = map_path + ".scdb";
    if (!loadScanContextDatabase(db_path, m_db) || m_db.empty()) {
        std::cerr << "[SCGlobal] no ScanContext db: " << db_path << std::endl;
        return false;
    }
    return buildMapSide(global_map);
}

bool ScanContextGlobalLocalizer::setMapWithKeyframes(
        const std::string& map_path, const CloudType::Ptr& global_map,
        const std::vector<LocalKeyframe>& keyframes) {
    m_ready = false;
    m_db.clear();
    if (!m_sc) configure(m_cfg);

    const std::string db_path = map_path + ".reloc2.scdb";   // gen2: 切视野建库(与 gen1 关键帧点云口径不同)
    if (loadScanContextDatabase(db_path, m_db) && !m_db.empty()) {
        std::cout << "[SCGlobal] loaded db: " << m_db.size() << " kfs" << std::endl;
        return buildMapSide(global_map);
    }
    if (keyframes.empty()) {
        // 没有 .scdb 也没有关键帧 (例如工作图模式): 无法建库 -> 交给上层退回有界恢复
        std::cerr << "[SCGlobal] no db and no keyframes: " << db_path << std::endl;
        return false;
    }

    // ---- 现场建库: 按关键帧位姿**从全局图切视野**, 与查询同口径 ----
    // 为什么不再直接用 kf.cloud: 那是建图侧的累积点云(voxel + 高度带裁过), 而查询是
    // 单帧扫描; 实测同地点描述子距离 0.75~0.81 (阈值 0.4), 召回不到。切图视野后两边
    // 都是"某点位的 360° 视野", 只剩遮挡差异。
    // 代价: 每地点一次半径搜索+变换+描述子 (~10~30ms), 条目按 sc_db_spacing 抽稀.
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<ScanContextDatabaseEntry> db;
    int skipped = 0;
    CloudType::Ptr base = m_cfg.sc_db_from_map && global_map
        ? downsampleCloud(filterFinite(global_map), std::max(0.4, m_cfg.voxel_resolution))
        : CloudType::Ptr();
    if (base && base->size() >= 1000) {
        pcl::KdTreeFLANN<PointType> kdt;
        kdt.setInputCloud(base);
        // 位姿抽稀: 贪心保证两两间距 >= sc_db_spacing (关键帧本身 1~2m 一个)
        const double min_d = std::max(1.0, m_cfg.sc_db_spacing);
        std::vector<V3D> picked;
        picked.reserve(static_cast<size_t>(m_cfg.sc_db_max_places));
        for (const auto& kf : keyframes) {
            if (static_cast<int>(picked.size()) >= m_cfg.sc_db_max_places) break;
            bool keep = true;
            for (const auto& q : picked) {
                if ((kf.pose.trans - q).norm() < min_d) { keep = false; break; }
            }
            if (!keep) continue;
            CloudType::Ptr sub = cropRadius(kdt, base, kf.pose.trans,
                                            m_cfg.sc_db_radius, 12000);
            if (!sub || sub->size() < 100) { ++skipped; continue; }
            // 世界 -> 车体系 -> 水平系 (与查询端 toBodyFrame + m_level_r 完全同一条链)
            const Eigen::Matrix3d R_bw = kf.pose.rot.transpose();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = m_level_r * R_bw;
            T.block<3, 1>(0, 3) = -m_level_r * R_bw * kf.pose.trans;
            CloudType::Ptr lvl(new CloudType());
            pcl::transformPointCloud(*sub, *lvl, T);
            ScanContextDatabaseEntry e;
            e.id = "place" + std::to_string(db.size());
            e.rot = kf.pose.rot;
            e.trans = kf.pose.trans;
            e.desc = m_sc->makeDescriptor(lvl);
            db.push_back(std::move(e));
            picked.push_back(kf.pose.trans);
        }
    }
    if (db.empty() && !keyframes.empty()) {
        // 全局图不可用 (工作图模式等): 退回"关键帧原始点云"建库, 至少能跑
        std::cerr << "[SCGlobal] map crops unavailable, fallback to keyframe clouds" << std::endl;
        for (const auto& kf : keyframes) {
            if (!kf.cloud || kf.cloud->size() < 50) { ++skipped; continue; }
            ScanContextDatabaseEntry e;
            e.id = "kf" + std::to_string(db.size());
            e.rot = kf.pose.rot;
            e.trans = kf.pose.trans;
            Eigen::Matrix4d T_lvl = Eigen::Matrix4d::Identity();
            T_lvl.block<3, 3>(0, 0) = m_level_r;
            CloudType::Ptr lvl(new CloudType());
            pcl::transformPointCloud(*kf.cloud, *lvl, T_lvl);
            e.desc = m_sc->makeDescriptor(lvl);
            db.push_back(std::move(e));
        }
    }
    if (db.empty()) {
        std::cerr << "[SCGlobal] build db failed: all keyframes too small" << std::endl;
        return false;
    }
    m_db.swap(db);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    std::cout << "[SCGlobal] built db from map crops: " << m_db.size()
              << " places (skipped " << skipped << ", spacing " << m_cfg.sc_db_spacing
              << "m, radius " << m_cfg.sc_db_radius << "m) in " << ms << " ms" << std::endl;
    // 落盘缓存: 下次启动直接 load (设备上如果 map_path 在 /dev/shm 每次重建也无妨)
    if (saveScanContextDatabase(db_path, m_db))
        std::cout << "[SCGlobal] cached db -> " << db_path << std::endl;
    return buildMapSide(global_map);
}

bool ScanContextGlobalLocalizer::buildMapSide(const CloudType::Ptr& global_map) {
    m_map = downsampleCloud(filterFinite(global_map), m_cfg.voxel_resolution);
    if (!m_map || m_map->empty()) return false;
    m_kdt.setInputCloud(m_map);
    // 粗层: 六级精配的候选淘汰放在这上面跑 (点少 => 单次配准 10~20x 便宜)
    m_map_coarse = downsampleCloud(m_map, 0.6);
    if (!m_map_coarse || m_map_coarse->size() < 100) m_map_coarse = m_map;
    m_kdt_coarse.setInputCloud(m_map_coarse);

    m_ready = true;
    std::cout << "[SCGlobal] map ready: " << m_map->size() << " pts, db="
              << m_db.size() << " kfs" << std::endl;
    return true;
}

int ScanContextGlobalLocalizer::localizeTopK(const CloudType::Ptr& scan, int max_k,
                                             std::vector<GlobalCandidate>& out) {
    out.clear();
    if (!m_ready || !m_sc || !scan || scan->empty()) return 0;

    // 查询云先按与建库相同的 voxel 降采样, 再转到水平系算描述子.
    // 为什么必须降采样: 描述子取"每格最大高度/占用", 对点密度敏感 —— 原始扫描 11.7k 点
    // vs 关键帧 0.3m 体素(≈3.7k 点), 占用模式系统性不同, 同地点距离也会被拉到 0.7+.
    CloudType::Ptr scan_desc = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!scan_desc || scan_desc->size() < 10) scan_desc = scan;
    const ScanContext::Descriptor cur = [&] {
        // 查询云同样转到水平系再算描述子 (与建库时的变换一致)
        Eigen::Matrix4d T_lvl = Eigen::Matrix4d::Identity();
        T_lvl.block<3, 3>(0, 0) = m_level_r;
        CloudType::Ptr lvl(new CloudType());
        pcl::transformPointCloud(*scan_desc, *lvl, T_lvl);
        return m_sc->makeDescriptor(lvl);
    }();

    struct Candidate { size_t idx; double rk; double d; double rank; int shift; };
    std::vector<Candidate> cands;
    cands.reserve(m_db.size());
    for (size_t i = 0; i < m_db.size(); ++i)
        cands.push_back({i, ScanContext::ringKeyDistance(m_db[i].desc.ring_key, cur.ring_key),
                         0.0, 0.0, 0});
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.rk < b.rk; });
    const size_t num = std::max<size_t>(1, static_cast<size_t>(cands.size() * m_cfg.sc_candidate_ratio));

    // 查询侧"有东西"的格子数: 用来算重叠率, 防止"只比了几格 → 距离虚低"的假候选拿头名
    int cur_occ = 0;
    for (int r = 0; r < cur.rows(); ++r)
        for (int s = 0; s < cur.cols(); ++s)
            if (cur.data(r, s) > 0.05) ++cur_occ;
    const double occ_ref = std::max(1, cur_occ);

    std::vector<Candidate> hits;
    for (size_t ci = 0; ci < num; ++ci) {
        // 掩码距离: 只比"两边都非空"的格子 (空 vs 有 不罚分), 见 ScanContext::distanceMasked
        const auto mm = ScanContext::distanceMasked(m_db[cands[ci].idx].desc, cur, m_cfg.sc_min_overlap_cells);
        if (mm.compared <= 0) continue;
        if (mm.dist < m_cfg.sc_dist_thresh) {
            Candidate c = cands[ci];
            c.d = mm.dist;
            c.shift = mm.shift;
            // 排序键: 距离 + 低重叠惩罚 (重叠率 0.35 以下按比例加罚), 避免"重叠少→距离虚低"
            const double overlap = static_cast<double>(mm.compared) / occ_ref;
            c.rank = mm.dist + 0.30 * std::max(0.0, 1.0 - overlap / 0.35);
            hits.push_back(c);
        }
    }
    std::sort(hits.begin(), hits.end(), [](const Candidate& a, const Candidate& b) { return a.rank < b.rank; });

    // 召回兜底: sc_dist_thresh 只是"省时间"的预筛, 不能当"这里有没有地方"的判决.
    // 现场同一地点的描述子距离会被"关键帧降采样 vs 原始扫描""货架变动"整体抬高,
    // 一旦全被阈值卡掉就永久召回失败(且以前是静默的). 这里退化为"按 ringkey 取前 K 个",
    // 精度完全交给后面的覆盖率 + GICP 精配 + 歧义门控 —— 它们才是物理判据.
    bool fallback = false;
    if (hits.empty() && !cands.empty()) {
        fallback = true;
        const int kfall = std::max(3, std::min<int>(8, static_cast<int>(cands.size())));
        for (int ci = 0; ci < kfall; ++ci) {
            const auto mm = ScanContext::distanceMasked(m_db[cands[ci].idx].desc, cur,
                                                        m_cfg.sc_min_overlap_cells);
            Candidate c = cands[ci];
            c.d = (mm.compared > 0) ? mm.dist : 1e9;
            c.shift = mm.shift;
            const double overlap = static_cast<double>(mm.compared) / occ_ref;
            c.rank = c.d + 0.30 * std::max(0.0, 1.0 - overlap / 0.35);
            hits.push_back(c);
        }
        std::sort(hits.begin(), hits.end(),
                  [](const Candidate& a, const Candidate& b) { return a.rank < b.rank; });
    }

    // [SCProbe] 召回诊断 (每次查询一条): 没有命中时现场看不出是"描述子差"还是"候选被
    // 覆盖率/精配淘汰". 这几个量就够了: 近邻 ringkey 距离、命中数、最好描述子距离.
    {
        static int probe_n = 0;
        if (++probe_n % 3 == 0) {
            // 描述子量纲自检: 两边 max 值差很多 => 两边点云的 z 口径不一致
            // (例如关键帧做过高度带裁剪, 查询没做) -> 距离永远过不了阈值.
            double qmax = cur.data.size() ? cur.data.maxCoeff() : 0.0;
            double kmax = m_db.empty() ? 0.0 : m_db.front().desc.data.maxCoeff();
            std::cout << "[SCProbe] db=" << m_db.size()
                      << " scan_pts=" << scan->size()
                      << " occ=" << cur_occ
                      << " rk_best=" << (cands.empty() ? -1.0 : cands.front().rk)
                      << " hits=" << hits.size()
                      << " best_d=" << (hits.empty() ? -1.0 : hits.front().d)
                      << " best_rank=" << (hits.empty() ? -1.0 : hits.front().rank)
                      << " thr=" << m_cfg.sc_dist_thresh
                      << " fallback=" << (fallback ? 1 : 0)
                      << " qmax=" << qmax << " kmax=" << kmax << std::endl;
        }
    }

    CloudType::Ptr scan_down = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!scan_down || scan_down->size() < 10) return 0;
    // 精配用的源点云: 比评分用的更稀 (GICP 代价 ~ 源点数 x 目标点数 x 迭代),
    // 位姿精度由最终 coverage 与管线二级精配保证.
    CloudType::Ptr scan_icp = downsampleCloud(
        scan_down, std::max(m_cfg.voxel_resolution, 0.4));
    if (!scan_icp || scan_icp->size() < 10) scan_icp = scan_down;

    // 两级精配 + 提前结束:
    //   阶段1 粗层 (0.6 m) 淘汰绝大多数错误候选; 阶段2 只在粗层通过的候选上做全精度.
    //   以前每个候选都在 0.3 m 全图上跑完整 GICP, 15 个候选 = 10~40 s.
    const int try_limit = std::max(1, max_k * 3);
    // 粗筛门限: 描述子只给"大致哪个地方", 候选位置可能差 1~3m (库是 1m 抽稀的地点位姿).
    // 所以粗筛要松一点(1.2m 容差/0.25 覆盖): 它的作用只是淘汰"完全不对的地方",
    // 真正的精度由后面的 GICP + 覆盖率 + 歧义门控负责.
    const double coarse_gate = std::max(0.20, 0.50 * m_cfg.min_score);
    const double coarse_inlier = 1.2;
    int refined = 0;      // 成功产出候选数
    int fine_tried = 0;   // 实际精配次数 (无论成功), 决定最坏耗时
    // 只精配最靠前的几个候选: 覆盖率打分已经排序, 排在 4 名之后的基本是重复结构.
    // 管线只需要 best/second 做歧义门控, 3 个足够; 5 个会把最坏耗时顶到 5 s 以上.
    const int fine_budget = std::max(1, std::min(3, max_k));
    for (int ci = 0; ci < static_cast<int>(hits.size()) && ci < try_limit; ++ci) {
        const auto& match = m_db[hits[ci].idx];
        const double yaw = 2.0 * M_PI * hits[ci].shift / m_cfg.sc_num_sectors;
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        // 列偏移 = 绕**水平系 z** 的旋转; 存下来的 pose 是"车体系->世界", 所以要把它
        // 共轭回车体系再左乘: R_body(θ) = R_lb^T * Rz(θ) * R_lb (俯装时不能用 Rz 直接乘).
        const Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d R_body = m_level_r.transpose() * Rz * m_level_r;
        guess.block<3,3>(0,0) = (match.rot * R_body).cast<float>();
        guess.block<3,1>(0,3) = match.trans.cast<float>();

        // ---- 阶段1: 用候选位姿直接打覆盖分 ----
        // 不在这里跑粗 GICP: GICP 每次 setInputTarget 都要重算目标协方差,
        // 十几个候选 = 几秒~几十秒 (实测最坏 25 s). 覆盖打分只有 2000 次 1-NN,
        // 单次 ~10 ms, 用 0.8 m 容差足够把 "差 1~2 m" 的错误候选淘汰掉.
        Eigen::Matrix4f guess2 = guess;
        {
            const Eigen::Matrix4d T0 = guess.cast<double>();
            const CoverageStats cs0 = evaluateCoverage(m_kdt, *scan_down, T0,
                                                      coarse_inlier, /*max_points=*/2000);
            if (cs0.coverage < coarse_gate) continue;
        }

        // ---- 阶段2: 全精度 ----
        if (fine_tried >= fine_budget) break;   // 最坏耗时的上界 (自相似场景候选很多)
        ++fine_tried;
        // 精配目标点数上限 1.2 万: GICP 代价 ~ 源点数 x 目标点数, 点太多会把
        // 最坏情况拖到几秒 (实测 6 万点 x 15 候选 = 35 s).
        CloudType::Ptr target = m_cfg.reloc_submap_radius > 0.0
            ? cropRadius(m_kdt, m_map, V3D(match.trans), m_cfg.reloc_submap_radius,
                         /*max_points=*/10000)
            : m_map;
        if (!target || target->size() < 100) target = m_map;
        m_gicp.setInputSource(scan_icp);
        m_gicp.setInputTarget(target);
        CloudType aligned;
        m_gicp.align(aligned, guess2);
        if (!m_gicp.hasConverged()) continue;

        const Eigen::Matrix4d T = m_gicp.getFinalTransformation().cast<double>();
        const CoverageStats cs = evaluateCoverage(m_kdt, *scan_down, T, m_cfg.voxel_resolution);
        if (cs.coverage < m_cfg.min_score) continue;

        GlobalCandidate out_c;
        out_c.pose = matrixToPose(T);
        out_c.score = cs.coverage;
        out_c.id = static_cast<int>(hits[ci].idx);
        out.push_back(std::move(out_c));
        ++refined;
        // 高置信命中不必再试其它候选; 但至少给 2 个, 否则管线拿不到"次优分数",
        // 自相似场景里的歧义门控 (`best - second < margin`) 就失效了.
        if (cs.coverage >= 0.88 && refined >= std::min(2, max_k)) break;

    }
    std::sort(out.begin(), out.end(),
              [](const GlobalCandidate& a, const GlobalCandidate& b) { return a.score > b.score; });
    if (static_cast<int>(out.size()) > max_k) out.resize(max_k);
    return static_cast<int>(out.size());
}

bool ScanContextGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    std::vector<GlobalCandidate> cands;
    if (localizeTopK(scan, 1, cands) < 1) return false;
    out = cands.front();
    return true;
}

} // namespace rosiwit_slam
