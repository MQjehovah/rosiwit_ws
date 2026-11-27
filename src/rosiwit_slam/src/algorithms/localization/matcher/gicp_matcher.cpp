// src/algorithms/localization/matcher/gicp_matcher.cpp
#include "algorithms/localization/matcher/gicp_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <chrono>
#include <cstdlib>

namespace rosiwit_slam {

// 诊断探针开关 (与 localization_pipeline.cpp 同一环境变量, 默认静默)
static const bool kLocProbe = std::getenv("ROSIWIT_LOC_PROBE") != nullptr;

void GicpMatcher::configure(const ScanMatcherConfig& cfg) {
    m_cfg = cfg;
    if (!m_gicp) return;   // 子类自带求解器 (如 LioMatcher), 不配置引擎
    m_gicp_lsq = dynamic_cast<fast_gicp::LsqRegistration<PointType, PointType>*>(m_gicp.get());
    m_gicp->setMaximumIterations(m_cfg.max_iterations);
    m_gicp->setMaxCorrespondenceDistance(m_cfg.max_correspondence_dist);
    m_gicp->setTransformationEpsilon(m_cfg.transformation_epsilon);
    // 2026-11-26: 关闭 fitness epsilon —— PCL 的收敛判据在它 >0 时每轮迭代都再扫
    //   一遍近邻算 MSE (迭代成本翻倍), 而位姿收敛由 transformation_epsilon 判定即可。
    m_gicp->setEuclideanFitnessEpsilon(1e-6);   // 2026-11-26 20:5x: 回到原值 (静止自转回归)
}

void GicpMatcher::setTarget(const CloudType::Ptr& target) {
    m_target = downsampleCloud(target, m_cfg.voxel_resolution);
    if (!m_target) m_target.reset(new CloudType());
    // 退化目标守卫: 点数过少时 GICP 协方差估计无意义且会段错误, 置空让 match() 走拒绝分支
    if (m_target->size() < static_cast<size_t>(std::max(1, m_cfg.min_match_points)))
        m_target->clear();
    if (!m_target->empty()) m_kdtree.setInputCloud(m_target);
}

bool GicpMatcher::align(const CloudType::Ptr& src, const Eigen::Matrix4d& guess_T,
                        Eigen::Matrix4d& T_out, int& iterations) {
    Eigen::Matrix4f g = guess_T.cast<float>();
    // 调试: ROSIWIT_GICP_LM_TRACE=N -> 前 N 次对齐打印 fast_gicp 的逐轮 LM 表 (到 stderr)。
    // 2026-11-27: 用它确认真实扫描下求解器第 ~4 轮就到了对应关系切换造成的步长噪声底 (1~5mm),
    // 之后的迭代步长不再变小 —— 按 1e-6 的"收敛"判据永远跑满 max_iterations。
    static int lm_trace_left = [] {
        const char* e = std::getenv("ROSIWIT_GICP_LM_TRACE");
        return e ? std::atoi(e) : 0;
    }();
    if (m_gicp_lsq) m_gicp_lsq->setDebugPrint(lm_trace_left-- > 0);
    m_gicp->setInputSource(src);
    // 目标指针没变就别重设: PCL 1.8 setInputTarget 无条件置脏 target_cloud_updated_
    // (每帧重建 ~6k 点目标 KD 树) 且 GICP 子类再 reset target_covariances_
    // (每帧重算 20NN 协方差) —— 实测占每帧 37ms 的绝大部分。目标缓存命中时
    // 指针跨帧不变, PCL 内部缓存(树+协方差)自动复用, 结果逐位一致。
    if (m_gicp->getInputTarget() != m_target) m_gicp->setInputTarget(m_target);
    CloudType aligned;
    m_gicp->align(aligned, g);
    iterations = m_gicp->getMaximumIterations();
    // fast_gicp: iterations = 实际执行的外层迭代次数 (收敛了还是跑满上限 —— 求解器成本的主项)
    if (m_name == "fast_gicp" && m_gicp_lsq) iterations = m_gicp_lsq->performedIterations();
    // fast_gicp 的 LsqRegistration 从不设置 converged_ (基类初值恒 false) —— 不能用它
    // 判失败; 只校验解的有限性, 收敛与否交给评分 (覆盖率/残差) 判。
    if (m_name != "fast_gicp" && !m_gicp->hasConverged()) return false;
    T_out = m_gicp->getFinalTransformation().cast<double>();
    return T_out.allFinite();
}

bool GicpMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                        MatchResult& out) {
    // [MatchProbe] 每 512 次打一条分解计时 —— 定位输出 10Hz(雷达 15.8Hz)瓶颈定位用
    static int probe_n = 0;
    const bool probe = (++probe_n % 512 == 0);
    const auto t0 = std::chrono::steady_clock::now();
    double ms_ds = 0, ms_df = 0, ms_gicp = 0, ms_eval = 0;
    out.ms_solver = 0.0;
    out.ms_eval = 0.0;
    auto ms_since = [](const std::chrono::steady_clock::time_point& t) {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t).count();
    };

    out = MatchResult();
    if (!m_target || m_target->empty()) return false;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!src || src->empty()) return false;
    out.visible = static_cast<int>(src->size());   // 动态滤除前的可见点数
    CloudType::Ptr src_all = src;                  // 诚实覆盖率用(不剔除)
    if (probe) ms_ds = ms_since(t0);
    const Eigen::Matrix4d guess_T = poseToMatrix(guess);
    // 高度带裁剪 (map 系): 去掉地面/贴地点, 上界默认 10000 = 不裁天花板.
    // 地图没有地面点, 扫描地面点参与配准只会永远"配不上", 把 coverage 分母撑大
    // (实测地面约占降采样后源点的 13%, 匹配度因此被压低 ~10 个百分点).
    // 注意: 裁剪按**猜测位姿**投到 map 系, 与后面配准用的是同一个 T, 不会引入额外误差.
    if (m_cfg.match_z_filter && m_cfg.match_z_max > m_cfg.match_z_min) {
        CloudType::Ptr kept(new CloudType());
        kept->reserve(src->size());
        if (m_cfg.crop_r_valid) {
            // 安装外参已注入: 转到 base_footprint 系裁剪 (地面 z=0), 与位姿 z 无关.
            // 离地高度 = (R·p + t).z: 平移项 t 是雷达安装高度 (本机 0.844m), 不能省 ——
            // 省了裁剪面就在 0.99m 而不是 0.15m (2026-11-23).
            const Eigen::Matrix3d& R = m_cfg.crop_r;
            const double z_off = m_cfg.crop_t.z();
            for (const auto& p : src->points) {
                const double zf = R(2, 0) * p.x + R(2, 1) * p.y + R(2, 2) * p.z + z_off;
                if (zf > m_cfg.match_z_min && zf < m_cfg.match_z_max) kept->push_back(p);
            }
        } else {
            // 兜底(无外参): 按 guess 投到 map 系; 依赖位姿 z, 仅用于单测/仿真.
            const Eigen::Matrix3d Rz = guess_T.block<3, 3>(0, 0);
            const double tz = guess_T(2, 3);
            for (const auto& p : src->points) {
                const double wz = Rz(2, 0) * p.x + Rz(2, 1) * p.y + Rz(2, 2) * p.z + tz;
                if (wz > m_cfg.match_z_min && wz < m_cfg.match_z_max) kept->push_back(p);
            }
        }
        src = kept;
    }
    if (m_cfg.dynamic_filter && m_cfg.dynamic_dist > 0) {
        // 源点在雷达系, 目标地图在世界系; 必须用 guess 变换后再查地图.
        src = filterByMapProximity(src, m_kdtree, guess_T, m_cfg.dynamic_dist);
        if (!src || src->empty()) return false;
    }
    if (probe) ms_df = ms_since(t0) - ms_ds;

    // 退化扫描守卫(崩溃根因): 16 点扫描进 PCL GICP = 段错误, 见 ScanMatcherConfig 注释
    if (src->size() < static_cast<size_t>(std::max(1, m_cfg.min_match_points))) return false;

    const auto t1 = std::chrono::steady_clock::now();
    Eigen::Matrix4d T;
    // 求解核心可替换 (GICP 引擎 / LioMatcher 的 iEKF 点到面); 评分在下面共用
    if (!align(src, guess_T, T, out.iterations)) {
        out.converged = false;
        return false;
    }
    ms_gicp = ms_since(t1);                    // 每帧都记: 帧耗时分解 (FramePerf) 要用
    out.ms_solver = ms_gicp;
    out.pose = matrixToPose(T, guess.time);
    const auto t2 = std::chrono::steady_clock::now();
    const CoverageStats cs = evaluateCoverage(m_kdtree, *src, T, m_cfg.inlier_dist);
    ms_eval = ms_since(t2);
    out.ms_eval = ms_eval;
    out.fitness = cs.coverage;
    // 退化扫描打折 (2026-11-23): 参与配准的点太少时 "覆盖率" 没有统计意义 ——
    // 被近处物体包围 (实车: 雷达四面 0.6~3.7m 全是遮挡, 降采样后仅 240 点) 时任何位姿
    // 都能拿 1.000, 节点据此判 "localized after 3 healthy frames" 并把位姿冻住.
    // 按点数线性下调, 让分数诚实; 正常扫描 (4400~5100 点) 不受影响.
    if (m_cfg.min_coverage_points > 0 &&
        src->size() < static_cast<size_t>(m_cfg.min_coverage_points) &&
        cs.inliers < m_cfg.min_inliers_full_score) {   // 绝对内点够多 -> 不打折
        out.fitness = cs.coverage * static_cast<double>(src->size()) /
                      static_cast<double>(m_cfg.min_coverage_points);
    }
    // [FitSweep] 低分帧诊断: 以当前位姿为中心, 在 yaw/roll/pitch 上小范围扫描覆盖率,
    // 判断偏差主轴 —— 哪个轴能把覆盖率救回来, 偏的就是哪个轴 (约定/时序/安装角).
    if (kLocProbe && out.fitness < 0.6 && m_target && !m_target->empty()) {
        static int sweep_n = 0;
        if (++sweep_n % 5 == 0) {
            std::cout << "[FitSweep] base=" << out.fitness;
            for (double dy : {-10.0, -5.0, -2.0, 0.0, 2.0, 5.0, 10.0}) {
                Eigen::Matrix4d Tp = T;
                Tp.block<3,3>(0,0) =
                    Eigen::AngleAxisd(dy * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                    * T.block<3,3>(0,0);
                std::cout << " yaw" << dy << "="
                          << evaluateCoverage(m_kdtree, *src, Tp, m_cfg.inlier_dist).coverage;
            }
            for (double dx : {-0.5, -0.25, 0.25, 0.5}) {
                Eigen::Matrix4d Tp = T;
                Tp(0,3) += dx;
                std::cout << " x" << dx << "="
                          << evaluateCoverage(m_kdtree, *src, Tp, m_cfg.inlier_dist).coverage;
            }
            for (double dy : {-0.5, -0.25, 0.25, 0.5}) {
                Eigen::Matrix4d Tp = T;
                Tp(1,3) += dy;
                std::cout << " y" << dy << "="
                          << evaluateCoverage(m_kdtree, *src, Tp, m_cfg.inlier_dist).coverage;
            }
            for (double dz : {-0.2, 0.2}) {
                Eigen::Matrix4d Tp = T;
                Tp(2,3) += dz;
                std::cout << " z" << dz << "="
                          << evaluateCoverage(m_kdtree, *src, Tp, m_cfg.inlier_dist).coverage;
            }
            std::cout << std::endl;
        }
    }
    // 诚实覆盖率: 同一阈值, 但分母是未做动态剔除的全部降采样点
    if (src_all && src_all != src)
        out.fitness_all = evaluateCoverage(m_kdtree, *src_all, T, m_cfg.inlier_dist).coverage;
    else
        out.fitness_all = cs.coverage;
    out.rmse = cs.mean_dist;
    out.inliers = cs.inliers;
    out.w_support = cs.w_support;   // P4: 加权锚定判据 (已确认结构支撑比例)
    out.correspondences = cs.total;
    out.residual = evaluatePoint2PlaneResidual(m_kdtree, *src, T,
                                               m_cfg.max_correspondence_dist);
    // ---- 方向可观测性 (2026-11-25): gicp 本身不提供 JtJ, 所以 degeneracy 相关
    // 判据在 gicp 下一直是空转(日志里 eig 恒为 1.000 = 默认值). 这里用内点的局部
    // 平面法向补算一个 6x6 信息矩阵并上报, 先用于诊断/标定; 不置 has_hessian
    // (原因见下面的注释). 有效平面点不足时保持原行为.
    Eigen::Matrix<double, 6, 6> H_obs = Eigen::Matrix<double, 6, 6>::Zero();
    int obs_used = 0;
    {
        Eigen::Matrix<double, 6, 1> lam;
        Eigen::Matrix<double, 6, 6> vecs;
        double ratio = 1.0;
        int dirs = 0;
        if (evaluateObservability(m_kdtree, *src, T,
                                  m_cfg.max_correspondence_dist, m_cfg.inlier_dist,
                                  lam, vecs, ratio, dirs, &H_obs, &obs_used)) {
            // **故意不置 out.has_hessian** (2026-11-25 回归实测):
            // 置位后 pipeline 的 degeneracy_partial_update 会立刻按
            // degeneracy_ratio_soft(0.05) 削最弱方向; 而本估计器在正常帧算出的
            // λmin/λmax 只有 ~4e-3 (>=0.05 的帧为 0%), 于是每帧都把修正压到 ~8%,
            // 位姿跟不上 -> SUCCESS 99.8% 掉到 85.6%. 两者量纲/加权口径不同,
            // 要启用必须先用现场数据重标定 soft 门限, 不能直接打开.
            out.hess_eigvals = lam;
            out.hess_eigvecs = vecs;
            out.smallest_eig_ratio = ratio;
            out.degenerate_dirs = dirs;
            out.degenerate = (ratio < 2e-3);
        }
    }
    if (probe) {
        std::cout << "[MatchProbe] match_index=" << probe_n
                  << " scan_points=" << scan->size()
                  << " source_points=" << src->size()
                  << " target_points=" << m_target->size()
                  << " downsample_ms=" << ms_ds
                  << " dynamic_filter_ms=" << ms_df
                  << " gicp_ms=" << ms_gicp
                  << " evaluate_ms=" << ms_eval
                  << " total_ms=" << ms_ds + ms_df + ms_gicp + ms_eval << std::endl;
    }
    const double s = out.residual > 0 ? out.residual * out.residual
                                      : (out.rmse > 0 ? out.rmse * out.rmse : 1.0);
    out.cov = Eigen::Matrix<double, 6, 6>::Identity() * s;
    // ★方向协方差 (2026-11-26 根因修复): 匹配是"预测的函数", 不是绝对观测 ——
    //   在弱观测方向 (对称走廊的 yaw) 它几乎不含信息, 各向同性协方差却把它当
    //   全信息用 (增益~0.95), 闭环积分出恒定的匹配偏置 -> 重定位后 yaw 自转/子图糊。
    //   旋转块用内点几何的真实信息 H_rot = Σ((q-t)×n)(...)ᵀ/N:
    //     cov_rot = (residual²/N)·H_rot⁻¹   [rad²], 弱方向自动放大, 增益随之趋零;
    //   信息下限取 λmax·1e-3 (防无限大), 与 rot_cov_floor 配合。
    {
        const Eigen::Matrix3d H_rot = H_obs.block<3, 3>(0, 0);
        if (obs_used >= 30 && H_rot.allFinite()) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_rot(H_rot);
            if (es_rot.info() == Eigen::Success) {
                const Eigen::Vector3d lv = es_rot.eigenvalues();   // 升序
                const double lm = lv(2);
                if (lm > 1e-12) {
                    Eigen::Vector3d cv;
                    for (int i = 0; i < 3; ++i) {
                        const double li = std::max(lv(i), lm * 1e-3);
                        cv(i) = (s / static_cast<double>(obs_used)) / li;
                    }
                    const Eigen::Matrix3d C =
                        es_rot.eigenvectors() * cv.asDiagonal() *
                        es_rot.eigenvectors().transpose();
                    for (int i = 0; i < 3; ++i) {
                        out.cov(i, i) = std::max(C(i, i), s);
                    }
                }
            }
        }
    }
    out.converged = true;
    return true;
}

double GicpMatcher::evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) {
    if (!m_target || m_target->empty()) return -1.0;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    return evaluatePoint2PlaneResidual(m_kdtree, *src, poseToMatrix(pose),
                                       m_cfg.max_correspondence_dist);
}

} // namespace rosiwit_slam
