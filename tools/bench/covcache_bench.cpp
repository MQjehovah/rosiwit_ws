// covcache_bench: 目标协方差复用缓存的正确性与收益 (真实地图)。
//   covcache_bench <map.overlay.pcd> <scan.pcd> [threads=4]
// 1) 数值: 命中的协方差 vs 现算, 最大差;  2) 配准: 同一帧扫描, 缓存版 vs 全量版的位姿差;
// 3) 耗时: 冷/热 setInputTarget;           4) clearTargetCovarianceCache 之后确实全部未命中。
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>

using P = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<P>;
using Covs = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;

static double ms_since(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}
static Cloud::Ptr voxel(const Cloud::Ptr& in, float leaf) {
    pcl::VoxelGrid<P> vg; vg.setLeafSize(leaf, leaf, leaf); vg.setInputCloud(in);
    Cloud::Ptr out(new Cloud()); vg.filter(*out); return out;
}
static Cloud::Ptr crop(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map, double cx, double cy, double r) {
    Cloud::Ptr c(new Cloud());
    for (const auto& p : map->points) {
        if (std::hypot(p.x - cx, p.y - cy) > r) continue;
        P q; q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = p.intensity;
        q.normal_x = q.normal_y = q.normal_z = 0; q.curvature = 0;
        c->push_back(q);
    }
    return voxel(c, 0.3f);
}
struct Run { double ms; Covs covs; size_t hits, miss, entries, redo; };
static Run build(const Cloud::Ptr& tgt, bool reuse, int th) {
    fast_gicp::setTargetCovarianceReuseEnabled<P>(reuse);
    fast_gicp::FastGICP<P, P> g;
    g.setNumThreads(th); g.setMaximumIterations(12); g.setMaxCorrespondenceDistance(1.2); g.setTransformationEpsilon(5e-3);
    auto t0 = std::chrono::steady_clock::now();
    g.setInputTarget(tgt);
    Run r; r.ms = ms_since(t0); r.covs = g.getTargetCovariances();
    fast_gicp::targetCovCacheStats<P>(r.hits, r.miss, r.entries, &r.redo);
    return r;
}
static Eigen::Matrix4f align(const Cloud::Ptr& tgt, const Cloud::Ptr& src, const Eigen::Matrix4f& guess, bool reuse, int th, int& iters) {
    fast_gicp::setTargetCovarianceReuseEnabled<P>(reuse);
    fast_gicp::FastGICP<P, P> g;
    g.setNumThreads(th); g.setMaximumIterations(12); g.setMaxCorrespondenceDistance(1.2); g.setTransformationEpsilon(5e-3);
    g.setInputTarget(tgt); g.setInputSource(src);
    Cloud out; g.align(out, guess); iters = g.performedIterations();
    return g.getFinalTransformation();
}
static int64_t key_of(const P& p) { return fast_gicp::targetCovKey(p.x, p.y, p.z); }

int main(int argc, char** argv) {
    const int th = argc > 3 ? std::atoi(argv[3]) : 4;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    Cloud::Ptr scan(new Cloud());
    pcl::io::loadPCDFile(argv[1], *map); pcl::io::loadPCDFile(argv[2], *scan);
    Cloud::Ptr src = voxel(scan, 0.3f);
    {
        const Eigen::Matrix3d R = (Eigen::AngleAxisd(1.569, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.020, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(0.687, Eigen::Vector3d::UnitX())).toRotationMatrix();
        Cloud::Ptr kept(new Cloud());
        for (const auto& p : src->points) {
            const double z = R(2, 0) * p.x + R(2, 1) * p.y + R(2, 2) * p.z + 0.914;
            if (z > 0.15) kept->push_back(p);
        }
        src = kept;
    }
    Cloud::Ptr T1 = crop(map, 5.98, -1.61, 30.0);
    Cloud::Ptr T2 = crop(map, 10.98, -1.61, 30.0);
    // 300 个"新提交的体素": 取 T2 里的点, 在 xy 上偏 10~20cm (落进别的体素), z 不动
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> off(0.10f, 0.20f), dir(0.f, 6.283f);
    Cloud::Ptr novel(new Cloud());
    for (int i = 0; i < 300; ++i) {
        const P& p = T2->points[rng() % T2->size()];
        if (std::hypot(p.x - 10.98, p.y + 1.61) > 12.f) { --i; continue; }
        P q = p; const double a = dir(rng), d = off(rng);
        q.x += (float)(d * std::cos(a)); q.y += (float)(d * std::sin(a));
        novel->push_back(q);
    }
    *T2 += *novel;
    std::printf("目标 T1 %zu 点; T2 %zu 点 (窗口平移 5m + 新增 %zu 点); 源 %zu 点; %d 线程\n", T1->size(), T2->size(), novel->size(), src->size(), th);

    fast_gicp::clearTargetCovarianceCache<P>();
    Run A = build(T1, false, th);            // 全量 (不复用)
    Run B = build(T1, true, th);             // 缓存空: 全量 + 入表
    Run C = build(T1, true, th);             // 全命中
    auto maxdiff = [](const Covs& a, const Covs& b) { double m = 0; for (size_t i = 0; i < a.size(); ++i) m = std::max(m, (a[i] - b[i]).cwiseAbs().maxCoeff()); return m; };
    std::printf("[T1] 全量 %.0f ms | 缓存空 %.0f ms (命中 %zu 未命中 %zu, 入表后 %zu 条) | 再来一次 %.0f ms (命中 %zu 未命中 %zu)\n",
                A.ms, B.ms, B.hits, B.miss, B.entries, C.ms, C.hits, C.miss);
    std::printf("     数值: 全量 vs 缓存空 最大差 %.2e; 命中重构 vs 现算 最大差 %.2e\n", maxdiff(A.covs, B.covs), maxdiff(B.covs, C.covs));

    Run D = build(T2, false, th);            // 全量
    Run W = build(T2, true, th);             // 命中 T1 里有的
    std::printf("[T2] 全量 %.0f ms | 复用 %.0f ms (命中 %zu = %.1f%%, 其中近邻变了重算 %zu, 新位置 %zu, 入表后 %zu 条)\n",
                D.ms, W.ms, W.hits, 100.0 * W.hits / T2->size(), W.redo, W.miss, W.entries);
    {
        std::map<int64_t, int> k1; for (int i = 0; i < (int)T1->size(); ++i) k1[key_of(T1->points[i])] = i;
        double m_same = 0, m_all = 0; size_t n_same = 0, n_stale = 0;
        for (int i = 0; i < (int)T2->size(); ++i) {
            auto it = k1.find(key_of(T2->points[i]));
            if (it == k1.end()) continue;
            ++n_same;
            const double d = (W.covs[i] - D.covs[i]).cwiseAbs().maxCoeff();
            m_same = std::max(m_same, d);
            bool near_novel = false;
            for (const auto& q : novel->points) if (std::hypot(q.x - T2->points[i].x, q.y - T2->points[i].y, q.z - T2->points[i].z) < 0.5) { near_novel = true; break; }
            if (d > 1e-9) ++n_stale;
            (void)near_novel;
        }
        for (int i = 0; i < (int)T2->size(); ++i) m_all = std::max(m_all, (W.covs[i] - D.covs[i]).cwiseAbs().maxCoeff());
        std::printf("     T2 里与 T1 同位置的 %zu 点: 协方差最大差 %.2e; 与现算差 >1e-9 的 %zu 点 (邻域里来了新点, 沿用旧值)\n", n_same, m_same, n_stale);
        std::printf("     全部 %zu 点最大差 %.2e  (复用路径与全量计算应当一致)\n", T2->size(), m_all);
    }
    {
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); guess(0, 3) = 10.0f; guess(1, 3) = -2.0f; guess(2, 3) = 0.98f;
        int it1 = 0, it2 = 0, it0 = 0;
        const Eigen::Matrix4f Tref = align(T1, src, guess, false, th, it0);     // 参照: 没有新点的干净地图
        const Eigen::Matrix4f Tf = align(T2, src, guess, false, th, it1);      // T2 全量
        const Eigen::Matrix4f Tc = align(T2, src, guess, true, th, it2);       // T2 复用
        auto delta = [](const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
            const double dp = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
            const Eigen::Matrix3f dR = a.block<3, 3>(0, 0) * b.block<3, 3>(0, 0).transpose();
            return std::make_pair(dp, std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5))) * 57.2957795);
        };
        const auto d_f_c = delta(Tf, Tc), d_ref_f = delta(Tref, Tf), d_ref_c = delta(Tref, Tc);
        std::printf("[配准] 轮数 T1全量 %d / T2全量 %d / T2复用 %d\n", it0, it1, it2);
        std::printf("     T2 复用 vs T2 全量: %.2e m, %.2e deg   (应当 ~0)\n", d_f_c.first, d_f_c.second);
        std::printf("     T2 全量 vs 干净地图 T1: %.3f m, %.3f deg | T2 复用 vs T1: %.3f m, %.3f deg   (新点对配准的扰动, 两种算法应当一样)\n",
                    d_ref_f.first, d_ref_f.second, d_ref_c.first, d_ref_c.second);
    }
    fast_gicp::clearTargetCovarianceCache<P>();
    Run E = build(T1, true, th);
    std::printf("[清空] clearTargetCovarianceCache 后 T1: 命中 %zu 未命中 %zu (应全未命中)\n", E.hits, E.miss);
    return 0;
}
