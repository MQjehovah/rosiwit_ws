// tilt_probe: 一帧的匹配对横滚/俯仰有多大约束力 —— 回答"匹配器的 roll/pitch 解能不能当测量用"。
//   tilt_probe <dump_dir> [theta_max_deg=8] [step_deg=1] [threads=4]
// dump_dir: scan.pcd (雷达系) + poses.txt (guess_trans / guess_rot / target) —— 节点 [DumpSeq] 的格式。
//
// 做三件事:
//  1) 从猜测出发, 用节点的设置跑 fast_gicp, 报告解相对"水平"姿态的倾斜 (与 updateMapTilt 同口径);
//  2) 倾斜代价曲线: 沿解的倾斜轴把倾斜钉在 θ, 只优化 x/y/z/偏航 到收敛, 报告代价 —— 谷深 = 约束力;
//  3) 在最优 θ 处, 由求解器自己的 Hessian 算 roll/pitch 的边缘不确定度 (雷达中心参数化, 其余自由度边缘化掉),
//     再按实测残差标定 (GICP 的噪声模型认为点到面残差 σ=3.2cm, 实际因 0.3m 体素量化远大于此)。
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

using P = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<P>;
using M3 = Eigen::Matrix3d;
using V3 = Eigen::Vector3d;
using Iso = Eigen::Isometry3d;

static const double kPhi = 0.687, kThe = 0.020, kPsi = 1.569;   // URDF: base_link<-lidar rpy
static M3 mountR() {
    return (Eigen::AngleAxisd(kPsi, V3::UnitZ()) * Eigen::AngleAxisd(kThe, V3::UnitY()) * Eigen::AngleAxisd(kPhi, V3::UnitX())).toRotationMatrix();
}
// 姿态 R (雷达->map) 分解: R = exp(v) * Rz(yaw) * R_mount ; updateMapTilt 取 v 的 x/y 分量当倾斜
static void decompose(const M3& R, double& yaw, V3& v) {
    const M3 Rb = R * mountR().transpose();
    yaw = std::atan2(Rb(1, 0), Rb(0, 0));
    const M3 level = Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix() * mountR();
    const Eigen::AngleAxisd aa(R * level.transpose());
    v = aa.axis() * aa.angle();
}
static M3 skew(const V3& p) { M3 s; s << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0; return s; }
static Cloud::Ptr voxel(const Cloud::Ptr& in, float leaf) {
    pcl::VoxelGrid<P> vg; vg.setLeafSize(leaf, leaf, leaf); vg.setInputCloud(in);
    Cloud::Ptr out(new Cloud()); vg.filter(*out); return out;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: tilt_probe <dump_dir> [theta_max_deg] [step_deg] [threads]\n"); return 1; }
    const std::string dir = argv[1];
    const double th_max = argc > 2 ? std::atof(argv[2]) : 8.0;
    const double th_step = argc > 3 ? std::atof(argv[3]) : 1.0;
    const int th = argc > 4 ? std::atoi(argv[4]) : 4;
    std::string tgt_path;
    double gt[3] = {0, 0, 0}, gr[4] = {0, 0, 0, 1};
    {
        std::ifstream f(dir + "/poses.txt");
        std::string k;
        while (f >> k) {
            if (k == "target") f >> tgt_path;
            else if (k == "guess_trans") f >> gt[0] >> gt[1] >> gt[2];
            else if (k == "guess_rot") f >> gr[0] >> gr[1] >> gr[2] >> gr[3];
            else { std::string s; std::getline(f, s); }
        }
    }
    if (tgt_path.empty() || (gt[0] == 0 && gt[1] == 0 && gt[2] == 0 && gr[0] == 0 && gr[1] == 0 && gr[2] == 0)) {
        std::printf("poses.txt 里缺 target 或 guess\n"); return 1;
    }
    Cloud::Ptr tgt(new Cloud()), scan(new Cloud());
    pcl::io::loadPCDFile(tgt_path, *tgt);
    pcl::io::loadPCDFile(dir + "/scan.pcd", *scan);
    // 匹配器预处理: 0.3m 体素 -> 按 URDF 投到 base 系去地面 (z<0.15)
    Cloud::Ptr src = voxel(scan, 0.3f);
    const size_t n_vox = src->size();
    {
        const M3 R = mountR(); const V3 t(0.414, 0.019, 0.914);
        Cloud::Ptr kept(new Cloud());
        for (const auto& p : src->points) {
            const V3 q = R * V3(p.x, p.y, p.z) + t;
            if (q.z() > 0.15) kept->push_back(p);
        }
        src = kept;
    }
    double zmin = 1e9, zmax = -1e9, rmax = 0;
    for (const auto& p : src->points) { zmin = std::min(zmin, (double)p.z); zmax = std::max(zmax, (double)p.z); rmax = std::max(rmax, (double)std::hypot(p.x, p.y)); }
    std::printf("帧 %s: 目标 %zu 点; 扫描 %zu -> 体素 %zu -> 去地面 %zu 点 (雷达系 z %.2f~%.2f m, 最远 %.1f m)\n",
                dir.substr(dir.rfind('/') + 1).c_str(), tgt->size(), scan->size(), n_vox, src->size(), zmin, zmax, rmax);

    const M3 Rm = mountR();
    auto guess_iso = [&]() {
        Iso x = Iso::Identity();
        x.linear() = Eigen::Quaterniond(gr[3], gr[0], gr[1], gr[2]).toRotationMatrix();
        x.translation() = V3(gt[0], gt[1], gt[2]);
        return x;
    };

    // ---- 1) 节点设置下的求解 ----
    Iso x_ref;
    int iters = 0;
    V3 v_ref = V3::Zero();
    double yaw_ref = 0;
    {
        fast_gicp::FastGICP<P, P> g;
        g.setNumThreads(th); g.setMaximumIterations(12); g.setMaxCorrespondenceDistance(1.2); g.setTransformationEpsilon(5e-3);
        g.setInputTarget(tgt); g.setInputSource(src);
        Cloud out;
        g.align(out, guess_iso().matrix().cast<float>());
        const Eigen::Matrix4d T = g.getFinalTransformation().cast<double>();
        x_ref = Iso::Identity(); x_ref.linear() = T.block<3, 3>(0, 0); x_ref.translation() = T.block<3, 1>(0, 3);
        iters = g.performedIterations();
        decompose(x_ref.linear(), yaw_ref, v_ref);
        std::printf("节点设置 (12 轮, 5mm 停机) 从水平猜测出发: 迭代 %d 轮, 解的倾斜 %.3f° = (vx %.3f°, vy %.3f°); 位置修正 (%.0f, %.0f, %.0f) mm\n",
                    iters, std::hypot(v_ref.x(), v_ref.y()) * 180 / M_PI, v_ref.x() * 180 / M_PI, v_ref.y() * 180 / M_PI,
                    1000 * (x_ref.translation().x() - gt[0]), 1000 * (x_ref.translation().y() - gt[1]), 1000 * (x_ref.translation().z() - gt[2]));
    }
    V3 axis = V3(v_ref.x(), v_ref.y(), 0);
    if (axis.norm() < 1e-6) axis = V3::UnitX(); else axis.normalize();

    // ---- 2) 倾斜代价曲线 ----
    fast_gicp::FastGICP<P, P> g;
    g.setNumThreads(th); g.setTransformationEpsilon(1e-9); g.setMaxCorrespondenceDistance(1.2);
    g.setInputTarget(tgt); g.setInputSource(src);
    // evaluateCost 不做预处理 (协方差在 align/computeTransformation 里算): 先跑 1 轮 align 把协方差建好
    {
        Cloud warm;
        g.setMaximumIterations(1);
        g.align(warm, x_ref.matrix().cast<float>());
    }
    pcl::KdTreeFLANN<P> kdtree; kdtree.setInputCloud(tgt);
    auto count_corr = [&](const Iso& x, double& rms) {
        int n = 0; double s2 = 0;
        std::vector<int> idx(1); std::vector<float> d2(1);
        for (const auto& p : src->points) {
            const V3 q = x * V3(p.x, p.y, p.z);
            P qq; qq.x = (float)q.x(); qq.y = (float)q.y(); qq.z = (float)q.z(); qq.intensity = 0;
            if (kdtree.nearestKSearch(qq, 1, idx, d2) > 0 && d2[0] < 1.2 * 1.2) { ++n; s2 += d2[0]; }
        }
        rms = n ? std::sqrt(s2 / n) : 0;
        return n;
    };
    // 自由参数 q=(dψ, dp): 姿态 exp(θâ)·Rz(ψ)·R_mount, 位置 p; 左扰动 d = B q (绕地图原点参数化)
    auto solve_fixed_tilt = [&](double theta, Iso& x) {
        double lam = 1e-6;
        Eigen::Matrix<double, 6, 6> H; Eigen::Matrix<double, 6, 1> b;
        double cost = g.evaluateCost(x.matrix().cast<float>(), &H, &b);
        const M3 Rt = (Eigen::AngleAxisd(theta, axis)).toRotationMatrix();
        const V3 u = Rt * V3::UnitZ();                         // 倾斜后的"竖直" = 偏航轴
        for (int it = 0; it < 80; ++it) {
            const V3 p = x.translation();
            Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity();
            J.block<3, 3>(3, 0) = skew(p);
            const Eigen::Matrix<double, 6, 6> Hp = J.transpose() * H * J;
            const Eigen::Matrix<double, 6, 1> bp = J.transpose() * b;
            Eigen::Matrix<double, 6, 4> B = Eigen::Matrix<double, 6, 4>::Zero();
            B.block<3, 1>(0, 0) = u; B.block<3, 3>(3, 1).setIdentity(); B.block<3, 1>(3, 0) = skew(p) * u;
            const Eigen::Matrix<double, 4, 4> Hf = B.transpose() * Hp * B;
            const Eigen::Matrix<double, 4, 1> bf = B.transpose() * bp;
            bool ok = false;
            for (int tries = 0; tries < 10 && !ok; ++tries) {
                const Eigen::Vector4d q = -(Hf + lam * Hf.diagonal().asDiagonal().toDenseMatrix()).ldlt().solve(bf);
                if (!q.allFinite()) { lam *= 10; continue; }
                Eigen::Matrix<double, 6, 1> d = B * q;
                Iso delta = Iso::Identity();
                const double wn = d.head<3>().norm();
                delta.linear() = wn > 1e-12 ? M3(Eigen::AngleAxisd(wn, d.head<3>() / wn)) : M3::Identity();
                delta.translation() = d.tail<3>();
                Iso xn = delta * x;
                const double cn = g.evaluateCost(xn.matrix().cast<float>());
                if (cn < cost) { x = xn; cost = cn; lam = std::max(1e-9, lam * 0.3); ok = true; }
                else lam *= 10;
            }
            if (!ok) break;
            // 收敛判据: 雷达位移 < 0.05mm, 偏航步 < 1e-5 rad
            const V3 p2 = x.translation();
            cost = g.evaluateCost(x.matrix().cast<float>(), &H, &b);
            if ((p2 - p).norm() < 5e-5) break;        // 雷达位移 < 0.05mm 视为收敛
        }
        return cost;
    };
    std::printf("倾斜代价曲线 (倾斜钉在 θ, 其余优化到收敛):\n");
    struct Row { double th, cost, rms; int n; V3 p; };
    std::vector<Row> rows;
    for (double t = 0; t <= th_max + 1e-9; t += th_step) {
        Iso x = x_ref;
        // 位置从参考解出发, 姿态按 θ 重建 (偏航取参考解的)
        const M3 Rt = Eigen::AngleAxisd(t * M_PI / 180.0, axis).toRotationMatrix();
        x.linear() = Rt * Eigen::AngleAxisd(yaw_ref, V3::UnitZ()).toRotationMatrix() * Rm;
        const double c = solve_fixed_tilt(t * M_PI / 180.0, x);
        double rms = 0; const int n = count_corr(x, rms);
        rows.push_back({t, c, rms, n, x.translation()});
        std::printf("   θ=%.1f°: 代价 %12.1f  关联 %5zu  每点 %.3f  最近邻 rms %.3f m  位置 (%.3f, %.3f, %.3f)\n",
                    t, c, (size_t)n, n ? c / n : 0.0, rms, x.translation().x(), x.translation().y(), x.translation().z());
    }
    // ---- 3) 信息量 (最优 θ 处) ----
    {
        size_t best = 0;
        for (size_t i = 1; i < rows.size(); ++i) if (rows[i].cost < rows[best].cost) best = i;
        Iso x = x_ref;
        x.linear() = Eigen::AngleAxisd(rows[best].th * M_PI / 180.0, axis).toRotationMatrix() * Eigen::AngleAxisd(yaw_ref, V3::UnitZ()).toRotationMatrix() * Rm;
        x.translation() = rows[best].p;
        Eigen::Matrix<double, 6, 6> H; Eigen::Matrix<double, 6, 1> b;
        const double c = g.evaluateCost(x.matrix().cast<float>(), &H, &b);
        const V3 p = x.translation();
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Identity(); J.block<3, 3>(3, 0) = skew(p);
        const Eigen::Matrix<double, 6, 6> Hp = J.transpose() * H * J;
        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(Hp);
        std::printf("最优 θ* = %.1f°; 雷达中心参数化 Hessian 特征值 (小->大): ", rows[best].th);
        const Eigen::VectorXd ev = es.eigenvalues().real();
        std::vector<double> sv(ev.data(), ev.data() + 6); std::sort(sv.begin(), sv.end());
        for (double v : sv) std::printf("%.3g ", v);
        std::printf("\n");
        Eigen::Matrix<double, 6, 6> cov = Hp.inverse();
        const double var = cov(0, 0) + cov(1, 1);
        const double sig = std::sqrt(var) * 180 / M_PI;
        const double scale = std::sqrt((rows[best].cost / rows[best].n) / 1.0);     // 实测每点 Mahalanobis / 模型期望(1)
        std::printf("roll/pitch 边缘不确定度 σ = %.3f° (GICP 噪声模型); 每点残差是模型的 %.1f 倍 -> 标定后 σ ≈ %.3f°; 每帧信息 I = 1/σ² = %.0f rad⁻²\n",
                    sig, rows[best].cost / rows[best].n, sig * scale, 1.0 / std::pow(sig * scale * M_PI / 180.0, 2));
        // 2x2 块: 两个主方向上的 σ 及其方向 (map 系, 相绕 x 轴/绕 y 轴)
        const Eigen::Matrix2d c2 = cov.block<2, 2>(0, 0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(c2);
        for (int e = 0; e < 2; ++e) {
            const V3 dir = V3(es2.eigenvectors()(0, e), es2.eigenvectors()(1, e), 0.0);
            std::printf("   roll/pitch 协方差主方向 %d: σ = %.3f°, 方向 (绕x %.2f, 绕y %.2f)°  [该方向上的信息 %.0f rad⁻²]\n", e,
                        std::sqrt(std::max(0.0, es2.eigenvalues()(e))) * 180 / M_PI, dir.x(), dir.y(),
                        es2.eigenvalues()(e) > 0 ? 1.0 / es2.eigenvalues()(e) : 0.0);
        }
    }
    return 0;
}
