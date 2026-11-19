// include/slam_core/ground_posegraph.h
// 后端 z-only 地面位姿图: 手持建图把地面约束到 z=0, 不依赖前端硬锁.
// 节点 = 关键帧 z; 边 = 相邻 z 增量 + 地面 unary (z_k = h_k);
// 只修 z, xy/yaw 不动; 地图点按最近关键帧的 delta 平移 z.
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>   // ConjugateGradient (SparseCholesky 在 EIGEN_MPL2_ONLY 下不可用)
#include <pcl/kdtree/kdtree_flann.h>

#include "slam_core/i_grid_map_generator.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

inline bool applyGroundPosegraph(const std::vector<Observation>& obs, CloudType::Ptr& cloud) {
    const size_t N = obs.size();
    if (N < 3 || !cloud || cloud->empty()) return false;

    // 5m 分块 2% 分位作为局部地面 (比单帧圆锥稳).
    constexpr double B = 5.0;
    auto bkey = [](int bx, int by) -> long long {
        return (static_cast<long long>(bx) << 32) ^ static_cast<unsigned int>(by);
    };
    std::unordered_map<long long, std::vector<float>> zsmap;
    zsmap.reserve(cloud->size() / 8 + 1);
    for (const auto& p : cloud->points) {
        const int bx = static_cast<int>(std::floor(p.x / B));
        const int by = static_cast<int>(std::floor(p.y / B));
        zsmap[bkey(bx, by)].push_back(p.z);
    }
    std::unordered_map<long long, float> bground;
    bground.reserve(zsmap.size());
    for (auto& kv : zsmap) {
        auto& v = kv.second;
        std::sort(v.begin(), v.end());
        bground[kv.first] = v[std::min(v.size() - 1, static_cast<size_t>(0.02 * v.size()))];
    }
    auto getg = [&](double x, double y) -> float {
        const int bx = static_cast<int>(std::floor(x / B));
        const int by = static_cast<int>(std::floor(y / B));
        for (int r = 0; r <= 8; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    if (r > 0 && std::abs(dx) != r && std::abs(dy) != r) continue;
                    auto it = bground.find(bkey(bx + dx, by + dy));
                    if (it != bground.end()) return it->second;
                }
            }
        }
        return 0.0f;
    };

    std::vector<double> zold(N, 0.0), h(N, 0.0);
    std::vector<char> valid(N, 0);
    for (size_t i = 0; i < N; ++i) {
        zold[i] = obs[i].origin.z();
        const double hgt = obs[i].origin.z() - getg(obs[i].origin.x(), obs[i].origin.y());
        if (hgt < 0.15 || hgt > 2.0) continue;
        h[i] = hgt;
        valid[i] = 1;
    }
    size_t nvalid = 0;
    for (char v : valid) nvalid += v;
    if (nvalid < 10) {
        std::cout << "[ground_posegraph] too few ground frames (" << nvalid << ")\n";
        return false;
    }

    const double wu = 1.0 / 0.05;   // 地面 unary 权重
    const double wo = 1.0 / 0.10;   // 相邻 z 增量权重
    std::vector<double> wun(N, 1.0), wod(N > 1 ? N - 1 : 0, 1.0);
    Eigen::VectorXd znew;
    const int max_iter = 6;
    for (int it = 0; it < max_iter; ++it) {
        const size_t rows = nvalid + (N - 1);
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(nvalid + (N - 1) * 2);
        Eigen::VectorXd b(rows);
        b.setZero();
        size_t r = 0;
        for (size_t i = 0; i < N; ++i) {
            if (!valid[i]) continue;
            const double w = wu * wun[i];
            trips.emplace_back(static_cast<int>(r), static_cast<int>(i), w);
            b[r] = w * h[i];
            ++r;
        }
        for (size_t i = 0; i + 1 < N; ++i) {
            const double w = wo * wod[i];
            trips.emplace_back(static_cast<int>(r), static_cast<int>(i), -w);
            trips.emplace_back(static_cast<int>(r), static_cast<int>(i + 1), w);
            b[r] = w * (zold[i + 1] - zold[i]);
            ++r;
        }
        Eigen::SparseMatrix<double> A(static_cast<int>(rows), static_cast<int>(N));
        A.setFromTriplets(trips.begin(), trips.end());
        Eigen::SparseMatrix<double> ATA = A.transpose() * A;
        Eigen::VectorXd ATb = A.transpose() * b;
        for (size_t i = 0; i < N; ++i) ATA.coeffRef(static_cast<int>(i), static_cast<int>(i)) += 1e-6;
        // [可移植性] PCL 会定义 EIGEN_MPL2_ONLY, 此时 <Eigen/SparseCholesky> 直接报错,
        // SimplicialLDLT 不可用。这里改用共轭梯度 (IterativeLinearSolvers 在 MPL2 下可用);
        // ATA + 1e-6·I 是 SPD, CG 收敛良好。
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
                                 Eigen::Lower | Eigen::Upper> solver;
        solver.setTolerance(1e-8);
        solver.setMaxIterations(200);
        solver.compute(ATA);
        if (solver.info() != Eigen::Success) {
            std::cout << "[ground_posegraph] solve failed at iter " << it << "\n";
            return false;
        }
        znew = solver.solve(ATb);
        // Huber 重加权
        for (size_t i = 0; i < N; ++i) {
            if (!valid[i]) continue;
            const double rr = std::fabs(znew[i] - h[i]);
            wun[i] = rr <= 0.15 ? 1.0 : 0.15 / rr;
        }
        for (size_t i = 0; i + 1 < N; ++i) {
            const double rr = std::fabs((znew[i + 1] - znew[i]) - (zold[i + 1] - zold[i]));
            wod[i] = rr <= 0.20 ? 1.0 : 0.20 / rr;
        }
    }

    std::vector<float> delta(N, 0.0f);
    for (size_t i = 0; i < N; ++i) delta[i] = static_cast<float>(znew[i] - zold[i]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr origins(new pcl::PointCloud<pcl::PointXYZ>());
    origins->reserve(N);
    for (const auto& o : obs)
        origins->push_back(pcl::PointXYZ(static_cast<float>(o.origin.x()),
                                         static_cast<float>(o.origin.y()), 0.0f));
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(origins);
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    float zmin0 = cloud->points[0].z, zmax0 = zmin0;
    for (const auto& p : cloud->points) {
        zmin0 = std::min(zmin0, p.z);
        zmax0 = std::max(zmax0, p.z);
    }
    for (auto& p : cloud->points) {
        pcl::PointXYZ q(p.x, p.y, 0.0f);
        if (kdtree.nearestKSearch(q, 1, idx, d2) > 0) p.z += delta[idx[0]];
    }
    float zmin1 = cloud->points[0].z, zmax1 = zmin1;
    for (const auto& p : cloud->points) {
        zmin1 = std::min(zmin1, p.z);
        zmax1 = std::max(zmax1, p.z);
    }
    std::cout << "[ground_posegraph] " << nvalid << "/" << N << " ground frames; z ["
              << zmin0 << ", " << zmax0 << "] -> [" << zmin1 << ", " << zmax1 << "]\n";
    return true;
}

} // namespace rosiwit_slam
