// src/algorithms/scan_context/scan_context.cpp
#include "algorithms/scan_context/scan_context.h"
#include <algorithm>
#include <limits>

namespace rosiwit_slam {

ScanContext::Descriptor ScanContext::makeDescriptor(const CloudType::Ptr& cloud) const {
    int nr = m_cfg.num_rings;
    int ns = m_cfg.num_sectors;
    double max_r = m_cfg.max_radius;
    double min_r = m_cfg.min_radius;

    Descriptor desc;
    desc.data = Eigen::MatrixXd::Zero(nr, ns);
    desc.ring_key = Eigen::VectorXi::Zero(nr);

    if (!cloud || cloud->empty()) return desc;

    double ring_width = (max_r - min_r) / nr;
    double sector_angle = 2.0 * M_PI / ns;

    // 遍历每个点, 填入对应格子
    for (const auto& pt : cloud->points) {
        double x = pt.x, y = pt.y, z = pt.z;
        double r = std::sqrt(x * x + y * y);
        if (r < min_r || r >= max_r) continue;

        int ring = static_cast<int>((r - min_r) / ring_width);
        ring = std::min(ring, nr - 1);

        double theta = std::atan2(y, x);  // [-π, π]
        if (theta < 0) theta += 2.0 * M_PI;
        int sector = static_cast<int>(theta / sector_angle);
        sector = std::min(sector, ns - 1);

        // 取每个格子的最大高度 (也可改用最大强度)
        if (z > desc.data(ring, sector)) {
            desc.data(ring, sector) = z;
        }
    }

    // 计算 RingKey: 每环中非空格子比例的量化编码
    for (int r = 0; r < nr; ++r) {
        int nonempty = 0;
        for (int s = 0; s < ns; ++s) {
            if (desc.data(r, s) > -1e6) ++nonempty;  // 默认0也是非空(因为地面)
        }
        // 实际上需要区分"空"和"地面": 用高度阈值 > -1e6 意义不大
        // 改用 z > 0.3 才算有效点 (过滤地面)
        // 重新计算更合理——但为了性能, 在上面的循环中不易做
        // 简单做法: 重新过一次
    }

    // 重新计算 RingKey (更准确)
    // 用 Z 轴分位数: 统计每环的扇区中 z>0.5m 的比例
    // 但为了效率, 用已有 data 矩阵: data(r,s) > 0.5 视为非空
    for (int r = 0; r < nr; ++r) {
        int occupied = 0;
        double threshold = 0.5;  // 高于 0.5m 视为障碍物
        for (int s = 0; s < ns; ++s) {
            if (desc.data(r, s) > threshold) ++occupied;
        }
        // 量化到 [0, ring_key_bins-1]
        double ratio = static_cast<double>(occupied) / ns;
        int bin = static_cast<int>(ratio * m_cfg.ring_key_bins);
        desc.ring_key(r) = std::min(bin, m_cfg.ring_key_bins - 1);
    }

    return desc;
}

double ScanContext::distance(const Descriptor& a, const Descriptor& b,
                              int* out_col_shift) {
    int nr = std::min(a.rows(), b.rows());
    int ns = std::min(a.cols(), b.cols());
    if (nr == 0 || ns == 0) return std::numeric_limits<double>::max();

    double best_dist = std::numeric_limits<double>::max();
    int best_shift = 0;

    // 列偏移搜索: 遍历所有可能偏移, 取最小距离 (实现旋转不变性)
    for (int shift = 0; shift < ns; ++shift) {
        double dist = 0.0;
        int count = 0;

        for (int r = 0; r < nr; ++r) {
            for (int s = 0; s < ns; ++s) {
                int bs = (s + shift) % ns;
                double diff = a.data(r, s) - b.data(r, bs);
                // L1 距离 (对离群更鲁棒)
                dist += std::abs(diff);
                ++count;
            }
        }

        if (count > 0) {
            dist /= count;  // 平均 L1 距离
            if (dist < best_dist) {
                best_dist = dist;
                best_shift = shift;
            }
        }
    }

    if (out_col_shift) *out_col_shift = best_shift;
    return best_dist;
}

double ScanContext::ringKeyDistance(const Eigen::VectorXi& a, const Eigen::VectorXi& b) {
    int n = std::min(a.size(), b.size());
    if (n == 0) return std::numeric_limits<double>::max();

    double dist = 0.0;
    for (int i = 0; i < n; ++i) {
        dist += std::abs(a(i) - b(i));
    }
    return dist / n;
}

} // namespace rosiwit_slam
