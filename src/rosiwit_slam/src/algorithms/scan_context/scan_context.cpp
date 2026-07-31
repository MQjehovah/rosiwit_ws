// src/algorithms/scan_context/scan_context.cpp
#include "algorithms/scan_context/scan_context.h"
#include <algorithm>
#include <limits>
#include <fstream>
#include <cstring>

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
    if (nr <= 0 || ns <= 0) { std::cerr << "[SC] bad cfg: nr=" << nr << " ns=" << ns << std::endl; return desc; }

    double ring_width = (max_r - min_r) / nr;
    double sector_angle = 2.0 * M_PI / ns;

    // 遍历每个点, 填入对应格子
    for (size_t pi = 0; pi < cloud->size(); ++pi) {
        const auto& pt = cloud->points[pi];
        double x = pt.x, y = pt.y, z = pt.z;
        if (!std::isfinite(x) || !std::isfinite(y)) continue;
        double r = std::sqrt(x * x + y * y);
        if (r < min_r || r >= max_r) continue;

        int ring = static_cast<int>((r - min_r) / ring_width);
        if (ring < 0) ring = 0;
        if (ring >= nr) ring = nr - 1;

        double theta = std::atan2(y, x);
        if (theta < 0) theta += 2.0 * M_PI;
        int sector = static_cast<int>(theta / sector_angle);
        if (sector < 0) sector = 0;
        if (sector >= ns) sector = ns - 1;

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

bool saveScanContextDatabase(const std::string& path, const std::vector<ScanContextDatabaseEntry>& db) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) {
        std::cerr << "[SC] Failed to open for write: " << path << std::endl;
        return false;
    }

    auto writeMat = [&ofs](const Eigen::MatrixXd& m) {
        int rows = m.rows(), cols = m.cols();
        ofs.write(reinterpret_cast<const char*>(&rows), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&cols), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(m.data()), sizeof(double) * rows * cols);
    };
    auto writeVec = [&ofs](const Eigen::VectorXi& v) {
        int n = v.size();
        ofs.write(reinterpret_cast<const char*>(&n), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(v.data()), sizeof(int) * n);
    };

    int num_entries = static_cast<int>(db.size());
    ofs.write(reinterpret_cast<const char*>(&num_entries), sizeof(int));
    for (const auto& e : db) {
        int id_len = static_cast<int>(e.id.size());
        ofs.write(reinterpret_cast<const char*>(&id_len), sizeof(int));
        ofs.write(e.id.c_str(), id_len);
        for (int i = 0; i < 9; ++i)
            ofs.write(reinterpret_cast<const char*>(e.rot.data() + i), sizeof(double));
        for (int i = 0; i < 3; ++i)
            ofs.write(reinterpret_cast<const char*>(e.trans.data() + i), sizeof(double));
        writeMat(e.desc.data);
        writeVec(e.desc.ring_key);
    }
    ofs.close();
    return true;
}

bool loadScanContextDatabase(const std::string& path, std::vector<ScanContextDatabaseEntry>& db) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "[SC] Failed to open for read: " << path << std::endl;
        return false;
    }

    auto readMat = [&ifs](Eigen::MatrixXd& m) -> bool {
        int rows = 0, cols = 0;
        if (!ifs.read(reinterpret_cast<char*>(&rows), sizeof(int))) return false;
        if (!ifs.read(reinterpret_cast<char*>(&cols), sizeof(int))) return false;
        if (rows <= 0 || rows > 1000 || cols <= 0 || cols > 1000) return false;
        m.resize(rows, cols);
        ifs.read(reinterpret_cast<char*>(m.data()), sizeof(double) * rows * cols);
        return ifs.good();
    };
    auto readVec = [&ifs](Eigen::VectorXi& v) -> bool {
        int n = 0;
        if (!ifs.read(reinterpret_cast<char*>(&n), sizeof(int))) return false;
        if (n < 0 || n > 100000) return false;
        v.resize(n);
        ifs.read(reinterpret_cast<char*>(v.data()), sizeof(int) * n);
        return ifs.good();
    };

    db.clear();
    int num_entries = 0;
    if (!ifs.read(reinterpret_cast<char*>(&num_entries), sizeof(int))) return false;
    if (num_entries < 0 || num_entries > 10000000) return false;
    db.reserve(num_entries);

    for (int i = 0; i < num_entries; ++i) {
        ScanContextDatabaseEntry e;
        int id_len = 0;
        if (!ifs.read(reinterpret_cast<char*>(&id_len), sizeof(int))) return false;
        if (id_len < 0 || id_len > 100000) return false;
        e.id.resize(id_len);
        ifs.read(&e.id[0], id_len);
        ifs.read(reinterpret_cast<char*>(e.rot.data()), sizeof(double) * 9);
        ifs.read(reinterpret_cast<char*>(e.trans.data()), sizeof(double) * 3);
        if (!readMat(e.desc.data)) return false;
        if (!readVec(e.desc.ring_key)) return false;
        if (!ifs.good()) return false;
        db.push_back(std::move(e));
    }
    return true;
}

} // namespace rosiwit_slam
