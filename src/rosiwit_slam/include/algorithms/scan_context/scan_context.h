// include/algorithms/scan_context/scan_context.h
// ScanContext 描述符: 将点云编码为环-扇区 2D 矩阵
//
// 原理:
//   1. 将点云按半径划分为 N_RINGS 个同心环
//   2. 每个环按方位角划分为 N_SECTORS 个扇区
//   3. 每个格子取最大高度 / 最大强度作为描述子值
//   4. RingKey: 每个环的扇区非空比例 (用于快速预筛)
//   5. 匹配: 列偏移搜索实现旋转不变性
//
// 参考: G. Kim and A. Kim, "Scan Context: Egocentric 3D Place Recognition
//       for 3D LiDARs", IROS 2018.

#pragma once
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstdint>
#include <string>
#include <cmath>

namespace rosiwit_slam {

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;

class ScanContext {
public:
    struct Config {
        int num_rings = 20;           // 同心环数
        int num_sectors = 60;         // 每环扇区数
        double max_radius = 80.0;     // 最大有效半径 (m)
        double min_radius = 2.0;      // 最小有效半径 (m), 去除近处噪声
        int ring_key_bins = 10;       // RingKey 直方图分箱数
        double ring_key_ratio = 0.3;  // RingKey 非空判定阈值 (>此比例=非空)
    };

    struct Descriptor {
        Eigen::MatrixXd data;         // num_rings × num_sectors
        Eigen::VectorXi ring_key;     // num_rings × 1, 每环量化编码

        int rows() const { return static_cast<int>(data.rows()); }
        int cols() const { return static_cast<int>(data.cols()); }
    };

    ScanContext() : m_cfg(Config{20, 60, 80.0, 2.0, 10, 0.3}) {}
    explicit ScanContext(const Config& cfg) : m_cfg(cfg) {}

    /// 从点云构造描述符
    Descriptor makeDescriptor(const CloudType::Ptr& cloud) const;

    /// 距离: 两描述符之间的列偏移对齐距离 (0=完全相同)
    static double distance(const Descriptor& a, const Descriptor& b,
                           int* out_col_shift = nullptr);

    /// RingKey 距离 (用于快速筛选候选)
    static double ringKeyDistance(const Eigen::VectorXi& a, const Eigen::VectorXi& b);

private:
    Config m_cfg;
};

} // namespace rosiwit_slam
