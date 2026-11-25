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
        // 高度上限 (m): >0 时把每格高度截到该值再比较.
        // 为什么需要: 描述子取"每格最大高度", 而远处点的高度会随俯仰/距离线性变化
        // (本机雷达俯装 41.67°, 50m 外的地面点在雷达系 z≈31m), 不截断的话"同地点"
        // 的查询和关键帧描述子能差到 20+ m, 召回永远过不了阈值 (实测 best_d=1.63).
        // 0 = 不截断 (建图侧 loop closure 保持原行为).
        double z_cap = 0.0;
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

    /// 掩码距离结果: 只统计"两边都非空"的格子, 并记录参与比较的格子数.
    struct MaskedMatch {
        double dist = 1e9;   // 平均高度差(m), 只算共同非空格
        int    shift = 0;    // 列偏移(旋转)
        int    compared = 0; // 参与比较的格子数(重叠量)
    };

    /// 掩码距离 (推荐用于重定位): 空 vs 有不罚分, 只比"两边都看到的格子".
    /// 为什么: 全格 L1 会把"这格有东西/没东西"算成 z_cap 那么大的差, 同地点距离
    /// 被顶到 0.4~0.5 (阈值边缘, 时灵时不灵). 只比共同非空格后, 距离反映真实几何差.
    /// compared 太小(< min_compared) 视为"重叠不足", 该偏移不参与竞争.
    static MaskedMatch distanceMasked(const Descriptor& a, const Descriptor& b,
                                      int min_compared = 60, double eps = 0.05);

    /// RingKey 距离 (用于快速筛选候选)
    static double ringKeyDistance(const Eigen::VectorXi& a, const Eigen::VectorXi& b);

private:
    Config m_cfg;
};

/// 数据库条目: 关键帧 id + 全局位姿 + 描述子
struct ScanContextDatabaseEntry {
    std::string id;
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    ScanContext::Descriptor desc;
};

/// 序列化 ScanContext 数据库到二进制文件 (用于重定位时加载)
bool saveScanContextDatabase(const std::string& path, const std::vector<ScanContextDatabaseEntry>& db);

/// 从二进制文件加载 ScanContext 数据库
bool loadScanContextDatabase(const std::string& path, std::vector<ScanContextDatabaseEntry>& db);

} // namespace rosiwit_slam
