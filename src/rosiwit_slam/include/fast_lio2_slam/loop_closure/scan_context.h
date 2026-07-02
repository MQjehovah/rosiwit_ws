/**
 * @file scan_context.h
 * @brief FAST-LIO2 SLAM - Scan Context闭环检测
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 实现Scan Context算法进行闭环检测:
 * 1. 从点云生成极坐标编码
 * 2. 计算相似度进行匹配
 * 3. 估计相对位姿
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief Scan Context配置
 */
struct ScanContextConfig {
    int ring_num = 20;           // 环数
    int sector_num = 60;         // 扇区数
    double max_range = 80.0;     // 最大距离
    double ring_height = 2.0;    // 环高度

    // 匯配参数
    double threshold = 0.3;      // 匯配阈值
    int min_match_count = 3;     // 最小匹配数

    // 搜索参数
    int exclude_near_scan = 50;  // 排除近邻帧
};

/**
 * @brief Scan Context描述子
 */
struct ScanContextDescriptor {
    Eigen::MatrixXd context;      // 描述矩阵 (ring_num x sector_num)
    Eigen::VectorXd ring_key;     // 环键值
    double timestamp;
    int scan_id;
    SE3d pose;

    ScanContextDescriptor() : timestamp(0), scan_id(-1) {}
};

/**
 * @brief Scan Context闭环检测类
 */
class ScanContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScanContext();
    explicit ScanContext(const ScanContextConfig& config);
    ~ScanContext() = default;

    /**
     * @brief 初始化
     */
    void initialize(const ScanContextConfig& config);

    /**
     * @brief 从点云生成描述子
     */
    ScanContextDescriptor makeDescriptor(const PointCloudPtr& cloud,
                                          double timestamp,
                                          int scan_id,
                                          const SE3d& pose);

    /**
     * @brief 添加关键帧
     */
    void addKeyframe(const ScanContextDescriptor& desc);

    /**
     * @brief 检测闭环
     */
    bool detectLoop(const ScanContextDescriptor& query,
                    LoopConstraint& constraint);

    /**
     * @brief 计算两个描述子之间的相似度
     */
    double computeSimilarity(const Eigen::MatrixXd& desc1,
                              const Eigen::MatrixXd& desc2);

    /**
     * @brief 计算旋转角度偏移
     */
    int computeYawOffset(const Eigen::MatrixXd& desc1,
                          const Eigen::MatrixXd& desc2);

    /**
     * @brief 获取所有关键帧
     */
    std::vector<ScanContextDescriptor> getKeyframes() const;

    /**
     * @brief 清空关键帧
     */
    void clearKeyframes();

    /**
     * @brief 获取关键帧数量
     */
    size_t keyframeCount() const;

private:
    /**
     * @brief 将点投影到极坐标网格
     */
    void projectToPolarGrid(const PointCloudPtr& cloud, Eigen::MatrixXd& context);

    /**
     * @brief 计算环键值
     */
    Eigen::VectorXd computeRingKey(const Eigen::MatrixXd& context);

    /**
     * @brief 旋转描述子
     */
    Eigen::MatrixXd rotateContext(const Eigen::MatrixXd& context, int shift);

private:
    ScanContextConfig config_;
    std::vector<ScanContextDescriptor> keyframes_;
    bool initialized_;
};

} // namespace fast_lio2_slam
