/**
 * @file map_quality.h
 * @brief FAST-LIO2 SLAM - 地图质量评估模块
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 提供地图质量评估功能，包括:
 * - 点密度分布分析
 * - 地图覆盖度评估
 * - 空洞检测
 * - 一致性检查
 * - 地图质量评分
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/map_manager/map_manager.h"

namespace fast_lio2_slam {

/**
 * @brief 质量评估报告
 */
struct QualityReport {
    // 总体评分 [0, 1]
    double overall_score = 0.0;

    // 详细评分
    double coverage_score = 0.0;       // 覆盖度评分
    double density_score = 0.0;        // 密度评分
    double consistency_score = 0.0;    // 一致性评分
    double completeness_score = 0.0;   // 完整性评分
    double uniformity_score = 0.0;     // 均匀性评分

    // 统计数据
    int total_points = 0;
    int total_submaps = 0;
    double coverage_area = 0.0;        // 覆盖面积 (平方米)
    double avg_density = 0.0;          // 平均点密度 (点/平方米)
    double max_density = 0.0;
    double min_density = 0.0;
    double density_std = 0.0;          // 密度标准差

    // 问题区域
    std::vector<Eigen::Vector3d> holes;              // 空洞位置
    std::vector<Eigen::Vector3d> low_density_areas;  // 低密度区域
    std::vector<Eigen::Vector3d> high_density_areas; // 高密度区域
    std::vector<Eigen::Vector3d> overlapping_areas;  // 重叠区域

    // 密度分布 (网格化)
    std::vector<double> density_distribution;        // 每个网格的密度
    int grid_count = 0;                              // 网格数量

    // 建议改进措施
    std::vector<std::string> recommendations;

    // 时间戳
    double evaluation_time = 0.0;     // 评估耗时 (秒)

    /**
     * @brief 生成文本报告
     */
    std::string generateTextReport() const;
};

/**
 * @brief 地图质量评估配置
 */
struct QualityEvaluatorConfig {
    // 评估参数
    double grid_size = 5.0;           // 网格大小 (米)
    double min_density_threshold = 50.0;  // 最小密度阈值 (点/平方米)
    double hole_threshold = 2.0;      // 空洞阈值 (米)
    double neighbor_radius = 1.0;     // 邻域搜索半径

    // 权重配置
    double coverage_weight = 0.2;
    double density_weight = 0.3;
    double consistency_weight = 0.2;
    double completeness_weight = 0.15;
    double uniformity_weight = 0.15;

    // 过滤参数
    double outlier_threshold = 3.0;   // 离群点阈值 (标准差)
    int min_neighbors = 5;            // 最小邻居数

    // 输出参数
    bool generate_heatmap = false;    // 生成密度热力图
    bool detailed_report = true;      // 详细报告
};

/**
 * @brief 地图质量评估器
 *
 * 评估地图质量，检测问题区域，生成改进建议
 */
class MapQualityEvaluator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<MapQualityEvaluator>;

    /**
     * @brief 构造函数
     */
    MapQualityEvaluator();
    explicit MapQualityEvaluator(const QualityEvaluatorConfig& config);

    /**
     * @brief 初始化
     */
    void initialize(const QualityEvaluatorConfig& config);

    // ============ 主要评估接口 ============

    /**
     * @brief 评估地图质量
     * @param map_manager 地图管理器
     * @return 质量报告
     */
    QualityReport evaluate(std::shared_ptr<MapManager> map_manager);

    /**
     * @brief 评估点云质量
     * @param cloud 点云
     * @return 质量报告
     */
    QualityReport evaluatePointCloud(const PointCloudPtr& cloud);

    // ============ 详细评估接口 ============

    /**
     * @brief 计算覆盖度评分
     * @param cloud 点云
     * @return 覆盖度评分 [0, 1]
     */
    double computeCoverageScore(const PointCloudPtr& cloud);

    /**
     * @brief 计算密度评分
     * @param cloud 点云
     * @return 密度评分 [0, 1]
     */
    double computeDensityScore(const PointCloudPtr& cloud);

    /**
     * @brief 计算一致性评分
     * @param submaps 子地图列表
     * @param poses 位姿列表
     * @return 一致性评分 [0, 1]
     */
    double computeConsistencyScore(
        const std::vector<Submap>& submaps,
        const std::vector<SE3d>& poses);

    /**
     * @brief 计算完整性评分
     * @param cloud 点云
     * @return 完整性评分 [0, 1]
     */
    double computeCompletenessScore(const PointCloudPtr& cloud);

    /**
     * @brief 计算均匀性评分
     * @param cloud 点云
     * @return 均匀性评分 [0, 1]
     */
    double computeUniformityScore(const PointCloudPtr& cloud);

    // ============ 检测接口 ============

    /**
     * @brief 计算点密度分布
     * @param cloud 点云
     * @return 密度分布 (每个网格的密度)
     */
    std::vector<double> computeDensityDistribution(const PointCloudPtr& cloud);

    /**
     * @brief 检测地图空洞
     * @param cloud 点云
     * @return 空洞位置列表
     */
    std::vector<Eigen::Vector3d> detectHoles(const PointCloudPtr& cloud);

    /**
     * @brief 检测低密度区域
     * @param cloud 点云
     * @param threshold 密度阈值
     * @return 低密度区域位置列表
     */
    std::vector<Eigen::Vector3d> detectLowDensityAreas(
        const PointCloudPtr& cloud,
        double threshold);

    /**
     * @brief 检测高密度区域
     * @param cloud 点云
     * @param threshold 密度阈值
     * @return 高密度区域位置列表
     */
    std::vector<Eigen::Vector3d> detectHighDensityAreas(
        const PointCloudPtr& cloud,
        double threshold);

    /**
     * @brief 检测离群点
     * @param cloud 点云
     * @return 离群点索引列表
     */
    std::vector<int> detectOutliers(const PointCloudPtr& cloud);

    // ============ 统计接口 ============

    /**
     * @brief 计算地图边界
     * @param cloud 点云
     * @return 边界 [min, max]
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> computeBounds(
        const PointCloudPtr& cloud);

    /**
     * @brief 计算覆盖面积
     * @param cloud 点云
     * @return 覆盖面积 (平方米)
     */
    double computeCoverageArea(const PointCloudPtr& cloud);

    /**
     * @brief 计算平均密度
     * @param cloud 点云
     * @return 平均密度 (点/平方米)
     */
    double computeAverageDensity(const PointCloudPtr& cloud);

    /**
     * @brief 计算密度统计
     * @param density_distribution 密度分布
     * @return [avg, min, max, std]
     */
    std::tuple<double, double, double, double> computeDensityStatistics(
        const std::vector<double>& density_distribution);

    // ============ 辅助接口 ============

    /**
     * @brief 生成改进建议
     * @param report 质量报告
     * @return 建议列表
     */
    std::vector<std::string> generateRecommendations(const QualityReport& report);

    /**
     * @brief 保存评估报告
     * @param report 质量报告
     * @param path 保存路径
     */
    bool saveReport(const QualityReport& report, const std::string& path);

    /**
     * @brief 获取配置
     */
    QualityEvaluatorConfig getConfig() const { return config_; }

private:
    // 配置
    QualityEvaluatorConfig config_;

    // KD树 (用于邻域搜索)
    pcl::search::KdTree<PointType>::Ptr kdtree_;

    // 辅助函数
    void setupKdTree(const PointCloudPtr& cloud);
    int countNeighbors(const PointType& point, double radius);
    double normalizeScore(double value, double min, double max);
};

} // namespace fast_lio2_slam
