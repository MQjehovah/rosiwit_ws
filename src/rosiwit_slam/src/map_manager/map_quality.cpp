/**
 * @file map_quality.cpp
 * @brief FAST-LIO2 SLAM - 地图质量评估模块实现
 */

#include "fast_lio2_slam/map_manager/map_quality.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace fast_lio2_slam {

std::string QualityReport::generateTextReport() const {
    std::stringstream ss;
    ss << "=== 地图质量评估报告 ===\n";
    ss << "\n总体评分: " << overall_score << " / 1.0\n";
    ss << "\n详细评分:\n";
    ss << "  - 覆盖度: " << coverage_score << "\n";
    ss << "  - 密度: " << density_score << "\n";
    ss << "  - 一致性: " << consistency_score << "\n";
    ss << "  - 完整性: " << completeness_score << "\n";
    ss << "  - 均匀性: " << uniformity_score << "\n";
    ss << "\n统计数据:\n";
    ss << "  - 总点数: " << total_points << "\n";
    ss << "  - 总子地图: " << total_submaps << "\n";
    ss << "  - 覆盖面积: " << coverage_area << " 平方米\n";
    ss << "  - 平均密度: " << avg_density << " 点/平方米\n";
    ss << "  - 密度范围: [" << min_density << ", " << max_density << "]\n";
    ss << "  - 密度标准差: " << density_std << "\n";
    ss << "\n问题区域:\n";
    ss << "  - 空洞数量: " << holes.size() << "\n";
    ss << "  - 低密度区域: " << low_density_areas.size() << "\n";
    ss << "  - 高密度区域: " << high_density_areas.size() << "\n";
    ss << "\n改进建议:\n";
    for (const auto& rec : recommendations) {
        ss << "  - " << rec << "\n";
    }
    ss << "\n评估耗时: " << evaluation_time << " 秒\n";
    return ss.str();
}

MapQualityEvaluator::MapQualityEvaluator() {
    kdtree_.reset(new pcl::search::KdTree<PointType>());
}

MapQualityEvaluator::MapQualityEvaluator(
    const QualityEvaluatorConfig& config)
    : config_(config) {
    kdtree_.reset(new pcl::search::KdTree<PointType>());
}

void MapQualityEvaluator::initialize(
    const QualityEvaluatorConfig& config) {
    config_ = config;
}

double MapQualityEvaluator::computeCoverageScore(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    double area = computeCoverageArea(cloud);

    // 计算期望面积 (基于点的分布范围)
    auto bounds = computeBounds(cloud);
    Eigen::Vector3d size = bounds.second - bounds.first;
    double expected_area = size.x() * size.y();

    // 实际覆盖面积与期望面积的比值
    double coverage_ratio = std::min(1.0, area / expected_area);

    return coverage_ratio;
}

double MapQualityEvaluator::computeDensityScore(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    double avg_density = computeAverageDensity(cloud);

    // 目标密度: 500点/平方米 (高质量室内地图)
    double target_density = 500.0;

    // 计算密度评分
    double score = std::min(1.0, avg_density / target_density);

    return score;
}

double MapQualityEvaluator::computeCompletenessScore(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    // 检测空洞数量
    auto holes = detectHoles(cloud);

    // 计算地图面积
    double area = computeCoverageArea(cloud);

    // 空洞比例
    double hole_ratio = holes.size() * config_.hole_threshold * config_.hole_threshold / area;

    // 完整性评分 = 1 - 空洞比例
    double score = std::max(0.0, 1.0 - hole_ratio);

    return score;
}

double MapQualityEvaluator::computeUniformityScore(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    auto density_dist = computeDensityDistribution(cloud);
    if (density_dist.empty()) return 0.0;

    auto [avg, min, max, std_dev] = computeDensityStatistics(density_dist);

    // 均匀性评分基于标准差的相对大小
    // 标准差越小，均匀性越好
    double relative_std = std_dev / avg;
    double score = 1.0 - std::min(1.0, relative_std);

    return score;
}

void MapQualityEvaluator::setupKdTree(const PointCloudPtr& cloud) {
    kdtree_->setInputCloud(cloud);
}

int MapQualityEvaluator::countNeighbors(
    const PointType& point, double radius) {
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree_->radiusSearch(point, radius, indices, distances);
    return indices.size();
}

double MapQualityEvaluator::normalizeScore(
    double value, double min, double max) {
    if (max <= min) return 0.0;
    return std::clamp((value - min) / (max - min), 0.0, 1.0);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
MapQualityEvaluator::computeBounds(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
        return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }

    Eigen::Vector3d min_bound(std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    Eigen::Vector3d max_bound(std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest(),
                               std::numeric_limits<double>::lowest());

    for (const auto& point : cloud->points) {
        min_bound.x() = std::min(min_bound.x(), static_cast<double>(point.x));
        min_bound.y() = std::min(min_bound.y(), static_cast<double>(point.y));
        min_bound.z() = std::min(min_bound.z(), static_cast<double>(point.z));

        max_bound.x() = std::max(max_bound.x(), static_cast<double>(point.x));
        max_bound.y() = std::max(max_bound.y(), static_cast<double>(point.y));
        max_bound.z() = std::max(max_bound.z(), static_cast<double>(point.z));
    }

    return {min_bound, max_bound};
}

double MapQualityEvaluator::computeCoverageArea(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    auto bounds = computeBounds(cloud);
    Eigen::Vector3d size = bounds.second - bounds.first;

    // 计算覆盖面积 (考虑网格)
    int grid_x = static_cast<int>(std::ceil(size.x() / config_.grid_size));
    int grid_y = static_cast<int>(std::ceil(size.y() / config_.grid_size));

    // 有效网格数量
    auto density_dist = computeDensityDistribution(cloud);
    int valid_grids = 0;
    for (const double density : density_dist) {
        if (density > config_.min_density_threshold) {
            valid_grids++;
        }
    }

    double area = valid_grids * config_.grid_size * config_.grid_size;
    return area;
}

double MapQualityEvaluator::computeAverageDensity(
    const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;

    double area = computeCoverageArea(cloud);
    if (area <= 0) return 0.0;

    return cloud->size() / area;
}

std::tuple<double, double, double, double>
MapQualityEvaluator::computeDensityStatistics(
    const std::vector<double>& density_distribution) {
    if (density_distribution.empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    double sum = 0.0;
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();

    for (const double d : density_distribution) {
        sum += d;
        min_val = std::min(min_val, d);
        max_val = std::max(max_val, d);
    }

    double avg = sum / density_distribution.size();

    double variance = 0.0;
    for (const double d : density_distribution) {
        variance += (d - avg) * (d - avg);
    }
    double std_dev = std::sqrt(variance / density_distribution.size());

    return {avg, min_val, max_val, std_dev};
}

QualityReport MapQualityEvaluator::evaluate(
    std::shared_ptr<MapManager> map_manager) {
    QualityReport report;

    if (!map_manager) return report;

    auto cloud = map_manager->getFullMap();
    auto submaps = map_manager->getAllSubmaps();

    report.total_points = cloud ? cloud->size() : 0;
    report.total_submaps = submaps.size();

    // 计算各项评分
    report.coverage_score = computeCoverageScore(cloud);
    report.density_score = computeDensityScore(cloud);
    report.completeness_score = computeCompletenessScore(cloud);
    report.uniformity_score = computeUniformityScore(cloud);

    // 计算总体评分
    report.overall_score =
        config_.coverage_weight * report.coverage_score +
        config_.density_weight * report.density_score +
        config_.completeness_weight * report.completeness_score +
        config_.uniformity_weight * report.uniformity_score;

    // 计算统计数据
    report.coverage_area = computeCoverageArea(cloud);
    report.avg_density = computeAverageDensity(cloud);
    auto density_dist = computeDensityDistribution(cloud);
    auto [avg, min, max, std_dev] = computeDensityStatistics(density_dist);
    report.min_density = min;
    report.max_density = max;
    report.density_std = std_dev;
    report.density_distribution = density_dist;

    // 检测问题区域
    report.holes = detectHoles(cloud);
    report.low_density_areas = detectLowDensityAreas(cloud, config_.min_density_threshold);
    report.high_density_areas = detectHighDensityAreas(cloud, config_.min_density_threshold * 10);

    // 生成改进建议
    report.recommendations = generateRecommendations(report);

    return report;
}

std::vector<std::string> MapQualityEvaluator::generateRecommendations(
    const QualityReport& report) {
    std::vector<std::string> recommendations;

    if (report.overall_score < 0.5) {
        recommendations.push_back("地图质量较低，建议重新采集数据");
    }

    if (report.coverage_score < 0.6) {
        recommendations.push_back("地图覆盖度不足，建议增加采集范围");
    }

    if (report.density_score < 0.5) {
        recommendations.push_back("点密度过低，建议增加采集时间或降低移动速度");
    }

    if (report.holes.size() > 10) {
        recommendations.push_back("存在较多空洞区域，建议在这些区域重新采集");
    }

    if (report.uniformity_score < 0.5) {
        recommendations.push_back("密度分布不均匀，建议在低密度区域增加采集");
    }

    if (report.high_density_areas.size() > 5) {
        recommendations.push_back("存在冗余采集区域，可适当清理重复点云");
    }

    if (recommendations.empty()) {
        recommendations.push_back("地图质量良好，无需改进");
    }

    return recommendations;
}

bool MapQualityEvaluator::saveReport(
    const QualityReport& report, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) return false;

    file << report.generateTextReport();
    file.close();

    return true;
}

} // namespace fast_lio2_slam
