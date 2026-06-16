// ============================================================
// Diffbot Navigation - 窄道检测器实现
// ============================================================

#include "diffbot_navigation/narrow_passage/narrow_passage_detector.hpp"

#include <algorithm>
#include <cmath>

namespace diffbot_navigation
{
namespace narrow_passage
{

NarrowPassageDetector::NarrowPassageDetector()
{
  // 初始化默认配置
  config_.robot_width = 0.4;
  config_.robot_length = 0.5;
  config_.min_passage_width = 0.6;
  config_.detection_range = 3.0;
  config_.safety_margin = 0.1;
  config_.width_threshold = 0.5;
  config_.length_threshold = 0.5;
  config_.laser_min_range = 0.1;
  config_.laser_max_range = 10.0;
  config_.laser_angular_resolution = 0.017;
  config_.precision_mode = true;
  config_.precision_safety_distance = 0.05;
}

void NarrowPassageDetector::configure(const DetectorConfig & config)
{
  config_ = config;
  RCLCPP_INFO(logger_,
    "Narrow passage detector configured with robot_width=%.2f, min_passage_width=%.2f",
    config_.robot_width, config_.min_passage_width);
}

void NarrowPassageDetector::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  latest_laser_scan_ = scan;
}

void NarrowPassageDetector::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  latest_map_ = map;
}

std::vector<NarrowPassage> NarrowPassageDetector::detectPassages()
{
  // 优先从激光扫描检测，如果激光扫描无效则从地图检测
  if (latest_laser_scan_) {
    return detectPassagesFromLaserScan(latest_laser_scan_);
  } else if (latest_map_) {
    return detectPassagesFromMap(latest_map_);
  }

  return std::vector<NarrowPassage>();
}

std::vector<NarrowPassage> NarrowPassageDetector::detectFrontPassages(
  const geometry_msgs::msg::PoseStamped & pose,
  double range)
{
  if (!latest_laser_scan_) {
    return std::vector<NarrowPassage>();
  }

  std::vector<NarrowPassage> front_passages;

  // 检测前方一定范围内的窄道
  std::vector<NarrowPassage> all_passages = detectPassages();

  // 过滤出前方窄道
  double robot_theta = 0.0;  // 默认朝向，实际应从pose获取
  robot_theta = std::atan2(
    2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
      pose.pose.orientation.x * pose.pose.orientation.y),
    1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y +
      pose.pose.orientation.z * pose.pose.orientation.z));

  for (const auto & passage : all_passages) {
    double dx = passage.centerline_x - pose.pose.position.x;
    double dy = passage.centerline_y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double angle = std::atan2(dy, dx) - robot_theta;

    // 规范化角度
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }

    // 检查是否在前方范围内
    if (std::abs(angle) < M_PI / 3 && dist < range) {
      front_passages.push_back(passage);
    }
  }

  return front_passages;
}

NarrowPassage NarrowPassageDetector::getClosestPassage(
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::vector<NarrowPassage> passages = detectPassages();

  if (passages.empty()) {
    return NarrowPassage();
  }

  NarrowPassage closest = passages[0];
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & passage : passages) {
    double dx = passage.centerline_x - pose.pose.position.x;
    double dy = passage.centerline_y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest = passage;
    }
  }

  return closest;
}

bool NarrowPassageDetector::isInNarrowPassage(const geometry_msgs::msg::PoseStamped & pose)
{
  // 检查当前位置是否在窄道中
  std::vector<NarrowPassage> passages = detectPassages();

  for (const auto & passage : passages) {
    if (isPointInPassage(pose.pose.position.x, pose.pose.position.y, passage)) {
      return true;
    }
  }

  return false;
}

std::vector<NarrowPassage> NarrowPassageDetector::checkPathForPassages(
  const std::vector<geometry_msgs::msg::Point> & path)
{
  std::vector<NarrowPassage> path_passages;

  // 检查路径上的每个点是否在窄道中
  std::vector<NarrowPassage> all_passages = detectPassages();

  for (const auto & path_point : path) {
    for (const auto & passage : all_passages) {
      if (isPointInPassage(path_point.x, path_point.y, passage)) {
        path_passages.push_back(passage);
        break;  // 避免重复添加同一通道
      }
    }
  }

  return path_passages;
}

bool NarrowPassageDetector::isPassageTraversable(const NarrowPassage & passage)
{
  // 检查通道宽度是否足够机器人通过
  double effective_width = passage.width - 2.0 * config_.safety_margin;

  return effective_width > config_.robot_width;
}

void NarrowPassageDetector::computeCenterline(NarrowPassage & passage)
{
  // 计算通道中心线
  passage.centerline_x = (passage.start_x + passage.end_x) / 2.0;
  passage.centerline_y = (passage.start_y + passage.end_y) / 2.0;

  // 计算通道方向
  passage.orientation = computePassageOrientation(passage);
}

double NarrowPassageDetector::getRecommendedVelocity(const NarrowPassage & passage)
{
  // 根据通道宽度计算建议通过速度
  // 通道越窄，速度越低

  double width_ratio = (passage.width - config_.robot_width) / config_.robot_width;

  // 速度公式：v = base_velocity * min(width_ratio, 1.0)
  double base_velocity = 0.2;  // 基础速度
  double velocity = base_velocity * std::min(width_ratio, 1.0);

  // 确保速度在合理范围内
  return std::clamp(velocity, 0.05, 0.2);
}

std::vector<NarrowPassage> NarrowPassageDetector::detectPassagesFromLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<NarrowPassage> passages;

  // 在激光扫描中找到间隙
  std::vector<std::pair<int, int>> gaps = findGapsInLaserScan(scan);

  // 将间隙转换为窄道
  for (const auto & gap : gaps) {
    double gap_width = computeGapWidth(scan, gap.first, gap.second);

    // 检查间隙宽度是否在窄道范围内
    if (gap_width >= config_.robot_width + 2.0 * config_.safety_margin &&
      gap_width < config_.min_passage_width + config_.safety_margin) {

      // 创建窄道
      NarrowPassage passage;

      // 计算间隙的起点和终点位置
      double start_angle = scan->angle_min + gap.first * scan->angle_increment;
      double end_angle = scan->angle_min + gap.second * scan->angle_increment;

      // 计算间隙两侧障碍物的位置
      double start_dist = scan->ranges[gap.first];
      double end_dist = scan->ranges[gap.second];

      passage.start_x = start_dist * std::cos(start_angle);
      passage.start_y = start_dist * std::sin(start_angle);
      passage.end_x = end_dist * std::cos(end_angle);
      passage.end_y = end_dist * std::sin(end_angle);

      passage.width = gap_width;
      passage.length = std::sqrt(
        std::pow(passage.end_x - passage.start_x, 2) +
        std::pow(passage.end_y - passage.start_y, 2));

      passage.is_traversable = isPassageTraversable(passage);
      passage.recommended_velocity = getRecommendedVelocity(passage);
      passage.safety_margin = config_.safety_margin;

      // 计算中心线
      computeCenterline(passage);

      // 添加通道
      if (passage.is_traversable) {
        passages.push_back(passage);
        RCLCPP_DEBUG(logger_,
          "Detected narrow passage at center (%.2f, %.2f), width %.2f m",
          passage.centerline_x, passage.centerline_y, passage.width);
      }
    }
  }

  // 合并相邻的通道
  passages = mergeAdjacentPassages(passages);

  detected_passages_ = passages;

  return passages;
}

std::vector<NarrowPassage> NarrowPassageDetector::detectPassagesFromMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // 从地图检测窄道（简化实现）
  std::vector<NarrowPassage> passages;

  // TODO: 实现完整的地图窄道检测

  // 检测地图中的空白区域宽度
  // 找到机器人宽度附近的通道

  return passages;
}

std::vector<std::pair<int, int>> NarrowPassageDetector::findGapsInLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<std::pair<int, int>> gaps;

  // 检测激光扫描中的间隙
  int gap_start = -1;
  double prev_range = scan->ranges[0];

  for (size_t i = 1; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // 检查范围有效性
    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    // 检测间隙（距离突然增大）
    if (range - prev_range > config_.width_threshold) {
      gap_start = static_cast<int>(i);
    }

    // 检测间隙结束（距离突然减小）
    if (gap_start >= 0 && prev_range - range > config_.width_threshold) {
      gaps.push_back(std::make_pair(gap_start, static_cast<int>(i)));
      gap_start = -1;
    }

    prev_range = range;
  }

  return gaps;
}

double NarrowPassageDetector::computeGapWidth(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  int start_index,
  int end_index)
{
  // 计算间隙的物理宽度
  double start_angle = scan->angle_min + start_index * scan->angle_increment;
  double end_angle = scan->angle_min + end_index * scan->angle_increment;

  // 使用扫描中有效的最小距离
  double min_dist = std::numeric_limits<double>::max();

  for (int i = start_index; i <= end_index; ++i) {
    if (scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max) {
      if (scan->ranges[i] < min_dist) {
        min_dist = scan->ranges[i];
      }
    }
  }

  // 使用三角几何计算间隙宽度
  // gap_width = 2 * min_dist * sin(angle_diff / 2)
  double angle_diff = std::abs(end_angle - start_angle);
  double gap_width = 2.0 * min_dist * std::sin(angle_diff / 2.0);

  return gap_width;
}

std::vector<NarrowPassage> NarrowPassageDetector::mergeAdjacentPassages(
  const std::vector<NarrowPassage> & passages)
{
  if (passages.size() <= 1) {
    return passages;
  }

  std::vector<NarrowPassage> merged;
  merged.push_back(passages[0]);

  for (size_t i = 1; i < passages.size(); ++i) {
    // 检查是否与前一个通道相邻
    double dx = passages[i].start_x - merged.back().end_x;
    double dy = passages[i].start_y - merged.back().end_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < config_.width_threshold) {
      // 合并通道
      merged.back().end_x = passages[i].end_x;
      merged.back().end_y = passages[i].end_y;
      merged.back().width = std::min(merged.back().width, passages[i].width);
      merged.back().length += passages[i].length;
      computeCenterline(merged.back());
    } else {
      merged.push_back(passages[i]);
    }
  }

  return merged;
}

double NarrowPassageDetector::computePassageOrientation(const NarrowPassage & passage)
{
  // 计算通道方向（从起点到终点的方向）
  double dx = passage.end_x - passage.start_x;
  double dy = passage.end_y - passage.start_y;

  return std::atan2(dy, dx);
}

bool NarrowPassageDetector::isPointInPassage(double x, double y, const NarrowPassage & passage)
{
  // 检查点是否在通道内
  // 计算点到通道中心线的距离
  double lateral_dist = distanceToCenterline(x, y, passage);

  // 计算点沿通道方向的距离
  double dx = x - passage.start_x;
  double dy = y - passage.start_y;
  double along_dist = dx * std::cos(passage.orientation) + dy * std::sin(passage.orientation);

  // 检查是否在通道范围内
  bool within_width = lateral_dist < passage.width / 2.0;
  bool within_length = along_dist >= 0.0 && along_dist <= passage.length;

  return within_width && within_length;
}

double NarrowPassageDetector::distanceToCenterline(double x, double y, const NarrowPassage & passage)
{
  // 计算点到中心线的横向距离
  // 使用点到线的距离公式

  // 中心线方程：ax + by + c = 0
  // 从起点到终点的方向向量
  double dx = passage.end_x - passage.start_x;
  double dy = passage.end_y - passage.start_y;

  // 线段长度
  double line_length = std::sqrt(dx * dx + dy * dy);

  if (line_length < 0.001) {
    // 通道长度太短
    double pt_dx = x - passage.centerline_x;
    double pt_dy = y - passage.centerline_y;
    return std::sqrt(pt_dx * pt_dx + pt_dy * pt_dy);
  }

  // 点到线的距离
  // distance = |(x - x1) * dy - (y - y1) * dx| / line_length
  double dist = std::abs(
    (x - passage.start_x) * dy - (y - passage.start_y) * dx) / line_length;

  return dist;
}

} // namespace narrow_passage
} // namespace diffbot_navigation