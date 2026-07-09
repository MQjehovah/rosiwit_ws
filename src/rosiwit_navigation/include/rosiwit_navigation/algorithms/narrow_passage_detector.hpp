// ============================================================
// Diffbot Navigation - 窄道检测器
// 检测和识别窄道区域
// ============================================================

#ifndef ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_DETECTOR_HPP_
#define ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_DETECTOR_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace rosiwit_navigation
{
namespace narrow_passage
{

/**
 * @brief 窄道信息
 */
struct NarrowPassage
{
  double start_x;              // 起点 x
  double start_y;              // 起点 y
  double end_x;                // 终点 x
  double end_y;                // 终点 y
  double width;                // 通道宽度 (m)
  double length;               // 通道长度 (m)
  double orientation;          // 通道方向 (rad)
  bool is_traversable;         // 是否可通过
  double safety_margin;        // 安全余量 (m)
  double recommended_velocity; // 建议速度 (m/s)
  double centerline_x;         // 中心线 x
  double centerline_y;         // 中心线 y

  NarrowPassage()
  : start_x(0.0), start_y(0.0), end_x(0.0), end_y(0.0),
    width(0.0), length(0.0), orientation(0.0),
    is_traversable(false), safety_margin(0.1),
    recommended_velocity(0.15), centerline_x(0.0), centerline_y(0.0) {}
};

/**
 * @brief 窄道检测参数
 */
struct DetectorConfig
{
  // 机器人参数
  double robot_width;          // 机器人宽度 (m)
  double robot_length;         // 机器人长度 (m)
  
  // 窄道检测参数
  double min_passage_width;    // 最小通道宽度 (m)
  double detection_range;      // 检测范围 (m)
  double safety_margin;        // 安全余量 (m)
  
  // 通道识别参数
  double width_threshold;      // 宽度阈值
  double length_threshold;     // 长度阈值
  
  // 激光雷达参数
  double laser_min_range;
  double laser_max_range;
  double laser_angular_resolution;
  
  // 精确模式参数
  bool precision_mode;
  double precision_safety_distance;
};

/**
 * @class NarrowPassageDetector
 * @brief 窄道检测器
 */
class NarrowPassageDetector
{
public:
  /**
   * @brief 构造函数
   */
  NarrowPassageDetector();

  /**
   * @brief 析构函数
   */
  ~NarrowPassageDetector() = default;

  /**
   * @brief 配置检测器
   */
  void configure(const DetectorConfig & config);

  /**
   * @brief 更新激光扫描数据
   */
  void updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief 更新地图数据
   */
  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  /**
   * @brief 检测窄道
   * @return 检测到的窄道列表
   */
  std::vector<NarrowPassage> detectPassages();

  /**
   * @brief 检测前方窄道
   * @param pose 当前位姿
   * @param range 检测范围
   * @return 前方窄道列表
   */
  std::vector<NarrowPassage> detectFrontPassages(
    const geometry_msgs::msg::PoseStamped & pose,
    double range = 3.0);

  /**
   * @brief 获取最近的窄道
   * @param pose 当前位姿
   * @return 最近的窄道
   */
  NarrowPassage getClosestPassage(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 检查是否在窄道中
   * @param pose 当前位姿
   * @return 是否在窄道中
   */
  bool isInNarrowPassage(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 检查路径是否经过窄道
   * @param path 待检查路径
   * @return 窄道列表
   */
  std::vector<NarrowPassage> checkPathForPassages(
    const std::vector<geometry_msgs::msg::Point> & path);

  /**
   * @brief 检查窄道是否可通过
   * @param passage 窄道
   * @return 是否可通过
   */
  bool isPassageTraversable(const NarrowPassage & passage);

  /**
   * @brief 计算窄道中心线
   */
  void computeCenterline(NarrowPassage & passage);

  /**
   * @brief 获取建议通过速度
   * @param passage 窄道
   * @return 建议速度 (m/s)
   */
  double getRecommendedVelocity(const NarrowPassage & passage);

  /**
   * @brief 清除检测结果
   */
  void clearPassages() { detected_passages_.clear(); }

  /**
   * @brief 获取所有检测到的窄道
   */
  const std::vector<NarrowPassage> & getAllPassages() const { return detected_passages_; }

private:
  /**
   * @brief 从激光扫描检测通道
   */
  std::vector<NarrowPassage> detectPassagesFromLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief 从地图检测通道
   */
  std::vector<NarrowPassage> detectPassagesFromMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  /**
   * @brief 在激光扫描中找到间隙
   */
  std::vector<std::pair<int, int>> findGapsInLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief 计算间隙宽度
   */
  double computeGapWidth(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    int start_index,
    int end_index);

  /**
   * @brief 合并相邻的通道
   */
  std::vector<NarrowPassage> mergeAdjacentPassages(
    const std::vector<NarrowPassage> & passages);

  /**
   * @brief 计算通道方向
   */
  double computePassageOrientation(const NarrowPassage & passage);

  /**
   * @brief 检查点是否在通道内
   */
  bool isPointInPassage(
    double x,
    double y,
    const NarrowPassage & passage);

  /**
   * @brief 计算点到通道中心线的距离
   */
  double distanceToCenterline(
    double x,
    double y,
    const NarrowPassage & passage);

  // 成员变量
  DetectorConfig config_;
  std::vector<NarrowPassage> detected_passages_;

  // 传感器数据
  sensor_msgs::msg::LaserScan::SharedPtr latest_laser_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("narrow_passage_detector")};
};

} // namespace narrow_passage
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_DETECTOR_HPP_