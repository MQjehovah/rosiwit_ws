// ============================================================
// 在线时间/外参标定（不依赖 ROS）
//
// 1) OnlineTimeCalibrator: 用 IMU 角速度与 LIO 位姿角速度做互相关，
//    估计传感器/管线时间偏移。
// 2) YawExtrinsicCalibrator: 用成对的 IMU/LiDAR 朝向样本估计安装 yaw 偏差。
//
// 设计为"只估计、不偷偷改状态"：上层确认 confidence 后再写回配置，
// 避免在线标定把正在工作的系统带偏。
// ============================================================
#ifndef ROSIWIT_SLAM__ONLINE_CALIBRATOR_H_
#define ROSIWIT_SLAM__ONLINE_CALIBRATOR_H_

#include <cstddef>
#include <deque>
#include <utility>

namespace rosiwit_slam
{

struct TimeCalibrationResult
{
  bool valid = false;
  double offset = 0.0;      // seconds; true_time = lio_time + offset
  double confidence = 0.0;  // 0..1
  int samples = 0;
};

class OnlineTimeCalibrator
{
public:
  struct Config
  {
    double window = 20.0;       // seconds
    double search_range = 0.20; // +/- seconds
    double search_step = 0.002;
    size_t min_samples = 20;
    size_t max_samples = 2000;
  };

  OnlineTimeCalibrator() = default;
  explicit OnlineTimeCalibrator(const Config & config) : config_(config) {}

  void setConfig(const Config & config) { config_ = config; }
  void addImu(double stamp, double gyro_z);
  /// LIO 位姿: 内部用相邻 yaw 差分得到角速度。
  void addLioPose(double stamp, double yaw);
  TimeCalibrationResult estimate() const;
  void reset();

private:
  double imuAt(double stamp) const;

  Config config_;
  std::deque<std::pair<double, double>> imu_;   // (t, gyro_z)
  std::deque<std::pair<double, double>> lio_;   // (t, yaw)
};

struct YawCalibrationResult
{
  bool valid = false;
  double yaw_offset = 0.0;
  int samples = 0;
};

class YawExtrinsicCalibrator
{
public:
  void addSample(double imu_yaw, double lidar_yaw);
  YawCalibrationResult estimate() const;
  void reset();

private:
  double sum_sin_ = 0.0;
  double sum_cos_ = 0.0;
  int samples_ = 0;
};

}  // namespace rosiwit_slam

#endif  // ROSIWIT_SLAM__ONLINE_CALIBRATOR_H_
