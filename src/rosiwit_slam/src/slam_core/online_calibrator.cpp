#include "slam_core/online_calibrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rosiwit_slam
{

namespace
{
double normalizeAngle(double a)
{
  constexpr double kPi = 3.14159265358979323846;
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
}  // namespace

void OnlineTimeCalibrator::addImu(double stamp, double gyro_z)
{
  if (!std::isfinite(stamp) || !std::isfinite(gyro_z)) return;
  if (!imu_.empty() && stamp <= imu_.back().first) return;
  imu_.emplace_back(stamp, gyro_z);
  while (!imu_.empty() && imu_.front().first < stamp - config_.window) imu_.pop_front();
  while (imu_.size() > config_.max_samples) imu_.pop_front();
}

void OnlineTimeCalibrator::addLioPose(double stamp, double yaw)
{
  if (!std::isfinite(stamp) || !std::isfinite(yaw)) return;
  if (!lio_.empty() && stamp <= lio_.back().first) return;
  lio_.emplace_back(stamp, yaw);
  while (!lio_.empty() && lio_.front().first < stamp - config_.window) lio_.pop_front();
  while (lio_.size() > config_.max_samples) lio_.pop_front();
}

double OnlineTimeCalibrator::imuAt(double stamp) const
{
  if (imu_.size() < 2 || stamp < imu_.front().first || stamp > imu_.back().first) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto it = std::lower_bound(imu_.begin(), imu_.end(), stamp,
                             [](const std::pair<double, double> & p, double t) {
                               return p.first < t;
                             });
  if (it == imu_.begin()) return it->second;
  if (it == imu_.end()) return imu_.back().second;
  const auto prev = std::prev(it);
  const double dt = it->first - prev->first;
  if (dt <= 1e-9) return it->second;
  const double a = (stamp - prev->first) / dt;
  return prev->second * (1.0 - a) + it->second * a;
}

TimeCalibrationResult OnlineTimeCalibrator::estimate() const
{
  TimeCalibrationResult result;
  if (imu_.size() < config_.min_samples || lio_.size() < 3) return result;

  struct RateSample { double t; double rate; };
  std::vector<RateSample> rates;
  rates.reserve(lio_.size());
  for (size_t i = 1; i < lio_.size(); ++i) {
    const double dt = lio_[i].first - lio_[i - 1].first;
    if (dt <= 1e-4 || dt > 0.5) continue;
    const double rate = normalizeAngle(lio_[i].second - lio_[i - 1].second) / dt;
    rates.push_back({0.5 * (lio_[i].first + lio_[i - 1].first), rate});
  }
  if (rates.size() < config_.min_samples) return result;

  // 以 IMU 角速度的能量归一化；MSE 越小、相对零偏移改善越大，confidence 越高。
  double best_mse = std::numeric_limits<double>::infinity();
  double best_offset = 0.0;
  double zero_mse = 0.0;
  int used = 0;
  double energy = 0.0;

  const int steps = static_cast<int>(
    std::max(1.0, std::ceil(2.0 * config_.search_range / config_.search_step)));
  for (int si = 0; si <= steps; ++si) {
    const double offset = -config_.search_range +
      (2.0 * config_.search_range) * si / std::max(1, steps);
    double mse = 0.0;
    int n = 0;
    for (const auto & r : rates) {
      const double imu = imuAt(r.t + offset);
      if (!std::isfinite(imu)) continue;
      const double e = imu - r.rate;
      mse += e * e;
      ++n;
    }
    if (n < 5) continue;
    mse /= n;
    if (std::abs(offset) < 1e-9) zero_mse = mse;
    if (mse < best_mse) {
      best_mse = mse;
      best_offset = offset;
      used = n;
    }
  }

  if (!std::isfinite(best_mse)) return result;
  result.offset = best_offset;
  result.samples = used;
  result.valid = used >= static_cast<int>(config_.min_samples);
  if (zero_mse > 1e-9) {
    result.confidence = std::max(0.0, std::min(1.0, 1.0 - best_mse / zero_mse));
  } else {
    result.confidence = best_mse < 1e-6 ? 1.0 : 0.0;
  }
  return result;
}

void OnlineTimeCalibrator::reset()
{
  imu_.clear();
  lio_.clear();
}

void YawExtrinsicCalibrator::addSample(double imu_yaw, double lidar_yaw)
{
  if (!std::isfinite(imu_yaw) || !std::isfinite(lidar_yaw)) return;
  const double d = normalizeAngle(lidar_yaw - imu_yaw);
  sum_sin_ += std::sin(d);
  sum_cos_ += std::cos(d);
  ++samples_;
}

YawCalibrationResult YawExtrinsicCalibrator::estimate() const
{
  YawCalibrationResult result;
  result.samples = samples_;
  result.valid = samples_ >= 10;
  if (samples_ > 0) {
    result.yaw_offset = std::atan2(sum_sin_, sum_cos_);
  }
  return result;
}

void YawExtrinsicCalibrator::reset()
{
  sum_sin_ = 0.0;
  sum_cos_ = 0.0;
  samples_ = 0;
}

}  // namespace rosiwit_slam
