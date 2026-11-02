// 多目标 3D 跟踪: world 系 2D 恒速 KF + 匈牙利关联 + 生命周期 + 双速度输出
//
// 与类别无关: track 记录 class_id/class_name, 关联时要求同类；
// 新增语义类别不需要改这里。
#pragma once

#include <cstdint>
#include <vector>

#include "object_semantics/object_types.h"

namespace rosiwit_perception {

class MultiObjectTracker {
public:
  MultiObjectTracker() = default;
  explicit MultiObjectTracker(const TrackerParams& params) : params_(params) {}
  void configure(const TrackerParams& params) { params_ = params; }

  /// @param stamp            彩色帧时间（秒）
  /// @param observations     本帧 3D 观测（base_link 系，来自 DepthAssociator）
  /// @param robot            同一时刻的机器人世界位姿/速度；无效时退化为 base 系跟踪
  /// @param tracks           输出：confirmed + 仍在 coasting 的目标
  void update(double stamp, const std::vector<ObjectObservation>& observations,
              const RobotState& robot, std::vector<ObjectTrack>& tracks);

  size_t trackCount() const { return tracks_.size(); }
  void reset() { tracks_.clear(); next_id_ = 1; }

private:
  struct Kf {
    Eigen::Vector4d x = Eigen::Vector4d::Zero();  // [px, py, vx, vy] world
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity() * 0.5;

    Eigen::Vector2d position() const { return Eigen::Vector2d(x(0), x(1)); }
    void predict(double dt, double accel) {
      Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
      F(0, 2) = dt;
      F(1, 3) = dt;
      Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
      const double q = accel * accel;
      const double dt2 = dt * dt;
      const double dt3 = dt2 * dt;
      Q(0, 0) = dt3 * q / 3.0;
      Q(1, 1) = dt3 * q / 3.0;
      Q(2, 2) = dt * q;
      Q(3, 3) = dt * q;
      Q(0, 2) = Q(2, 0) = dt2 * q / 2.0;
      Q(1, 3) = Q(3, 1) = dt2 * q / 2.0;
      x = F * x;
      P = F * P * F.transpose() + Q;
    }
    void update(const Eigen::Vector2d& meas, const Eigen::Matrix2d& R) {
      Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
      H(0, 0) = 1.0;
      H(1, 1) = 1.0;
      const Eigen::Matrix2d S = H * P * H.transpose() + R;
      const Eigen::Matrix2d S_inv = S.inverse();
      const Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S_inv;
      x += K * (meas - H * x);
      P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }
    double mahalanobisSq(const Eigen::Vector2d& meas, const Eigen::Matrix2d& R) const {
      Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
      H(0, 0) = 1.0;
      H(1, 1) = 1.0;
      const Eigen::Matrix2d S = H * P * H.transpose() + R;
      const Eigen::Vector2d d = meas - H * x;
      return d.transpose() * S.inverse() * d;
    }
  };

  struct Track {
    uint32_t id = 0;
    Kf kf;
    double first_stamp = 0.0;
    double last_update_stamp = 0.0;
    int hits = 0;
    int misses = 0;
    bool confirmed = false;
    bool coasting = false;
    bool depth_valid = false;
    ObjectObservation last_obs;
  };

  /// 匈牙利分配：cost 为 n x m，输出 pairs
  static void solveAssignment(const std::vector<std::vector<double>>& cost,
                              std::vector<int>& row_to_col);

  void toOutput(const Track& tr, double stamp, const RobotState& robot, ObjectTrack& out) const;

  TrackerParams params_;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 1;
  bool have_last_stamp_ = false;
  double last_stamp_ = 0.0;
};

}  // namespace rosiwit_perception
