// 深度关联实现
#include "object_semantics/depth_associator.h"

#include <algorithm>
#include <cmath>

namespace rosiwit_perception {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// 双三次不必；ROI 内取中位数 + 10/90 分位即可（方案 §5.3.4）
void percentile(const std::vector<float>& sorted, double q, double& out) {
  if (sorted.empty()) {
    out = 0.0;
    return;
  }
  const double pos = q * static_cast<double>(sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, sorted.size() - 1);
  const double frac = pos - static_cast<double>(lo);
  out = static_cast<double>(sorted[lo]) * (1.0 - frac) + static_cast<double>(sorted[hi]) * frac;
}

}  // namespace

void DepthAssociator::configure(const DepthParams& params, const CameraIntrinsics& color,
                                const CameraIntrinsics& depth, const Extrinsics& base_from_color) {
  params_ = params;
  color_ = color;
  depth_ = depth;
  base_from_color_ = base_from_color;
}

bool DepthAssociator::mapColorToDepthPixel(double u_color, double v_color,
                                           const CameraIntrinsics& color,
                                           const CameraIntrinsics& depth,
                                           double& u_depth, double& v_depth) {
  if (!color.valid() || depth.width <= 0 || depth.height <= 0) {
    return false;
  }
  // 光心基本重合假设下的比例映射（方案 §5.3.2）
  u_depth = (u_color - color.cx) * depth.fx / color.fx + depth.cx;
  v_depth = (v_color - color.cy) * depth.fy / color.fy + depth.cy;
  return std::isfinite(u_depth) && std::isfinite(v_depth);
}

double DepthAssociator::roiDepthStats(const DepthImage& image, const DepthParams& params,
                                     int u0, int v0, int u1, int v1,
                                     double& median_m, double& near_m, double& far_m,
                                     int* valid_count, int* total_count) {
  median_m = near_m = far_m = 0.0;
  if (valid_count) *valid_count = 0;
  if (total_count) *total_count = 0;
  if (!image.valid()) {
    return 0.0;
  }
  u0 = std::max(0, u0);
  v0 = std::max(0, v0);
  u1 = std::min(image.width - 1, u1);
  v1 = std::min(image.height - 1, v1);
  if (u1 < u0 || v1 < v0) {
    return 0.0;
  }

  std::vector<float> depths;
  depths.reserve(static_cast<size_t>(u1 - u0 + 1) * static_cast<size_t>(v1 - v0 + 1));
  int total = 0;
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      ++total;
      const int mm = static_cast<int>(image.at(u, v) * image.scale_mm);
      if (mm < params.depth_min_mm || mm > params.depth_max_mm) {
        continue;
      }
      depths.push_back(static_cast<float>(mm) * 0.001f);
    }
  }
  if (total_count) *total_count = total;
  if (valid_count) *valid_count = static_cast<int>(depths.size());
  if (depths.empty()) {
    return 0.0;
  }
  std::sort(depths.begin(), depths.end());
  percentile(depths, 0.5, median_m);
  percentile(depths, 0.1, near_m);
  percentile(depths, 0.9, far_m);
  return static_cast<double>(depths.size()) / static_cast<double>(total);
}

bool DepthAssociator::associate(const DetectionBox& box, const DepthImage& image,
                                const SemanticClass& cls, ObjectObservation& out) const {
  out = ObjectObservation{};
  out.box = box;
  out.confidence = static_cast<float>(box.score);
  if (box.width() <= 1.0 || box.height() <= 1.0) {
    return false;
  }

  // ---- 1) 目标中心 ROI（彩色坐标；人取躯干段，其他类别沿用同一比例）----
  const double half_w = 0.5 * box.width() * params_.roi_w_ratio;
  const double cu = box.cx();
  const double roi_c_u0 = cu - half_w;
  const double roi_c_u1 = cu + half_w;
  const double roi_c_v0 = box.v0 + box.height() * params_.roi_v_min;
  const double roi_c_v1 = box.v0 + box.height() * params_.roi_v_max;

  // ---- 2) 彩色 -> 深度像素 ----
  double du0 = 0.0, dv0 = 0.0, du1 = 0.0, dv1 = 0.0;
  const double feet_c_u = cu;
  const double feet_c_v = box.v1;
  const bool mapped = mapColorToDepthPixel(roi_c_u0, roi_c_v0, color_, depth_, du0, dv0) &&
                      mapColorToDepthPixel(roi_c_u1, roi_c_v1, color_, depth_, du1, dv1);

  double median_m = 0.0, near_m = 0.0, far_m = 0.0;
  if (mapped && image.valid()) {
    out.valid_ratio = roiDepthStats(image, params_,
                                    static_cast<int>(std::lround(std::min(du0, du1))),
                                    static_cast<int>(std::lround(std::min(dv0, dv1))),
                                    static_cast<int>(std::lround(std::max(du0, du1))),
                                    static_cast<int>(std::lround(std::max(dv0, dv1))),
                                    median_m, near_m, far_m);
    out.depth_median_m = median_m;
    out.depth_near_m = near_m;
    out.depth_far_m = far_m;
    out.depth_valid = out.valid_ratio >= params_.min_valid_ratio && median_m > 0.0;
    // 多人同框: ROI 深度分布出现双峰 (p10 与 p90 差 > 0.5 m 且两端都占 20% 以上)
    out.ambiguous = out.depth_valid && (far_m - near_m) > 0.5 && out.valid_ratio > 0.4 &&
                    (median_m - near_m) > 0.15 && (far_m - median_m) > 0.15;
  }

  // ---- 3) 反投影（方案 §5.4: 用彩色像素 + 彩色内参 + 深度中位数）----
  Eigen::Vector3d p_cam = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_base = Eigen::Vector3d::Zero();
  // 深度相机量到的是朝向相机的前表面; 按类别补偿到目标中心（0 = 不补偿）
  const double center_depth = median_m + cls.surface_offset_m;
  if (out.depth_valid) {
    // 光学系: x 右 / y 下 / z 前；base_link: x 前 / y 左 / z 上
    p_cam << (cu - color_.cx) / color_.fx * center_depth,
             (box.cy() - color_.cy) / color_.fy * center_depth,
             center_depth;
    p_base = base_from_color_.R * p_cam + base_from_color_.t;
    out.z = p_base.z();
    // 尺寸合理性: 用框顶点的射线在 center 深度处估计目标顶部高度,
    // 超出该类别先验区间只降置信不丢点（蹲坐的人 / 远距离货车都能保留）
    const double top_v = box.v0;
    const double top_y_cam = (top_v - color_.cy) / color_.fy * center_depth;
    const Eigen::Vector3d top_cam((cu - color_.cx) / color_.fx * center_depth, top_y_cam,
                                  center_depth);
    const double top_z = (base_from_color_.R * top_cam + base_from_color_.t).z();
    if (top_z < cls.min_height_m || top_z > cls.max_height_m) {
      out.confidence *= 0.5f;
    }
  }

  // ---- 4) 地面投影（脚点射线与 z=0 求交）----
  double ground_x = 0.0, ground_y = 0.0;
  bool have_ground = false;
  if (color_.valid()) {
    Eigen::Vector3d ray_cam((feet_c_u - color_.cx) / color_.fx,
                            (feet_c_v - color_.cy) / color_.fy, 1.0);
    const Eigen::Vector3d dir = base_from_color_.R * ray_cam;
    if (std::fabs(dir.z()) > 1e-6) {
      const double s = -base_from_color_.t.z() / dir.z();
      if (s > 0.05 && s < params_.ground_max_range) {
        ground_x = dir.x() * s + base_from_color_.t.x();
        ground_y = dir.y() * s + base_from_color_.t.y();
        have_ground = true;
      }
    }
  }

  if (out.depth_valid) {
    out.position = p_base.head<2>();
    out.ground = have_ground ? Eigen::Vector2d(ground_x, ground_y) : out.position;
    out.distance = out.position.norm();

    // ---- 5) 协方差（深度噪声 + 像素量化，转到 base 平面）----
    const double sigma_z = params_.sigma_z_a * center_depth * center_depth + params_.sigma_z_b;
    const double sigma_lat = center_depth * params_.px_sigma / std::max(1.0, color_.fx);
    Eigen::Matrix3d cov_cam = Eigen::Matrix3d::Zero();
    cov_cam(0, 0) = sigma_lat * sigma_lat;   // 横向（cam x）
    cov_cam(1, 1) = sigma_lat * sigma_lat;   // 纵向（cam y）
    cov_cam(2, 2) = sigma_z * sigma_z;       // 深度（cam z）
    const Eigen::Matrix3d cov_base = base_from_color_.R * cov_cam * base_from_color_.R.transpose();
    out.cov_xy = cov_base.block<2, 2>(0, 0) + Eigen::Matrix2d::Identity() * 1e-4;
  } else if (have_ground) {
    // 深度无效: 退回地面投影（低置信、大协方差）
    out.position = Eigen::Vector2d(ground_x, ground_y);
    out.ground = out.position;
    out.z = 1.0;
    out.distance = out.position.norm();
    out.cov_xy = Eigen::Matrix2d::Identity() * 0.36;  // σ≈0.6 m
    out.confidence *= 0.5f;
  } else {
    return false;
  }
  return true;
}

}  // namespace rosiwit_perception
