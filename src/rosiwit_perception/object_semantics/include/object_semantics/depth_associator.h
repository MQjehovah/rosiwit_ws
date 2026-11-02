// 深度关联: 2D 框 -> 深度 ROI -> 稳健深度 -> 反投影 -> base_link 位置/协方差
//
// 与类别无关: 框上带 class_id，类别只影响尺寸先验（SemanticClass）。
// 关键约束（方案 §3）: 彩色 640x480 与深度 320x200 分辨率/FOV 不同、未对齐,
// 必须按内参把彩色像素映射到深度像素, 不能直接套像素坐标。
#pragma once

#include <vector>

#include "object_semantics/object_types.h"

namespace rosiwit_perception {

class DepthAssociator {
public:
  DepthAssociator() = default;
  DepthAssociator(const DepthParams& params,
                  const CameraIntrinsics& color,
                  const CameraIntrinsics& depth,
                  const Extrinsics& base_from_color)
      : params_(params), color_(color), depth_(depth), base_from_color_(base_from_color) {}

  void configure(const DepthParams& params, const CameraIntrinsics& color,
                 const CameraIntrinsics& depth, const Extrinsics& base_from_color);

  /// 彩色像素 -> 深度像素（方案 §5.3 的映射公式）
  static bool mapColorToDepthPixel(double u_color, double v_color,
                                   const CameraIntrinsics& color,
                                   const CameraIntrinsics& depth,
                                   double& u_depth, double& v_depth);

  /// ROI 统计（深度像素坐标下的矩形），返回有效点比例；median/near/far 单位为米
  static double roiDepthStats(const DepthImage& image, const DepthParams& params,
                              int u0, int v0, int u1, int v1,
                              double& median_m, double& near_m, double& far_m,
                              int* valid_count = nullptr, int* total_count = nullptr);

  /// 单框完整关联。返回 false 表示框退化（宽高 <=0）。
  /// depth_valid=false 时仍会尝试地面投影（低置信回退）。
  /// @param cls 该框的类别定义（只用于尺寸合理性降置信）
  bool associate(const DetectionBox& box, const DepthImage& image,
                 const SemanticClass& cls, ObjectObservation& out) const;

private:
  DepthParams params_;
  CameraIntrinsics color_;
  CameraIntrinsics depth_;
  Extrinsics base_from_color_;
};

}  // namespace rosiwit_perception
