// YOLO 输出解码 + 类别筛选 + NMS（不含 NPU 绑定，便于单测）
//
// 支持两种常见导出布局（均为 [1, C, N] 或 [1, N, C] 展平后的一维数组）:
//   * person-only: C=5 -> [cx, cy, w, h, conf]，类别恒为 0
//   * 多类别（YOLOv8）: C=4+nc -> [cx, cy, w, h, cls0..cls(nc-1)]，取 argmax 作为类别
// 只保留 wanted_class_ids 里的类别；空表示全部保留。
#pragma once

#include <vector>

#include "object_semantics/object_types.h"

namespace rosiwit_perception {

/// letterbox 前处理参数（RGA/OpenCV 同款）
struct Letterbox {
  double scale = 1.0;   ///< 原图 -> 网络输入的缩放
  int pad_x = 0;        ///< 左侧填充
  int pad_y = 0;        ///< 上方填充
  int in_w = 416;
  int in_h = 416;
  int img_w = 640;
  int img_h = 480;

  /// 计算 letterbox 参数（保持长宽比，居中填充）
  static Letterbox fit(int img_w, int img_h, int in_w, int in_h);
  /// 网络坐标 -> 原图坐标
  void toImage(double x, double y, double& u, double& v) const;
};

/// 解码 + 置信度过滤 + 按类别 NMS。返回按 score 降序、最多 max_objects 个框。
/// @param wanted_class_ids 关心的类别索引；空 = 全部（节点一般只传配置里的类别）
std::vector<DetectionBox> decodeDetections(const std::vector<float>& out,
                                           int channels, int num_boxes,
                                           bool row_major,
                                           float conf_threshold, float nms_iou,
                                           int max_objects,
                                           const Letterbox& letterbox,
                                           const std::vector<int>& wanted_class_ids = {});

/// 全局 NMS（不区分类别）
void nmsInPlace(std::vector<DetectionBox>& boxes, double iou_threshold, int max_out);

/// 按类别 NMS：不同类别的重叠框互不抑制
void nmsPerClassInPlace(std::vector<DetectionBox>& boxes, double iou_threshold, int max_out);

double boxIou(const DetectionBox& a, const DetectionBox& b);

}  // namespace rosiwit_perception
