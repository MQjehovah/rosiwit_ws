// YOLO 解码 + 类别筛选 + NMS 实现
#include "object_semantics/detector_postprocess.h"

#include <algorithm>
#include <cmath>

namespace rosiwit_perception {
namespace {

bool classWanted(int class_id, const std::vector<int>& wanted) {
  if (wanted.empty()) {
    return true;
  }
  return std::find(wanted.begin(), wanted.end(), class_id) != wanted.end();
}

}  // namespace

Letterbox Letterbox::fit(int img_w, int img_h, int in_w, int in_h) {
  Letterbox lb;
  lb.img_w = img_w;
  lb.img_h = img_h;
  lb.in_w = in_w;
  lb.in_h = in_h;
  if (img_w <= 0 || img_h <= 0) {
    return lb;
  }
  lb.scale = std::min(static_cast<double>(in_w) / img_w, static_cast<double>(in_h) / img_h);
  const int new_w = static_cast<int>(std::lround(img_w * lb.scale));
  const int new_h = static_cast<int>(std::lround(img_h * lb.scale));
  lb.pad_x = (in_w - new_w) / 2;
  lb.pad_y = (in_h - new_h) / 2;
  return lb;
}

void Letterbox::toImage(double x, double y, double& u, double& v) const {
  u = (x - pad_x) / std::max(1e-6, scale);
  v = (y - pad_y) / std::max(1e-6, scale);
}

double boxIou(const DetectionBox& a, const DetectionBox& b) {
  const double x0 = std::max(a.u0, b.u0);
  const double y0 = std::max(a.v0, b.v0);
  const double x1 = std::min(a.u1, b.u1);
  const double y1 = std::min(a.v1, b.v1);
  const double inter = std::max(0.0, x1 - x0) * std::max(0.0, y1 - y0);
  const double area_a = std::max(0.0, a.width()) * std::max(0.0, a.height());
  const double area_b = std::max(0.0, b.width()) * std::max(0.0, b.height());
  const double uni = area_a + area_b - inter;
  return uni > 1e-9 ? inter / uni : 0.0;
}

void nmsInPlace(std::vector<DetectionBox>& boxes, double iou_threshold, int max_out) {
  std::sort(boxes.begin(), boxes.end(),
            [](const DetectionBox& a, const DetectionBox& b) { return a.score > b.score; });
  std::vector<DetectionBox> kept;
  for (const auto& box : boxes) {
    bool suppressed = false;
    for (const auto& k : kept) {
      if (boxIou(box, k) > iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(box);
      if (static_cast<int>(kept.size()) >= max_out) {
        break;
      }
    }
  }
  boxes.swap(kept);
}

void nmsPerClassInPlace(std::vector<DetectionBox>& boxes, double iou_threshold, int max_out) {
  std::sort(boxes.begin(), boxes.end(),
            [](const DetectionBox& a, const DetectionBox& b) { return a.score > b.score; });
  std::vector<DetectionBox> kept;
  for (const auto& box : boxes) {
    bool suppressed = false;
    for (const auto& k : kept) {
      if (k.class_id != box.class_id) {
        continue;  // 人类别框不会被车类别框抑制
      }
      if (boxIou(box, k) > iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(box);
      if (static_cast<int>(kept.size()) >= max_out) {
        break;
      }
    }
  }
  boxes.swap(kept);
}

std::vector<DetectionBox> decodeDetections(const std::vector<float>& out,
                                           int channels, int num_boxes,
                                           bool row_major,
                                           float conf_threshold, float nms_iou,
                                           int max_objects,
                                           const Letterbox& letterbox,
                                           const std::vector<int>& wanted_class_ids) {
  std::vector<DetectionBox> boxes;
  if (channels < 5 || num_boxes <= 0 || out.empty()) {
    return boxes;
  }
  const size_t need = static_cast<size_t>(channels) * static_cast<size_t>(num_boxes);
  if (out.size() < need) {
    return boxes;
  }
  // row_major = true: out[i*channels + c]（[N, C]）；false: out[c*num_boxes + i]（[C, N]）
  auto at = [&](int i, int c) -> double {
    return row_major ? out[static_cast<size_t>(i) * channels + c]
                     : out[static_cast<size_t>(c) * num_boxes + i];
  };

  boxes.reserve(static_cast<size_t>(num_boxes));
  for (int i = 0; i < num_boxes; ++i) {
    const double cx = at(i, 0);
    const double cy = at(i, 1);
    const double w = at(i, 2);
    const double h = at(i, 3);
    double score = 0.0;
    int class_id = 0;
    if (channels == 5) {
      score = at(i, 4);  // person-only 导出
    } else {
      // YOLOv8 多类别: 没有 objectness，直接对类别分数取 argmax
      for (int c = 4; c < channels; ++c) {
        const double s = at(i, c);
        if (s > score) {
          score = s;
          class_id = c - 4;
        }
      }
    }
    if (!std::isfinite(score) || score < conf_threshold) {
      continue;
    }
    if (!classWanted(class_id, wanted_class_ids)) {
      continue;
    }
    if (!(w > 1.0 && h > 1.0)) {
      continue;
    }
    DetectionBox box;
    double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
    letterbox.toImage(cx - w * 0.5, cy - h * 0.5, u0, v0);
    letterbox.toImage(cx + w * 0.5, cy + h * 0.5, u1, v1);
    box.u0 = std::max(0.0, u0);
    box.v0 = std::max(0.0, v0);
    box.u1 = std::min(static_cast<double>(letterbox.img_w - 1), u1);
    box.v1 = std::min(static_cast<double>(letterbox.img_h - 1), v1);
    box.score = score;
    box.class_id = class_id;
    if (box.width() < 4.0 || box.height() < 8.0) {
      continue;
    }
    boxes.push_back(box);
  }
  nmsPerClassInPlace(boxes, nms_iou, max_objects);
  return boxes;
}

}  // namespace rosiwit_perception
