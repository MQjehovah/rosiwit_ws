// 核心单测: YOLO 解码 / 类别筛选 / 按类别 NMS / letterbox
#include <gtest/gtest.h>

#include <vector>

#include "object_semantics/detector_postprocess.h"

namespace rp = rosiwit_perception;

TEST(DetectorPostprocess, LetterboxFitKeepsAspect) {
  const auto lb = rp::Letterbox::fit(640, 480, 416, 416);
  EXPECT_NEAR(lb.scale, 416.0 / 640.0, 1e-9);
  EXPECT_EQ(lb.pad_y, static_cast<int>((416 - 480 * 416.0 / 640.0) / 2 + 0.5));
  EXPECT_EQ(lb.pad_x, 0);

  double u = 0.0, v = 0.0;
  lb.toImage(0.0, 0.0, u, v);
  EXPECT_NEAR(u, 0.0, 1e-9);
  EXPECT_NEAR(v, -lb.pad_y / lb.scale, 1e-6);
}

TEST(DetectorPostprocess, DecodesPersonOnlyLayout) {
  // 2 个框, layout [N, C] (row_major), C=5
  std::vector<float> out = {
      208.0f, 208.0f, 100.0f, 200.0f, 0.9f,   // 中心, 正常
      100.0f, 100.0f, 100.0f, 200.0f, 0.10f,  // 低置信 -> 被过滤
  };
  const auto lb = rp::Letterbox::fit(640, 480, 416, 416);
  auto boxes = rp::decodeDetections(out, 5, 2, true, 0.35f, 0.45f, 20, lb);
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_NEAR(boxes[0].score, 0.9, 1e-6);
  EXPECT_EQ(boxes[0].class_id, 0);
  EXPECT_GT(boxes[0].width(), 50.0);
}

TEST(DetectorPostprocess, DecodesChannelMajorLayout) {
  // 与上面同一组数据, layout [C, N]
  std::vector<float> out = {
      208.0f, 100.0f,   // cx
      208.0f, 100.0f,   // cy
      100.0f, 100.0f,   // w
      200.0f, 200.0f,   // h
      0.9f, 0.10f,      // conf
  };
  const auto lb = rp::Letterbox::fit(640, 480, 416, 416);
  auto boxes = rp::decodeDetections(out, 5, 2, false, 0.35f, 0.45f, 20, lb);
  ASSERT_EQ(boxes.size(), 1u);
}

TEST(DetectorPostprocess, DecodesMultiClassLayout) {
  // channels = 4 + nc, nc = 2 (0=person, 1=car)，layout [C, N]
  const int nc = 2;
  const int channels = 4 + nc;
  const int n = 2;
  std::vector<float> out(static_cast<size_t>(channels) * n, 0.0f);
  auto set = [&](int c, int i, float v) { out[static_cast<size_t>(c) * n + i] = v; };
  // 框 0: 人
  set(0, 0, 208.0f); set(1, 0, 208.0f); set(2, 0, 100.0f); set(3, 0, 200.0f);
  set(4, 0, 0.9f);   set(5, 0, 0.1f);
  // 框 1: 车
  set(0, 1, 100.0f); set(1, 1, 208.0f); set(2, 1, 80.0f); set(3, 1, 120.0f);
  set(4, 1, 0.2f);   set(5, 1, 0.8f);

  const auto lb = rp::Letterbox::fit(640, 480, 416, 416);
  auto boxes = rp::decodeDetections(out, channels, n, false, 0.35f, 0.45f, 20, lb);
  ASSERT_EQ(boxes.size(), 2u);
  // 按 score 降序: 人 0.9 -> class 0, 车 0.8 -> class 1
  EXPECT_EQ(boxes[0].class_id, 0);
  EXPECT_EQ(boxes[1].class_id, 1);
}

TEST(DetectorPostprocess, FiltersByWantedClass) {
  const int nc = 2;
  const int channels = 4 + nc;
  const int n = 2;
  std::vector<float> out(static_cast<size_t>(channels) * n, 0.0f);
  auto set = [&](int c, int i, float v) { out[static_cast<size_t>(c) * n + i] = v; };
  set(0, 0, 208.0f); set(1, 0, 208.0f); set(2, 0, 100.0f); set(3, 0, 200.0f);
  set(4, 0, 0.9f);   set(5, 0, 0.1f);   // person
  set(0, 1, 100.0f); set(1, 1, 208.0f); set(2, 1, 80.0f); set(3, 1, 120.0f);
  set(4, 1, 0.2f);   set(5, 1, 0.8f);   // car

  const auto lb = rp::Letterbox::fit(640, 480, 416, 416);
  auto only_person = rp::decodeDetections(out, channels, n, false, 0.35f, 0.45f, 20, lb, {0});
  ASSERT_EQ(only_person.size(), 1u);
  EXPECT_EQ(only_person[0].class_id, 0);

  auto only_car = rp::decodeDetections(out, channels, n, false, 0.35f, 0.45f, 20, lb, {1});
  ASSERT_EQ(only_car.size(), 1u);
  EXPECT_EQ(only_car[0].class_id, 1);

  auto none = rp::decodeDetections(out, channels, n, false, 0.35f, 0.45f, 20, lb, {7});
  EXPECT_TRUE(none.empty());
}

TEST(DetectorPostprocess, NmsSuppressesOverlap) {
  std::vector<rp::DetectionBox> boxes(3);
  boxes[0] = {100, 100, 200, 300, 0.9, 0, "person"};
  boxes[1] = {105, 102, 205, 302, 0.8, 0, "person"};  // 与 0 高度重叠
  boxes[2] = {400, 100, 500, 300, 0.7, 0, "person"};  // 独立
  rp::nmsInPlace(boxes, 0.45, 20);
  ASSERT_EQ(boxes.size(), 2u);
  EXPECT_NEAR(boxes[0].score, 0.9, 1e-6);
  EXPECT_NEAR(boxes[1].score, 0.7, 1e-6);
}

TEST(DetectorPostprocess, NmsIsPerClass) {
  std::vector<rp::DetectionBox> boxes(2);
  boxes[0] = {100, 100, 200, 300, 0.9, 0, "person"};
  boxes[1] = {105, 102, 205, 302, 0.8, 2, "car"};  // 重叠但类别不同 -> 都保留
  rp::nmsPerClassInPlace(boxes, 0.45, 20);
  ASSERT_EQ(boxes.size(), 2u);
}

TEST(DetectorPostprocess, MaxObjectsClamp) {
  std::vector<rp::DetectionBox> boxes;
  for (int i = 0; i < 30; ++i) {
    boxes.push_back({10.0 + i * 20, 100.0, 30.0 + i * 20, 300.0, 0.5 + 0.01 * i, 0, "person"});
  }
  rp::nmsInPlace(boxes, 0.45, 20);
  EXPECT_EQ(boxes.size(), 20u);
}

TEST(DetectorPostprocess, BoxIouIsSane) {
  rp::DetectionBox a{0, 0, 10, 10, 1.0, 0, "person"};
  rp::DetectionBox b{0, 0, 10, 10, 1.0, 0, "person"};
  rp::DetectionBox c{100, 100, 110, 110, 1.0, 0, "person"};
  EXPECT_NEAR(rp::boxIou(a, b), 1.0, 1e-9);
  EXPECT_NEAR(rp::boxIou(a, c), 0.0, 1e-9);
}
