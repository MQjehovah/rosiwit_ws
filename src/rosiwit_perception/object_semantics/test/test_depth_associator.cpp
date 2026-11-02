// 核心单测: 像素映射 / ROI 深度 / 反投影 / 协方差 / 地面投影
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "object_semantics/depth_associator.h"

namespace rp = rosiwit_perception;

namespace {

rp::CameraIntrinsics makeColor() {
  rp::CameraIntrinsics c;
  c.fx = 478.118;
  c.fy = 477.981;
  c.cx = 324.378;
  c.cy = 242.398;
  c.width = 640;
  c.height = 480;
  return c;
}

rp::CameraIntrinsics makeDepth() {
  rp::CameraIntrinsics d;
  d.fx = 153.504;
  d.fy = 153.504;
  d.cx = 159.589;
  d.cy = 99.437;
  d.width = 320;
  d.height = 200;
  return d;
}

// 相机装在 base_link 前 0.525 m、高 0.713 m、俯仰 0（方案 §3）
// base_link(x 前,y 左,z 上) <- camera_optical(x 右,y 下,z 前)
rp::Extrinsics makeExtrinsics() {
  rp::Extrinsics e;
  e.R << 0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
         0.0, -1.0, 0.0;
  e.t = Eigen::Vector3d(0.525, 0.0, 0.713);
  return e;
}

rp::DepthImage makeDepthImage(int w, int h, uint16_t value_mm, double valid_ratio,
                              uint32_t seed = 1) {
  rp::DepthImage img;
  img.width = w;
  img.height = h;
  img.data.assign(static_cast<size_t>(w) * h, value_mm);
  // 按比例把部分像素置 0（模拟深度空洞）
  uint32_t state = seed;
  const int holes = static_cast<int>((1.0 - valid_ratio) * w * h);
  for (int i = 0; i < holes; ++i) {
    state = state * 1664525u + 1013904223u;
    img.data[state % img.data.size()] = 0;
  }
  return img;
}

}  // namespace

TEST(DepthAssociator, MapsColorPixelToDepthPixel) {
  double u = 0.0, v = 0.0;
  ASSERT_TRUE(rp::DepthAssociator::mapColorToDepthPixel(
      324.378, 242.398, makeColor(), makeDepth(), u, v));
  EXPECT_NEAR(u, 159.589, 1e-6);
  EXPECT_NEAR(v, 99.437, 1e-6);

  // 彩色右移 100 px -> 深度按 fx 比例右移 ~32 px
  ASSERT_TRUE(rp::DepthAssociator::mapColorToDepthPixel(
      424.378, 242.398, makeColor(), makeDepth(), u, v));
  EXPECT_NEAR(u, 159.589 + 100.0 * 153.504 / 478.118, 1e-6);
}

TEST(DepthAssociator, RoiMedianAndValidRatio) {
  rp::DepthParams p;
  auto img = makeDepthImage(320, 200, 2000, 0.8);
  double median = 0.0, near_m = 0.0, far_m = 0.0;
  int valid = 0, total = 0;
  const double ratio = rp::DepthAssociator::roiDepthStats(img, p, 120, 60, 200, 160,
                                                          median, near_m, far_m, &valid, &total);
  EXPECT_NEAR(ratio, 0.8, 0.05);
  EXPECT_NEAR(median, 2.0, 0.05);
  EXPECT_NEAR(near_m, 2.0, 0.05);
  EXPECT_LE(near_m, median);
  EXPECT_GE(far_m, median);
  EXPECT_GT(total, valid);
}

TEST(DepthAssociator, BackProjectionAtTwoMeters) {
  rp::DepthAssociator assoc;
  assoc.configure(rp::DepthParams{}, makeColor(), makeDepth(), makeExtrinsics());
  auto img = makeDepthImage(320, 200, 2000, 1.0);
  // 人站在相机正前方 2 m、身高 1.7 m: 框顶 ~v=6(1.7m), 框底 ~v=413(0m)
  rp::DetectionBox box;
  box.u0 = 300;
  box.u1 = 350;
  box.v0 = 6;
  box.v1 = 413;
  box.score = 0.9;
  rp::ObjectObservation ob;
  ASSERT_TRUE(assoc.associate(box, img, rp::SemanticClass{}, ob));
  ASSERT_TRUE(ob.depth_valid);
  EXPECT_NEAR(ob.position.x(), 0.525 + 2.0, 0.05);   // base 在相机后 0.525 m
  EXPECT_NEAR(ob.position.y(), 0.0, 0.05);
  // 2 m 处深度噪声 σ_z≈0.02 m, 横向远小于深度
  EXPECT_LT(ob.cov_xy(0, 0), 0.01);
}

TEST(DepthAssociator, CovarianceGrowsWithDistance) {
  rp::DepthAssociator assoc;
  auto img_near = makeDepthImage(320, 200, 1000, 1.0);
  auto img_far = makeDepthImage(320, 200, 4000, 1.0);
  assoc.configure(rp::DepthParams{}, makeColor(), makeDepth(), makeExtrinsics());
  // 1 m 用 1.7 m 身高换算的框图; 4 m 同一比例缩小 (只关心协方差趋势)
  rp::DetectionBox box_near{300, 120, 350, 350, 0.9};
  rp::DetectionBox box_far{300, 60, 350, 400, 0.9};
  rp::ObjectObservation near_ob, far_ob;
  ASSERT_TRUE(assoc.associate(box_near, img_near, rp::SemanticClass{}, near_ob));
  ASSERT_TRUE(assoc.associate(box_far, img_far, rp::SemanticClass{}, far_ob));
  EXPECT_GT(far_ob.cov_xy.trace(), near_ob.cov_xy.trace());
}

TEST(DepthAssociator, InvalidDepthFallsBackToGround) {
  rp::DepthAssociator assoc;
  assoc.configure(rp::DepthParams{}, makeColor(), makeDepth(), makeExtrinsics());
  auto img = makeDepthImage(320, 200, 0, 0.0);  // 全空洞
  rp::DetectionBox box;
  box.u0 = 300; box.u1 = 350; box.v0 = 380; box.v1 = 470; box.score = 0.8;
  rp::ObjectObservation ob;
  ASSERT_TRUE(assoc.associate(box, img, rp::SemanticClass{}, ob));
  EXPECT_FALSE(ob.depth_valid);
  EXPECT_GT(ob.cov_xy(0, 0), 0.1);   // 回退到低置信地面投影
  EXPECT_LT(ob.confidence, 0.5f);
}

TEST(DepthAssociator, OffCenterPersonHasLateralOffset) {
  rp::DepthAssociator assoc;
  assoc.configure(rp::DepthParams{}, makeColor(), makeDepth(), makeExtrinsics());
  auto img = makeDepthImage(320, 200, 3000, 1.0);
  rp::DetectionBox box;
  box.u0 = 424 - 25; box.u1 = 424 + 25; box.v0 = 200; box.v1 = 470; box.score = 0.9;
  rp::ObjectObservation ob;
  ASSERT_TRUE(assoc.associate(box, img, rp::SemanticClass{}, ob));
  ASSERT_TRUE(ob.depth_valid);
  // 相机前 3 m -> base x = 3 + 0.525; 像素右移 100 px -> base y 为负
  EXPECT_NEAR(ob.position.x(), 0.525 + 3.0, 0.1);
  EXPECT_NEAR(ob.position.y(), -100.0 / 478.118 * 3.0, 0.1);
}

TEST(DepthAssociator, HeightPriorIsPerClass) {
  // 1.7 m 高的人在 2 m 处: person 类先验 (0.8-2.2) 不降置信;
  // 换成"车窗高度 0.2-1.2 m"的类别先验则应降置信, 但观测仍保留。
  rp::DepthAssociator assoc;
  assoc.configure(rp::DepthParams{}, makeColor(), makeDepth(), makeExtrinsics());
  auto img = makeDepthImage(320, 200, 2000, 1.0);
  rp::DetectionBox box;
  box.u0 = 300; box.u1 = 350; box.v0 = 6; box.v1 = 413; box.score = 0.9;

  rp::SemanticClass person;
  person.id = 0;
  person.name = "person";
  person.min_height_m = 0.8;
  person.max_height_m = 2.2;
  rp::ObjectObservation person_ob;
  ASSERT_TRUE(assoc.associate(box, img, person, person_ob));
  EXPECT_NEAR(person_ob.confidence, 0.9f, 1e-5);

  rp::SemanticClass low_object;
  low_object.id = 5;
  low_object.name = "low_object";
  low_object.min_height_m = 0.2;
  low_object.max_height_m = 1.2;
  rp::ObjectObservation low_ob;
  ASSERT_TRUE(assoc.associate(box, img, low_object, low_ob));
  EXPECT_LT(low_ob.confidence, 0.5f);
  EXPECT_TRUE(low_ob.depth_valid);   // 只降置信, 不丢点
}
