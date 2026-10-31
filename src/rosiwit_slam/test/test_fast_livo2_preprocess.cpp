#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include "algorithms/fast_livo2/fast_livo2_config.h"
#include "algorithms/fast_livo2/preprocess.h"

namespace {

rosiwit_slam::CloudType makeCloud(const std::vector<std::array<float, 3>>& xyz,
                                  const std::vector<float>& curvature) {
    rosiwit_slam::CloudType cloud;
    for (size_t i = 0; i < xyz.size(); ++i) {
        rosiwit_slam::PointType p;
        p.x = xyz[i][0];
        p.y = xyz[i][1];
        p.z = xyz[i][2];
        p.curvature = curvature[i];
        cloud.push_back(p);
    }
    return cloud;
}

}  // namespace

TEST(FastLivo2Preprocess, DropsBlindAndNaNAndKeepsCurvature) {
    rosiwit_slam::FastLivo2Config cfg;
    cfg.point_filter_num = 1;
    cfg.blind = 0.5;

    rosiwit_slam::FastLivo2Preprocess pre;
    pre.configure(cfg);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto in = makeCloud({{0.1f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {nan, 0.f, 0.f}, {0.f, 0.f, 3.f}},
                        {0.f, 10.f, 20.f, 30.f});
    rosiwit_slam::CloudType out;
    ASSERT_TRUE(pre.process(in, out));

    ASSERT_EQ(out.size(), 2u);
    EXPECT_FLOAT_EQ(out[0].curvature, 10.f);
    EXPECT_FLOAT_EQ(out[1].curvature, 30.f);
}

TEST(FastLivo2Preprocess, AppliesPointStride) {
    rosiwit_slam::FastLivo2Config cfg;
    cfg.point_filter_num = 2;
    cfg.blind = 0.1;

    rosiwit_slam::FastLivo2Preprocess pre;
    pre.configure(cfg);

    auto in = makeCloud({{1.f, 0.f, 0.f}, {2.f, 0.f, 0.f}, {3.f, 0.f, 0.f},
                         {4.f, 0.f, 0.f}, {5.f, 0.f, 0.f}},
                        {0.f, 1.f, 2.f, 3.f, 4.f});
    rosiwit_slam::CloudType out;
    ASSERT_TRUE(pre.process(in, out));

    ASSERT_EQ(out.size(), 3u);  // indices 0, 2, 4
    EXPECT_FLOAT_EQ(out[0].curvature, 0.f);
    EXPECT_FLOAT_EQ(out[1].curvature, 2.f);
    EXPECT_FLOAT_EQ(out[2].curvature, 4.f);
}

TEST(FastLivo2Preprocess, EmptyInputIsEmpty) {
    rosiwit_slam::FastLivo2Config cfg;
    rosiwit_slam::FastLivo2Preprocess pre;
    pre.configure(cfg);

    rosiwit_slam::CloudType in;
    rosiwit_slam::CloudType out;
    EXPECT_FALSE(pre.process(in, out));
    EXPECT_TRUE(out.empty());
}
