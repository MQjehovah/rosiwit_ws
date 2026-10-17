// ============================================================================
// LostMonitor 单测 (计划 T3 的 8 条)
// ============================================================================
#include "lost_monitor.h"

#include <gtest/gtest.h>

namespace rosiwit_slam_ros1 {
namespace {

constexpr int8_t asInt(LocStatus s) { return static_cast<int8_t>(s); }

// 1
TEST(LostMonitor, NotLoadedIsRecovering) {
    LostMonitor m;
    m.setMapLoaded(false);

    EXPECT_EQ(asInt(m.update(true)), 1);
    EXPECT_FALSE(m.consumeLostEdge());
    EXPECT_EQ(asInt(m.update(false)), 1);   // 地图没加载时连"丢失"都谈不上
    EXPECT_FALSE(m.consumeLostEdge());
}

// 2 —— INIT 陷阱的回归测试: 核心在 INIT 期 getPoseCov() 返回 true 且给默认位姿,
//      绝不能被当成"定位成功"
TEST(LostMonitor, LoadedButNeverLocalizedIsRecovering) {
    LostMonitor m;
    m.setMapLoaded(true);

    EXPECT_EQ(asInt(m.update(true)), 1);
    EXPECT_EQ(asInt(m.update(true)), 1);
    EXPECT_FALSE(m.consumeLostEdge());
}

// 3
TEST(LostMonitor, AfterLocalizedIsSuccess) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();

    EXPECT_EQ(asInt(m.update(true)), 0);
}

// 4
TEST(LostMonitor, LostAfterLocalizedIsFailed) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();

    EXPECT_EQ(asInt(m.update(false)), 2);
}

// 5 —— 边沿只报一次, 连续丢失不能刷屏
TEST(LostMonitor, LostEdgeFiresExactlyOnce) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();

    EXPECT_EQ(asInt(m.update(false)), 2);
    EXPECT_TRUE(m.consumeLostEdge());
    EXPECT_EQ(asInt(m.update(false)), 2);
    EXPECT_FALSE(m.consumeLostEdge());
    EXPECT_EQ(asInt(m.update(false)), 2);
    EXPECT_FALSE(m.consumeLostEdge());
}

// 6 —— 丢失还没被消费就恢复了: 这个边沿作废(恢复不算新丢失)
TEST(LostMonitor, RecoveryReturnsToSuccessAndDoesNotRefire) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();

    EXPECT_EQ(asInt(m.update(false)), 2);
    EXPECT_EQ(asInt(m.update(true)), 0);
    EXPECT_FALSE(m.consumeLostEdge());
}

// 7 —— reset() 回到"还没定位上"; 注意它不该清掉 map_loaded_
TEST(LostMonitor, ResetReturnsToRecovering) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();
    EXPECT_EQ(asInt(m.update(true)), 0);

    m.reset();
    EXPECT_EQ(asInt(m.update(true)), 1);
    EXPECT_FALSE(m.consumeLostEdge());
}

// 8 —— 恢复之后再次丢失, 边沿必须还能报(不能因为报过一次就永久哑掉)
TEST(LostMonitor, LostAgainAfterRecoveryFiresAgain) {
    LostMonitor m;
    m.setMapLoaded(true);
    m.onLocalizedPose();

    EXPECT_EQ(asInt(m.update(false)), 2);
    EXPECT_TRUE(m.consumeLostEdge());          // 第一次丢失
    EXPECT_EQ(asInt(m.update(true)), 0);       // 恢复
    EXPECT_FALSE(m.consumeLostEdge());

    EXPECT_EQ(asInt(m.update(false)), 2);      // 再次丢失
    EXPECT_TRUE(m.consumeLostEdge());          // 必须再报一次
    EXPECT_FALSE(m.consumeLostEdge());
}

} // namespace
} // namespace rosiwit_slam_ros1

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
