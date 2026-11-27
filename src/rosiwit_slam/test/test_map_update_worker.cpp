// test/test_map_update_worker.cpp
// 地图更新线程 (2026-11-27): 账本写入挪出跟踪线程之后的并发契约。
//
//   1. 队列按提交顺序执行 (账本按时间序记账);
//   2. 观测任务 (Localized) 有界: 满了丢**最旧的观测**, 结算任务 (Merge / Lifelong) 永远不丢;
//   3. flush(): 丢掉排队的, 等手里那个做完 (换图前调用的保证);
//   4. 维护回调 (空闲时跑的工作图分片标注): 任务在做时绝不跑; 空闲时按节拍 (~50ms) 跑;
//      任务源源不断时也照样按节拍跑 (不会被饿死, 也不会更频繁);
//   5. 任务抛异常不杀线程 (计数后继续);
//   6. 析构会等在途任务;
//   7. 真实账本: 跟踪线程读的视图 (revision / filterDeadBase / cachedPersistencyCoverage)
//      不拿账本主锁 —— 别人持锁做一次长标注时, 这些读必须立刻返回; 且缓存值 == 现算值。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "algorithms/localization/maintainer/map_update_worker.h"
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"

using namespace rosiwit_slam;
using Kind = MapUpdateWorker::Kind;
using Job = MapUpdateWorker::Job;
using clk = std::chrono::steady_clock;

namespace {

Job mkJob(Kind k, int tag)
{
    Job j;
    j.kind = k;
    j.pose.time = static_cast<double>(tag);   // 借 time 字段当任务编号
    return j;
}
int tagOf(const Job& j) { return static_cast<int>(j.pose.time); }

// 等条件成立 (最多 ~2s); 返回是否等到
template <typename F>
bool waitUntil(F f, int timeout_ms = 2000)
{
    const auto t0 = clk::now();
    while (!f()) {
        if (clk::now() - t0 > std::chrono::milliseconds(timeout_ms)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

MaintainerConfig testCfg()
{
    MaintainerConfig c;
    c.enabled = true;
    c.voxel = 0.3;
    c.cell = 10.0;
    c.radius = 60.0;
    c.fitness_trigger = 0.99;
    c.min_confirm = 1;
    c.min_keyframe_dist = 0.0;
    c.persist_hits = 1;
    c.delta_min = 0.0;
    c.novel_min_cluster = 1;
    c.carve_stride = 1;
    c.decay_frames = 0;
    c.persistency_anchor = 128;
    c.shadow = true;
    c.persistency_unit = 0;
    c.persistency_hit_gain = 8;
    c.persistency_miss_confirm = 1;
    c.persistency_miss_action = 1;
    c.base_voxel_w = 100;
    return c;
}

PoseStamped mkPose(double x, double y = 0.0, double z = 0.0)
{
    PoseStamped p;
    p.trans = V3D(x, y, z);
    p.rot = M3D::Identity();
    return p;
}

CloudType::Ptr mkCloud(std::vector<V3D> pts)
{
    CloudType::Ptr c(new CloudType());
    for (const V3D& p : pts) {
        PointType q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        c->push_back(q);
    }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

MatchResult lowFitnessMatch()
{
    MatchResult mr;
    mr.converged = true;
    mr.fitness = 0.1;
    return mr;
}

}  // namespace

TEST(MapUpdateWorker, RunsJobsInSubmitOrder)
{
    std::mutex mu;
    std::vector<int> order;
    MapUpdateWorker w([&](Job& j) {
        std::lock_guard<std::mutex> lk(mu);
        order.push_back(tagOf(j));
    }, nullptr);
    // 不超载地投递 (队列没满才投下一帧): 一帧都不该丢, 且严格按提交顺序执行
    for (int i = 0; i < 40; ++i) {
        ASSERT_TRUE(waitUntil([&] { return w.stats().queued < 3; })) << "第 " << i << " 个之前队列一直是满的";
        w.submit(mkJob(Kind::Localized, i));
    }
    ASSERT_TRUE(waitUntil([&] { return w.stats().done == 40; }));
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(order.size(), 40u);
    for (int i = 0; i < 40; ++i) EXPECT_EQ(order[i], i) << "第 " << i << " 个";
    EXPECT_EQ(w.stats().dropped, 0u);
}

TEST(MapUpdateWorker, DropsOldestObservationWhenFull)
{
    std::mutex mu;
    std::vector<int> order;
    std::promise<void> gate;
    std::shared_future<void> released = gate.get_future().share();
    std::atomic<bool> first_started(false);
    const auto handler = [&](Job& j) {
        if (tagOf(j) == 1) {
            first_started.store(true);
            released.wait();
        }
        std::lock_guard<std::mutex> lk(mu);
        order.push_back(tagOf(j));
    };
    MapUpdateWorker w(handler, nullptr, /*capacity=*/2);
    w.submit(mkJob(Kind::Localized, 1));           // 立刻开始, 卡在栅栏上
    ASSERT_TRUE(waitUntil([&] { return first_started.load(); }));
    for (int i = 2; i <= 6; ++i) w.submit(mkJob(Kind::Localized, i));
    // 容量 2: 2,3 排队; 4 挤掉 2; 5 挤掉 3; 6 挤掉 4 -> 队列里是 5,6
    EXPECT_EQ(w.stats().queued, 2u);
    gate.set_value();
    ASSERT_TRUE(waitUntil([&] { return w.stats().done == 3; }));
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], 1);
    EXPECT_EQ(order[1], 5);
    EXPECT_EQ(order[2], 6);
    EXPECT_EQ(w.stats().dropped, 3u);
}

TEST(MapUpdateWorker, NeverDropsSettlementJobs)
{
    std::mutex mu;
    std::vector<int> order;
    std::promise<void> gate;
    std::shared_future<void> released = gate.get_future().share();
    std::atomic<bool> first_started(false);
    const auto handler = [&](Job& j) {
        if (tagOf(j) == 1) {
            first_started.store(true);
            released.wait();
        }
        std::lock_guard<std::mutex> lk(mu);
        order.push_back(tagOf(j));
    };
    MapUpdateWorker w(handler, nullptr, /*capacity=*/1);
    w.submit(mkJob(Kind::Localized, 1));
    ASSERT_TRUE(waitUntil([&] { return first_started.load(); }));
    w.submit(mkJob(Kind::Localized, 2));           // 占满唯一的观测名额
    w.submit(mkJob(Kind::Merge, 3));               // 结算: 直接入队
    w.submit(mkJob(Kind::Localized, 4));           // 挤掉最旧的观测 2 (结算 3 不动)
    w.submit(mkJob(Kind::Localized, 5));           // 挤掉观测 4
    w.submit(mkJob(Kind::Lifelong, 6));            // 结算: 直接入队
    gate.set_value();
    ASSERT_TRUE(waitUntil([&] { return w.stats().done == 4; }));
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order[0], 1);
    EXPECT_EQ(order[1], 3);
    EXPECT_EQ(order[2], 5);
    EXPECT_EQ(order[3], 6);
    EXPECT_EQ(w.stats().dropped, 2u);              // 只丢了观测 2 和 4
}

TEST(MapUpdateWorker, FlushDropsQueuedAndWaitsForInflight)
{
    std::promise<void> gate;
    std::shared_future<void> released = gate.get_future().share();
    std::atomic<bool> started(false);
    std::atomic<bool> finished(false);
    MapUpdateWorker w([&](Job&) {
        started.store(true);
        released.wait();
        finished.store(true);
    }, nullptr);
    w.submit(mkJob(Kind::Localized, 1));
    ASSERT_TRUE(waitUntil([&] { return started.load(); }));
    w.submit(mkJob(Kind::Localized, 2));
    w.submit(mkJob(Kind::Merge, 3));
    auto flushing = std::async(std::launch::async, [&] { w.flush(); });
    // 在途任务没做完: flush 不能返回
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(flushing.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
    EXPECT_EQ(w.stats().queued, 0u);               // 排队的已经丢掉
    gate.set_value();
    ASSERT_EQ(flushing.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_TRUE(finished.load());
    EXPECT_EQ(w.stats().done, 1u);
    EXPECT_EQ(w.stats().dropped, 1u);              // 丢掉的观测任务计数 (结算任务 3 也被清掉, 但不计入"丢弃")
}

TEST(MapUpdateWorker, IdleHookNeverRunsDuringAJobAndTicksWhenIdle)
{
    std::promise<void> gate;
    std::shared_future<void> released = gate.get_future().share();
    std::atomic<bool> started(false);
    std::atomic<int> idle_calls(0);
    MapUpdateWorker w([&](Job&) {
        started.store(true);
        released.wait();
    }, [&] { ++idle_calls; });
    w.submit(mkJob(Kind::Localized, 1));
    ASSERT_TRUE(waitUntil([&] { return started.load(); }));
    // 任务在做 (好几个节拍的时间): 维护回调一次都不能跑
    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    EXPECT_EQ(idle_calls.load(), 0);
    gate.set_value();
    // 没有任务了: 维护回调按节拍 (50ms) 推进
    EXPECT_TRUE(waitUntil([&] { return idle_calls.load() >= 3; }));
}

TEST(MapUpdateWorker, IdleHookStillTicksUnderContinuousJobStream)
{
    std::atomic<int> jobs(0);
    std::atomic<int> idle_calls(0);
    MapUpdateWorker w([&](Job&) { ++jobs; }, [&] { ++idle_calls; });
    // 每 5ms 来一个任务, 持续 400ms: "等满一拍才跑"的话永远等不到那一拍 —— 维护回调必须照样推进,
    // 而且仍然受节拍约束 (每个任务做完才查一次, 不会变成每个任务跑一次)。
    const auto t_end = clk::now() + std::chrono::milliseconds(400);
    while (clk::now() < t_end) {
        w.submit(mkJob(Kind::Localized, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_GT(jobs.load(), 20);
    EXPECT_GE(idle_calls.load(), 3);
    EXPECT_LE(idle_calls.load(), 11);
}

TEST(MapUpdateWorker, HandlerExceptionDoesNotKillThread)
{
    std::atomic<int> ok(0);
    MapUpdateWorker w([&](Job& j) {
        if (tagOf(j) == 1) throw std::runtime_error("boom");
        ++ok;
    }, nullptr);
    w.submit(mkJob(Kind::Localized, 1));
    w.submit(mkJob(Kind::Localized, 2));
    w.submit(mkJob(Kind::Merge, 3));
    ASSERT_TRUE(waitUntil([&] { return w.stats().done == 3; }));
    EXPECT_EQ(ok.load(), 2);
    EXPECT_EQ(w.stats().failed, 1u);
}

TEST(MapUpdateWorker, DestructorWaitsForInflightJob)
{
    std::promise<void> gate;
    std::shared_future<void> released = gate.get_future().share();
    std::atomic<bool> started(false);
    std::atomic<bool> finished(false);
    auto w = std::unique_ptr<MapUpdateWorker>(new MapUpdateWorker([&](Job&) {
        started.store(true);
        released.wait();
        finished.store(true);
    }, nullptr));
    w->submit(mkJob(Kind::Localized, 1));
    ASSERT_TRUE(waitUntil([&] { return started.load(); }));
    auto destroying = std::async(std::launch::async, [&] { w.reset(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    EXPECT_EQ(destroying.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
    EXPECT_FALSE(finished.load());
    gate.set_value();
    ASSERT_EQ(destroying.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_TRUE(finished.load());
    EXPECT_EQ(w, nullptr);
}

// ---- 真实账本 ----

TEST(OverlayMaintainerLockFreeViews, CachedCoverageEqualsComputed)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();

    // 还没写过: 缓存是 n/a (与现算一致)
    EXPECT_LT(m.cachedPersistencyCoverage(), 0.0);
    EXPECT_LT(m.persistencyCoverage(pose), 0.0);

    const uint64_t rev0 = m.revision();
    for (int i = 0; i < 16; ++i) m.onLocalized(pose, scan, mr, true);   // w = 8 + 15*8 = 128 -> 达标
    int age_c = -1, age = -1;
    size_t pts_c = 0, pts = 0;
    const double s_c = m.cachedPersistencyCoverage(&age_c, &pts_c);
    const double s = m.persistencyCoverage(pose, &age, &pts);
    ASSERT_DOUBLE_EQ(s, 1.0);
    EXPECT_DOUBLE_EQ(s_c, s);
    EXPECT_EQ(pts_c, pts);
    EXPECT_EQ(age_c, age);
    EXPECT_GE(m.revision(), rev0);
}

TEST(OverlayMaintainerLockFreeViews, ReadersDoNotWaitForMainLock)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0), V3D(1.3, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    for (int i = 0; i < 4; ++i) m.onLocalized(pose, scan, mr, true);
    // 一片含账本体素的底图: filterDeadBase 要真的去查判死集, 而不是"空集原样返回"的捷径
    CloudType::Ptr base(new CloudType());
    for (int i = 0; i < 2000; ++i) {
        PointType p;
        p.x = 0.001f * static_cast<float>(i);
        p.y = 0.0f;
        p.z = 0.0f;
        base->push_back(p);
    }
    base->width = static_cast<uint32_t>(base->size());
    base->height = 1;

    // 另一个线程持主锁做一次长标注 (300 万点, 实测 ~33ns/点 -> ~100ms)
    CloudType::Ptr big(new CloudType());
    for (long i = 0; i < 3000000; ++i) {
        PointType p;
        p.x = 0.001f * static_cast<float>(i % 3000);
        p.y = 50.0f + 0.001f * static_cast<float>(i / 3000);
        p.z = 0.0f;
        big->push_back(p);
    }
    big->width = static_cast<uint32_t>(big->size());
    big->height = 1;
    std::atomic<bool> holding(false);
    std::atomic<bool> holder_done(false);
    double holder_ms = 0.0;
    std::thread holder([&] {
        holding.store(true);
        const auto t0 = clk::now();
        m.annotateBaseWeights(*big);
        holder_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        holder_done.store(true);
    });
    ASSERT_TRUE(waitUntil([&] { return holding.load(); }));
    // 持锁期间: 跟踪线程的读视图必须立刻返回 (不拿主锁)
    const auto t0 = clk::now();
    int age = -1;
    size_t pts = 0;
    const double cov = m.cachedPersistencyCoverage(&age, &pts);
    const uint64_t rev = m.revision();
    const CloudType::Ptr filtered = m.filterDeadBase(base);
    const double reader_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    const bool overlapped = holding.load() && !holder_done.load();
    holder.join();
    EXPECT_GE(rev, 0u);
    EXPECT_TRUE(filtered != nullptr);
    (void)cov; (void)age; (void)pts;
    // 这次读与持锁区间重叠才有意义 (机器太快/太慢都会让测试失效, 那就明确报出来)
    ASSERT_TRUE(overlapped) << "持锁线程在读者之前就做完了 (holder_ms=" << holder_ms << ")";
    EXPECT_LT(reader_ms, holder_ms * 0.5)
        << "读视图用了 " << reader_ms << "ms, 持锁线程 " << holder_ms << "ms: 读视图在等主锁";
}

// 账本写入在地图更新线程上, 跟踪线程同时读视图 + 换图线程式的 flush: 不崩、不挂。
TEST(OverlayMaintainerLockFreeViews, ConcurrentWritesAndLockFreeReads)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0), V3D(1.3, 0.0, 0.0), V3D(0.7, 0.4, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    std::atomic<bool> stop(false);
    std::atomic<int> reads(0);
    {
        MapUpdateWorker w([&](Job&) { m.onLocalized(pose, scan, mr, true); },
                          [&] { m.reclaim(); });
        std::thread reader([&] {
            uint64_t last = 0;
            while (!stop.load()) {
                const uint64_t r = m.revision();
                EXPECT_GE(r, last);            // 版本号单调 (原子量)
                last = r;
                int age = 0;
                size_t pts = 0;
                const double c = m.cachedPersistencyCoverage(&age, &pts);
                EXPECT_TRUE(c < 0.0 || (c >= 0.0 && c <= 1.0));
                m.filterDeadBase(scan);
                ++reads;
            }
        });
        for (int i = 0; i < 300; ++i) {
            w.submit(mkJob(Kind::Localized, i));
            if (i % 25 == 24) {
                w.flush();
                Job j = mkJob(Kind::Merge, i);
                j.cloud = scan;
                w.submit(j);
            }
        }
        w.flush();
        stop.store(true);
        reader.join();
        EXPECT_GT(w.stats().done, 0u);
    }
    EXPECT_GT(reads.load(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
