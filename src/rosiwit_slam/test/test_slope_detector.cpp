// SlopeDetector: 双阈值迟滞 + 地面优先、IMU 仅在无地面时兜底。
//
// 断言:
//   1) 平地不触发
//   2) 超过 enter 阈值但不足 enter_frames 帧 -> 仍平地
//   3) 满 enter_frames 帧 -> 进坡
//   4) 回落到 enter/exit 之间, 不足 exit_frames 帧 -> 仍坡上
//   5) 满 exit_frames 帧 -> 回平地
//   6) 无地面且 IMU 倾角异常(80deg) -> 保持原状态, 不误判
#include <cmath>
#include <cstdio>

#include "algorithms/fast_lio2/slope_detector.h"

using namespace rosiwit_slam;

namespace {
int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %s %s\n", ok ? "[ok]" : "[FAIL]", what);
    if (!ok) ++fails;
}
constexpr double kDeg = M_PI / 180.0;
}  // namespace

int main() {
    SlopeGateConfig cfg;
    cfg.enter_deg = 10.0;
    cfg.exit_deg = 6.0;
    cfg.enter_frames = 5;
    cfg.exit_frames = 8;

    SlopeDetector det(cfg);

    check(!det.onSlope(), "平地初始为不在坡上");
    for (int i = 0; i < 20; ++i) det.update(true, 0.5 * kDeg, 0.5 * kDeg);
    check(!det.onSlope(), "平地持续输入不触发");

    for (int i = 0; i < cfg.enter_frames - 1; ++i) det.update(true, 12.0 * kDeg, 12.0 * kDeg);
    check(!det.onSlope(), "12deg 但帧数不足 enter_frames, 仍平地(迟滞)");

    det.update(true, 12.0 * kDeg, 12.0 * kDeg);
    check(det.onSlope(), "满 enter_frames 帧后进坡");

    for (int i = 0; i < cfg.exit_frames - 1; ++i) det.update(true, 3.0 * kDeg, 3.0 * kDeg);
    check(det.onSlope(), "回落到 3deg 但不足 exit_frames, 仍在坡上");

    det.update(true, 3.0 * kDeg, 3.0 * kDeg);
    check(!det.onSlope(), "满 exit_frames 帧后回平地");

    // 重新进坡, 再用"无地面 + 异常 IMU"检查不会误改状态
    for (int i = 0; i < cfg.enter_frames; ++i) det.update(true, 15.0 * kDeg, 15.0 * kDeg);
    check(det.onSlope(), "再次进坡");
    for (int i = 0; i < 50; ++i) det.update(false, 0.0, 80.0 * kDeg);
    check(det.onSlope(), "无地面且 IMU=80deg 时保持坡上, 不误判");

    std::printf(fails == 0 ? "ALL PASS\n" : "FAILED %d\n", fails);
    return fails == 0 ? 0 : 1;
}
