#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行驶报告: 从 xz_3d_slam 节点的 stdout 日志里, 把一次行驶的关键指标拆开。

    python3 tools/drive_report.py                                 # 最新启动的日志; 从起步前开始统计
    python3 tools/drive_report.py <日志路径> 02:39:40             # 指定日志与起始时刻 (今天)
    python3 tools/drive_report.py <日志> 02:39:40 --speed 0.6 0.95 --by-speed

各节看什么 (背景见 docs/SCHEME_PERSISTENT_MAP.md §3.1):
  侵蚀     被 w 账本判死的工作图点 (底图 + 已升格提交点)。两个精确来源: 账本的 [maint] dead_base (全局判死体素总数, 每 100 帧
           一条) 和 [TargetBuild] (每次建目标时这份裁剪里剔除了多少点)。健康: ~0 且不随时间涨;
           涨到上千还在线性增长 = 账本在删真实的墙。旧日志没有这两行时回退到近似配对 (见 report_erosion)。
  fitness  按主航向往返切趟、只取行驶中的直行帧, 在同一速度段内逐趟对比。fitness 是扫描侧统计的 (有
           地图点落在 inlier_dist 内的扫描点占比), 还随方向 (本机雷达安装朝向使两个方向差 ~0.05)、
           随速度 (0.3 m/s 与 0.8 m/s 差 ~0.05)、随位置变化 —— 不控速、不分方向就看不出地图的变化。
           --speed 指定速度段 (默认自动选"同速的完整趟"最多的 0.3 m/s 宽的一段); --by-speed 另印
           "速度 x 趟"的二维表。
  帧耗时   行驶中实际处理帧率 (输入 10 Hz)、单帧耗时分位数、队列积压, 以及匹配器的分段计时。
  新息/零偏 航向新息 (掉头时最大); 零偏三个分量应停在 ~0。
  子图     session_map 打开 / 切到子图 / 锚回底图 的事件。

只依赖 Python3 标准库。日志里多数行没有时间戳, 用最近一条带 ROS 时间 ([1795...]:) 的行对齐。
"""
import glob
import math
import os
import re
import sys
import time

# std::cout 打印的 double 小于 1e-4 时是科学计数法 (innov_yaw=6.1e-05): 只认 [\d.]+ 会把 6.1e-05 读成 6.1
NUM = r'(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)'

# 事件行: 维护器 / 子图 / 流水线自己的 [tag] 行。[quality]/[DropProbe] 等探针行里也有
# "session_map=" 这样的字段, 不是事件 -> 按 PROBE_PREFIX 排除。
EVENT_KEYS = ('unhealthy frames=', 'bounded recovery', 'grid recovery', 'localized after',
              '[session_map]', '[submap]', 'bridging', 're-anchored to overlay', '[anchor]',
              'save FAILED', 'working_map] saved', 'working_map] annotation done')
PROBE_PREFIX = re.compile(r'^\[(quality|DropProbe|StatusProbe|LocProbe|TargetBuild|YawTrans|pose_cmp|InsProbe|'
                          r'LocPerf|maint|odom|ImuDiag|FusProbe|MahalanobisProbe|YawTrace|FlightRecorder\])')

MIN_TREND_FRAMES = 40        # 一趟在一个速度段内至少这么多帧, 它的均值才有统计意义


def hms(t):
    return time.strftime('%H:%M:%S', time.localtime(t))


def pct(vals, p):
    return vals[min(len(vals) - 1, int(p * len(vals)))] if vals else float('nan')


def mean(vals):
    return sum(vals) / len(vals) if vals else float('nan')


def linfit(xs, ys):
    """最小二乘斜率 (点数 < 3 或 xs 无散布时返回 nan)。"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if n >= 3 and sxx > 0 else float('nan')


def sampled(seq, n):
    """等间隔抽最多 n 个 (保留首尾)。"""
    if len(seq) <= n:
        return seq
    return [seq[int(round(i * (len(seq) - 1) / float(n - 1)))] for i in range(n)]


def parse_args(argv):
    """[日志路径 [HH:MM:SS]] [--speed VMIN VMAX] [--by-speed] -> (位置参数, 选项)"""
    opts = {'speed': None, 'by_speed': False}
    pos, i = [], 0
    while i < len(argv):
        if argv[i] == '--speed' and i + 2 < len(argv):
            opts['speed'] = (float(argv[i + 1]), float(argv[i + 2]))
            i += 3
        elif argv[i] == '--by-speed':
            opts['by_speed'] = True
            i += 1
        else:
            pos.append(argv[i])
            i += 1
    return pos, opts


def section(title):
    print('\n' + '=' * 8 + ' ' + title + ' ' + '=' * 8)


class Log(object):
    """把日志解析成若干带时间戳的记录表 (dict 列表)。"""

    def __init__(self, path, t_start=None):
        any_ts = re.compile(r'\[(\d{10})\.\d+\]:')
        rx_loc = re.compile(r'\[locstate\] mode=\S+ st=(\d) matching=' + NUM +
                            r' pos=\(' + NUM + r',' + NUM + r',' + NUM + r'deg\) health=(\d)')
        rx_probe = re.compile(r'StatusProbe\].*?accepted=(\d).*?fitness=' + NUM +
                              r' innov_t=' + NUM + r' innov_yaw=' + NUM + r'.*?rejection=(\w+)')
        rx_imu = re.compile(r'ImuDiag\].*?yaw_rate=' + NUM)
        rx_perf = re.compile(r'\[LocPerf\] frames=(\d+) s2m_ms_ema=' + NUM +
                             r' last_ms=' + NUM + r' queue=(\d+)')
        # 2026-11-27 追加的字段 (旧日志没有): 帧龄 = 这帧在 m_loc_buf 里等了多久; 累计丢弃 / 最近 50 帧丢弃
        rx_perf2 = re.compile(r'age_ms_ema=' + NUM + r' drop_total=(\d+) drop_50=(\d+)')
        # 地图更新线程 (账本写入, 2026-11-27 起不在跟踪线程上): 任务耗时 / 丢弃的观测 / 排队
        rx_mupd = re.compile(r'mapupd_ms=' + NUM + r' mapupd_ema=' + NUM + r' mapupd_max=' + NUM +
                             r' mapupd_drop=(\d+) mapupd_fail=(\d+) mapupd_q=(\d+)')
        rx_qual = re.compile(r'\[quality\].*?cov_base=' + NUM + r'.*?novel_votes=(\d+) novel_commits=(\d+)'
                             r'.*?anchor=(\w+) inl=(\d+) vis=(\d+)')
        rx_build = re.compile(r'\[LocProbe\] local_map_points=(\d+) pose=\(' + NUM + r',\s*' + NUM + r'\)')
        rx_tbuild = re.compile(r'\[TargetBuild\] pre=(\d+) removed=(\d+) pose=\(' + NUM + r',\s*' + NUM + r'\)')
        rx_maint = re.compile(r'\[maint\] .*?commits=(\d+)(?: dead_base=(\d+))?')
        rx_base = re.compile(r'\[overlay\] base voxels in w ledger: (\d+) \(dead=(\d+)')
        rx_drop = re.compile(r'\[DropProbe\] t=[\d.]+ .*?target_points=(\d+)')
        rx_yaw = re.compile(r'\[YawTrans\].*?bias3=\(' + NUM + r',' + NUM + r',' + NUM + r'\)')
        rx_mprobe = re.compile(r'\[MatchProbe\] match_index=\d+ scan_points=(\d+) source_points=(\d+) '
                               r'target_points=(\d+) downsample_ms=' + NUM + r' dynamic_filter_ms=' + NUM +
                               r' gicp_ms=' + NUM + r' evaluate_ms=' + NUM + r' total_ms=' + NUM)
        self.poses, self.probes, self.perfs, self.quals = [], [], [], []
        self.builds, self.drops, self.yaws, self.events, self.mprobe = [], [], [], [], []
        self.tbuilds, self.dead, self.base_voxels = [], [], None
        self.wr = 0.0            # 最近一帧的 |机体角速度| (ImuDiag, 2s 一条)
        last = None
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                m = any_ts.search(line)
                if m:
                    last = int(m.group(1))
                t = last
                if t is None or (t_start is not None and t < t_start):
                    # [maint] 的 dead_base 与加载时的底图体素数是全程累计量: 起始时刻之前的也要收
                    if line.startswith('[maint]'):
                        mm = rx_maint.search(line)
                        if mm and mm.group(2) is not None:
                            self.dead.append((t, int(mm.group(2))))
                    elif line.startswith('[overlay] base voxels in w ledger:'):
                        mm = rx_base.search(line)
                        if mm:
                            self.base_voxels = int(mm.group(1))
                    continue
                if line.startswith('[quality]'):
                    m = rx_qual.search(line)
                    if m:
                        mu = rx_mupd.search(line)
                        self.quals.append(dict(t=t, cov=float(m.group(1)), nc=int(m.group(3)),
                                               anchor=m.group(4), inl=int(m.group(5)), vis=int(m.group(6)),
                                               w=self.wr,
                                               mu=(float(mu.group(2)), float(mu.group(3)), int(mu.group(4)),
                                                   int(mu.group(5)), int(mu.group(6))) if mu else None))
                elif line.startswith('[DropProbe'):
                    m = rx_drop.search(line)
                    if m:
                        self.drops.append(dict(t=t, post=int(m.group(1))))
                elif 'StatusProbe' in line:
                    m = rx_probe.search(line)
                    if m:
                        self.probes.append(dict(t=t, acc=int(m.group(1)), iy=float(m.group(4)), rej=m.group(5)))
                elif line.startswith('[LocPerf'):
                    m = rx_perf.search(line)
                    if m:
                        m2 = rx_perf2.search(line)
                        self.perfs.append(dict(t=t, n=int(m.group(1)), ema=float(m.group(2)), ms=float(m.group(3)),
                                               q=int(m.group(4)),
                                               age=float(m2.group(1)) if m2 else None,
                                               dtot=int(m2.group(2)) if m2 else None,
                                               d50=int(m2.group(3)) if m2 else None))
                elif line.startswith('[MatchProbe]'):
                    m = rx_mprobe.search(line)
                    if m:
                        self.mprobe.append(dict(t=t, src=int(m.group(2)), tgt=int(m.group(3)), gicp=float(m.group(6)),
                                                ev=float(m.group(7)), tot=float(m.group(8))))
                elif line.startswith('[TargetBuild]'):
                    m = rx_tbuild.search(line)
                    if m:
                        self.tbuilds.append(dict(t=t, pre=int(m.group(1)), rm=int(m.group(2)),
                                                 x=float(m.group(3)), y=float(m.group(4))))
                elif line.startswith('[LocProbe] local_map_points='):
                    m = rx_build.search(line)
                    if m:
                        self.builds.append(dict(t=t, pre=int(m.group(1)), x=float(m.group(2)), y=float(m.group(3))))
                elif line.startswith('[maint]'):
                    m = rx_maint.search(line)
                    if m and m.group(2) is not None:
                        self.dead.append((t, int(m.group(2))))
                elif line.startswith('[overlay] base voxels in w ledger:'):
                    m = rx_base.search(line)
                    if m:
                        self.base_voxels = int(m.group(1))
                elif line.startswith('[YawTrans]'):
                    m = rx_yaw.search(line)
                    if m:
                        self.yaws.append([abs(float(v)) for v in m.groups()])
                elif 'ImuDiag' in line:
                    m = rx_imu.search(line)
                    if m:
                        self.wr = abs(float(m.group(1)))
                elif 'locstate] mode' in line:
                    m = rx_loc.search(line)
                    if m:
                        self.poses.append(dict(t=t, fit=float(m.group(2)), x=float(m.group(3)),
                                               y=float(m.group(4)), yaw=float(m.group(5)), health=int(m.group(6))))
                elif not PROBE_PREFIX.match(line) and any(k in line for k in EVENT_KEYS):
                    self.events.append((t, line.rstrip()))
        for i, p in enumerate(self.poses):
            if i == 0:
                p['v'] = 0.0
            else:
                dt = p['t'] - self.poses[i - 1]['t']
                p['v'] = math.hypot(p['x'] - self.poses[i - 1]['x'], p['y'] - self.poses[i - 1]['y']) / dt \
                    if dt > 0 else 0.0

    def nearest_pose(self, t):
        """二分找最近的位姿 (poses 按时间有序)。"""
        ps = self.poses
        if not ps:
            return None
        lo, hi = 0, len(ps)
        while lo < hi:
            mid = (lo + hi) // 2
            if ps[mid]['t'] < t:
                lo = mid + 1
            else:
                hi = mid
        best = None
        for k in (lo - 1, lo):
            if 0 <= k < len(ps) and (best is None or abs(ps[k]['t'] - t) < abs(best['t'] - t)):
                best = ps[k]
        return best


def split_laps(poses):
    """按主航向往返切趟: 沿主轴的前进方向反转 (每个方向至少走 3m) 即新的一趟。给每个位姿加
    's' (沿主轴坐标) 和 'lap'。返回趟数。"""
    mx = mean([p['x'] for p in poses])
    my = mean([p['y'] for p in poses])
    sxx = sum((p['x'] - mx) ** 2 for p in poses)
    syy = sum((p['y'] - my) ** 2 for p in poses)
    sxy = sum((p['x'] - mx) * (p['y'] - my) for p in poses)
    th = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ax, ay = math.cos(th), math.sin(th)
    for p in poses:
        p['s'] = (p['x'] - mx) * ax + (p['y'] - my) * ay
    starts, dirn, ext, t0 = [], 0, poses[0]['s'], poses[0]['t']
    for p in poses:
        s = p['s']
        if dirn == 0:
            if abs(s - ext) > 3.0:
                dirn = 1 if s > ext else -1
            continue
        if (s - ext) * dirn > 0:
            ext = s
            continue
        if abs(s - ext) > 3.0:
            starts.append(t0)
            t0, dirn, ext = p['t'], -dirn, s
    starts.append(t0)
    for p in poses:
        p['lap'] = max(0, sum(1 for a in starts if p['t'] >= a) - 1)
    return len(starts)


def lap_shape(poses, n_laps):
    """每趟的 (起始时刻, 方向 A/B, 行驶范围 m, 行驶帧数)。方向 A = 第 1 趟的行驶方向;
    范围 = 行驶帧 (v>0.15) 沿主轴的 s_max - s_min。"""
    info = []
    for k in range(n_laps):
        mv = [p for p in poses if p['lap'] == k and p['v'] > 0.15]
        rng = (max(p['s'] for p in mv) - min(p['s'] for p in mv)) if len(mv) >= 20 else 0.0
        info.append(dict(t=mv[0]['t'] if mv else poses[-1]['t'],
                         sign=(1 if mv[-1]['s'] >= mv[0]['s'] else -1) if len(mv) >= 2 else 0,
                         rng=rng, n=len(mv)))
    ref = next((i['sign'] for i in info if i['sign']), 1)
    for i in info:
        i['dir'] = 'A' if i['sign'] == ref else 'B'
    return info


def report_erosion(log):
    section('侵蚀 (被 w 账本判死的工作图点)')
    # 1) 全局: 账本自己记的判死体素总数 (底图 + 已升格进工作图的提交点; 精确;
    #    每 100 帧一条, 统计区间之前的也收, 因为是累计量)
    if log.dead:
        dead = log.dead
        print('判死的工作图体素总数 (账本 [maint], 每 100 帧一条): %d (%s) -> %d (%s)%s' % (
            dead[0][1], hms(dead[0][0]), dead[-1][1], hms(dead[-1][0]),
            ', 占底图 %.2f%%' % (100.0 * dead[-1][1] / log.base_voxels) if log.base_voxels else ''))
        print('  ' + '  '.join('%s:%d' % (hms(t), d) for t, d in sampled(dead, 12)))
        moving = [d for t, d in dead if log.nearest_pose(t) is not None and log.nearest_pose(t)['v'] > 0.1]
        if len(moving) >= 4 and moving[-1] - moving[0] > 500:
            print('  [警告] 行驶期间判死数持续增长 (+%d): 账本在删工作图。真有东西被挪走时应该很快饱和, '
                  '线性增长 = 在删真实的墙' % (moving[-1] - moving[0]))
    # 2) 每次建目标: 这份裁剪里剔除了多少 (精确)
    if log.tbuilds:
        print('每次建目标 (账本 [TargetBuild]):')
        print('  %-9s %10s %9s %7s   目标中心' % ('时刻', '裁剪点数', '判死剔除', '占比'))
        for b in sampled(log.tbuilds, 40):
            print('  %-9s %10d %9d %6.1f%%   (%.0f, %.0f)' % (
                hms(b['t']), b['pre'], b['rm'], 100.0 * b['rm'] / max(1, b['pre']), b['x'], b['y']))
        worst = max(log.tbuilds, key=lambda b: b['rm'] / max(1, b['pre']))
        print('-> %d 次构建, 单次剔除占比最大 %.1f%% (%s, 判死 %d / %d)%s' % (
            len(log.tbuilds), 100.0 * worst['rm'] / max(1, worst['pre']), hms(worst['t']), worst['rm'],
            worst['pre'], '   [注意] 单次构建剔除了 5% 以上' if worst['rm'] >= 0.05 * worst['pre'] else ''))
    elif log.builds:
        # 近似 (旧日志): [LocProbe] 是每 50 次调用采样一条的"裁剪点数", 目标按 15m 重建半径滚动 ——
        # 它和之后某帧的 target_points 常常不是同一份裁剪, 差值带有位置造成的误差 (几个百分点)。
        print('近似 (旧日志, 没有 [TargetBuild]): 采样的裁剪点数 -> 1~20s 后目标点数, 两个数可能来自不同位置的裁剪')
        print('  (11-27 那趟的近似值给出 "判死 47~1109", 但文件逐点对比是 0: 当时判死点落盘时被写成 128'
              ' (11-28 修复, 见 docs/HANDOVER_2026-11-27.md §1.6); 现在以 tools/wmap_diff.py 为准)')
        rows = []
        for b in log.builds:
            d = next((d for d in log.drops if b['t'] + 1.0 <= d['t'] <= b['t'] + 20.0), None)
            if d is not None:
                rows.append((b, d['post']))
        if not rows:
            print('区间内没有目标构建 (机器人没移动超过重建半径)。')
            return
        print('  %-9s %10s %10s %9s %7s   采样位置' % ('时间', '裁剪点数', '目标点数', '差值', '占比'))
        for b, post in rows:
            print('  %-9s %10d %10d %9d %6.1f%%   (%.0f, %.0f)' % (
                hms(b['t']), b['pre'], post, b['pre'] - post, 100.0 * (b['pre'] - post) / max(1, b['pre']),
                b['x'], b['y']))
        diff = [b['pre'] - post for b, post in rows]
        if len(diff) >= 3 and min(diff) > 0.03 * min(b['pre'] for b, _ in rows):
            print('-> [警告] 每次采样都有 3% 以上的底图点被剔除: 账本在侵蚀底图')
    else:
        print('区间内没有目标构建 (机器人没移动超过重建半径)。')


def report_fitness(log, ps, n_laps, opts):
    # fitness 是扫描侧统计的 (有地图点落在 inlier_dist 内的扫描点占比): 地图点被判死 -> 真实表面上的
    # 扫描点失去地图邻居 -> 直接掉分。但 fitness 还随方向 / 速度 / 位置变化, 只能拿"同方向、同速度"的
    # 完整趟来比。
    info = lap_shape(ps, n_laps)
    full = {k for k, i in enumerate(info) if i['rng'] >= 0.6 * max(i['rng'] for i in info)}
    items = []
    for r in log.quals:
        p = log.nearest_pose(r['t'])
        if p is not None and r['vis'] and r['w'] < 0.12 and p['v'] > 0.15:
            items.append((p['lap'], p['v'], r['inl'] / r['vis'], r['cov'], r['nc'], r['anchor'] != 'OK'))

    def band_stats(a0, a1):
        """速度段 [a0, a1) 内每趟的 (帧数, 平均速度, fitness, cov_base, 提交, anchor 弱帧数)。"""
        st = {}
        for k in range(n_laps):
            sel = [it for it in items if it[0] == k and a0 <= it[1] < a1]
            if len(sel) >= 5:
                st[k] = (len(sel), mean([it[1] for it in sel]), mean([it[2] for it in sel]),
                         mean([it[3] for it in sel if it[3] >= 0]), sum(it[4] for it in sel),
                         sum(1 for it in sel if it[5]))
        return st

    def trend_laps(st, a0, a1, central):
        """参与趋势的趟: 完整、帧数够, 且 (自动选段时) 平均速度落在速度段中段 —— 各趟真是同速。"""
        return [k for k in sorted(st) if k in full and st[k][0] >= MIN_TREND_FRAMES
                and (not central or a0 + 0.05 <= st[k][1] < a1 - 0.05)]

    if opts['speed']:
        lo, hi = opts['speed']
        st = band_stats(lo, hi)
        keep = trend_laps(st, lo, hi, False)
    elif items:
        # 自动: 在 0.3 m/s 宽的速度段里, 选"同速的完整趟"最多的一段 (并列取这些趟的总帧数多者)
        best = None
        for c in range(1, 16):
            a0, a1 = c / 10.0, (c + 3) / 10.0
            s0 = band_stats(a0, a1)
            k0 = trend_laps(s0, a0, a1, True)
            key = (len(k0), sum(s0[k][0] for k in k0))
            if best is None or key > best[0]:
                best = (key, (a0, a1), s0, k0)
        if best[0][0] == 0:                 # 没有往返 (或速度很杂): 退回全部行驶帧
            lo, hi = 0.0, float('inf')
            st, keep = band_stats(lo, hi), []
            keep = trend_laps(st, lo, hi, False)
        else:
            (lo, hi), st, keep = best[1], best[2], best[3]
    else:
        lo, hi, st, keep = 0.0, float('inf'), {}, []

    section('fitness 逐趟 (直行帧 |角速度|<0.12 rad/s, 同速段 %.2f~%.2f m/s; fitness = inliers/visible)' % (lo, hi))
    print('方向 A = 第 1 趟的行驶方向, B = 反向; * = 不参与趋势 (不完整的趟 / 该速度段内不足 %d 帧 / 各趟均速不齐)'
          % MIN_TREND_FRAMES)
    print('%-4s %-9s %-4s %5s %6s %7s %6s %6s %7s' % ('趟', '起', '方向', '帧数', '平均v', 'fitness', 'cov_b', '提交', 'anchor弱'))
    for k in sorted(st):
        n, v, f, cov, nc, weak = st[k]
        print('%-3d%s %-9s %-4s %5d %6.2f %7.3f %6.2f %6d %7d' % (
            k + 1, ' ' if k in keep else '*', hms(info[k]['t']), info[k]['dir'], n, v, f, cov, nc, weak))
    if not keep:
        print('-> 没有可跨趟比较的完整趟 (需要往返行驶, 且各趟速度相近; 可用 --speed 指定速度段)')
    for d in ('A', 'B'):
        ks = [k for k in keep if info[k]['dir'] == d]
        if len(ks) < 3:
            continue
        drop = mean([st[k][2] for k in ks[:2]]) - mean([st[k][2] for k in ks[-2:]])
        print('-> 方向 %s (%d 趟, 均速 %.2f~%.2f m/s): 首 %.3f -> 末 %.3f, 斜率 %+.4f / 趟%s' % (
            d, len(ks), min(st[k][1] for k in ks), max(st[k][1] for k in ks),
            st[ks[0]][2], st[ks[-1]][2], linfit(ks, [st[k][2] for k in ks]),
            '   [警告] 同速同向的 fitness 随趟数下降 (前两趟均值比末两趟高 %.3f)' % drop if drop > 0.02 else ''))

    if opts['by_speed']:
        section('fitness 速度 x 趟 (0.1 m/s 一档; 每档 >= 8 帧才显示)')
        grid = {}
        for k, v, f, _, _, _ in items:
            grid.setdefault((k, int(v * 10)), []).append(f)
        cols = sorted(set(c for _, c in grid))
        print('%-7s' % '' + ''.join('%6.1f' % (c / 10.0) for c in cols) + '   <- m/s')
        for k in range(n_laps):
            if k not in st:
                continue
            print('%-3d %-4s ' % (k + 1, info[k]['dir'] + ('' if k in full else '*')) + ''.join(
                '%6.3f' % (sum(g) / len(g)) if len(g) >= 8 else '     -'
                for g in (grid.get((k, c), []) for c in cols)))


def report_timing(log, ps):
    section('帧耗时 (行驶中; 10Hz 输入, 持续 >100ms 即开始积压)')
    moving_t = [p['t'] for p in ps if p['v'] > 0.1]
    perfs = [p for p in log.perfs if moving_t and moving_t[0] <= p['t'] <= moving_t[-1]]
    ms = sorted(p['ms'] for p in perfs if p['ms'] > 0)
    if not ms:
        print('区间内没有行驶帧。')
        return
    rate = sorted((b['n'] - a['n']) / float(b['t'] - a['t']) for a, b in zip(perfs, perfs[1:])
                  if b['n'] > a['n'] and b['t'] > a['t'])
    print('单帧 (每 200 帧取 1 帧): p50=%.0f  p90=%.0f  p99=%.0f  max=%.0f ms   队列: 平均 %.2f 最大 %d' % (
        pct(ms, 0.5), pct(ms, 0.9), pct(ms, 0.99), ms[-1], mean([p['q'] for p in perfs]),
        max(p['q'] for p in perfs)))
    if rate:
        print('实际处理帧率: 最低 %.1f  中位 %.1f  最高 %.1f Hz   (低于 10 = 丢帧 + 位姿延迟)' % (
            rate[0], pct(rate, 0.5), rate[-1]))
    # 2026-11-27 起节点自己报 m_loc_buf 的丢弃与帧龄 (之前这里的丢弃在日志里看不见: [slam_algo] input 1s
    # 报的是 ROS 层通道, 不是跟踪线程的队列)
    aged = [p for p in perfs if p.get('age') is not None]
    if aged:
        drop = aged[-1]['dtot'] - aged[0]['dtot']
        age = sorted(p['age'] for p in aged)
        print('跟踪队列: 区间内丢弃 %d 帧 (输入 10Hz x %.0fs = %d 帧)   帧龄 (入队→开始处理) ema: p50 %.0f  最大 %.0f ms'
              % (drop, aged[-1]['t'] - aged[0]['t'], int((aged[-1]['t'] - aged[0]['t']) * 10),
                 pct(age, 0.5), age[-1]))
    mu = [q['mu'] for q in log.quals if q.get('mu') and moving_t[0] <= q['t'] <= moving_t[-1]]
    if mu:   # 每项: (任务耗时 ema, 单次最大, 累计丢弃观测, 累计失败, 当前排队)
        print('地图更新线程 (账本写入, 不在跟踪线程上): 任务耗时 ema p50 %.0f  最大 %.0f ms, 单次最大 %.0f ms; '
              '区间内丢弃观测 %d, 失败 %d, 排队最大 %d'
              % (pct(sorted(m[0] for m in mu), 0.5), max(m[0] for m in mu), max(m[1] for m in mu),
                 mu[-1][2] - mu[0][2], mu[-1][3] - mu[0][3], max(m[4] for m in mu)))
    mp = [m for m in log.mprobe if moving_t[0] <= m['t'] <= moving_t[-1]]
    if mp:
        print('匹配器分段 (每 512 帧取 1 帧; gicp = 求解, 评估 = 覆盖率统计, 每个扫描点一次最近邻查询):')
        print('  %-9s %8s %9s %9s %9s %9s' % ('时刻', '扫描点', '地图点', 'gicp ms', '评估 ms', '合计 ms'))
        for m in mp:
            print('  %-9s %8d %9d %9.1f %9.1f %9.1f' % (hms(m['t']), m['src'], m['tgt'], m['gicp'], m['ev'], m['tot']))


def main():
    args, opts = parse_args(sys.argv[1:])
    if not args:
        logs = glob.glob('/tmp/[0-9a-f]*-*/xz_3d_slam-*-stdout.log')
        if not logs:
            sys.exit('找不到日志 (默认路径 /tmp/<uuid>-*/xz_3d_slam-*-stdout.log)')
        args = [max(logs, key=os.path.getmtime)]
    path = args[0]
    t0 = int(time.mktime(time.strptime(time.strftime('%Y-%m-%d ') + args[1], '%Y-%m-%d %H:%M:%S'))) \
        if len(args) > 1 else None
    log = Log(path, t0)
    if not log.poses:
        sys.exit('日志里没有位姿行: ' + path)
    if t0 is None:
        # 默认从起步前开始 (之前是静止 / 启动定位)。起步 = 连续 10 个位姿里 >= 6 个在动;
        # 启动瞬间定位跳变只动 1 个位姿, 不算。
        first = next((i for i in range(len(log.poses) - 10)
                      if sum(1 for p in log.poses[i:i + 10] if p['v'] > 0.1) >= 6), None)
        if first is not None and first > 5:
            log = Log(path, log.poses[first - 1]['t'])
    ps = log.poses
    n_laps = split_laps(ps)

    section('概览')
    print('日志: %s' % path)
    print('统计区间: %s ~ %s (%d 秒), 其中行驶 %d 秒, 路径长 %.0f m, 往返 %d 趟' % (
        hms(ps[0]['t']), hms(ps[-1]['t']), ps[-1]['t'] - ps[0]['t'], sum(1 for p in ps if p['v'] > 0.1),
        sum(math.hypot(ps[i]['x'] - ps[i - 1]['x'], ps[i]['y'] - ps[i - 1]['y']) for i in range(1, len(ps))), n_laps))
    ev = log.events

    def cnt(*keys):
        return sum(1 for _, l in ev if any(k in l for k in keys))

    print('LOST: %d   网格恢复: %d   落盘: %d   落盘失败: %d' % (
        cnt('unhealthy frames='), cnt('bounded recovery'), cnt('working_map] saved'), cnt('save FAILED')))
    print('子图: 打开 %d   切到子图(桥接) %d   锚回底图 %d' % (
        cnt('[session_map] open'), cnt('enter bridging', '-> bridging'), cnt('leave bridging', 're-anchored to overlay')))

    report_erosion(log)
    report_fitness(log, ps, n_laps, opts)
    report_timing(log, ps)

    section('航向新息 / 零偏 / 健康')
    iy = sorted(p['iy'] for p in log.probes)
    if iy:
        print('新息 yaw: mean=%.3f  p99=%.2f  max=%.2f (度)   >2°: %d   >5°: %d   被拒采样: %d / %d' % (
            mean(iy), pct(iy, 0.99), iy[-1], sum(1 for v in iy if v > 2), sum(1 for v in iy if v > 5),
            sum(1 for p in log.probes if not p['acc']), len(iy)))
    if log.yaws:
        print('零偏 |max|: x=%.4f  y=%.4f  z=%.4f rad/s   (应停在 ~0)' % (
            max(y[0] for y in log.yaws), max(y[1] for y in log.yaws), max(y[2] for y in log.yaws)))
    print('health != 1 的秒数: %d / %d' % (sum(1 for p in ps if p['health'] != 1), len(ps)))

    section('子图 / 桥接 / 丢失 事件')
    sub = [(t, l) for t, l in ev if any(k in l for k in ('[session_map]', '[submap]', 'bridging', 're-anchor',
                                                         '[anchor]', 'LOST', 'recovery'))]
    for t, l in sub[:40]:
        print('%s  %s' % (hms(t), re.sub(r'^\[[^]]*\]\[[A-Z]\]\[[A-Za-z]+\] ', '', l)[:150]))
    print('(共 %d 条)' % len(sub) if sub else '(无)')


if __name__ == '__main__':
    main()
