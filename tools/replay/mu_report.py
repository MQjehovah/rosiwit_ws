#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回放对比 (地图更新线程 / RT, 2026-11-27): 帧耗时、队列积压与丢弃、账本行为。

    python3 mu_report.py <TAG> [<TAG> ...]

每个 TAG 对应一次 run_replay.sh 的产物: node_<TAG>.log / .mark。只统计回放窗口 (mark 之后)。
"""
import io
import re
import sys

R = '/tmp/opencode/replay'
ANSI = re.compile(r'\x1b\[[0-9;]*m')
NUM = r'-?[\d.]+(?:e[-+]\d+)?'
RE_FP1 = re.compile(r'\[FramePerf\] n=(\d+) ms p50/p90/max: wall=(%s)/(%s)/(%s) cpu=(%s)/(%s)/(%s) notrun=(%s)/(%s)/(%s)'
                    % ((NUM,) * 9))
RE_FP1B = re.compile(r'preempted/frame p50=(\d+) sum=([\d.]+)\s+blocked/frame p50=(\d+) sum=([\d.]+)')
RE_FP2 = re.compile(r'stages p50/p90/max: lock=[\d./]+ pre=[\d./]+ solver=(%s)/(%s)/(%s) eval=(%s)/(%s)/(%s) post=(%s)/(%s)/(%s)'
                    % ((NUM,) * 9))
RE_SUB = re.compile(r'(\w+)=([\d.]+)/([\d.]+)/([\d.]+)')
RE_LP = re.compile(r'\[LocPerf\] frames=(\d+) s2m_ms_ema=(%s) last_ms=(%s) queue=(\d+)' % (NUM, NUM))
RE_LP_NEW = re.compile(r'age_ms_ema=(%s) drop_total=(\d+) drop_50=(\d+)' % NUM)
RE_MU = re.compile(r'mapupd_ms=(%s) mapupd_ema=(%s) mapupd_max=(%s) mapupd_drop=(\d+) mapupd_fail=(\d+) mapupd_q=(\d+)' % (NUM, NUM, NUM))
RE_Q = re.compile(r'\[quality\] persistency_cov=(n/a|%s).*?novel_votes=(\d+) novel_commits=(\d+).*? fitness=(%s)' % (NUM, NUM))
RE_DEAD = re.compile(r'base voxels in w ledger: (\d+) \(dead=(\d+)')
RE_LOC = re.compile(r'\[locstate\] mode=\S+ st=(\d) matching=(%s)' % NUM)


def load(tag):
    lines = io.open('%s/node_%s.log' % (R, tag), encoding='utf-8', errors='replace').read().split('\n')
    mark = int(io.open('%s/node_%s.mark' % (R, tag)).read().strip() or 0)
    return [ANSI.sub('', l) for l in lines[mark:]], [ANSI.sub('', l) for l in lines[:mark]]


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, int(p * (len(s) - 1)))]


def report(tag):
    win, pre = load(tag)
    fp1 = [RE_FP1.search(l) for l in win if l.startswith('[FramePerf] n=')]
    fp1 = [m for m in fp1 if m]
    fp1b = [RE_FP1B.search(l) for l in win if l.startswith('[FramePerf] n=')]
    fp2 = [RE_FP2.search(l) for l in win if 'stages p50/p90/max' in l]
    print('== %s ==' % tag)
    if fp1:
        wall50 = [float(m.group(2)) for m in fp1]
        wall90 = [float(m.group(3)) for m in fp1]
        wallmax = [float(m.group(4)) for m in fp1]
        cpu50 = [float(m.group(5)) for m in fp1]
        cpu90 = [float(m.group(6)) for m in fp1]
        nr50 = [float(m.group(8)) for m in fp1]
        nrmax = [float(m.group(10)) for m in fp1]
        pre = [float(m.group(2)) for m in fp1b if m]
        blk = [float(m.group(4)) for m in fp1b if m]
        print('  [FramePerf] %d 个窗口 (每 100 帧), 共 %d 帧' % (len(fp1), sum(int(m.group(1)) for m in fp1)))
        print('    墙钟 p50: 中位 %.0f 最大 %.0f | 墙钟 p90: 中位 %.0f 最大 %.0f | 单帧最大 %.0f'
              % (pct(wall50, .5), max(wall50), pct(wall90, .5), max(wall90), max(wallmax)))
        print('    线程CPU p50: 中位 %.0f 最大 %.0f | CPU p90: 中位 %.0f' % (pct(cpu50, .5), max(cpu50), pct(cpu90, .5)))
        print('    没在运行 p50: 中位 %.0f 最大 %.0f | 单帧没在运行最大 %.0f' % (pct(nr50, .5), max(nr50), max(nrmax)))
        print('    每帧被抢占 均值 %.0f (合计 %.0f) | 每帧主动阻塞 均值 %.0f' % (
            sum(pre) / max(1, len(pre)), sum(pre), sum(blk) / max(1, len(blk))))
        print('    墙钟 p90 >= 200ms 的窗口: %d 个' % sum(1 for x in wall90 if x >= 200))
        print('    各窗口墙钟 p50/p90: ' + ' '.join('%.0f/%.0f' % (a, b) for a, b in zip(wall50, wall90)))
    if fp2:
        s50 = [float(m.group(1)) for m in fp2]
        s90 = [float(m.group(2)) for m in fp2]
        p50 = [float(m.group(7)) for m in fp2]
        p90 = [float(m.group(8)) for m in fp2]
        print('    求解 p50 中位 %.0f / p90 中位 %.0f | post p50 中位 %.0f / p90 中位 %.0f' % (
            pct(s50, .5), pct(s90, .5), pct(p50, .5), pct(p90, .5)))
    if any('post 内 p50/p90/max' in l for l in win):
        # 每个窗口的 post 子阶段 (p50/p90/max)
        rows = []
        for l in win:
            if 'post 内 p50/p90/max' in l:
                d = dict((k, (float(a), float(b), float(c))) for k, a, b, c in RE_SUB.findall(l.split('p50/p90/max:')[1]))
                rows.append(d)
        print('    各窗口 ledger(p50/p90/max): ' + ' '.join('%.1f/%.1f/%.1f' % r['ledger'] for r in rows if 'ledger' in r))
        print('    各窗口 tail  (p50/p90/max): ' + ' '.join('%.1f/%.1f/%.1f' % r['tail'] for r in rows if 'tail' in r))
        for name in ('covbase', 'session', 'merge'):
            v = [r[name] for r in rows if name in r]
            if v:
                print('    %-8s max 列最大 %.1f, p90 列最大 %.1f' % (name, max(x[2] for x in v), max(x[1] for x in v)))
    lp = []
    for l in win:
        if l.startswith('[LocPerf]'):
            m, n = RE_LP.search(l), RE_LP_NEW.search(l)
            if m:
                d = m.groups()
                extra = (float(n.group(1)), int(n.group(2)), int(n.group(3))) if n else None
                lp.append((int(d[0]), float(d[1]), float(d[2]), int(d[3]), extra))
    if lp:
        q = [x[3] for x in lp]
        print('  [LocPerf] %d 条, 处理到第 %d 帧 | s2m_ms_ema 最大 %.0f 末值 %.0f | queue 最大 %d, 队列深度>=2 的占 %.0f%%'
              % (len(lp), lp[-1][0], max(x[1] for x in lp), lp[-1][1], max(q), 100.0 * sum(1 for x in q if x >= 2) / len(q)))
        if lp[-1][4] is not None:
            age = [x[4][0] for x in lp]
            print('    帧龄 (队列等待) ema: 最大 %.0f ms, 均值 %.0f ms | m_loc_buf 累计丢弃 %d (窗口内 drop_50 合计 %d)'
                  % (max(age), sum(age) / len(age), lp[-1][4][1], sum(x[4][2] for x in lp)))
    mu = [RE_MU.search(l) for l in win if '[quality]' in l]
    mu = [m for m in mu if m]
    if mu:
        print('  地图更新线程: 任务耗时 ema 最大 %.0f ms (末值 %.0f), 单次最大 %.0f ms | 丢弃观测 %d, 失败 %d, 排队最大 %d'
              % (max(float(m.group(2)) for m in mu), float(mu[-1].group(2)), max(float(m.group(3)) for m in mu),
                 int(mu[-1].group(4)), int(mu[-1].group(5)), max(int(m.group(6)) for m in mu)))
    q = [RE_Q.search(l) for l in win if l.startswith('[quality]')]
    q = [m for m in q if m]
    dead = [RE_DEAD.search(l) for l in win]
    dead = [m for m in dead if m]
    commits = sum(int(m.group(3)) for m in q)
    cov = [float(m.group(1)) for m in q if m.group(1) != 'n/a']
    print('  账本: 新结构提交 (novel_commits 采样求和) %d | persistency_cov 均值 %s (n=%d) | dead_base 窗口内 %s -> %s'
          % (commits, ('%.3f' % (sum(cov) / len(cov))) if cov else 'n/a', len(cov),
             dead[0].group(2) if dead else '?', dead[-1].group(2) if dead else '?'))
    lost = sum(1 for l in win if 'unhealthy frames=' in l)
    recov = sum(1 for l in win if 'bounded recovery' in l or 'grid recovery' in l)
    st = [RE_LOC.search(l) for l in win]
    fits = [float(m.group(2)) for m in st if m and m.group(1) == '1' and float(m.group(2)) > 0]
    print('  定位: LOST %d 次, 恢复搜索 %d 次, 跟踪期 fitness 均值 %s 最低 %s (n=%d)' % (
        lost, recov, ('%.3f' % (sum(fits) / len(fits))) if fits else 'n/a', ('%.3f' % min(fits)) if fits else 'n/a', len(fits)))
    tb = [re.search(r'\[TargetBuild\] pre=(\d+) removed=(\d+)', l) for l in win if l.startswith('[TargetBuild]')]
    tb = [m for m in tb if m]
    if tb:
        rem = [int(m.group(2)) for m in tb]
        print('  建目标 %d 次: 判死剔除点数 末值 %d 最大 %d 中位 %d' % (len(tb), rem[-1], max(rem), pct(rem, .5)))
    ins = [re.search(r'\[InsProbe\] fit=(%s) anchored=(\d) shadow=\d coast=(\d).*? stable=(\d) tracking=(\d) acc=(\d).*? anchorOk=(\d)' % NUM, l)
           for l in win if l.startswith('[InsProbe]')]
    ins = [m for m in ins if m]
    if ins:
        print('  写图门 [InsProbe] %d 条: anchored %d, stable %d, tracking %d, accepted %d, anchorOk %d (都满足 = 本来就该写)'
              % (len(ins), sum(int(m.group(2)) for m in ins), sum(int(m.group(4)) for m in ins),
                 sum(int(m.group(5)) for m in ins), sum(int(m.group(6)) for m in ins), sum(int(m.group(7)) for m in ins)))
    print('  [quality] 行数 (1Hz, 仅被接受的帧才打): %d' % len(q))
    try:
        import json
        sm = json.load(open('%s/sampler_%s.json' % (R, tag)))
        w = sm['wall_s']
        print('  线程采样 %.0fs (整机CPU忙 %.0f%%):' % (w, sm['cpu_busy_pct']))
        for nm in ('slam_loc', 'slam_mapupd', 'slam_fe'):
            tl = sorted([t for t in sm['threads'] if t['name'] == nm], key=lambda t: -t['cpu_s'])
            for i, t in enumerate(tl):
                print('    %-12s #%d  %-12s prio=%-2d  CPU %5.1f%%  等调度 %5.1f%%  阻塞 %5.1f%%  被抢占 %.0f 次/s  主动阻塞 %.0f 次/s'
                      % (nm, i, t['policy'], t['rt_prio'], 100 * t['cpu_s'] / w, 100 * t['rq_wait_s'] / w,
                         100 * (t['sleep_s'] + t['d_s']) / w, t['nivcsw'] / w, t['vcsw'] / w))
    except (IOError, ValueError, KeyError) as e:
        print('  (无线程采样: %s)' % e)
    wm = [l for l in win if l.startswith('[working_map]')]
    for l in wm[:6]:
        print('  ' + l[:200])
    return win, None


for i, t in enumerate(sys.argv[1:]):
    if i:
        print()
    report(t)
