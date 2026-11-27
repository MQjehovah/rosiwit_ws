#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""线程状态采样: 把一个进程里各线程的墙钟时间分成 在 CPU 上 / 在就绪队列里等调度 / 阻塞。

    python3 thread_sampler.py <pid> <秒数> <输出.json> [采样率 Hz=500] [线程名过滤=slam]

本机内核 (PREEMPT_RT) 没编 SCHEDSTATS / SCHED_DEBUG, 拿不到内核的"就绪队列等待时间"计数器,
所以从外面高频读 /proc/<pid>/task/<tid>/stat:
    state  R = 正在运行或在就绪队列里;  S = 可中断睡眠 (等锁 / 等工作线程 / 等 IO);  D = 不可中断
    utime + stime  线程的 CPU 时间 (时钟中断采样, 10ms 粒度; 窗口长了以后统计上是准的)
    ->  在 CPU 上 ~= (utime+stime) 的增量;  等调度 ~= R 状态的时间 - 在 CPU 上;  阻塞 ~= S (+D) 状态的时间
只能给线程自己做不了的决定: "没在运行的那些时间, 是在排队还是在等别人"。
与节点自己的 [FramePerf] (墙钟 / 本线程 CPU / 被抢占次数) 交叉核对。
"""
import json
import os
import sys
import time


def read_stat(path):
    try:
        with open(path, 'rb') as f:
            s = f.read().decode('ascii', 'replace')
    except IOError:
        return None
    rp = s.rfind(')')
    comm = s[s.find('(') + 1:rp]
    rest = s[rp + 2:].split(' ')
    # rest[0] = state (第 3 字段); 第 n 字段 = rest[n - 3]: 14/15 = utime/stime, 41 = policy, 42 = rt_priority
    return comm, rest[0], int(rest[11]), int(rest[12]), int(rest[38]), int(rest[39])


def policy_name(p):
    return {0: 'SCHED_OTHER', 1: 'SCHED_FIFO', 2: 'SCHED_RR', 3: 'SCHED_BATCH', 5: 'SCHED_IDLE'}.get(p, str(p))


def read_ctx(tid_path):
    """-> (voluntary, nonvoluntary) 上下文切换累计次数 (/proc/<pid>/task/<tid>/status)"""
    vc = nvc = 0
    try:
        with open(tid_path + '/status') as f:
            for line in f:
                if line.startswith('voluntary_ctxt_switches:'):
                    vc = int(line.split()[1])
                elif line.startswith('nonvoluntary_ctxt_switches:'):
                    nvc = int(line.split()[1])
    except IOError:
        pass
    return vc, nvc


def list_threads(pid, name_filter):
    out = {}
    base = '/proc/%d/task' % pid
    try:
        for tid in os.listdir(base):
            r = read_stat(base + '/' + tid + '/stat')
            if r and (not name_filter or r[0] in name_filter):
                out[tid] = r
    except OSError:
        pass
    return out


def cpu_busy_delta():
    with open('/proc/stat') as f:
        parts = f.readline().split()[1:]
    vals = [int(x) for x in parts]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def main():
    pid, dur, out_path = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    hz = float(sys.argv[4]) if len(sys.argv) > 4 else 500.0
    # 线程名精确匹配 (逗号分隔)。不能用子串: 进程名 rosiwit_slam_ros1_node 自己就含 "slam", 所有线程都会被选中。
    name_filter = sys.argv[5].split(',') if len(sys.argv) > 5 else ['slam_loc', 'slam_fe']
    clk_tck = os.sysconf('SC_CLK_TCK')
    rescan = list_threads(pid, name_filter)
    first = {tid: (r[2], r[3]) for tid, r in rescan.items()}
    ctx0 = {tid: read_ctx(base_t + '/' + tid) for tid in rescan for base_t in ['/proc/%d/task' % pid]}
    state_time = {}          # tid -> {R: 秒, S: 秒, D: 秒}
    last = {}
    samples = {tid: 0 for tid in rescan}
    names = {tid: r[0] for tid, r in rescan.items()}
    t_begin = time.monotonic()
    tot0, idle0 = cpu_busy_delta()
    period = 1.0 / hz
    next_scan = t_begin
    prev = {tid: r[1] for tid, r in rescan.items()}
    next_t = t_begin
    n = 0
    while True:
        now = time.monotonic()
        if now - t_begin >= dur:
            break
        if now >= next_scan:
            # 新线程 (OpenMP 工作线程在第一次并行区才建) 每 0.5s 重新扫一遍
            for tid, r in list_threads(pid, name_filter).items():
                if tid not in first:
                    first[tid] = (r[2], r[3])
                    names[tid] = r[0]
                    samples[tid] = 0
                    prev[tid] = r[1]
            next_scan = now + 0.5
        cur = list_threads(pid, name_filter)
        for tid, r in cur.items():
            if tid not in first:
                continue
            st = r[1]
            if tid not in last:
                last[tid] = now
                prev[tid] = st
                continue
            dt = now - last[tid]
            if dt <= 0 or dt > 0.2:
                last[tid] = now
                prev[tid] = st
                continue
            d = state_time.setdefault(tid, {'R': 0.0, 'S': 0.0, 'D': 0.0})
            d['R' if st == 'R' else 'S' if st == 'S' else 'D'] += dt
            samples[tid] += 1
            last[tid] = now
        for tid in list(last):
            if tid not in cur:
                del last[tid]
        n += 1
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()
    wall = time.monotonic() - t_begin
    tot1, idle1 = cpu_busy_delta()
    threads = []
    for tid, (u0, s0) in first.items():
        r = read_stat('/proc/%d/task/%s/stat' % (pid, tid))
        cpu = ((r[2] - u0) + (r[3] - s0)) / float(clk_tck) if r else 0.0
        st = state_time.get(tid, {'R': 0.0, 'S': 0.0, 'D': 0.0})
        vc1, nvc1 = read_ctx('/proc/%d/task/%s' % (pid, tid))
        vc0, nvc0 = ctx0.get(tid, (vc1, nvc1))
        threads.append(dict(tid=int(tid), name=names[tid], cpu_s=round(cpu, 3), run_s=round(st['R'], 3),
                            sleep_s=round(st['S'], 3), d_s=round(st['D'], 3),
                            rq_wait_s=round(max(0.0, st['R'] - cpu), 3), samples=samples.get(tid, 0),
                            vcsw=vc1 - vc0, nivcsw=nvc1 - nvc0,
                            policy=policy_name(r[4]) if r else '?', rt_prio=r[5] if r else -1))
    threads.sort(key=lambda t: -t['cpu_s'])
    load = open('/proc/loadavg').read().split()[:3]
    json.dump(dict(pid=pid, wall_s=round(wall, 2), hz=hz, samples=n, loadavg=load,
                   cpu_busy_pct=round(100.0 * ((tot1 - tot0) - (idle1 - idle0)) / float(tot1 - tot0), 1),
                   threads=threads), open(out_path, 'w'), indent=1)
    print('窗口 %.1fs, 采样 %d 次, 整机 CPU 忙 %.1f%%, 负载 %s' % (
        wall, n, 100.0 * ((tot1 - tot0) - (idle1 - idle0)) / float(tot1 - tot0), ' '.join(load)))
    print('  %-6s %-9s %-12s %4s %8s %8s %9s %8s   %9s %9s' % (
        'tid', '线程', '调度策略', 'prio', 'CPU s', 'R 共', '等调度', '阻塞', '主动阻塞次', '被抢占次'))
    for t in threads:
        print('  %-6d %-9s %-12s %4d %8.2f %8.2f %9.2f %8.2f   %9d %9d' % (
            t['tid'], t['name'], t['policy'], t['rt_prio'], t['cpu_s'], t['run_s'], t['rq_wait_s'],
            t['sleep_s'] + t['d_s'], t['vcsw'], t['nivcsw']))


if __name__ == '__main__':
    main()
