#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工作图落盘对比: 行驶后的 map.overlay.pcd vs 行驶前的版本 (intensity = 账本的 w, 0~255 整数)。

    python3 tools/wmap_diff.py <行驶前.pcd> <行驶后.pcd>
    python3 tools/wmap_diff.py <工作图.pcd>              # 只看 w 分布

落盘时账本给每个底图点写上当前的 w, w==0 的点不写盘; 新提交的结构追加在后面。
两个文件的底图点顺序一致, 所以顺序比对 (不建大表 —— 机器内存紧):
    坐标相同 -> 存活, 比较前后的 w;  前版有而后版没有 -> 被账本判死;  后版多出来的 -> 新提交。
这是"账本是否在侵蚀底图"的确切证据 (日志里的剔除点数都是近似的, 见 drive_report.py)。
"""
import mmap
import struct
import sys

BINS = [(0, 0, 'w<=0 (落盘时应已丢弃)'), (1, 15, '1-15'), (16, 31, '16-31'), (32, 63, '32-63'), (64, 95, '64-95'),
        (96, 111, '96-111'), (112, 127, '112-127'), (128, 128, '128 (中性: 没收到证据)'), (129, 143, '129-143'),
        (144, 159, '144-159'), (160, 191, '160-191'), (192, 239, '192-239'), (240, 254, '240-254'), (255, 255, '255')]
DBINS = [(-10 ** 9, -64, '<= -64'), (-63, -33, '-63..-33'), (-32, -17, '-32..-17'), (-16, -9, '-16..-9'),
         (-8, -1, '-8..-1'), (0, 0, '0'), (1, 8, '1..8'), (9, 16, '9..16'), (17, 32, '17..32'),
         (33, 64, '33..64'), (65, 10 ** 9, '>= 65')]


def bin_of(w, bins):
    for k, (lo, hi, _) in enumerate(bins):
        if lo <= w <= hi:
            return k
    return len(bins) - 1


def pcd_header(mm):
    """-> (点数, 每点字节, 数据区起点)。数据区起点是 'DATA ...' 那一行**之后**的换行。"""
    end = mm.find(b'\nDATA ')
    hdr = mm[:end].decode('ascii', 'replace').split('\n')
    n = int([l for l in hdr if l.startswith('POINTS')][0].split()[1])
    size = sum(int(v) for v in [l for l in hdr if l.startswith('SIZE')][0].split()[1:])
    fields = [l for l in hdr if l.startswith('FIELDS')][0].split()[1:]
    return n, size, mm.find(b'\n', end + 1) + 1, fields


def open_pcd(path):
    f = open(path, 'rb')
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    n, pt, body, fields = pcd_header(mm)
    assert fields[:4] == ['x', 'y', 'z', 'intensity'] and pt == 32, (fields, pt)
    return f, mm, n, pt, body


def main():
    fa, a, na, pa, ba = open_pcd(sys.argv[1])
    two = len(sys.argv) > 2
    if two:
        fb, b, nb, pb, bb = open_pcd(sys.argv[2])
    if two:
        print('%s (%d 点)   vs   %s (%d 点)' % (sys.argv[1].split('/')[-1], na, sys.argv[2].split('/')[-1], nb))
    else:
        print('%s (%d 点)' % (sys.argv[1].split('/')[-1], na))
    ua = struct.Struct('<4f')
    ha = [0] * len(BINS)
    if two:
        hb, hd = [0] * len(BINS), [0] * len(DBINS)
    alive = dead = added = desync = 0
    dead_cells, dead_list, added_cells = {}, [], {}
    i = j = 0
    while i < na and (not two or j < nb):
        x1, y1, z1, w1 = ua.unpack_from(a, ba + i * pa)
        if not two:
            ha[bin_of(int(round(w1)), BINS)] += 1
            i += 1
            continue
        x2, y2, z2, w2 = ua.unpack_from(b, bb + j * pb)
        if x1 == x2 and y1 == y2 and z1 == z2:
            alive += 1
            k1, k2 = int(round(w1)), int(round(w2))
            ha[bin_of(k1, BINS)] += 1
            hb[bin_of(k2, BINS)] += 1
            hd[bin_of(k2 - k1, DBINS)] += 1
            i += 1
            j += 1
            continue
        # 不一致: 看新版是否与"前版的下一个点"对上 (= 前版这个点被判死), 或前版与"新版的下一个点"对上 (= 插入了新点)
        nxt_a = ua.unpack_from(a, ba + (i + 1) * pa) if i + 1 < na else None
        nxt_b = ua.unpack_from(b, bb + (j + 1) * pb) if j + 1 < nb else None
        if nxt_a is not None and nxt_a[:3] == (x2, y2, z2):
            dead += 1
            dead_list.append((x1, y1, z1, int(round(w1))))
            key = (int(x1 // 5) * 5, int(y1 // 5) * 5)
            dead_cells[key] = dead_cells.get(key, 0) + 1
            i += 1
            continue
        if nxt_b is not None and (x1, y1, z1) == nxt_b[:3]:
            added += 1
            key = (int(x2 // 5) * 5, int(y2 // 5) * 5)
            added_cells[key] = added_cells.get(key, 0) + 1
            j += 1
            continue
        desync += 1
        if desync <= 5:
            print('  [错位] 前版 #%d (%.3f, %.3f, %.3f) w=%d   后版 #%d (%.3f, %.3f, %.3f) w=%d' % (
                i, x1, y1, z1, round(w1), j, x2, y2, z2, round(w2)))
        i += 1
        j += 1
    if two:
        dead += na - i
        added += nb - j
        print('存活 %d   判死(被丢弃) %d   新增 %d   无法对齐 %d' % (alive, dead, added, desync))
        print('\n%-28s %12s %12s' % ('底图点的 w', '行驶前', '行驶后'))
        for k, (lo, hi, name) in enumerate(BINS):
            print('  %-26s %12d %12d   %6.2f%% -> %6.2f%%' % (name, ha[k], hb[k], 100.0 * ha[k] / max(1, alive),
                                                           100.0 * hb[k] / max(1, alive)))
        print('\n这一趟每个底图点的 w 变化 (共 %d 点):' % alive)
        for k, (lo, hi, name) in enumerate(DBINS):
            print('  %-10s %12d  %6.2f%%' % (name, hd[k], 100.0 * hd[k] / max(1, alive)))
        up = sum(hd[6:])
        down = sum(hd[:5])
        print('  -> 上升 %d (%.1f%%)   不变 %d (%.1f%%)   下降 %d (%.1f%%)' % (
            up, 100.0 * up / max(1, alive), hd[5], 100.0 * hd[5] / max(1, alive), down, 100.0 * down / max(1, alive)))
    else:
        print('\n%-28s %12s' % ('底图点的 w', '点数'))
        for k, (lo, hi, name) in enumerate(BINS):
            print('  %-26s %12d  %6.2f%%' % (name, ha[k], 100.0 * ha[k] / max(1, na)))
    if dead_list:
        print('\n判死点位置 (5m 网格, 最多的 12 格; 共 %d 格):' % len(dead_cells))
        for key, v in sorted(dead_cells.items(), key=lambda kv: -kv[1])[:12]:
            print('  x[%3d,%3d) y[%3d,%3d): %6d' % (key[0], key[0] + 5, key[1], key[1] + 5, v))
        zs = sorted(p[2] for p in dead_list)
        ws = sorted(p[3] for p in dead_list)
        print('  高度 z: 最小 %.2f  中位 %.2f  最大 %.2f ;  行驶前的 w: 最小 %d  中位 %d  最大 %d' % (
            zs[0], zs[len(zs) // 2], zs[-1], ws[0], ws[len(ws) // 2], ws[-1]))
        print('  判死点数 / 网格数 = %d / %d (平均每格 %.1f; 真有东西被挪走应是少数几格里的密集簇, 侵蚀则是沿墙散布)' % (
            len(dead_list), len(dead_cells), float(len(dead_list)) / max(1, len(dead_cells))))
    if added_cells:
        print('\n新增点位置 (5m 网格, 最多的 10 格; 共 %d 点 / %d 格):' % (added, len(added_cells)))
        for key, v in sorted(added_cells.items(), key=lambda kv: -kv[1])[:10]:
            print('  x[%3d,%3d) y[%3d,%3d): %6d' % (key[0], key[0] + 5, key[1], key[1] + 5, v))


if __name__ == '__main__':
    main()
