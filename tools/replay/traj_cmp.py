#!/usr/bin/env python3
# 两条回放轨迹按录包时间配对比较, 并把位置差分解成 纵向 (沿运动方向; 多半是输出延迟差) / 横向 (真实的岔开)。
#   python3 traj_cmp.py <基准TAG> <TAG> [<TAG> ...]
import re, io, bisect, math, sys
ANSI = re.compile(r'\x1b\[[0-9;]*m')
NUM = r'-?[\d.]+'
RE = re.compile(r'\[\d{10}\.\d+, (\d{10}\.\d+)[^\]]*\]: \[locstate\] mode=\S+ st=(\d) matching=(%s) pos=\((%s),(%s),(%s)deg\)' % ((NUM,) * 4))
def load(tag):
    lines = io.open('/tmp/opencode/replay/node_%s.log' % tag, encoding='utf-8', errors='replace').read().split('\n')
    mark = int(io.open('/tmp/opencode/replay/node_%s.mark' % tag).read().strip() or 0)
    rows = []
    for l in lines[mark:]:
        m = RE.search(ANSI.sub('', l))
        if m:
            rows.append((float(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6))))
    return rows
def q(xs, p):
    xs = sorted(xs); return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else float('nan')
def compare(a, b, base=None):
    A, B = load(a), (load(b) if base is None else base)
    tb = [r[0] for r in B]
    out = []
    for k, r in enumerate(A):
        i = bisect.bisect_left(tb, r[0])
        if i > 0 and (i >= len(tb) or tb[i] - r[0] > r[0] - tb[i - 1]):
            i -= 1
        if not (i < len(tb) and abs(tb[i] - r[0]) < 0.6) or k == 0 or i == 0:
            continue
        bp, bn = B[i - 1], B[min(i + 1, len(B) - 1)]
        dx, dy = bn[3] - bp[3], bn[4] - bp[4]
        L = math.hypot(dx, dy)
        v = L / max(1e-3, (bn[0] - bp[0]))
        ex, ey = (dx / L, dy / L) if L > 1e-6 else (0.0, 0.0)
        ddx, ddy = r[3] - B[i][3], r[4] - B[i][4]
        dyaw = abs((r[5] - B[i][5] + 180) % 360 - 180)
        out.append((r[0] - A[0][0], v, ddx * ex + ddy * ey, -ddx * ey + ddy * ex, math.hypot(ddx, ddy), dyaw))
    return B, out
base = None
for tag in sys.argv[2:]:
    base, out = compare(tag, sys.argv[1], base)
    mv = [o for o in out if o[1] > 0.2]; st = [o for o in out if o[1] <= 0.2]
    print('== %s 相对 %s: 成对 %d (行驶中 %d, 近静止 %d)' % (tag, sys.argv[1], len(out), len(mv), len(st)))
    for lab, grp in (('行驶中', mv), ('近静止', st)):
        if not grp: continue
        print('   %s: 纵向差 p50 %+.2f |纵向| p90 %.2f max %.2f | 横向|差| p50 %.2f p90 %.2f max %.2f | 总差 p50 %.2f p90 %.2f max %.2f m | 航向差 p90 %.1f max %.1f°'
              % (lab, q([o[2] for o in grp], .5), q([abs(o[2]) for o in grp], .9), max(abs(o[2]) for o in grp),
                 q([abs(o[3]) for o in grp], .5), q([abs(o[3]) for o in grp], .9), max(abs(o[3]) for o in grp),
                 q([o[4] for o in grp], .5), q([o[4] for o in grp], .9), max(o[4] for o in grp),
                 q([o[5] for o in grp], .9), max(o[5] for o in grp)))
