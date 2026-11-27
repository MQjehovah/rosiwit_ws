import re, io, sys, bisect
ansi = re.compile(r'\x1b\[[0-9;]*m')
def load(tag):
    rows = []; last_t = None
    for line in io.open('/tmp/opencode/replay/node_%s.log' % tag, encoding='utf-8', errors='replace'):
        s = ansi.sub('', line)
        d = re.search(r'\[DropProbe\] t=([\d.]+) fitness=([\d.]+)', s)
        if d: last_t = float(d.group(1)); continue
        if s.startswith('[quality]'):
            f = re.search(r' fitness=([\d.]+)', s)
            if f and last_t is not None: rows.append((last_t, float(f.group(1))))
    return rows
def cmp(a, b):
    A, B = load(a), load(b)
    tb = [r[0] for r in B]
    pairs = []
    for t, f in A:
        i = bisect.bisect_left(tb, t)
        if i < len(tb) and abs(tb[i] - t) < 0.15: pairs.append((f, B[i][1]))
    d = sorted(bb - aa for aa, bb in pairs)
    n = len(d)
    print('%-9s vs %-9s: 成对 %4d, 差均值 %+.4f 中位 %+.4f | 更低 %4d 更高 %4d' % (a, b, n, sum(d) / n, d[n // 2], sum(1 for x in d if x < -0.005), sum(1 for x in d if x > 0.005)))
for pair in sys.argv[1:]:
    cmp(*pair.split(':'))
