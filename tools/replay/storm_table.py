import re, io, sys
ansi = re.compile(r'\x1b\[[0-9;]*m')
tag = sys.argv[1]
last_t = None
per = []; tgt = []
T0 = None
for line in io.open('/tmp/opencode/replay/node_%s.log' % tag, encoding='utf-8', errors='replace'):
    s = ansi.sub('', line)
    d = re.search(r'\[DropProbe\] t=([\d.]+)', s)
    if d:
        last_t = float(d.group(1))
        if T0 is None: T0 = last_t
    p = re.search(r'\[FramePerf\] n=(\d+) ms p50/p90/max: wall=([\d]+)/([\d]+)/(\d+)', s)
    if p:
        per.append([last_t - T0] + [int(x) for x in p.groups()]); continue
    st = re.search(r'solver=([\d.]+)/([\d.]+)/([\d.]+) eval=([\d.]+)/([\d.]+)/([\d.]+) post=([\d.]+)/([\d.]+)/([\d.]+)', s)
    if st and per and len(per[-1]) == 5 and 'stages' in s:
        it = re.search(r'iter p50/max=(\d+)/(\d+)', s)
        per[-1] += [float(x) for x in st.groups()] + [int(it.group(1)) if it else -1]; continue
    m = re.search(r'\[target\] background build: pts=(\d+) ms=([\d.]+)', s)
    if m: tgt.append(last_t - T0)
print('窗口 起始s  墙钟 p50/p90/max      求解 p50/p90/max     评估p50/p90   收尾p50/p90   迭代p50  窗内后台建目标')
prev = -1e9
for i, p in enumerate(per):
    t = p[0]
    n = sum(1 for x in tgt if prev <= x < t) * 5
    prev = t
    print('  %2d %5.0fs  %4d/%4d/%5d ms   %3.0f/%4.0f/%4.0f ms   %3.0f/%4.0f     %3.0f/%4.0f      %2d      %s' % (
        i + 1, t, p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[11], p[12], p[14], ('~%d 次' % n) if n else '-'))
