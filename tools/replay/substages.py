import re, io, sys
ansi = re.compile(r'\x1b\[[0-9;]*m')
tag = sys.argv[1]
M = int(open('/tmp/opencode/replay/node_%s.mark' % tag).read())
lines = io.open('/tmp/opencode/replay/node_%s.log' % tag, encoding='utf-8', errors='replace').read().split('\n')[M - 1:]
names = ['accept', 'fuse', 'outlim', 'misc', 'prep', 'ledger', 'state', 'covbase', 'session', 'merge', 'save', 'tail']
print('窗口  墙钟 p50/p90/max       | 各子阶段最大值 (ms): ' + ' '.join('%6s' % n for n in names) + ' | p50: prep ledger covbase session')
nwin = 0
i = 0
while i < len(lines):
    s = ansi.sub('', lines[i])
    m = re.search(r'\[FramePerf\] n=\d+ ms p50/p90/max: wall=(\d+)/(\d+)/(\d+)', s)
    if m:
        nwin += 1
        blk = [s]
        j = i + 1
        while j < len(lines) and ansi.sub('', lines[j]).startswith('[FramePerf]'):
            blk.append(ansi.sub('', lines[j])); j += 1
        sub = dict((a, (float(b), float(c), float(d))) for a, b, c, d in re.findall(r'(\w+)=([\d.]+)/([\d.]+)/([\d.]+)', blk[2])) if len(blk) > 2 else {}
        flag = ' <<<' if int(m.group(3)) > 300 else ''
        print('  %2d  %4s/%4s/%5s ms | %s | %s %s %s %s%s' % (
            nwin, m.group(1), m.group(2), m.group(3),
            ' '.join('%6.0f' % sub.get(k, (0, 0, float('nan')))[2] for k in names) if sub else '(无)',
            '%.0f' % sub.get('prep', (float('nan'),))[0] if sub else '-', '%.0f' % sub.get('ledger', (float('nan'),))[0] if sub else '-',
            '%.0f' % sub.get('covbase', (float('nan'),))[0] if sub else '-', '%.0f' % sub.get('session', (float('nan'),))[0] if sub else '-', flag))
        i = j
    else:
        i += 1
