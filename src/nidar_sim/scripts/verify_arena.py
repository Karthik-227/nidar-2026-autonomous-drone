import collections
import math
import os
import re
import sys

RES = 0.05

def wall_boxes(sdf_text):
    arena = sdf_text[sdf_text.index('<model name="arena">'):
                     sdf_text.index('<model name="launch_pad">')]
    out = []
    for name, body in re.findall(r'<link name="(\w+)">(.*?)</link>', arena, re.S):
        if name.startswith('net_'):
            continue
        px, py = map(float, re.search(
            r'<pose>(\S+) (\S+) ', body).groups())
        sx, sy = map(float, re.search(
            r'<size>(\S+) (\S+) ', body).groups())
        out.append((px, py, sx, sy))
    return out

def survivors(yaml_path):
    out, cur = [], None
    for line in open(yaml_path):
        m = re.match(r'\s*- id: (\S+)', line)
        if m:
            cur = m.group(1)
        m = re.match(r'\s*xy: \[([-\d.]+), ([-\d.]+)\]', line)
        if m and cur:
            out.append((cur, float(m.group(1)), float(m.group(2))))
            cur = None
    return out

def check(sdf_path):
    stem = os.path.splitext(sdf_path)[0]
    boxes = wall_boxes(open(sdf_path).read())
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    lo_x, hi_x = min(xs) - 3, max(xs) + 3
    lo_y, hi_y = min(ys) - 3, max(ys) + 3
    w = int((hi_x - lo_x) / RES) + 1
    h = int((hi_y - lo_y) / RES) + 1

    grid = bytearray(w * h)
    for px, py, sx, sy in boxes:
        i0 = max(0, int((px - sx / 2 - lo_x) / RES))
        i1 = min(w - 1, int(math.ceil((px + sx / 2 - lo_x) / RES)))
        j0 = max(0, int((py - sy / 2 - lo_y) / RES))
        j1 = min(h - 1, int(math.ceil((py + sy / 2 - lo_y) / RES)))
        for j in range(j0, j1 + 1):
            row = j * w
            for i in range(i0, i1 + 1):
                grid[row + i] = 1

    def idx(x, y):
        return int((y - lo_y) / RES) * w + int((x - lo_x) / RES)

    start = idx(0.0, 0.0)
    pad_free = grid[start] == 0
    seen = bytearray(w * h)
    escaped = False
    if pad_free:
        seen[start] = 1
        q = collections.deque([start])
        while q:
            k = q.popleft()
            j, i = divmod(k, w)
            if i <= 1 or i >= w - 2 or j <= 1 or j >= h - 2:
                escaped = True
                continue
            for nk in (k - 1, k + 1, k - w, k + w):
                if not seen[nk] and not grid[nk]:
                    seen[nk] = 1
                    q.append(nk)

    surv = survivors(stem + '.yaml')
    unreachable = [n for n, x, y in surv if not seen[idx(x, y)]]
    area = sum(seen) * RES * RES

    ok = pad_free and not escaped and not unreachable
    print('%-24s %s pad_free=%-5s sealed=%-5s survivors=%d/%d reachable=%.1f m^2'
          % (os.path.basename(stem), 'PASS' if ok else 'FAIL', pad_free,
             not escaped, len(surv) - len(unreachable), len(surv), area))
    if unreachable:
        print('    unreachable survivors: %s' % ', '.join(unreachable))
    if not pad_free:
        print('    the launch pad at (0,0) is inside a wall')
    if escaped:
        print('    the flood fill left the arena - there is a hole in the shell')
    return ok

if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('Usage: verify_arena.py WORLD.sdf [WORLD2.sdf ...]')
    raise SystemExit(0 if all([check(p) for p in sys.argv[1:]]) else 1)
