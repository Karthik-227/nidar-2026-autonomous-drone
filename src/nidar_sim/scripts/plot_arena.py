import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_arena import survivors, wall_boxes

PPM = 26
MARGIN = 1.2
LABEL_H = 26

def tile(stem):
    from PIL import Image, ImageDraw
    boxes = wall_boxes(open(stem + '.sdf').read())
    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    lo_x, hi_x = min(xs) - MARGIN, max(xs) + MARGIN
    lo_y, hi_y = min(ys) - MARGIN, max(ys) + MARGIN
    w, h = int((hi_x - lo_x) * PPM), int((hi_y - lo_y) * PPM)

    im = Image.new('RGB', (w, h + LABEL_H), (250, 250, 250))
    d = ImageDraw.Draw(im)

    def px(x, y):
        return (x - lo_x) * PPM, h - (y - lo_y) * PPM

    for cx, cy, sx, sy in boxes:
        x0, y0 = px(cx - sx / 2, cy + sy / 2)
        x1, y1 = px(cx + sx / 2, cy - sy / 2)
        d.rectangle([x0, y0, x1, y1], fill=(60, 62, 70))

    x0, y0 = px(-0.305, 0.305)
    x1, y1 = px(0.305, -0.305)
    d.rectangle([x0, y0, x1, y1], fill=(30, 190, 70))

    for sid, sx, sy in survivors(stem + '.yaml'):
        cx, cy = px(sx, sy)
        d.ellipse([cx - 7, cy - 7, cx + 7, cy + 7],
                  fill=(240, 110, 15), outline=(120, 50, 0))
        d.text((cx - 3, cy - 5), sid.rsplit('_', 1)[1], fill=(255, 255, 255))

    d.text((6, h + 6), '%s - green: launch pad / entry / exit at origin; '
                       'orange: survivors; north up' % os.path.basename(stem),
           fill=(20, 20, 20))
    return im

def main():
    ap = argparse.ArgumentParser(
        description='Render a generated arena to a PNG plan, straight from the emitted SDF.')
    ap.add_argument('sdf', nargs='+')
    ap.add_argument('-o', '--out', default='arenas.png')
    args = ap.parse_args()
    from PIL import Image

    tiles = [tile(os.path.splitext(p)[0]) for p in args.sdf]
    gap = 14
    w = sum(t.width for t in tiles) + gap * (len(tiles) + 1)
    h = max(t.height for t in tiles) + gap * 2
    out = Image.new('RGB', (w, h), (255, 255, 255))
    x = gap
    for t in tiles:
        out.paste(t, (x, gap))
        x += t.width + gap
    out.save(args.out)
    print('wrote %s (%dx%d)' % (args.out, out.width, out.height))

if __name__ == '__main__':
    main()
