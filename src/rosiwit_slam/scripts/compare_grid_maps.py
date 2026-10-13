#!/usr/bin/env python3
"""并排渲染多张 2D 栅格地图 (PGM) 以对比不同生成策略.

像素约定 (与 grid_map_io.cpp 一致):
  0   -> occupied (黑)
  254 -> free     (白)
  205 -> unknown  (灰)

用法:
  python3 compare_grid_maps.py out.png height.pgm ray.pgm octo.pgm
可选:
  --scale 2        放大倍数 (默认按高度自适应到 720px)
  --csv metrics.csv 输出各图统计

标题会显示: 文件名 + 分辨率/尺寸 + occupied/free/unknown 占比.
"""
import argparse
import csv
import os
import sys

import numpy as np
from PIL import Image, ImageDraw


def read_pgm(path):
    img = Image.open(path)
    if img.mode not in ("L", "I", "I;16"):
        img = img.convert("L")
    arr = np.frombuffer(img.tobytes(), dtype=np.uint8).reshape(img.height, img.width)
    return arr


def classify(arr):
    occ = arr == 0
    free = arr == 254
    unknown = ~occ & ~free
    return occ, free, unknown


def to_rgb(arr):
    occ, free, unknown = classify(arr)
    rgb = np.zeros((*arr.shape, 3), dtype=np.uint8)
    rgb[free] = (255, 255, 255)
    rgb[unknown] = (160, 160, 160)
    rgb[occ] = (0, 0, 0)
    return rgb


def metrics(name, arr):
    occ, free, unknown = classify(arr)
    total = arr.size
    return {
        "name": os.path.basename(name),
        "width": arr.shape[1],
        "height": arr.shape[0],
        "occupied_%": round(100.0 * occ.sum() / total, 2),
        "free_%": round(100.0 * free.sum() / total, 2),
        "unknown_%": round(100.0 * unknown.sum() / total, 2),
    }


def main():
    ap = argparse.ArgumentParser(description="并排对比多张 2D 占据栅格 PGM")
    ap.add_argument("output", help="输出 PNG 路径")
    ap.add_argument("maps", nargs="+", help="输入 PGM 列表 (顺序即从左到右)")
    ap.add_argument("--scale", type=float, default=0.0,
                    help="统一放大倍数; <=0 时自动缩放到目标高度 720px")
    ap.add_argument("--target-height", type=int, default=720)
    ap.add_argument("--csv", default="", help="输出统计 CSV 路径")
    args = ap.parse_args()

    images, stats, titles = [], [], []
    for path in args.maps:
        arr = read_pgm(path)
        images.append(arr)
        stats.append(metrics(path, arr))

    max_h = max(a.shape[0] for a in images)
    if args.scale > 0:
        scale = args.scale
    else:
        scale = args.target_height / max_h if max_h > 0 else 1.0
    scale = max(scale, 0.05)

    pad = 8
    label_h = 46
    rendered = []
    for arr, st in zip(images, stats):
        rgb = to_rgb(arr)
        h, w = rgb.shape[:2]
        nh, nw = max(1, int(round(h * scale))), max(1, int(round(w * scale)))
        im = Image.fromarray(rgb).resize((nw, nh), Image.NEAREST)
        rendered.append(im)
        titles.append(f"{st['name']}  {st['width']}x{st['height']}  "
                      f"occ {st['occupied_%']}% / free {st['free_%']}% / ? {st['unknown_%']}%")

    canvas_w = sum(im.width for im in rendered) + pad * (len(rendered) + 1)
    canvas_h = label_h + max(im.height for im in rendered) + pad
    canvas = Image.new("RGB", (canvas_w, canvas_h), (245, 245, 245))
    draw = ImageDraw.Draw(canvas)

    x = pad
    for im, title in zip(rendered, titles):
        draw.text((x + 2, 6), title, fill=(20, 20, 20))
        canvas.paste(im, (x, label_h))
        x += im.width + pad

    canvas.save(args.output)
    print(f"[compare] saved {args.output} ({canvas_w}x{canvas_h})")

    # 控制台统计表
    header = f"{'map':<28}{'size':>12}{'occupied%':>11}{'free%':>9}{'unknown%':>10}"
    print(header)
    for st in stats:
        size = f"{st['width']}x{st['height']}"
        print(f"{st['name']:<28}{size:>12}{st['occupied_%']:>11}"
              f"{st['free_%']:>9}{st['unknown_%']:>10}")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(stats[0].keys()))
            writer.writeheader()
            writer.writerows(stats)
        print(f"[compare] metrics -> {args.csv}")


if __name__ == "__main__":
    sys.exit(main())
