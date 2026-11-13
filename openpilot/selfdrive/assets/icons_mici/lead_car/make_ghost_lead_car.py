#!/usr/bin/env python3
"""Generate lead_car_ghost_dashed.png — Honda-style dashed-outline ghost of lead_car.png.

Mirrors hondacan.py HUD_LEAD: 2=solid lead car (radar-fixed), 1=ghost
(vision-only lead). The ghost is baked natively at the renderer's load size
(130 px wide) so dash/stroke geometry is tuned for the on-screen scale instead
of surviving a ~5x minification of the 636 px source (which shimmers badly).

NOTE: the SHIPPED ghost asset `lead_car_ghost.png` is now the pixel-art Boo-on-kart
sprite (200x172 RGBA, transparent background, taken from boo-ghost.gif). This
script is kept for reference and writes `lead_car_ghost_dashed.png` instead, so
re-running it never clobbers the live icon.

Run from this directory:  python3 make_ghost_lead_car.py [target_width]
"""
import os
import sys

import numpy as np
from PIL import Image

# dash/stroke parameters at the SOURCE resolution (636 px wide); scaled to the target
DASH_PERIOD = 80  # px along the diagonal stripe
DASH_DUTY = 0.60  # fraction of the period that is dash
STROKE_DILATE = 3  # outline thickness radius, px

SOURCE_WIDTH = 636
DEFAULT_TARGET_WIDTH = 130  # matches the renderer's gui_app.texture load size


def erode(mask):
  er = mask.copy()
  er[1:, :] &= mask[:-1, :]
  er[:-1, :] &= mask[1:, :]
  er[:, 1:] &= mask[:, :-1]
  er[:, :-1] &= mask[:, 1:]
  return er


def dilate(mask, radius):
  out = mask.copy()
  for _ in range(radius):
    d = out.copy()
    d[1:, :] |= out[:-1, :]
    d[:-1, :] |= out[1:, :]
    d[:, 1:] |= out[:, :-1]
    d[:, :-1] |= out[:, 1:]
    out = d
  return out


def main():
  here = os.path.dirname(os.path.abspath(__file__))
  src = Image.open(os.path.join(here, 'lead_car.png')).convert('RGBA')

  target_width = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TARGET_WIDTH
  scale = target_width / SOURCE_WIDTH
  target_height = round(src.height * scale)

  # downscale the soft alpha first, then threshold: keeps the silhouette clean at target scale
  small = src.resize((target_width, target_height), Image.LANCZOS)
  body = np.asarray(small)[:, :, 3] > 128

  outline = body & ~erode(body)
  stroke = dilate(outline, max(1, round(STROKE_DILATE * scale)))

  dash_period = max(2, round(DASH_PERIOD * scale))
  dash_len = max(1, round(dash_period * DASH_DUTY))
  yy, xx = np.mgrid[0:target_height, 0:target_width]
  dashes = stroke & (((xx + yy) % dash_period) < dash_len)

  out = np.zeros((target_height, target_width, 4), dtype=np.uint8)
  out[..., 0] = 255
  out[..., 1] = 255
  out[..., 2] = 255
  out[..., 3] = np.where(dashes, 255, 0)
  out_name = 'lead_car_ghost_dashed.png'
  Image.fromarray(out, 'RGBA').save(os.path.join(here, out_name))
  print(f"wrote {out_name} ({target_width}x{target_height}, "
          f"dash {dash_len}/{dash_period}px, stroke {2 * round(STROKE_DILATE * scale) + 1}px)")


if __name__ == '__main__':
  main()
