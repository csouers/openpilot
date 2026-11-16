#!/usr/bin/env python3
"""Generate lead_car_ghost.png — Honda-style dashed-outline ghost of lead_car.png.

Mirrors hondacan.py HUD_LEAD: 2=solid lead car (radar-fixed), 1=ghost
(vision-only lead). The ghost is baked natively at the renderer's load size
(260 px wide) so dash/stroke geometry is tuned for the on-screen scale instead
of surviving a ~5x minification of the 636 px source (which shimmers badly).

Two things this bake gets right that the old one did not:
  * the body is inset by PAD px, so the stroke is never flush against the canvas
    edge (where resampling and the onscreen scissor can shave it), and
  * the silhouette mask is padded before eroding, so body pixels that touch an
    edge still count as outline — the old erode() treated the canvas border as
    interior and left the roof crest and underbody unstroked ("cut off in spots").

NOTE: the shipped asset is no longer baked from lead_car.png. The live ghost is
`lead_car_ghost.svg` (a hand-drawn dashed outline: rounded body, solid mirror
blobs, larger dashes), rasterized to lead_car_ghost.png at 260x161 with a 5 px
transparent pad. Re-bake with:

  rsvg-convert -b none -w 2540 lead_car_ghost.svg -o /tmp/ghost_hi.png
  # then crop to the alpha bbox, resize the body to 250 px wide, pad 5 px,
  # force rgb=255 and hard-threshold alpha, save as lead_car_ghost.png

This script is kept for regenerating the old dashed lead_car.png outline if that
design is ever wanted back. The pixel-art Boo-on-kart sprite (200x172 RGBA, from
boo-ghost.gif) that was live between 4d497f2246 and the dashed outline is still
reachable at `git show 4d497f2246:.../lead_car_ghost.png`. The Boo sprite is still reachable at
`git show 4d497f2246:.../lead_car_ghost.png` if it is ever wanted back.

Run from this directory:  python3 make_ghost_lead_car.py [target_width]
"""
import os
import sys

import numpy as np
from PIL import Image

# dash/stroke parameters at the SOURCE resolution (636 px wide); scaled to the target.
# The stroke has to survive being drawn far smaller than the bake: the on-screen lead icon is
# typically 25-60 px wide (LEAD_MIN_ICON_WIDTH is 8.7 px), so a 3 px stroke in a 260 px canvas
# landed under half a screen pixel and read as a hairline. 5 px at 260 gives ~1.3 px at 40 px.
DASH_PERIOD = 90  # px along the diagonal stripe
DASH_DUTY = 0.75  # fraction of the period that is dash (longer dashes, shorter gaps)
STROKE_DILATE = 5  # outline thickness radius, px

SOURCE_WIDTH = 636
DEFAULT_TARGET_WIDTH = 260  # the shipped dashed asset's total width, padding included
PAD = 5  # transparent bleed around the body at the default target size, px


def erode(mask):
  """Erode by one pixel, treating everything outside the array as background."""
  m = np.pad(mask, 1, constant_values=False)
  er = m.copy()
  er[1:, :] &= m[:-1, :]
  er[:-1, :] &= m[1:, :]
  er[:, 1:] &= m[:, :-1]
  er[:, :-1] &= m[:, 1:]
  return er[1:-1, 1:-1]


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
  pad = max(1, round(PAD * target_width / DEFAULT_TARGET_WIDTH))
  body_width = target_width - 2 * pad
  scale = body_width / SOURCE_WIDTH
  body_height = round(src.height * scale)
  target_height = body_height + 2 * pad

  # downscale the soft alpha first, then threshold: keeps the silhouette clean at target scale
  small = src.resize((body_width, body_height), Image.LANCZOS)
  body = np.zeros((target_height, target_width), dtype=bool)
  body[pad:pad + body_height, pad:pad + body_width] = np.asarray(small)[:, :, 3] > 128

  outline = body & ~erode(body)
  stroke = dilate(outline, max(1, round(STROKE_DILATE * scale)))

  dash_period = max(2, round(DASH_PERIOD * scale))
  dash_len = max(1, round(dash_period * DASH_DUTY))
  yy, xx = np.mgrid[0:target_height, 0:target_width]

  # phase the dashes so one sits dead centre on the body's topmost point (the roof crest)
  # instead of a gap landing there
  top_y = np.argmax(body.any(axis=1))
  top_x = int(np.median(np.nonzero(body[top_y])[0]))
  phase = (top_x + top_y - dash_len / 2.0) % dash_period

  dashes = stroke & (((xx + yy - phase) % dash_period) < dash_len)

  out = np.zeros((target_height, target_width, 4), dtype=np.uint8)
  out[..., 0] = 255
  out[..., 1] = 255
  out[..., 2] = 255
  out[..., 3] = np.where(dashes, 255, 0)
  out_name = 'lead_car_ghost.png'
  Image.fromarray(out, 'RGBA').save(os.path.join(here, out_name))
  print(f"wrote {out_name} ({target_width}x{target_height}, pad {pad}px, "
        f"dash {dash_len}/{dash_period}px, stroke {2 * max(1, round(STROKE_DILATE * scale)) + 1}px)")


if __name__ == '__main__':
  main()
