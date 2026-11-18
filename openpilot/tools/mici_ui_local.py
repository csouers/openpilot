#!/usr/bin/env python3
"""
Run the real mici UI (openpilot/selfdrive/ui/ui.py) at the *device* resolution.

Why: the UI's projection uses the mici camera intrinsics (fx 2648 for a 1928x1208 image) but divides
by the window size. At the default 536x240 window everything is ~3.6x oversized, so lane lines and
road edges land off-screen and the lead car hits its size cap. Sizing the window like the device
makes the projection 1:1, which is what you want when eyeballing a local test.

  .venv/bin/python openpilot/tools/mici_test_sender.py            # fake bus, one terminal
  .venv/bin/python openpilot/tools/mici_ui_local.py               # this: the UI, device-sized

  --scale N   render the window at N x device size (e.g. 0.5 for a 964x604 window while the UI
              still thinks it is 1928 wide); this is gui_app's SCALE, applied by its draw matrix.

Pass-through: any extra args are ignored; set env like BIG=0, RECORD=1, RECORD_OUTPUT=... as usual.
"""

import argparse
import os
import sys

from openpilot.common.basedir import BASEDIR

sys.path.insert(0, os.path.join(BASEDIR, "openpilot"))

from openpilot.system.ui.lib.application import gui_app

DEVICE_W, DEVICE_H = 1928, 1208  # mici narrow-road camera resolution


def main():
  ap = argparse.ArgumentParser(description="run the real UI at device resolution")
  ap.add_argument("--scale", type=float, default=0.5, help="window scale (default 0.5 = half size)")
  ap.add_argument("--full", action="store_true", help="1:1 window (1928x1208), no scaling")
  args = ap.parse_args()

  scale = 1.0 if args.full else args.scale
  gui_app._scale = scale
  gui_app._width, gui_app._height = DEVICE_W, DEVICE_H
  gui_app._scaled_width = int(DEVICE_W * scale)
  gui_app._scaled_height = int(DEVICE_H * scale)
  gui_app._scaled_width += gui_app._scaled_width % 2
  gui_app._scaled_height += gui_app._scaled_height % 2
  os.environ["SCALE"] = str(scale)  # keep gui_app from re-deriving an auto scale

  from openpilot.selfdrive.ui.ui import main as ui_main
  ui_main()


if __name__ == "__main__":
  sys.exit(main())
