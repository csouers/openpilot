#!/usr/bin/env python3
"""Drive the aapi cluster arrow from the comma four's steering angle — over CAN.

Runs ON the comma four (AGNOS). Reads `carState` from openpilot's msgq and publishes a
`navInstruction`, which `card.py`'s NavCan encodes into OPNAV frames on the Honda body
bus. No WiFi, no HTTP.

Why publish navInstruction instead of writing CAN directly: msgq has ONE write pointer
per ring, so a second `sendcan` publisher corrupts card's stream. card.py is the single
owner of sendcan; everything else hands it something to send.

Gate: navd also publishes navInstruction when NavigationEnabled is set. This script
refuses to run in that case rather than fight it.

  ssh four '/usr/local/venv/bin/python3 /data/openpilot/tools/nav/steering_nav.py --dry-run'

Mapping: |angle| < hold -> STRAIGHT, above -> TURN LEFT/RIGHT. Hysteresis stops the
arrow flickering at the threshold; only transitions (and a slow re-assert) are published.
"""
from __future__ import annotations

import argparse
import sys
import time

# The nested tree must win: /data/openpilot_stock, when present, is a FLAT layout and
# its top-level `openpilot` package shadows the real one, so appending it breaks
# `openpilot.selfdrive.navcan`.
for extra in ("/data/openpilot/msgq_repo", "/data/openpilot/opendbc_repo", "/data/openpilot"):
  if extra not in sys.path:
    sys.path.insert(0, extra)
for extra in ("/data/openpilot_stock", "/data/safe_staging/merged"):
  if extra not in sys.path:
    sys.path.append(extra)

try:
  from openpilot.cereal.messaging import SubMaster, PubMaster  # nested layout
except ImportError:
  from cereal import messaging as _messaging  # stock flat layout
  SubMaster, PubMaster = _messaging.SubMaster, _messaging.PubMaster

from openpilot.selfdrive.navcan.steering import classify, instruction

DISTANCE_M = 50.0


def publish(pm, fields: dict) -> None:
  msg = messaging_new('navInstruction')
  ni = msg.navInstruction
  for k, v in fields.items():
    setattr(ni, k, v)
  pm.send('navInstruction', msg)


def messaging_new(name: str):
  # imported lazily so --dry-run works even if messaging is unavailable
  try:
    from openpilot.cereal import messaging
  except ImportError:
    from cereal import messaging
  msg = messaging.new_message(name, valid=True)
  msg.valid = True
  return msg


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  p.add_argument("--threshold-deg", type=float, default=20.0)
  p.add_argument("--hysteresis-deg", type=float, default=6.0)
  p.add_argument("--reassert-s", type=float, default=0.5)
  p.add_argument("--slight-deg", type=float, default=60.0)
  p.add_argument("--sharp-deg", type=float, default=180.0)
  p.add_argument("--uturn-deg", type=float, default=360.0)
  p.add_argument("--uturn-max-speed", type=float, default=4.0)
  p.add_argument("--distance-m", type=float, default=DISTANCE_M)
  p.add_argument("--dry-run", action="store_true", help="print, publish nothing")
  p.add_argument("--once", action="store_true", help="one carState sample, then exit")
  args = p.parse_args()

  if not args.dry_run:
    try:
      from openpilot.common.params import Params
      if Params().get_bool("NavigationEnabled"):
        print("refusing: NavigationEnabled is on (navd already publishes navInstruction)", file=sys.stderr)
        return 2
    except Exception as e:
      print(f"warning: could not read NavigationEnabled ({e})", file=sys.stderr)

  sm = SubMaster(["carState"])
  pm = None if args.dry_run else PubMaster(["navInstruction"])

  state = ""
  last_sent = 0.0
  while True:
    sm.update(1000)
    if not sm.updated["carState"]:
      continue
    angle = float(sm["carState"].steeringAngleDeg)

    v_ego = float(sm["carState"].vEgo)
    want = classify(angle, args.threshold_deg, args.hysteresis_deg,
                    slight_deg=args.slight_deg, sharp_deg=args.sharp_deg,
                    uturn_deg=args.uturn_deg, v_ego=v_ego,
                    uturn_max_speed=args.uturn_max_speed, prev=state)
    if want and want != state:
      state = want
      last_sent = 0.0

    now = time.monotonic()
    if state and (now - last_sent) >= args.reassert_s:
      fields = instruction(state, angle, args.distance_m)
      if args.dry_run:
        print("DRY", state, fields, flush=True)
      else:
        publish(pm, fields)
        print("sent", state, f"angle={angle:+.1f}", flush=True)
      last_sent = now

    if args.once:
      print(f"once: angle={angle:+.1f} state={state!r}", flush=True)
      return 0


if __name__ == "__main__":
  try:
    sys.exit(main())
  except KeyboardInterrupt:
    sys.exit(0)
