#!/usr/bin/env python3
"""Find the Honda B-CAN illumination / dimmer signal by watching bytes that move.

The body DBC has no *named* illumination field, only a handful of unnamed ones, so this
watches the whole body bus (panda bus 2 on the comma four) and prints every byte that
changes. Turn the dash dimmer knob / the light switch / headlights on-off while it runs and
whatever steps is the signal we want.

Passive listener: pandad owns the panda, this only reads openpilot's 'can' service.

  PYTHONPATH=/data/openpilot/openpilot:/data/openpilot /usr/local/venv/bin/python3 \
    /data/openpilot/openpilot/tools/illumination_hunt.py

Options:
  --bus N        panda bus to watch (default 2 = B-CAN on the four)
  --all          watch every id, not just the illumination candidates
  --summary      also print the current value of every candidate every 5 s
"""
import argparse
import time
from collections import defaultdict

from openpilot.cereal import messaging


# ids that plausibly carry a lighting/dimmer level (see topics/openpilot-aapi.md)
CANDIDATES = {
  0x0AF81118: "af811_Wipers_Lighting",      # HEADLIGHTS / LIGHTING_ON bits
  0x0EF82318: "ef823",
  0x0EF88318: "ef883",
  0x0EF91154: "ef911",
  0x12F81018: "12f810_Lighting",            # lamp output bits
  0x12F85250: "12f852",
  0x12F85450: "12f854_Lighting",
  0x12F91550: "12f915",
  0x12F96050: "12f960",
  0x12F96150: "12f961",
  0x12F96250: "12f962",
  0x12F97150: "12f971_Steer",
  0x12F97C50: "12f97c",
  0x12F9C554: "12f9c5",
}

SUMMARY_PERIOD = 5.0


def describe(name: str, old: bytes | None, new: bytes) -> str:
  diffs = []
  for i, (a, b) in enumerate(zip(old, new, strict=False) if old else []):
    if a != b:
      flipped = [bit for bit in range(8) if (a ^ b) >> bit & 1]
      diffs.append(f"byte{i} {a:#04x}->{b:#04x} (bits {flipped})")
  if old is not None and len(old) != len(new):
    diffs.append(f"dlc {len(old)}->{len(new)}")
  if not diffs:
    diffs.append("new")
  return f"{name:22s} {' '.join(new.hex(' ').split())}  | {', '.join(diffs)}"


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--bus", type=int, default=2)
  parser.add_argument("--all", action="store_true", help="watch every id on the bus")
  parser.add_argument("--summary", action="store_true", help="print candidate values every 5 s")
  args = parser.parse_args()

  sock = messaging.sub_sock('can', timeout=100)
  last: dict[int, bytes] = {}
  seen: set[int] = set()
  counts: dict[int, int] = defaultdict(int)
  last_summary = time.monotonic()

  what = "all ids" if args.all else f"{len(CANDIDATES)} candidates"
  hint = "Turn the dash dimmer knob / light switch now."
  print(f"watching bus {args.bus} ({what}); ctrl-c to stop. {hint}", flush=True)

  while True:
    for msg in messaging.drain_sock(sock, wait_for_one=True):
      for can in msg.can:
        if can.src != args.bus:
          continue
        addr = can.address
        if not args.all and addr not in CANDIDATES:
          continue
        dat = bytes(can.dat)
        seen.add(addr)
        counts[addr] += 1
        old = last.get(addr)
        if dat != old:
          name = CANDIDATES.get(addr, "")
          stamp = time.strftime("%H:%M:%S")
          print(f"{stamp} 0x{addr:08X} {describe(name, old, dat)}", flush=True)
          last[addr] = dat

    if args.summary and time.monotonic() - last_summary > SUMMARY_PERIOD:
      last_summary = time.monotonic()
      frames = sum(counts.values())
      print(f"--- {time.strftime('%H:%M:%S')} {len(seen)} ids seen, {frames} frames", flush=True)
      for addr in sorted(CANDIDATES):
        if addr in last:
          print(f"    0x{addr:08X} {CANDIDATES[addr]:22s} {last[addr].hex(' ')}", flush=True)


if __name__ == "__main__":
  main()
