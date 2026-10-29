#!/usr/bin/env python3
"""Passive extended-ID CAN capture for Honda display-mode investigation."""
import argparse
import json
import time
from pathlib import Path

from openpilot.cereal import messaging


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', required=True)
  parser.add_argument('--seconds', type=float, default=1200)
  args = parser.parse_args()
  sock = messaging.sub_sock('can', timeout=100)
  end = time.monotonic() + args.seconds
  counts = {}
  last_report = 0.0
  with Path(args.output).open('x', buffering=1) as out:
    print(f'READY passive capture: {args.output}', flush=True)
    while time.monotonic() < end:
      records = []
      now = time.time()
      for msg in messaging.drain_sock(sock, wait_for_one=True):
        for frame in msg.can:
          if frame.address > 0x7ff and frame.src < 128:
            records.append({'t': now, 'mono_ns': msg.logMonoTime, 'bus': frame.src,
                            'id': frame.address, 'data': bytes(frame.dat).hex()})
            counts[frame.src] = counts.get(frame.src, 0) + 1
      if records:
        out.write(''.join(json.dumps(r) + '\n' for r in records))
      if time.monotonic() - last_report >= 10:
        print(f'frames by bus: {counts}', flush=True)
        last_report = time.monotonic()
    print(f'DONE frames by bus: {counts}', flush=True)


if __name__ == '__main__':
  main()
