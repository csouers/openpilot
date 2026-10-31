"""Panel fixes for the comma four (mici) AMOLED.

On the DWO/Bantian DO0200FAT06 panel (RM67162 controller), nominally black areas emit a
red/grey floor instead of being off. The cause is four registers in the controller's
private FE05 page: they ship non-zero, and zeroing them removes the floor entirely.

The bootloader reprograms those registers on every boot, so this has to run once per boot.
It does not need (and is not helped by) a panel sleep/wake cycle -- see
https://github.com/commaai/openpilot/issues/38707 for the originating report, which
proposed exactly that sleep cycle as the fix.
"""

import struct
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog

PANEL_CMD = Path('/sys/kernel/debug/dsi_dwo_video_display/mipi_command')
BACKLIGHT = Path('/sys/class/backlight/panel0-backlight')

# The path above is derived from the kernel command line panel selection, so a unit with a
# different panel never matches it and this is a no-op there.
#
# Each record is: dtype, last, vc, ack, unused, len_hi, len_lo, payload...
# The delay byte (index 4) is ignored by the debugfs implementation.
PANEL_FIX_SEQUENCE = (
  '39 01 00 00 0A 00 02 FE 05',  # select page FE05
  '39 01 00 00 0A 00 02 05 00',
  '39 01 00 00 0A 00 02 2A 00',
  '39 01 00 00 0A 00 02 91 00',
  '39 01 00 00 0A 00 02 00 00',
  '39 01 00 00 0A 00 02 FE 00',  # back to page 0
)


def _record(command: str) -> bytes:
  payload = bytes(int(x, 16) for x in command.split())
  return struct.pack(f'i{len(payload)}s', len(payload), payload)


def fix_panel_black_floor(timeout: float = 10.) -> bool:
  """Zero the FE05 registers that give this panel a lifted black level. Idempotent."""
  deadline = time.monotonic() + timeout
  while not PANEL_CMD.exists() and time.monotonic() < deadline:
    time.sleep(0.5)
  if not PANEL_CMD.exists():
    return False

  try:
    brightness = int((BACKLIGHT / 'brightness').read_text().strip())
  except Exception:
    brightness = 0

  try:
    PANEL_CMD.write_bytes(b''.join(_record(command) for command in PANEL_FIX_SEQUENCE))
  except Exception:
    cloudlog.exception('panel black floor fix failed')
    return False

  # re-emit the DCS brightness write so the panel's brightness state matches sysfs again
  if brightness > 0:
    try:
      (BACKLIGHT / 'brightness').write_text('1')
      time.sleep(0.1)
      (BACKLIGHT / 'brightness').write_text(str(brightness))
    except Exception:
      pass

  return True
