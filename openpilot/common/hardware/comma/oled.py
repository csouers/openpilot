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
import subprocess
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog

PANEL_CMD = Path('/sys/kernel/debug/dsi_dwo_video_display/mipi_command')
BACKLIGHT = Path('/sys/class/backlight/panel0-backlight')

# The panel selection in the kernel command line determines this path, so a unit with a
# different panel never matches it and this becomes a no-op there.
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


def _panel_command_writable() -> bool:
  # /sys/kernel/debug is root-only, so we cannot even stat the file as the comma user.
  try:
    return subprocess.run(['sudo', '-n', 'test', '-w', str(PANEL_CMD)],
                          capture_output=True, timeout=5).returncode == 0
  except Exception:
    return False


def _write_panel(data: bytes) -> bool:
  # tee does a single write(2) for a payload this small, which the debugfs handler needs:
  # it parses one or more length-prefixed records out of a single write.
  try:
    ret = subprocess.run(['sudo', '-n', 'tee', str(PANEL_CMD)], input=data,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
    return ret.returncode == 0
  except Exception:
    return False


def _brightness() -> int:
  try:
    return int(subprocess.run(['sudo', '-n', 'cat', str(BACKLIGHT / 'brightness')],
                              capture_output=True, text=True, timeout=5).stdout.strip())
  except Exception:
    return 0


def _set_brightness(value: str) -> None:
  try:
    subprocess.run(['sudo', '-n', 'tee', str(BACKLIGHT / 'brightness')], input=value.encode(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
  except Exception:
    pass


def fix_panel_black_floor(timeout: float = 10.) -> bool:
  """Zero the FE05 registers that give this panel a lifted black level. Idempotent."""
  deadline = time.monotonic() + timeout
  while not _panel_command_writable() and time.monotonic() < deadline:
    time.sleep(0.5)
  if not _panel_command_writable():
    return False

  brightness = _brightness()

  if not _write_panel(b''.join(_record(command) for command in PANEL_FIX_SEQUENCE)):
    cloudlog.error('panel black floor fix: mipi command write failed')
    return False

  # re-emit the DCS brightness write so the panel's brightness state matches sysfs again
  if brightness > 0:
    _set_brightness('1')
    time.sleep(0.1)
    _set_brightness(str(brightness))

  return True
