import sys
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_ENUMERATION_GRACE, CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, cable_connected, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not chestnut_present():
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

def chestnut_expected(timeout: float = CHESTNUT_ENUMERATION_GRACE) -> bool:
  # A USB-C cable in the port is NOT evidence of a chestnut: that port (and its CC
  # orientation) is shared with accessories, e.g. the gamepad used by joystick debug
  # mode. Taking the big model path for a gamepad makes the load fail, and selfdrived
  # then soft-disables engagement on every attempt. Only a real enumeration counts,
  # with a short grace period for the cable-detected-before-enumeration race. Waiting
  # longer than this just stalls modeld on a bare cable and blows selfdrived's 6 s
  # initialization timeout.
  if not chestnut_compiled():
    return False
  if chestnut_present():
    return True
  if not cable_connected():
    return False
  try:
    wait_for_chestnut(timeout)
  except TimeoutError:
    return False
  return True

def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  return path.is_file() and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))
