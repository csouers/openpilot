import os
# import cereal.messaging as messaging
from selfdrive.hardware import PC
from common.params import Params

if not PC:
  OP_DIR = '/data/openpilot/'
else:
  OP_DIR = '/home/pi/openpilot/'


def load_car(cache):
  """Get lists of supported cars and load the cached car's values."""
  all_makes = []
  make = None
  car = None

  # get all cars
  for root, dirs, files in os.walk(OP_DIR + 'selfdrive/car'):
    if 'bodyvalues.py' in files:
      d = os.path.basename(root)
      d = d.upper()
      all_makes.append(d)

  # get our make
  for x in all_makes:
    if x in str(cache):
      make = x
      make_lower = x.lower()
      _path = ('selfdrive.car.%s' % make_lower)

  # load body
  bs = __import__(_path + '.bodystate', fromlist=['BodyState']).BodyState

  # get our car
  CAR = __import__('selfdrive.car.%s.bodyvalues' % make_lower, fromlist=['CAR']).CAR
  models = [getattr(CAR, c) for c in CAR.__dict__.keys() if not c.startswith("__")]

  for model in models:
        if model in str(cache):
            car = model
  return all_makes, make, car, bs


def is_offroad(p):
  offroad = True if p.get("IsOffroad") == b"1" else False
  return offroad


def is_panda_connected(p):
  panda = True if p.get("PandaDongleId") is not None else False
  return panda


def allowed():
  """Check if we're offroad and have a connected panda."""
  p = Params()
  offroad = is_offroad(p)
  panda = is_panda_connected(p)

  if offroad and panda:
    return True
  else:
    return False


def dbc_dict(body_dbc=None):
  return {'body': body_dbc}
