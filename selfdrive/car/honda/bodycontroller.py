from selfdrive.body.lib.bodyd_helpers import allowed
# from opendbc.can.packer import CANPacker
# from selfdrive.car.honda.bodyvalues import CAR, DBC


class BodyController():
  def __init__(self):
    raise Exception('BodyController: not implemented')
    # self.packer = CANPacker(dbc_name)

  def update(self):
    # Send CAN commands. Not for use when onroad
    if not allowed():
      return None
    can_sends = []
    # do stuff
    return can_sends
