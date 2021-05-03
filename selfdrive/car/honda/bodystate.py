from cereal import body
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.honda.bodyvalues import CAR, DBC
import cereal.messaging as messaging


Body = body.BodyState
Door = Body.Door
Window = Body.Window


def get_body_signals(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
    # signal , frame name, initial value
    # ajar state
    ('LF_OPEN', 'LEFT_DOORS', 0),
    ('LR_OPEN', 'LEFT_DOORS', 0),
    ('RF_OPEN', 'RIGHT_DOORS', 0),
    ('RR_OPEN', 'RIGHT_DOORS', 0),
    ('TRUNK_OPEN', 'TRUNK_2', 0),
    # locks
    ('LF_UNLOCKED', 'LF_DOOR', 0),
    ('UNLOCKED', 'RF_DOOR', 0),
    ('UNLOCKED', 'LR_DOOR', 0),
    ('UNLOCKED', 'RR_DOOR', 0),
    # windows
    ('LF_WINDOWSTATE', 'FRONT_WINDOWS', 0),
    ('RF_WINDOWSTATE', 'FRONT_WINDOWS', 0),
  ]

  if CP.carFingerprint in [CAR.ACCORD]:
    signals += [('SUNROOF_CLOSED', 'FRONT_WINDOWS', 0)]

  # check for gateway heartbeat?
  checks = []

  return signals, checks


class BodyState():
  def __init__(self, BP):
    super(BodyState, self).__init__()
    self.BP = BP
    # can_define = CANDefine(DBC[BP.carFingerprint]['body'])

  def update(self, cp):
    # ret = Body.new_message()
    ret = messaging.new_message('bodyState')
    ret.bodyState.frontLeftDoor.state = Door.State.open if bool((cp.vl["LEFT_DOORS"]['LF_OPEN'])) else Door.State.closed

    # ajar state
    ret.bodyState.frontLeftDoor.state = Door.State.open if bool((cp.vl["LEFT_DOORS"]['LF_OPEN'])) else Door.State.closed
    ret.bodyState.frontRightDoor.state = Door.State.open if bool((cp.vl["LEFT_DOORS"]['LR_OPEN'])) else Door.State.closed
    ret.bodyState.rearLeftDoor.state = Door.State.open if bool((cp.vl["RIGHT_DOORS"]['RF_OPEN'])) else Door.State.closed
    ret.bodyState.rearRightDoor.state = Door.State.open if bool((cp.vl["RIGHT_DOORS"]['RR_OPEN'])) else Door.State.closed
    # ret.bodyState.hood # TODO: find this
    ret.bodyState.trunk.state = Door.State.open if bool((cp.vl["TRUNK_2"]['TRUNK_OPEN'])) else Door.State.closed

    # locks
    ret.bodyState.frontLeftDoor.lock = Door.Lock.unlocked if bool((cp.vl["LF_DOOR"]['LF_UNLOCKED'])) else Door.Lock.locked
    ret.bodyState.frontRightDoor.lock = Door.Lock.unlocked if bool((cp.vl["RF_DOOR"]['UNLOCKED'])) else Door.Lock.locked
    ret.bodyState.rearLeftDoor.lock = Door.Lock.unlocked if bool((cp.vl["LR_DOOR"]['UNLOCKED'])) else Door.Lock.locked
    ret.bodyState.rearRightDoor.lock = Door.Lock.unlocked if bool((cp.vl["RR_DOOR"]['UNLOCKED'])) else Door.Lock.locked

    # windows
    ret.bodyState.frontLeftWindow = Window.closed if bool((cp.vl["FRONT_WINDOWS"]["LF_WINDOWSTATE"])) else Window.open
    ret.bodyState.frontRightWindow = Window.closed if bool((cp.vl["FRONT_WINDOWS"]["RF_WINDOWSTATE"])) else Window.open

    # not yet seen on CAN
    # ret.bodyState.backLeftWindow = Window.closed if bool((cp.vl["FRONT_WINDOWS"]["LF_WINDOWSTATE"])) else Window.open
    # ret.bodyState.backRightWindow = Window.closed if bool((cp.vl["FRONT_WINDOWS"]["LF_WINDOWSTATE"])) else Window.open

    # Accord reports sunroof state on CAN
    if self.BP.carFingerprint in [CAR.ACCORD]:
      ret.bodyState.sunroof = Window.closed if bool((cp.vl["FRONT_WINDOWS"]["SUNROOF_CLOSED"])) else Window.open

    return ret

  @staticmethod
  def get_bodyd_body_can_parser(BP):
    signals, checks = get_body_signals(BP)
    bus_body = 0
    return CANParser(DBC[BP.carFingerprint]['body'], signals, checks, bus_body)
