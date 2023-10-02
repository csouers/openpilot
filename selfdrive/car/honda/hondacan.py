from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CanBusBase
from openpilot.selfdrive.car.honda.values import HondaFlags, HONDA_BOSCH, HONDA_BOSCH_RADARLESS, CAR, CarControllerParams

# CAN bus layout with relay
# 0 = ACC-CAN - radar side
# 1 = F-CAN B - powertrain
# 2 = ACC-CAN - camera side
# 3 = F-CAN A - OBDII port


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    # use fingerprint if specified
    super().__init__(CP if fingerprint is None else None, fingerprint)

    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
      self._pt, self._radar = self.offset + 1, self.offset
    else:
      self._pt, self._radar = self.offset, self.offset + 1

  @property
  def pt(self) -> int:
    return self._pt

  @property
  def radar(self) -> int:
    return self._radar

  @property
  def camera(self) -> int:
    return self.offset + 2


def get_lkas_cmd_bus(CAN, car_fingerprint, radar_disabled=False):
  no_radar = car_fingerprint in HONDA_BOSCH_RADARLESS
  if radar_disabled or no_radar:
    # when radar is disabled, steering commands are sent directly to powertrain bus
    return CAN.pt
  # normally steering commands are sent to radar, which forwards them to powertrain bus
  return 0


def get_cruise_speed_conversion(car_fingerprint: str, is_metric: bool) -> float:
  # on certain cars, CRUISE_SPEED changes to imperial with car's unit setting
  return CV.MPH_TO_MS if car_fingerprint in HONDA_BOSCH_RADARLESS and not is_metric else CV.KPH_TO_MS


def create_brake_command(packer, CAN, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, fcw, car_fingerprint, stock_brake):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_1": 1,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": stock_brake["CHIME"] if fcw else 0,  # send the chime for stock fcw
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw?
    "AEB_REQ_1": 0,
    "AEB_REQ_2": 0,
    "AEB_STATUS": 0,
  }
  return packer.make_can_msg("BRAKE_COMMAND", CAN.pt, values)


def create_acc_commands(packer, CAN, enabled, active, accel, gas, vego, stopping_counter, braking_counter, car_fingerprint, test):
  commands = []
  min_gas_accel = CarControllerParams.BOSCH_GAS_LOOKUP_BP[0]
  engine_brake_accel = CarControllerParams.BOSCH_GAS_LOOKUP_BP[1]
  stopped = stopping_counter > 4 * 50 # allow idle stop after 4 seconds (50 Hz)

  accel = accel if not stopped else CarControllerParams.BOSCH_ACCEL_MIN
  gas_command = gas if active and accel > min_gas_accel else -3000
  accel_command = accel if active else 0
  braking = active and braking_counter > 0 and accel < 0
  standstill = active and stopped
  standstill_release = active and accel > 0 and vego < 0.5 # Hold high briefly

  # common ACC_CONTROL values
  acc_control_values = {
    'ACCEL_COMMAND': accel_command,
    'STANDSTILL': standstill, #stopping_counter > 5 * 50, # needs safety when combined with BRAKE_REQUEST
  }

  if car_fingerprint in HONDA_BOSCH_RADARLESS:
    acc_control_values.update({
      "CONTROL_ON": enabled,
      "IDLESTOP_ALLOW": standstill,
    })
  else:
    acc_control_values.update({
      # setting CONTROL_ON causes car to set POWERTRAIN_DATA->ACC_STATUS = 1
      "CONTROL_ON": 5 if enabled else 0,
      "GAS_COMMAND": gas_command,  # used for gas
      "BRAKE_LIGHTS": braking,
      "BRAKE_REQUEST": braking, # needs safety. holds brake pressure when high
      "STANDSTILL_RELEASE": standstill_release,
    })
    acc_control_on_values = {
      "SET_TO_3": 0x3, #test  % (0xff >> 1), # has no effect from user perspective
      "CONTROL_ON": enabled,
      "SET_TO_FF": 0xff, #test  % 0xff, # has no effect from user perspective
      "SET_TO_75": 0x75, #test  % 0xff, # has no effect from user perspective
      "SET_TO_30": 0x30, #test  % 0xff, # has no effect from user perspective
      # "ZEROS_BOH": test  % (0xff >> 3), # rescale and add other new one
      # "ZEROS_BOH2": test % 0xffff,
      # "ZEROS_BOH3": test % 2,
      "COAST_BRAKE": active and accel < -0.15 ,
      "ACCEL_KILL": 0, # ALWAYS ZERO! KILLS GAS PEDAL TOO! TAKES EFFECT AFTER ENABLING/MOVING FORWARD THE FIRST TIME
      # "POS_ACCEL": active and not braking,
      # "NEW_SIGNAL_1": test % 0x4,

    }
    commands.append(packer.make_can_msg("ACC_CONTROL_ON", CAN.pt, acc_control_on_values))

  commands.append(packer.make_can_msg("ACC_CONTROL", CAN.pt, acc_control_values))
  return commands


def create_steering_control(packer, CAN, apply_steer, lkas_active, car_fingerprint, radar_disabled):
  values = {
    "STEER_TORQUE": apply_steer if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
    # "RDM": lkas_active,
    # "STEER_DOWN_TO_ZERO": lkas_active,
    # "SET_ME_X00": 0xff >> 1, # kills steering
    # "SET_ME_X00_2": 0xff,
  }
  bus = get_lkas_cmd_bus(CAN, car_fingerprint, radar_disabled)
  return packer.make_can_msg("STEERING_CONTROL", bus, values)


def create_bosch_supplemental_1(packer, CAN, car_fingerprint):
  # non-active params
  values = {
    "SET_ME_X04": 0x04,
    "SET_ME_X80": 0x80,
    "SET_ME_X10": 0x10,
  }
  bus = get_lkas_cmd_bus(CAN, car_fingerprint)
  return packer.make_can_msg("BOSCH_SUPPLEMENTAL_1", bus, values)


def create_ui_commands(packer, CAN, CP, enabled, pcm_speed, hud, is_metric, acc_hud, lkas_hud, braking, test):
  commands = []
  radar_disabled = CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and CP.openpilotLongitudinalControl
  bus_lkas = get_lkas_cmd_bus(CAN, CP.carFingerprint, radar_disabled)

  if CP.openpilotLongitudinalControl:
    acc_hud_values = {
      'CRUISE_SPEED': hud.v_cruise if not braking else 252,
      'ENABLE_MINI_CAR': 1 if enabled else 0,
      # only moves the lead car without ACC_ON
      'HUD_DISTANCE': (hud.lead_distance_bars + 1) % 4,  # wraps to 0 at 4 bars
      'IMPERIAL_UNIT': int(not is_metric),
      'HUD_LEAD': 2 if enabled and hud.lead_visible else 1 if enabled else 0,
      'SET_ME_X01_2': 1,
    }

    if CP.carFingerprint in HONDA_BOSCH:
      acc_hud_values['ACC_ON'] = int(enabled)
      acc_hud_values['FCM_OFF'] = 1
      acc_hud_values['FCM_OFF_2'] = 1
      acc_hud_values['ICONS'] = int(bool(hud.e2e) << 1)
      #acc_hud_values['CRUISE_CONTROL_LABEL'] = not hud.e2e
    else:
      # Shows the distance bars, TODO: stock camera shows updates temporarily while disabled
      acc_hud_values['ACC_ON'] = int(enabled)
      acc_hud_values['PCM_SPEED'] = pcm_speed * CV.MS_TO_KPH
      acc_hud_values['PCM_GAS'] = hud.pcm_accel
      acc_hud_values['SET_ME_X01'] = 1
      acc_hud_values['FCM_OFF'] = acc_hud['FCM_OFF']
      acc_hud_values['FCM_OFF_2'] = acc_hud['FCM_OFF_2']
      acc_hud_values['FCM_PROBLEM'] = acc_hud['FCM_PROBLEM']
      acc_hud_values['ICONS'] = acc_hud['ICONS']
    commands.append(packer.make_can_msg("ACC_HUD", CAN.pt, acc_hud_values))

  lkas_hud_values = {
    'SET_ME_X41': 0x41,
    'STEERING_REQUIRED': hud.steer_required,
    'SOLID_LANES': hud.lanes_visible,
    'BEEP': 0,
    'RDM_OFF_MINI_ICON': hud.e2e,
    'LANE_DEPARTURE_WARNING': hud.ldw,
  }

  if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
    lkas_hud_values['LANE_LINES'] = 3
    lkas_hud_values['DASHED_LANES'] = hud.lanes_visible
    # car likely needs to see LKAS_PROBLEM fall within a specific time frame, so forward from camera
    lkas_hud_values['LKAS_PROBLEM'] = lkas_hud['LKAS_PROBLEM']

  if not (CP.flags & HondaFlags.BOSCH_EXT_HUD):
    lkas_hud_values['SET_ME_X48'] = 0x48

  if CP.flags & HondaFlags.BOSCH_EXT_HUD and not CP.openpilotLongitudinalControl:
    commands.append(packer.make_can_msg('LKAS_HUD_A', bus_lkas, lkas_hud_values))
    commands.append(packer.make_can_msg('LKAS_HUD_B', bus_lkas, lkas_hud_values))
  else:
    commands.append(packer.make_can_msg('LKAS_HUD', bus_lkas, lkas_hud_values))

  if radar_disabled:
    radar_hud_values = {
      'CMBS_OFF': 0x01,
      'SET_TO_1': 0x01,
    }
    commands.append(packer.make_can_msg('RADAR_HUD', CAN.pt, radar_hud_values))

    if CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
      legacy_brake_values = {
        'AEB_BRAKE_ALERT': hud.fcw,
      }
      commands.append(packer.make_can_msg("LEGACY_BRAKE_COMMAND", CAN.pt, legacy_brake_values))

  return commands


def spam_buttons_command(packer, CAN, button_val, car_fingerprint):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  # send buttons to camera on radarless cars
  bus = CAN.camera if car_fingerprint in HONDA_BOSCH_RADARLESS else CAN.pt
  return packer.make_can_msg("SCM_BUTTONS", bus, values)

def create_kwp_can_msg(packer, cmd, bus=0):
  if cmd == 'left':
    values = {'D0': 0x30,
              'D1': 0x0a,
              'D2': 0x0f,
             }
  elif cmd == 'right':
    values = {'D0': 0x30,
              'D1': 0x0b,
              'D2': 0x0f,
             }
  else:
    return [384899312, 0, b' ', bus] # default is a special 1 byte cancel
  return packer.make_can_msg("Tester_16f118_KWP_Req_BCM", bus, values)
