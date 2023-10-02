from collections import namedtuple

from cereal import car
from common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.honda import hondacan
from openpilot.selfdrive.car.honda.values import CruiseButtons, VISUAL_HUD, HONDA_BOSCH, HONDA_BOSCH_RADARLESS, HONDA_NIDEC_ALT_PCM_ACCEL, \
                                                 CarControllerParams, HondaFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import IMPERIAL_INCREMENT, rate_limit

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
BLINKER_INTERVAL_CANCEL = 20 # frames
BLINKER_INTERVAL = 30 # frames. The real interval is 0.350 seconds but ours must be shorter
FACTOR_TORQUE_TO_ACCEL = 0.015
MAX_GAS_DIFF = 1.0


def torque_accel(accel, speed, torque):
  # Send more brake when engine torque increases at low speeds
  # TODO: does radarless need this?
  if speed < 2.0 and not accel and torque:
    accel -= (torque * FACTOR_TORQUE_TO_ACCEL)
  return accel


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)


# TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value

  # *** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts

def round_hud_set_speed(v_cruise_kph, is_metric):
  if not is_metric:
    v_cruise_kph = v_cruise_kph / (CV.MPH_TO_KPH / IMPERIAL_INCREMENT)
    v_cruise_kph = round(v_cruise_kph / IMPERIAL_INCREMENT) * IMPERIAL_INCREMENT * 1.001
  return round(v_cruise_kph)

def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  ldw = 0

  # priority is: FCW, steer required, all others
  if hud_alert == VisualAlert.fcw:
    fcw_display = VISUAL_HUD[hud_alert.raw]
  elif hud_alert == VisualAlert.steerRequired:
    steer_required = VISUAL_HUD[hud_alert.raw]
  elif hud_alert == VisualAlert.ldw:
    ldw = VISUAL_HUD[hud_alert.raw]
  else:
    acc_alert = VISUAL_HUD[hud_alert.raw]

  return fcw_display, steer_required, acc_alert, ldw


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "lead_visible",
                      "lanes_visible", "fcw", "ldw", "acc_alert", "steer_required",
                      "lead_distance_bars", "e2e"])


def rate_limit_steer(new_steer, last_steer):
  # TODO just hardcoded ramp to min/max in 0.33s for all Honda
  MAX_DELTA = 3 * DT_CTRL
  return clip(new_steer, last_steer - MAX_DELTA, last_steer + MAX_DELTA)


class BlinkerController:
  def __init__(self):
    # Always cancel on startup to test communication with B-CAN
    self.disabled = True
    self.lockout = False
    self.control_prev = False
    self.queue = []

    self.left_last = 0
    # self.left_confirmed = False
    self.right_last = 0
    # self.right_confirmed = False

    self.left_next = -1
    self.right_next = -1
    self.cancel_next = 0

  def process(self, CS):
    if len(self.queue):
      if CS.bcm_kwp_started and self.queue[0] in ['left', 'right']:
        print(f'{self.queue[0]} accepted. removing {self.queue[0]} from queue')
        self.queue.remove(self.queue[0])
      elif CS.bcm_kwp_stopped and self.queue[0] == 'cancel':
        print(f'{self.queue[0]} accepted. removing {self.queue[0]} from queue')
        self.queue.remove(self.queue[0])
      if CS.bcm_kwp_failed:
        self.lockout = True

  def update(self, frame, CC, CS):
    # lamp state
    self.left_last = frame if CS.leftBlinker_lamp else self.left_last
    self.right_last = frame if CS.rightBlinker_lamp else self.right_last
    # stalk state
    stalk = CS.leftBlinker_stalk or CS.rightBlinker_stalk

    control = CC.leftBlinker or CC.rightBlinker
    # the only way to re-enable control after override is to turn off blinker control
    self.disabled = False if not control else self.disabled

    # control but no lamp
    if CC.leftBlinker:
      # light has been off and user input is off. send a pre-cancel
      if not self.disabled and not stalk:
        # cancel before the light re-flashes
        if frame >= (self.left_last + BLINKER_INTERVAL):
          if len(self.queue):
            if 'left' not in self.queue:
              self.left_next = frame
          else:
            self.left_next = frame
        elif frame >= (self.left_last + BLINKER_INTERVAL_CANCEL):
          if len(self.queue):
            if 'cancel' not in self.queue:
              self.cancel_next = frame
          else:
            self.cancel_next = frame
        # user override or car unresponsive
      elif CS.rightBlinker_stalk or frame > (self.left_last + 1 / DT_CTRL):
        self.disabled = True
        self.queue = []
        self.cancel_next = frame
    elif CC.rightBlinker:
      # light has been off and user input is off. send a pre-cancel
      if not self.disabled and not stalk:
        # cancel before the light re-flashes
        if frame >= (self.right_last + BLINKER_INTERVAL):
          if len(self.queue):
            if 'right' not in self.queue:
              self.right_next = frame
          else:
            self.right_next = frame
        elif frame >= (self.right_last + BLINKER_INTERVAL_CANCEL):
          if len(self.queue):
            if 'cancel' not in self.queue:
              self.cancel_next = frame
          else:
            self.cancel_next = frame
        # user override or car unresponsive
      elif CS.leftBlinker_stalk or frame > (self.right_last + 1 / DT_CTRL):
        self.disabled = True
        self.queue = []
        self.cancel_next = frame
    elif self.control_prev:
      # todo: cancel light when full on period has completed (aesthetic)
      # cancel (or cancel again) when blinker control disenages
      print('control off cancel')
      self.queue = []
      self.cancel_next = frame
    if not self.lockout:
      if frame == self.cancel_next:
        self.queue.append('cancel')
      elif frame == self.left_next:
        self.queue.append('left')
      elif frame == self.right_next:
        self.queue.append('right')
    self.control_prev = control
    return

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
    self.frame = 0

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0
    self.starting_counter = 0
    self.braking_counter = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_steer = 0.0
    self.last_gas = 0.0

    self.BLINK = None
    if self.CP.flags & HondaFlags.ENABLE_BLINKERS:
      self.BLINK = BlinkerController()

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    conversion = hondacan.get_cruise_speed_conversion(self.CP.carFingerprint, CS.is_metric)
    hud_v_cruise = hud_control.setSpeed / conversion if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if CC.longActive:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo, self.CP.carFingerprint)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # *** rate limit steer ***
    limited_steer = rate_limit_steer(actuators.steer, self.last_steer)
    self.last_steer = limited_steer

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    fcw_display, steer_required, acc_alert, ldw = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_steer = int(interp(-limited_steer * self.params.STEER_MAX,
                             self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    # Send CAN commands
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.frame % 10 == 0:
        can_sends.append((0x18DAB0F1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 1))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_steer, CC.latActive, self.CP.carFingerprint,
                                                      CS.CP.openpilotLongitudinalControl))

    # wind brake from air resistance decel at high speed
    wind_brake = interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
    # all of this is only relevant for HONDA NIDEC
    max_accel = interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    if not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    elif self.CP.carFingerprint in HONDA_NIDEC_ALT_PCM_ACCEL:
      pcm_speed_V = [0.0,
                     clip(CS.out.vEgo - 3.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 0.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = interp(gas - brake, pcm_speed_BP, pcm_speed_V)
      pcm_accel = int(1.0 * self.params.NIDEC_GAS_MAX)
    else:
      pcm_speed_V = [0.0,
                     clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = interp(gas - brake, pcm_speed_BP, pcm_speed_V)
      pcm_accel = int(clip((accel / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS:  # radarless cars don't have supplemental message
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN, self.CP.carFingerprint))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          accel = torque_accel(accel, CS.out.vEgo, CS.engine_torque)
          self.accel = clip(accel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX)
          gas = interp(accel, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V)
          self.gas = min(gas, self.last_gas + MAX_GAS_DIFF)
          self.last_gas = self.gas
          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping and CS.out.standstill else 0
          # Prevent brake lamp flashing. Hold brake_request flag for at least 1.2 seconds before releasing.
          self.braking = self.accel < self.params.BOSCH_GAS_LOOKUP_BP[0]
          self.braking_counter = 60 if self.braking else self.braking_counter - 1
          can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                        CS.out.vEgo, self.stopping_counter, self.braking_counter, self.CP.carFingerprint, int(self.frame / 50)))
        else:
          apply_brake = clip(self.brake_last - wind_brake, 0.0, 1.0)
          apply_brake = int(clip(apply_brake * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, fcw_display,
                                                         self.CP.carFingerprint, CS.stock_brake))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

    if self.BLINK is not None:
      self.BLINK.update(self.frame, CC, CS)

    # Send dashboard UI commands.
    if self.frame % 10 == 0:
      hud = HUDData(int(pcm_accel), round_hud_set_speed(hud_v_cruise, CS.is_metric), hud_control.leadVisible,
                    hud_control.lanesVisible, fcw_display, ldw, acc_alert, steer_required,
                    hud_control.leadDistanceBars, hud_control.e2e)
      can_sends.extend(hondacan.create_ui_commands(self.packer, self.CAN, self.CP, CC.enabled, pcm_speed, hud, CS.is_metric, CS.acc_hud, CS.lkas_hud, self.braking, int(self.frame / 50)))

      if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint not in HONDA_BOSCH:
        self.speed = pcm_speed
        self.gas = pcm_accel / self.params.NIDEC_GAS_MAXf

      if self.BLINK is not None:
        self.BLINK.process(CS)
        if len(self.BLINK.queue):
          can_sends.append(hondacan.create_kwp_can_msg(self.packer, cmd=self.BLINK.queue[0]))

    new_actuators = actuators.as_builder()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake
    new_actuators.steer = self.last_steer
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
