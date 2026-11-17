import numpy as np
from opendbc.car.structs import car
from opendbc.car.honda.values import HondaFlags
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Fork (Honda Bosch only): kp is hardcoded to 0, so the integral is the whole accel feedback term.
# Freeze it while the planner is asking for a real acceleration - that lag is the feedforward's job,
# and integrating through it charges the term and overshoots the set speed on roll-off. It is then
# only active near steady state, where it trims the residual accel error and holds the set speed.
INTEGRATOR_ENGAGE_ACCEL = 0.3  # m/s^2, |a_target| above this is treated as a transient
INTEGRATOR_MAX_ERROR = 0.5  # m/s^2, large errors are transient by definition
INTEGRATOR_MAX = 0.5  # m/s^2, hard bound on the integral contribution

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(active, long_control_state, should_stop, brake_pressed, cruise_standstill):
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.guard_integrator = CP.brand == 'honda' and bool(CP.flags & HondaFlags.BOSCH)
    self.a_target_prev = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(active, self.long_control_state, should_stop,
                                                       CS.brakePressed, CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      freeze_integrator = False
      if self.guard_integrator:
        ramp = abs(a_target - self.a_target_prev) / DT_CTRL  # m/s^3, a set speed change shows up here
        freeze_integrator = abs(a_target) > INTEGRATOR_ENGAGE_ACCEL or abs(error) > INTEGRATOR_MAX_ERROR or ramp > 2.0
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target, freeze_integrator=freeze_integrator)
      if self.guard_integrator:
        self.pid.i = float(np.clip(self.pid.i, -INTEGRATOR_MAX, INTEGRATOR_MAX))

    self.a_target_prev = a_target
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
