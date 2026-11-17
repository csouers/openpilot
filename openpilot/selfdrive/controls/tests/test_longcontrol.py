import numpy as np

from opendbc.car.honda.values import HondaFlags
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans, LongControl, \
                                                       INTEGRATOR_ENGAGE_ACCEL, INTEGRATOR_MAX


class TestLongControlIntegrator(OpenpilotTestCase):

  def setup_method(self):
    CP = car.CarParams(brand="honda", flags=int(HondaFlags.BOSCH), pcmCruise=False, stopAccel=-0.5)
    CP.longitudinalTuning.kiBP = [0.]
    CP.longitudinalTuning.kiV = [0.09]
    self.CP = CP
    self.loc = LongControl(CP)

  def step(self, a_target, a_ego, n=1):
    CS = car.CarState(brakePressed=False, cruiseState={"standstill": False})
    for _ in range(n):
      CS.aEgo = float(a_ego)
      self.loc.update(True, CS, a_target, should_stop=False, accel_limits=(-3.5, 2.0))

  def test_integral_holds_steady_state(self):
    """A small sustained accel error near steady state is integrated away."""
    self.step(a_target=0.2, a_ego=0.1, n=100)  # 1 s at a 0.1 m/s^2 error
    assert self.loc.pid.i > 0.0
    assert self.loc.last_output_accel > 0.2

  def test_integral_frozen_during_ramp(self):
    """No integral charge while the planner is ramping (|a_target| above the engage threshold)."""
    self.step(a_target=1.0, a_ego=0.4, n=300)
    assert self.loc.pid.i == 0.0
    assert abs(self.loc.last_output_accel - 1.0) < 1e-9

  def test_integral_bounded(self):
    self.step(a_target=INTEGRATOR_ENGAGE_ACCEL - 0.05, a_ego=0.0, n=5000)
    assert abs(self.loc.pid.i) <= INTEGRATOR_MAX + 1e-9

class TestLongControlStateTransition(OpenpilotTestCase):

  def test_stay_stopped(self):
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
