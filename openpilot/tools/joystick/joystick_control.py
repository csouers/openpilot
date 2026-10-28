#!/usr/bin/env python3
import argparse
import threading
import time
import numpy as np
from inputs import UnpluggedError, get_gamepad

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.hardware import PC
from openpilot.tools.lib.kbhit import KBHit

EXPO = 0.4
DEADZONE = 0.03

# A controller that stops reporting (switched off, power save, wedged USB) leaves the kernel's last
# axis values in place, which would pin the last steer/accel command forever. Release the axes when
# nothing has been received for this long. 0 disables it. A deliberately held stick emits no events
# either and gets released too, so keep it generous.
DEADMAN = 3.0

# Linux evdev ranges: analogue sticks are 16 bit signed, triggers are 8 bit unsigned
STICK_RANGE = (-32768., 32767.)
TRIGGER_RANGE = (0., 255.)


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def axes(self) -> list[float]:
    return [self.axes_values[ax] for ax in self.axes_order]

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False
    if key == 'r':
      self.axes_values = dict.fromkeys(self.axes_values, 0.)
    elif key == 'c':
      self.cancel = True
    elif key in self.axes_map:
      axis = self.axes_map[key]
      incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
      self.axes_values[axis] = float(np.clip(self.axes_values[axis] + incr, -1, 1))
    else:
      return False
    return True


def expo(value: float) -> float:
  norm = float(np.clip(value, -1, 1))
  return EXPO * norm ** 3 + (1 - EXPO) * norm


class Joystick:
  """Xbox-style controller -> testJoystick.axes = [accel, steer].

  Left stick X steers, the analogue triggers drive accel/brake (RT gas, LT brake).
  Sticks and triggers are read as raw evdev values, so the ranges below are the
  kernel's, not a calibrated one. Swap the axes here for other controllers.
  """

  def __init__(self, deadman: float = DEADMAN):
    # BTN_NORTH is Y on an Xbox pad and triangle on a DualSense
    self.cancel_button = 'BTN_NORTH'

    self.steer_axis = 'ABS_X'
    self.gas_axis = 'ABS_RZ'
    self.brake_axis = 'ABS_Z'

    # left on the stick is negative in evdev, but positive torque steers left
    self.invert_map = {self.steer_axis: True}

    # a silent controller must not keep commanding the last axes (a switched-off pad freezes them)
    self.deadman = deadman
    self.last_event_time = time.monotonic()
    self.released = False

    self.raw_values = dict.fromkeys((self.steer_axis, self.gas_axis, self.brake_axis), 0.)
    self.axes_values = {'accel': 0., 'steer': 0.}
    self.axes_order = ['accel', 'steer']
    self.cancel = False

  def axes(self) -> list[float]:
    if self.deadman > 0 and time.monotonic() - self.last_event_time > self.deadman:
      if not self.released and any(self.axes_values.values()):
        print(f'\nno controller input for {self.deadman:g}s (controller off?), releasing axes')
      self.released = True
      return [0. for _ in self.axes_order]
    return [self.axes_values[ax] for ax in self.axes_order]

  @staticmethod
  def _deadzone(value: float) -> float:
    # center and released triggers can be noisy
    return 0. if abs(value) < DEADZONE else value

  def _axis(self, axis: str, axis_range: tuple[float, float], out_range: tuple[float, float]) -> float:
    norm = float(np.interp(self.raw_values[axis], list(axis_range), list(out_range)))
    return -norm if self.invert_map.get(axis, False) else norm

  def _update_values(self):
    self.axes_values['steer'] = expo(self._deadzone(self._axis(self.steer_axis, STICK_RANGE, (-1., 1.))))
    gas = self._deadzone(self._axis(self.gas_axis, TRIGGER_RANGE, (0., 1.)))
    brake = self._deadzone(self._axis(self.brake_axis, TRIGGER_RANGE, (0., 1.)))
    self.axes_values['accel'] = expo(gas - brake)

  def _handle_event(self, code: str, state: int) -> bool:
    self.last_event_time = time.monotonic()
    self.released = False
    if code == self.cancel_button:
      self.cancel = state == 1
      return True
    if code in self.raw_values:
      self.raw_values[code] = float(state)
      self._update_values()
      return True
    return False

  def update(self):
    try:
      events = get_gamepad()
    except (OSError, UnpluggedError):
      self.raw_values = dict.fromkeys(self.raw_values, 0.)
      self._update_values()
      return False

    changed = False
    for event in events:
      changed = self._handle_event(event.code, event.state) or changed
    return changed


def send_thread(joystick):
  pm = messaging.PubMaster(['testJoystick'])
  params = Params()

  rk = Ratekeeper(100, print_delay_threshold=None)

  while True:
    # JoystickDebugMode is CLEAR_ON_OFFROAD_TRANSITION, so an ignition cycle drops it and the manager
    # stops joystickd/joystick. Re-assert it while this tool is alive so a detached copy survives
    # ignition cycles instead of needing a manual restart.
    if rk.frame % 100 == 0 and not params.get_bool('JoystickDebugMode'):
      params.put_bool('JoystickDebugMode', True, block=True)

    axes = joystick.axes()
    if rk.frame % 20 == 0:
      print('\n' + ', '.join(f'{name}: {round(value, 3)}' for name, value in zip(joystick.axes_order, axes, strict=True)), end='')

    joystick_msg = messaging.new_message('testJoystick')
    joystick_msg.valid = True
    joystick_msg.testJoystick.axes = axes

    pm.send('testJoystick', joystick_msg)

    rk.keep_time()


def joystick_control_thread(joystick):
  Params().put_bool('JoystickDebugMode', True, block=True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()


def main(deadman: float = DEADMAN):
  joystick_control_thread(Joystick(deadman=deadman))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joystick_control. This tool supports ' +
                                               'an Xbox-style controller (left stick steers, RT/LT accelerate and brake).',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  parser.add_argument('--deadman', type=float, default=DEADMAN,
                      help='Release the axes when no controller input has arrived for this many seconds (0 disables)')
  args = parser.parse_args()

  # openpilot must be offroad to start: joystick mode replaces controlsd entirely. Wait for it rather
  # than exiting, so a copy started by the manager (which starts it as soon as the param is set) comes
  # up on its own instead of dying onroad and never being restarted.
  if not PC:
    params = Params()
    if not params.get_bool("IsOffroad"):
      print('openpilot must be offroad to start joystick_control, waiting for the car to turn off...', end='', flush=True)
      while not params.get_bool("IsOffroad"):
        time.sleep(1.)
      print(' offroad', flush=True)

  print()
  if args.keyboard:
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
  else:
    print('Using joystick, make sure to run openpilot/system/webrtc/bridge.py if running over the network!')
    print('Mapping: left stick = steering, RT = gas, LT = brake, Y (BTN_NORTH) = cancel cruise')

  joystick = Keyboard() if args.keyboard else Joystick(deadman=args.deadman)
  joystick_control_thread(joystick)
