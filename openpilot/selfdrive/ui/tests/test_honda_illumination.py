from openpilot.selfdrive.ui.honda_illumination import HondaIllumination


def test_day_night_day():
  state = HondaIllumination()
  assert not state.seen
  for raw, daylight in [('d5360000b0160000', True), ('55360000b0550000', False), ('d5360000b0160000', True)]:
    state.update(0x12F85450, 0, bytes.fromhex(raw))
    assert state.seen and state.daylight_override == daylight
    assert state.parking_lights_on and state.step == 21


def test_offroad_updates_and_sleep_retention():
  state = HondaIllumination()
  state.update(0x12F85450, 2, bytes.fromhex('d5360000b0160000'))
  # No ignition or card process is required to apply a subsequent dimmer/night frame.
  state.update(0x12F85450, 2, bytes.fromhex('47360000b0470000'))
  assert not state.daylight_override and state.parking_lights_on and state.step == 7
  state.update(0x12F85450, 128, bytes(8))
  state.update(0x12F85450, 1, bytes(8))
  state.update(0x12F85450, 0, bytes(1))
  state.update(0x123, 0, bytes(8))
  assert state.parking_lights_on and state.step == 7


def test_lights_off():
  state = HondaIllumination()
  state.update(0x12F85450, 0, bytes.fromhex('16360000b0160000'))
  assert state.seen and not state.parking_lights_on and state.step == 22
