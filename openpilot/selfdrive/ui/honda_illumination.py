"""Passive Honda meter illumination state, independent of ignition/card lifecycle."""
from dataclasses import dataclass


@dataclass
class HondaIllumination:
  seen: bool = False
  daylight_override: bool = False
  parking_lights_on: bool = False
  step: int = 0

  def update(self, address: int, bus: int, data: bytes) -> None:
    # On the four the body harness is exposed on 0 and mirrored on 2. Never consume TX echoes.
    if address != 0x12F85450 or bus not in (0, 2) or len(data) != 8:
      return
    self.seen = True
    self.daylight_override = bool(data[0] & 0x80)
    self.parking_lights_on = bool(data[0] & 0x40)
    self.step = data[0] & 0x3f
