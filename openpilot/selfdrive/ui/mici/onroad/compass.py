import math

import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# rough heading smoothing (shortest-path, per frame)
HEADING_SMOOTHING = 0.2
MIN_HEADING_SPEED = 0.3  # m/s, below this the GPS heading is noise


def wrap180(deg: float) -> float:
  return (deg + 180.0) % 360.0 - 180.0


class Compass(Widget):
  """Small heading compass for the onroad side rail.

  North-up needle that rotates with the car: the white half points north, the grey half south,
  and the tick at the top of the ring is the car's own heading.
  """

  def __init__(self, demo_heading: float | None = None):
    super().__init__()
    self._demo_heading = demo_heading
    self._heading = 0.0
    self._has_heading = demo_heading is not None
    self._alpha_filter = FirstOrderFilter(0.0, 0.4, 1 / gui_app.target_fps)

  def _update_state(self):
    heading = self._demo_heading
    if heading is None:
      heading = self._gps_heading()

    if heading is not None:
      self._has_heading = True
      self._heading = (self._heading + wrap180(heading - self._heading) * HEADING_SMOOTHING) % 360.0

    self._alpha_filter.update(1.0 if self._has_heading else 0.0)

  def _gps_heading(self) -> float | None:
    sm = ui_state.sm
    # u-blox first, then the built-in GPS (qcomgpsd on the four)
    for service in ("gpsLocationExternal", "gpsLocation"):
      if sm.alive[service] and sm.valid[service]:
        loc = sm[service]
        if loc.hasFix:
          # heading-of-motion is only meaningful while moving: take the first fix so the
          # compass shows while parked, then only track it when actually driving
          if loc.speed > MIN_HEADING_SPEED or not self._has_heading:
            return loc.bearingDeg % 360.0
    return None

  def _render(self, _):
    alpha = self._alpha_filter.x
    if alpha < 0.02:
      return

    center = rl.Vector2(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2)
    radius = min(self._rect.width, self._rect.height) / 2 - 2

    ring_color = rl.Color(255, 255, 255, int(80 * alpha))
    rl.draw_ring(center, radius - 1.5, radius, 0.0, 360.0, 24, ring_color)

    # car heading reference (top of the ring)
    tick = rl.Color(255, 255, 255, int(200 * alpha))
    rl.draw_line_ex(rl.Vector2(center.x, center.y - radius - 3), rl.Vector2(center.x, center.y - radius + 2), 2.0, tick)

    # north points at -heading on screen (0 = up, clockwise)
    angle = math.radians(-self._heading)
    north = rl.Vector2(math.sin(angle), -math.cos(angle))
    perp = rl.Vector2(-north.y, north.x)
    base_l = rl.Vector2(center.x + perp.x * 3.5, center.y + perp.y * 3.5)
    base_r = rl.Vector2(center.x - perp.x * 3.5, center.y - perp.y * 3.5)

    north_tip = rl.Vector2(center.x + north.x * (radius - 3), center.y + north.y * (radius - 3))
    south_tip = rl.Vector2(center.x - north.x * (radius - 7), center.y - north.y * (radius - 7))
    rl.draw_triangle(north_tip, base_l, base_r, rl.Color(255, 255, 255, int(255 * alpha)))
    rl.draw_triangle(south_tip, base_l, base_r, rl.Color(255, 255, 255, int(90 * alpha)))
    rl.draw_circle_v(center, 2.0, rl.Color(255, 255, 255, int(255 * alpha)))
