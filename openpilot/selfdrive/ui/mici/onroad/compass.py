import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment, TextAlignmentVertical
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel

# rough heading smoothing (shortest-path, per frame)
HEADING_SMOOTHING = 0.2
MIN_HEADING_SPEED = 0.3  # m/s, below this the GPS heading is noise
CARDINALS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")  # 8 point, at most two letters
FONT_SIZE = 30


def wrap180(deg: float) -> float:
  return (deg + 180.0) % 360.0 - 180.0


class Compass(Widget):
  """Two-letter heading readout for the onroad side rail (N, NE, E, ...)."""

  def __init__(self, demo_heading: float | None = None):
    super().__init__()
    self._label = UnifiedLabel("", FONT_SIZE, FontWeight.BOLD,
                               text_color=rl.Color(255, 255, 255, 235),
                               alignment=TextAlignment.CENTER,
                               alignment_vertical=TextAlignmentVertical.MIDDLE)
    self._demo_heading = demo_heading
    self._heading = 0.0
    self._has_heading = demo_heading is not None
    self._alpha_filter = FirstOrderFilter(0.0, 0.4, 1 / gui_app.target_fps)

  def cardinal(self) -> str:
    return CARDINALS[int(((self._heading + 22.5) % 360.0) // 45.0)]

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

    self._label.set_text(self.cardinal())
    self._label.set_text_color(rl.Color(255, 255, 255, int(235 * alpha)))
    self._label.render(self._rect)
