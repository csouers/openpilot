import raylib_nb as rl

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.compass import Compass
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

LaneChangeState = log.LaneChangeState

# Vertical space the status icons occupy at the top of the rail
STATUS_ICONS_HEIGHT = 116
# extra downward nudge for the experimental icon out of the compass-block centre
EXPERIMENTAL_MODE_ICON_DROP = 6
# how long the experimental icon trails the compass when the pair slides in/out
EXPERIMENTAL_ICON_DELAY_S = 0.18
# orange overlay on the (white) experimental-mode icon so it reads as its own state
EXPERIMENTAL_MODE_ICON_COLOR = rl.Color(255, 165, 0, 255)
# Compass sits at the very top of the rail (wider than the rail so two big letters fit),
# the experimental icon right below it
COMPASS_W = 120  # 1.5x the original 80
COMPASS_H = 70   # fits the 70 px font; tightened from the 1.5x-scaled 78 so the ink tucks up toward the corner
COMPASS_MARGIN_TOP = 4   # tight to the top edge to match the set-speed box's inset (was 11)
COMPASS_MARGIN_RIGHT = 4


class StatusIconColumn(Widget):
  """Status rail: compass in the top-right corner, experimental mode icon next to it."""

  def __init__(self):
    super().__init__()

    self._experimental_texture = gui_app.texture("icons_mici/experimental_mode.png", 48, 48)
    self._compass = Compass()

    self._experimental_mode = False
    self._alert_present = False
    self._should_show = False
    # slides in/out through the right edge, but held back so the two move as a
    # sequence rather than together
    self._slide = FirstOrderFilter(0.0, 0.4, 1 / gui_app.target_fps)
    self._delay_left = 0.0

  def set_alert_present(self, alert_present: bool):
    # an alert banner draws across the top of the screen: it covers the compass and the
    # lane change prompts draw over this corner, so both step aside
    self._alert_present = alert_present
    self._compass.set_suppressed(alert_present)

  def _update_state(self):
    sm = ui_state.sm
    self._experimental_mode = (sm.alive["selfdriveState"] and sm.valid["selfdriveState"] and
                               sm["selfdriveState"].experimentalMode)

    # the icon is clutter in this corner while changing lanes: hide for any alert (the
    # lane change prompt is one) and for the maneuver itself, which outlives the prompt
    lane_changing = (sm.alive["modelV2"] and sm.valid["modelV2"] and
                     sm["modelV2"].meta.laneChangeState != LaneChangeState.off)

    was_on = self._should_show
    self._should_show = self._experimental_mode and not lane_changing and not self._alert_present

    if was_on != self._should_show:
      # geometry changes start with the compass; we just wait our turn
      self._delay_left = EXPERIMENTAL_ICON_DELAY_S

    dt = 1 / gui_app.target_fps
    if self._delay_left > 0.0:
      self._delay_left = max(0.0, self._delay_left - dt)
    else:
      # same rate as the compass slide, so the motion reads as one gesture
      self._slide.update_alpha(0.4)
      self._slide.update(1.0 if self._should_show else 0.0)

  @staticmethod
  def _draw_texture_centered(texture: rl.Texture, center: rl.Vector2, tint: rl.Color = rl.WHITE):
    pos = rl.Vector2(center.x - texture.width / 2, center.y - texture.height / 2)
    rl.draw_texture_ex(texture, pos, 0.0, 1.0, tint)

  def _render(self, _):
    panel = rl.Rectangle(
      self.rect.x + self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    # compass in the top-right corner (right-aligned to the screen edge)
    self._compass.render(rl.Rectangle(self.rect.x + self.rect.width - COMPASS_W - COMPASS_MARGIN_RIGHT,
                                     self.rect.y + COMPASS_MARGIN_TOP,
                                     COMPASS_W, COMPASS_H))

    slide = self._slide.x
    if slide < 0.02:
      return

    # horizontally centered on the compass (the 60 px rail is narrower than it), and below it
    center_x = self.rect.x + self.rect.width - COMPASS_W / 2 - COMPASS_MARGIN_RIGHT
    icons_top = panel.y + COMPASS_MARGIN_TOP + COMPASS_H
    center_y = icons_top + (panel.y + STATUS_ICONS_HEIGHT - icons_top) / 2 + EXPERIMENTAL_MODE_ICON_DROP

    # slide in from the right edge (the compass slides in through the top): at slide=0 the icon
    # sits fully beyond the right edge, so it emerges from the edge and tucks back out
    center_x += (1.0 - slide) * (COMPASS_W / 2 + self._experimental_texture.width)
    center = rl.Vector2(center_x, center_y)

    self._draw_texture_centered(self._experimental_texture, center, EXPERIMENTAL_MODE_ICON_COLOR)
