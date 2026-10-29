import pyray as rl

from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.compass import Compass
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# Vertical space the status icons occupy at the top of the rail
STATUS_ICONS_HEIGHT = 100
# Compass sits at the very top of the rail, the experimental icon right below it
COMPASS_W = 52
COMPASS_H = 36
COMPASS_MARGIN_TOP = 4


class StatusIconColumn(Widget):
  """Status rail: compass in the top-right corner, experimental mode icon next to it."""

  def __init__(self):
    super().__init__()

    self._experimental_texture = gui_app.texture("icons_mici/experimental_mode.png", 48, 48)
    self._compass = Compass()

    self._experimental_mode = False

  def _update_state(self):
    sm = ui_state.sm
    self._experimental_mode = (sm.alive["selfdriveState"] and sm.valid["selfdriveState"] and
                               sm["selfdriveState"].experimentalMode)

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

    # compass in the top-right corner
    self._compass.render(rl.Rectangle(panel.x + (panel.width - COMPASS_W) / 2,
                                     panel.y + COMPASS_MARGIN_TOP,
                                     COMPASS_W, COMPASS_H))

    if not self._experimental_mode:
      return

    center_x = panel.x + panel.width / 2
    icons_top = panel.y + COMPASS_MARGIN_TOP + COMPASS_H
    center = rl.Vector2(center_x, icons_top + (panel.y + STATUS_ICONS_HEIGHT - icons_top) / 2)

    self._draw_texture_centered(self._experimental_texture, center, rl.WHITE)
