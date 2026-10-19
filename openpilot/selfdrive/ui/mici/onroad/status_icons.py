import pyray as rl

from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


INACTIVE_COLOR = rl.Color(86, 86, 86, 255)

# Vertical space the status icon occupies at the top of the rail
STATUS_ICONS_HEIGHT = 100

class StatusIconColumn(Widget):
  """Status rail for the experimental mode icon."""

  def __init__(self):
    super().__init__()

    self._experimental_texture = gui_app.texture("icons_mici/experimental_mode.png", 48, 48)

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
    center_x = panel.x + panel.width / 2
    center = rl.Vector2(center_x, panel.y + STATUS_ICONS_HEIGHT / 2)

    experimental_tint = rl.WHITE if self._experimental_mode else INACTIVE_COLOR
    self._draw_texture_centered(self._experimental_texture, center, experimental_tint)
