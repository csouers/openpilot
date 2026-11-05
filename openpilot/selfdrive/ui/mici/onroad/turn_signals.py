import raylib_nb as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

ICON_W, ICON_H = 72, 66
MARGIN = 16


class TurnSignals(Widget):
  """Turn-signal arrows in the bottom corners, lit by the car's actual lamp output.

  Hidden when the signal is off; the car's flasher output already provides the blink.
  Falls back to the (filtered) blinker state on cars that don't report the flasher output.
  """

  def __init__(self):
    super().__init__()
    self._left = gui_app.texture('icons_mici/onroad/turn_signal_left.png', ICON_W, ICON_H)
    self._right = gui_app.texture('icons_mici/onroad/turn_signal_left.png', ICON_W, ICON_H, flip_x=True)
    self._left_on = False
    self._right_on = False

  def _update_state(self):
    sm = ui_state.sm
    body = sm['carBodyState']
    if body.blinkerLampValid:
      self._left_on = body.leftBlinkerLamp
      self._right_on = body.rightBlinkerLamp
    else:
      CS = sm['carState']
      self._left_on = CS.leftBlinker
      self._right_on = CS.rightBlinker

  def _render(self, _):
    bottom_y = self._rect.y + self._rect.height - ICON_H - MARGIN
    for texture, x, on in ((self._left, self._rect.x + MARGIN, self._left_on),
                           (self._right, self._rect.x + self._rect.width - MARGIN - ICON_W, self._right_on)):
      if on:
        rl.draw_texture_ex(texture, rl.Vector2(x, bottom_y), 0.0, 1.0, rl.WHITE)
