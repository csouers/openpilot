import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.turn_signal_icon import draw_turn_signal, turn_signal_color
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget

ICON_W, ICON_H = 72, 66
MARGIN = 16
DIM_ALPHA = 55  # idle brightness, so the corner indicators are always visible


class TurnSignals(Widget):
  """Debug turn-signal arrows in the bottom corners, lit by the car's actual lamp output.

  The off state is drawn dim rather than hidden so the placement is visible while debugging.
  Falls back to the (filtered) blinker state on cars that don't report the flasher output.
  """

  def __init__(self):
    super().__init__()
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
    left_box = rl.Rectangle(self._rect.x + MARGIN, bottom_y, ICON_W, ICON_H)
    right_box = rl.Rectangle(self._rect.x + self._rect.width - MARGIN - ICON_W, bottom_y, ICON_W, ICON_H)

    for box, side, on in ((left_box, "left", self._left_on), (right_box, "right", self._right_on)):
      draw_turn_signal(box, side, turn_signal_color(255 if on else DIM_ALPHA))
