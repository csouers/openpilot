"""The mici HUD's model-source icon.

The green one is a status while the large model drives: it stays up for as long as that is
true. The others (loading pulses, orange, waiting) are a brief "something changed" show,
gone after SET_SPEED_PERSISTENCE. Either way the icon slides off the bottom edge as it goes
rather than vanishing. None of it needs a raylib window: rl and ui_state are stubbed.
"""
import types

import pytest

from openpilot.common.filter_simple import FirstOrderFilter

hud_renderer = pytest.importorskip('openpilot.selfdrive.ui.mici.onroad.hud_renderer')
H = hud_renderer

RECT_H = 200.
WHEEL_H = 44.
ICON_H = 44.
BASE_Y = RECT_H - 14 - (WHEEL_H + ICON_H) / 2


class FakeRL:
  def __init__(self):
    self.t = 0.0
    self.drawn = []
    self.positions = []
    outer = self

    class Vector2:
      def __init__(self, x, y):
        self.x, self.y = x, y
        outer.positions.append((x, y))

    self.Vector2 = Vector2

  def get_time(self):
    return self.t

  def draw_texture_ex(self, icon, pos, rot, scale, color):
    self.drawn.append(icon)

  class Color:
    def __init__(self, *a):
      pass


class FakeUIState:
  def __init__(self, state):
    self.chestnut_state = state


def _run(*phases):
  """Drive the renderer for each (state, seconds) phase, e.g. _run((ACTIVE, 2), (DISCONNECTED, 2))."""
  fake_rl = FakeRL()
  H.rl = fake_rl
  H.ui_state = FakeUIState(phases[0][0])
  normal = types.SimpleNamespace(width=60, height=ICON_H)
  owner = types.SimpleNamespace(
    CHESTNUT_SLIDE_PX=H.HudRenderer.CHESTNUT_SLIDE_PX,
    _txt_chestnut=normal,
    _txt_chestnut_green=types.SimpleNamespace(width=60, height=ICON_H),
    _txt_chestnut_orange=types.SimpleNamespace(width=75, height=ICON_H),
    _txt_wheel=types.SimpleNamespace(width=50, height=WHEEL_H),
    _chestnut_icon=None,
    _chestnut_fade_time=0.0,
    _chestnut_alpha_filter=FirstOrderFilter(0.0, 0.1, 1 / 60),
  )
  rect = types.SimpleNamespace(x=0, y=0, width=300, height=RECT_H)
  for state, seconds in phases:
    H.ui_state.chestnut_state = state
    for _ in range(int(seconds * 60)):
      fake_rl.t += 1 / 60
      H.HudRenderer._draw_model_source(owner, rect)
  return owner, fake_rl


def test_active_stays_visible():
  owner, fake_rl = _run((H.ChestnutState.ACTIVE, 6.0))
  assert owner._chestnut_alpha_filter.x > 0.95
  assert fake_rl.drawn and fake_rl.drawn[-1] is owner._txt_chestnut_green
  # fully faded in means fully in place
  assert fake_rl.positions[-1][1] == BASE_Y


@pytest.mark.parametrize('state', [H.ChestnutState.UNCOMPILED, H.ChestnutState.WAITING])
def test_a_change_shows_briefly(state):
  owner, fake_rl = _run((state, 6.0))
  assert owner._chestnut_alpha_filter.x < 0.05
  # the last frame it drew had slid most of the way off the bottom edge
  assert fake_rl.positions[-1][1] > BASE_Y + 20


def test_loading_pulses_through():
  owner, fake_rl = _run((H.ChestnutState.LOADING, 6.0))
  assert owner._chestnut_alpha_filter.x > 0.95
  assert fake_rl.drawn[-1] is owner._txt_chestnut


def test_it_slides_off_when_the_icon_goes_away():
  # the green icon persists while active, then the state clears: it must fade and slide out,
  # not disappear on the first frame
  owner, fake_rl = _run((H.ChestnutState.ACTIVE, 3.0), (H.ChestnutState.DISCONNECTED, 3.0))
  assert owner._chestnut_alpha_filter.x < 0.05
  assert fake_rl.drawn[-1] is owner._txt_chestnut_green
  assert fake_rl.positions[-1][1] > BASE_Y + 20
