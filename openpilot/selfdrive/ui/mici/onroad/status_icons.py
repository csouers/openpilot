import pyray as rl

from openpilot.cereal import log
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


INACTIVE_COLOR = rl.Color(86, 86, 86, 255)
INACTIVE_TEXTURE_TINT = rl.Color(140, 140, 140, 255)
CRUISE_COLOR = rl.Color(0, 163, 255, 255)
LEAD_COLOR = rl.Color(255, 190, 0, 255)
E2E_COLOR = rl.Color(0, 255, 204, 255)

PlanSource = log.LongitudinalPlan.LongitudinalPlanSource


class StatusIconColumn(Widget):
  """Three-slot status rail for experimental mode, speed limiter, and eGPU use."""

  def __init__(self):
    super().__init__()

    self._experimental_texture = gui_app.texture("icons_mici/experimental_mode.png", 48, 48)

    self._experimental_mode = False
    self._plan_source = log.LongitudinalPlan.LongitudinalPlanSource.lead0
    self._speed_limiter_active = False
    self._using_egpu = False

  def _update_state(self):
    sm = ui_state.sm
    self._experimental_mode = (sm.alive["selfdriveState"] and sm.valid["selfdriveState"] and
                               sm["selfdriveState"].experimentalMode)
    self._plan_source = sm["longitudinalPlan"].longitudinalPlanSource
    self._speed_limiter_active = (sm.alive["carControl"] and sm.valid["carControl"] and
                                  sm["carControl"].longActive and
                                  sm.alive["longitudinalPlan"] and sm.valid["longitudinalPlan"])

  @staticmethod
  def _draw_texture_centered(texture: rl.Texture, center: rl.Vector2, tint: rl.Color = rl.WHITE):
    pos = rl.Vector2(center.x - texture.width / 2, center.y - texture.height / 2)
    rl.draw_texture_ex(texture, pos, 0.0, 1.0, tint)

  @staticmethod
  def _draw_cruise_icon(center: rl.Vector2, color: rl.Color):
    gauge_center = rl.Vector2(center.x, center.y + 4)
    rl.draw_ring(gauge_center, 14, 19, 180, 360, 32, color)
    rl.draw_line_ex(gauge_center, rl.Vector2(center.x + 11, center.y - 6), 4, color)
    rl.draw_circle(int(gauge_center.x), int(gauge_center.y), 4, color)

  @staticmethod
  def _draw_lead_icon(center: rl.Vector2, color: rl.Color, active: bool):
    # Front-view car silhouette.
    rl.draw_rectangle_rounded(rl.Rectangle(center.x - 12, center.y - 18, 24, 17), 0.35, 8, color)
    rl.draw_rectangle_rounded(rl.Rectangle(center.x - 18, center.y - 8, 36, 23), 0.3, 8, color)
    rl.draw_rectangle_rec(rl.Rectangle(center.x - 8, center.y - 14, 16, 9), rl.BLACK)
    detail_color = rl.WHITE if active else color
    rl.draw_circle(int(center.x - 10), int(center.y + 7), 3, detail_color)
    rl.draw_circle(int(center.x + 10), int(center.y + 7), 3, detail_color)
    rl.draw_rectangle_rec(rl.Rectangle(center.x - 19, center.y + 5, 4, 10), color)
    rl.draw_rectangle_rec(rl.Rectangle(center.x + 15, center.y + 5, 4, 10), color)

  @staticmethod
  def _draw_e2e_icon(center: rl.Vector2, color: rl.Color):
    # A compact network glyph for the end-to-end model.
    start = rl.Vector2(center.x, center.y + 18)
    left = rl.Vector2(center.x - 13, center.y)
    right = rl.Vector2(center.x + 13, center.y)
    end = rl.Vector2(center.x, center.y - 18)
    for first, second in ((start, left), (start, right), (left, end), (right, end)):
      rl.draw_line_ex(first, second, 4, color)
    for point in (start, left, right, end):
      rl.draw_circle(int(point.x), int(point.y), 5, color)

  def _draw_speed_limiter(self, center: rl.Vector2):
    if not self._speed_limiter_active:
      color = INACTIVE_COLOR
    elif self._plan_source in (PlanSource.lead0, PlanSource.lead1, PlanSource.lead2):
      color = LEAD_COLOR
    elif self._plan_source == PlanSource.e2e:
      color = E2E_COLOR
    else:
      color = CRUISE_COLOR

    if self._plan_source in (PlanSource.lead0, PlanSource.lead1, PlanSource.lead2):
      self._draw_lead_icon(center, color, self._speed_limiter_active)
    elif self._plan_source == PlanSource.e2e:
      self._draw_e2e_icon(center, color)
    else:
      self._draw_cruise_icon(center, color)

  def _render(self, _):
    panel = rl.Rectangle(
      self.rect.x,  # left side rail, opposite the confidence ball
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )
    center_x = panel.x + panel.width / 2
    centers = [rl.Vector2(center_x, panel.y + panel.height * (slot + 0.5) / 2) for slot in range(2)]

    experimental_tint = rl.WHITE if self._experimental_mode else INACTIVE_COLOR
    self._draw_texture_centered(self._experimental_texture, centers[0], experimental_tint)
    self._draw_speed_limiter(centers[1])
