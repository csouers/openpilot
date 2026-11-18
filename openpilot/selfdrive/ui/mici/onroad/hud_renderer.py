import math
import numpy as np
import raylib_nb as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad import DIMMED_ICON_ALPHA
from openpilot.selfdrive.ui.mici.onroad.status import STATUS_FADE_RC, lerp_color, status_fade
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus, ChestnutState
from openpilot.system.ui.lib.application import gui_app, FontWeight, ASSETS_DIR
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter


# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds
WHEEL_SIZE = 52
WHEEL_WHITE = rl.Color(255, 255, 255, 255)
WHEEL_GREEN = rl.Color(0, 255, 64, 255)
WHEEL_ORANGE = rl.Color(255, 115, 0, 255)


def _texture(rgba: np.ndarray, px: float) -> rl.Texture:
  texture = rl.load_texture_from_image(rl.Image(None, rgba.shape[1], rgba.shape[0], 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8))
  rl.update_texture(texture, np.ascontiguousarray(rgba))
  rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
  texture.width, texture.height = round(rgba.shape[1] / px), round(rgba.shape[0] / px)
  return texture


def _wheel_effect_textures(asset: str, size: int) -> tuple[rl.Texture, rl.Texture]:
  """White glow and center highlight for the wheel, matching the design's render. Tint the glow when drawing."""
  image = gui_app._load_image_from_path((ASSETS_DIR / asset).as_posix(), size, size)
  w, h, px = image.width, image.height, image.width / size
  alpha = np.frombuffer(image.to_bytes(w * h * 4), dtype=np.uint8).reshape(h, w, 4)[..., 3] / 255.0
  rl.unload_image(image)

  # glow: Figma's drop shadow (blur 24 at 128px) renders as a gaussian with sigma 10, its spread is ignored on vectors
  sigma = 10 * size / 128 * px
  pad = math.ceil(3 * sigma)
  glow = np.pad(alpha, pad)
  x = np.arange(-pad, pad + 1)
  kernel = np.exp(-x ** 2 / (2 * sigma ** 2))
  kernel /= kernel.sum()
  for axis in (0, 1):
    glow = np.apply_along_axis(np.convolve, axis, glow, kernel, mode='same')
  glow_rgba = np.full((*glow.shape, 4), 255, dtype=np.uint8)
  glow_rgba[..., 3] = np.clip(glow * 255, 0, 255)

  # highlight: the fill is a radial gradient from 35% lighter at the center to the base color at the rim
  yy, xx = np.mgrid[:h, :w]
  r = np.hypot(xx + 0.5 - w / 2, yy + 0.5 - h / 2) / (w / 2)
  highlight_rgba = np.full((h, w, 4), 255, dtype=np.uint8)
  highlight_rgba[..., 3] = np.clip(alpha * 89 * (1 - r), 0, 255)

  return _texture(glow_rgba, px), _texture(highlight_rgba, px)

# bottom row geometry: the model-source icon sits above the bottom edge, on the same row as the
# (undrawn) wheel icon reference. status_icons.py centres the experimental icon between this and
# the compass, so both files must agree on where this row is.
CHESTNUT_MARGIN_BOTTOM = 14
CHESTNUT_ROW_H = 50   # wheel.png reference height
CHESTNUT_ICON_H = 44


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 48  # 15% smaller than the 57 (which was 1.5x the original 38)
  set_speed: int = 95


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  # the model-source icon sits 14 px above the bottom edge; add its height so its whole
  # body clears that edge when it slides away
  CHESTNUT_SLIDE_PX = 14
  # the icon is centered on the compass's vertical axis, which status_icons.py draws
  # COMPASS_W/2 + COMPASS_MARGIN_RIGHT (60 + 4) in from the right edge of the screen
  CHESTNUT_CENTER_MARGIN_RIGHT = 64

  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False
    self._chestnut_fade_time: float = 0

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', WHEEL_SIZE, WHEEL_SIZE)
    self._txt_wheel_glow, self._txt_wheel_highlight = _wheel_effect_textures('icons_mici/wheel.png', WHEEL_SIZE)
    self._txt_chestnut: rl.Texture = gui_app.texture('icons_mici/chestnut.png', 48, 36)
    self._txt_chestnut_green: rl.Texture = gui_app.texture('icons_mici/chestnut_green.png', 48, 36)
    # same 48 px width as the other two states, so the single vertical axis is kept and the icon
    # stays clear of the blind-spot bar that lives in the outer 26 px of the screen edge
    self._txt_chestnut_orange: rl.Texture = gui_app.texture('icons_mici/chestnut_orange.png', 48, 36)
    self._chestnut_icon: rl.Texture | None = None
    self._right_panel_width: float = 0
    self._wheel_critical_filter = FirstOrderFilter(0, STATUS_FADE_RC, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._set_speed_alpha = 0.0
    self._chestnut_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  @classmethod
  def chestnut_center_y(cls, rect: rl.Rectangle) -> float:
    """Vertical centre of the bottom icon row (top edge relative to the full-height rect)."""
    return rect.y + rect.height - CHESTNUT_MARGIN_BOTTOM - (CHESTNUT_ROW_H + CHESTNUT_ICON_H) / 2

  def set_right_panel_width(self, width: float):
    """Width of the panel (navigation) that shrinks this renderer's rect on the right."""
    self._right_panel_width = width

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_torque_bar_hidden(self, hidden: bool):
    self._torque_bar.set_hidden(hidden)

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if not ui_state.received_since_started("carState"):
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    car_state = sm['carState']

    # the cluster value is what the car actually shows; fall back to the controller's set speed
    # when the car doesn't report one (upstream dropped this fallback)
    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = sm['controlsState'].deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    if engaged != self._engaged:
      self._chestnut_fade_time = rl.get_time() if engaged else 0
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    self._draw_set_speed(rect)

    self._draw_model_source(rect)

  def render_steering_wheel(self, rect: rl.Rectangle) -> None:
    """Draw the steering wheel centered at the bottom of rect, after render() has updated state."""
    self._draw_steering_wheel(rect)

  def _draw_model_source(self, rect: rl.Rectangle) -> None:
    state = ui_state.chestnut_state
    loading = state == ChestnutState.LOADING
    if loading:
      icon = self._txt_chestnut
      opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED, ChestnutState.UNAVAILABLE):
      icon = self._txt_chestnut_orange
      opacity = 1.0
    elif state == ChestnutState.WAITING:
      # attached with its engine up, only the swap window missing: steady, unlike the load pulse
      icon = self._txt_chestnut
      opacity = 1.0
    elif state == ChestnutState.ACTIVE:
      icon = self._txt_chestnut_green
      opacity = 1.0
    else:
      # nothing to show: keep fading the last one out, so it slides away instead of vanishing
      icon = None
      opacity = 1.0

    if icon is not None and icon is not self._chestnut_icon:
      self._chestnut_fade_time = rl.get_time()
      self._chestnut_icon = icon
    # the green icon is a status, not a notification: while the large model drives, it stays up
    # for as long as that is true. The other states keep the brief "something changed" show.
    visible = icon is not None and (loading or icon is self._txt_chestnut_green or
                                    rl.get_time() - self._chestnut_fade_time < SET_SPEED_PERSISTENCE)
    alpha = self._chestnut_alpha_filter.update(visible)
    if alpha < 1e-2 or self._chestnut_icon is None:
      return

    icon = self._chestnut_icon
    # slide with the fade, as the set-speed box does: up into place from the bottom edge when
    # it appears, and back off the bottom edge when it goes
    slide = (1.0 - alpha) * (icon.height + self.CHESTNUT_SLIDE_PX)
    # centered on the compass axis; the nav panel, when shown, narrows our rect by its width
    center_x = rect.x + rect.width - self._right_panel_width - self.CHESTNUT_CENTER_MARGIN_RIGHT
    pos = rl.Vector2(center_x - icon.width / 2,
                     self.chestnut_center_y(rect) - icon.height / 2 + slide)
    rl.draw_texture_ex(icon, pos, 0.0, 1.0, rl.Color(255, 255, 255, int(255 * opacity * alpha)))

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    # The wheel is the source of truth for engagement: green if and only if openpilot is engaged.
    # When disengaged it's grayed out, except for steerRequired alerts which show it orange.
    critical = self._wheel_critical_filter.update(self._show_wheel_critical)
    idle_color = lerp_color(WHEEL_WHITE, WHEEL_ORANGE, critical)
    wheel_color = status_fade.color({UIStatus.ENGAGED: WHEEL_GREEN, UIStatus.OVERRIDE: idle_color, UIStatus.DISENGAGED: idle_color})
    engaged = status_fade.weight(UIStatus.ENGAGED)
    glow = engaged + (1 - engaged) * critical
    wheel_txt = self._txt_wheel

    # pos
    pos_x = int(rect.x + rect.width / 2)
    pos_y = int(rect.y + rect.height - 10 - wheel_txt.height / 2)
    rotation = -ui_state.sm['carState'].steeringAngleDeg if ui_state.started and ui_state.received_since_started('carState') else 0.0

    alpha = 255 * (1 - (1 - DIMMED_ICON_ALPHA) * status_fade.weight(UIStatus.DISENGAGED) * (1 - critical))
    origin = rl.Vector2(self._txt_wheel_glow.width / 2, self._txt_wheel_glow.height / 2)
    rl.draw_texture_pro(self._txt_wheel_glow, rl.Rectangle(0, 0, self._txt_wheel_glow.width, self._txt_wheel_glow.height),
                        rl.Rectangle(pos_x, pos_y, self._txt_wheel_glow.width, self._txt_wheel_glow.height), origin, rotation,
                        rl.Color(wheel_color.r, wheel_color.g, wheel_color.b, int(alpha * glow)))
    rl.draw_texture_pro(wheel_txt, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(wheel_color.r, wheel_color.g, wheel_color.b, int(alpha)))
    rl.draw_texture_pro(self._txt_wheel_highlight, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(255, 255, 255, int(alpha * glow)))

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    # fork: the MAX indicator stays up while it may be drawn (7cd4f5d45 dropped the 2.5 s persistence)
    alpha = self._set_speed_alpha = self._set_speed_alpha_filter.update(float(self._can_draw_top_icons))
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y - (1.0 - alpha) * (FONT_SIZES.set_speed + FONT_SIZES.max_speed)

    # draw drop shadow
    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    set_speed_width = measure_text_cached(self._font_display, set_speed_text, FONT_SIZES.set_speed).x

    # both the number and the label share one center axis (the glow circle's center), so they
    # stay aligned regardless of glyph metrics — including the en dash used when cruise is off
    center_x = x + circle_radius
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(center_x - set_speed_width / 2, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("max")
    max_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(center_x - max_width / 2, y + FONT_SIZES.set_speed - 6),  # sliver of extra space between the number and the label (was set_speed - 11)
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
