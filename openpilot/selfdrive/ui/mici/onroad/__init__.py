import raylib_nb as rl

SIDE_PANEL_WIDTH = 72
DIMMED_ICON_ALPHA = 0.32  # side panel icons while disengaged

# Drop shadow: same recipe as the set-speed box (hud_renderer), a radial black gradient whose
# center is 50% black and which fades to nothing. Bars have no blur available, so they get
# stacked expanding rounded rects at falling alpha instead.
SHADOW_ALPHA = 0.5


def draw_icon_shadow(center: rl.Vector2, radius: float, alpha: float = 1.0) -> None:
  """Soft radial drop shadow behind an icon."""
  if alpha <= 1e-2 or radius <= 0:
    return
  rl.draw_circle_gradient(center, radius, rl.Color(0, 0, 0, int(255 * SHADOW_ALPHA * alpha)), rl.BLANK)


def draw_bar_shadow(rect: rl.Rectangle, roundness: float, segments: int, alpha: float = 1.0) -> None:
  """Soft drop shadow behind a rounded bar."""
  if alpha <= 1e-2:
    return
  for i in range(4):
    grow = 3.0 * (i + 1)
    a = int(255 * SHADOW_ALPHA * alpha / (4 * (i + 1)))
    rl.draw_rectangle_rounded(rl.Rectangle(rect.x - grow, rect.y - grow, rect.width + 2 * grow, rect.height + 2 * grow),
                              roundness, segments, rl.Color(0, 0, 0, a))


def blend_colors(a: rl.Color, b: rl.Color, f: float) -> rl.Color:
  h0, s0, v0 = (hsv0 := rl.color_to_hsv(a)).x, hsv0.y, hsv0.z
  h1, s1, v1 = (hsv1 := rl.color_to_hsv(b)).x, hsv1.y, hsv1.z
  dh = ((h1 - h0 + 180) % 360) - 180  # shortest hue delta
  color = rl.color_from_hsv((h0 + f * dh) % 360,
                            s0 + f * (s1 - s0),
                            v0 + f * (v1 - v0))
  color.a = round(a.a + f * (b.a - a.a))
  return color
