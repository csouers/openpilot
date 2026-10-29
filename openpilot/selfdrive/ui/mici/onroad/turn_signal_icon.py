import pyray as rl

# the turn signal icons are drawn, not textured: the shipped PNG is LFS-tracked (no LFS push
# credentials here) and carries a baked-in glow
TURN_SIGNAL_COLOR = (39, 255, 93)


def draw_turn_signal(box: rl.Rectangle, side: str, color: rl.Color) -> None:
  """Flat (no glow) turn signal arrow filling `box`, pointing left or right."""
  w, h = box.width, box.height
  cy = box.y + h / 2
  head_h = 0.80 * h
  bar_h = 0.32 * h

  def px(fx: float) -> float:
    return box.x + fx * w if side == "left" else box.x + (1.0 - fx) * w

  rl.draw_triangle(rl.Vector2(px(0.02), cy),
                   rl.Vector2(px(0.46), cy - head_h / 2),
                   rl.Vector2(px(0.46), cy + head_h / 2), color)

  x0, x1 = px(0.44), px(1.0)
  rl.draw_rectangle_rec(rl.Rectangle(min(x0, x1), cy - bar_h / 2, abs(x1 - x0), bar_h), color)


def turn_signal_color(alpha: int) -> rl.Color:
  return rl.Color(TURN_SIGNAL_COLOR[0], TURN_SIGNAL_COLOR[1], TURN_SIGNAL_COLOR[2], alpha)
