"""Steering-angle -> nav instruction, for exercising the cluster without a route.

Kept separate from the CAN encoder so it is testable and so tools/nav/steering_nav.py
and any in-process user share one definition.

Angle sign follows openpilot's ISO 8855 convention: positive is a LEFT turn.

Severity comes from the wheel angle and is deliberately heuristic — it is a dev source,
not a navigation system:
    |angle| < hold                 -> STRAIGHT
    hold .. slight                 -> slight left/right
    slight .. sharp                -> left/right (normal)
    sharp .. uturn                 -> sharp left/right
    >= uturn, and slow             -> uturn left/right
Bands are steering-wheel degrees (tuned 2026-10-21 on the car: the old 8/35/120/170
fired a band early at every step, so a small correction already drew a turn arrow).
Current bands: 20 / 60 / 180 / 360 (U-turn also requires vEgo <= 4 m/s).
The speed gate on the U-turn matters: at parking speed the wheel sits far past 170 deg
for an ordinary turn, so angle alone would call every parking manoeuvre a U-turn.
"""
from __future__ import annotations

LEFT, RIGHT, STRAIGHT = "LEFT", "RIGHT", "STRAIGHT"

# maneuver strings use the Modifier vocabulary Mapbox/AA already speak
SEVERITY = ("slight", "", "sharp", "uturn")


def _severity(a: float, prev_level: int, edges: tuple[float, float, float],
              hysteresis: float) -> int:
  level = sum(a >= e for e in edges)
  if prev_level > level:
    # only drop a band after falling hysteresis below the edge we came from
    edge = edges[prev_level - 1]
    if a >= edge - hysteresis:
      level = prev_level
  return level


def classify(angle: float, threshold: float, hysteresis: float,
             slight_deg: float = 60.0, sharp_deg: float = 180.0,
             uturn_deg: float = 360.0, v_ego: float = 0.0,
             uturn_max_speed: float = 4.0, prev: str = "") -> str:
  """Return '', 'straight left', 'straight', 'slight left', 'left', ....

  '' means "inside the hold band": the caller keeps whatever it had.
  """
  a = abs(angle)
  if a >= threshold:
    side = "left" if angle > 0 else "right"
    prev_side = prev.rsplit(maxsplit=1)[-1] if prev and prev != STRAIGHT.lower() else ""
    prev_level = 0
    if prev_side == side:
      words = prev.split()
      prev_level = SEVERITY.index(words[0]) if words[0] in SEVERITY else 1
    level = _severity(a, prev_level, (slight_deg, sharp_deg, uturn_deg), hysteresis)
    if level == 3 and v_ego > uturn_max_speed:
      level = 2  # fast + big angle is a sharp turn, not a U-turn
    word = SEVERITY[level]
    return f"{word} {side}".strip()
  if a <= threshold - hysteresis:
    return "straight"
  return ""


def instruction(state: str, angle: float, distance_m: float = 50.0) -> dict:
  """navInstruction fields for a steering-derived state."""
  if state == "straight":
    mtype, modifier = "continue", "straight"
  else:
    mtype, modifier = "turn", state
  return {
    "maneuverType": mtype,
    "maneuverModifier": modifier,
    "maneuverDistance": float(distance_m),
    "maneuverPrimaryText": f"steer {angle:+.0f}",
  }


def run_self_test() -> int:
  ok = True

  def check(name, cond):
    nonlocal ok
    if not cond:
      print(f"[fail] {name}")
      ok = False

  t, h = 20.0, 6.0
  # sign: ISO 8855 positive = left
  check("left", classify(25, t, h) == "slight left")
  check("right", classify(-25, t, h) == "slight right")
  check("straight", classify(0, t, h) == "straight")
  check("hold band", classify(17, t, h) == "")
  check("leave band", classify(13, t, h) == "straight")
  check("slight", classify(30, t, h) == "slight left")
  check("normal", classify(100, t, h) == "left")
  check("sharp", classify(250, t, h) == "sharp left")
  check("uturn slow", classify(380, t, h, v_ego=1.0) == "uturn left")
  check("uturn only when slow", classify(380, t, h, v_ego=10.0) == "sharp left")
  check("severity hysteresis", classify(178, t, h, prev="sharp left") == "sharp left")
  check("severity clears", classify(172, t, h, prev="sharp left") == "left")
  check("straight fields", instruction("straight", 0)["maneuverType"] == "continue")
  check("slight fields", instruction("slight right", -20)["maneuverModifier"] == "slight right")
  check("uturn fields", instruction("uturn left", 200)["maneuverModifier"] == "uturn left")
  print("steering self-test:", "ok" if ok else "FAILED")
  return 0 if ok else 1


if __name__ == "__main__":
  raise SystemExit(run_self_test())
