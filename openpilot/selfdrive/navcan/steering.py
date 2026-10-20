"""Steering-angle -> nav instruction, for exercising the cluster without a route.

Kept separate from the CAN encoder so it is testable and so tools/nav/steering_nav.py
and any in-process user share one definition.
"""
from __future__ import annotations

LEFT, RIGHT, STRAIGHT = "LEFT", "RIGHT", "STRAIGHT"


def classify(angle: float, threshold: float, hysteresis: float) -> str:
  """Return 'LEFT', 'RIGHT', 'STRAIGHT', or '' to keep the previous state.

  Empty means "inside the hysteresis band": the caller holds whatever it had, which is
  what keeps the arrow from flickering at the threshold.
  """
  if angle >= threshold:
    return RIGHT
  if angle <= -threshold:
    return LEFT
  if abs(angle) <= threshold - hysteresis:
    return STRAIGHT
  return ""


def instruction(state: str, angle: float, distance_m: float = 50.0) -> dict:
  """navInstruction fields for a steering-derived state."""
  if state == STRAIGHT:
    mtype, modifier = "continue", "straight"
  else:
    mtype, modifier = "turn", state.lower()
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

  t, h = 8.0, 4.0
  check("right", classify(9, t, h) == RIGHT)
  check("left", classify(-9, t, h) == LEFT)
  check("straight", classify(0, t, h) == STRAIGHT)
  check("hold right band", classify(6, t, h) == "")
  check("hold left band", classify(-6, t, h) == "")
  check("leave band", classify(3, t, h) == STRAIGHT)
  check("turn fields", instruction(RIGHT, 9)["maneuverModifier"] == "right")
  check("straight fields", instruction(STRAIGHT, 0)["maneuverType"] == "continue")
  print("steering self-test:", "ok" if ok else "FAILED")
  return 0 if ok else 1


if __name__ == "__main__":
  raise SystemExit(run_self_test())
