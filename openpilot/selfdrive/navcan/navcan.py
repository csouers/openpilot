"""Publish openpilot's nav instruction onto the Honda B-CAN as OPNAV frames.

openpilot already computes a `navInstruction` (navd, Mapbox). The aapi companion box
reads it with a white panda on the 125 kbit/s body bus and turns it into Android Auto
nav events for the instrument cluster. This module is the openpilot end of that link.

Wire format is defined in the DBC generator:
  opendbc_repo/opendbc/dbc/generator/honda/_body_2017.dbc  (OPNAV_*)

Constraints that shaped this:
  * ids are private (0x1A5A0001/2/3) and distinct from the head unit's own write-only
    nav family (0x12F9595x), so the aapi can tell the two sources apart.
  * the cluster only renders arrow + name + distance, so no angle is transmitted.
  * the frames go out on the body bus via the normal `sendcan` path, which means the
    panda safety whitelist must list them (HONDA_BOSCH_TX_MSGS).
"""
from __future__ import annotations

import math

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CAR as HONDA_CARS, DBC

OPNAV_STATUS = 0x1A5A0001
OPNAV_TEXT_CTL = 0x1A5A0002
OPNAV_TEXT_CHUNK = 0x1A5A0003

VERSION = 1
FLAG_VALID = 1 << 0
FLAG_ARRIVED = 1 << 1
FLAG_ROUTE_ACTIVE = 1 << 2

LEFT, RIGHT, UNSPECIFIED = 1, 2, 3
UNIT_METRES = 0
MAX_TEXT = 36
CHUNK_BYTES = 6

TTL_DS = 30          # 3 s: navd publishes at 1 Hz, so this tolerates jitter
STATUS_PERIOD = 50   # frames at 100 Hz = 2 Hz
TEXT_REPEAT = 20     # re-send a changed text after 200 ms

# Android Auto NextTurnEnum, exactly as the HU numbers them.
UNKNOWN, DEPART, NAME_CHANGE, SLIGHT_TURN, TURN, SHARP_TURN, U_TURN = 0, 1, 2, 3, 4, 5, 6
ON_RAMP, OFF_RAMP, FORK, MERGE, ROUNDABOUT_ENTER, ROUNDABOUT_EXIT = 7, 8, 9, 10, 11, 12
ROUNDABOUT_ENTER_AND_EXIT, STRAIGHT = 13, 14
FERRY_BOAT, FERRY_TRAIN, DESTINATION = 16, 17, 19


def _norm(s: str) -> str:
  return " ".join((s or "").lower().replace("_", " ").split())


def turn_side(modifier: str) -> int:
  m = _norm(modifier)
  if "left" in m:
    return LEFT
  if "right" in m:
    return RIGHT
  return UNSPECIFIED


def aa_event(maneuver_type: str, modifier: str) -> int:
  """Mapbox banner (type, modifier) -> AA NextTurnEnum."""
  t, m = _norm(maneuver_type), _norm(modifier)
  if "uturn" in m or "u turn" in m or "uturn" in t:
    return U_TURN
  if t in ("arrive", "destination"):
    return DESTINATION
  if t == "depart":
    return DEPART
  if t == "new name":
    return NAME_CHANGE
  if t == "ferry":
    return FERRY_BOAT
  if t == "ferry train":
    return FERRY_TRAIN
  if t == "merge":
    return MERGE
  if t == "fork":
    return FORK
  if t == "on ramp":
    return ON_RAMP
  if t == "off ramp":
    return OFF_RAMP
  if "roundabout" in t or "rotary" in t:
    return ROUNDABOUT_EXIT if "exit" in t else ROUNDABOUT_ENTER
  if t in ("continue", "straight") or m == "straight":
    return STRAIGHT
  if "sharp" in m:
    return SHARP_TURN
  if "slight" in m:
    return SLIGHT_TURN
  # "turn", "end of road", "notification", unknown -> a plain turn
  return TURN


def _clamp_u16(v: float) -> int:
  if not math.isfinite(v):
    return 0
  return max(0, min(65535, int(round(v))))


def _text_bytes(s: str) -> bytes:
  raw = (s or "").encode("utf-8")
  if len(raw) <= MAX_TEXT:
    return raw
  # cut on a character boundary so the aapi never decodes a broken glyph
  return raw[:MAX_TEXT].decode("utf-8", "ignore").encode("utf-8")


class NavCan:
  """Encodes navInstruction into sendcan frames. Returns None for non-Honda cars."""

  def __init__(self, CP):
    if CP.carFingerprint not in HONDA_CARS or Bus.body not in DBC[CP.carFingerprint]:
      return
    self.CP = CP
    self.bus = CanBus(CP).bcm
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.body])
    self.counter = 0
    self.text_id = 0
    self.last_status_frame = -STATUS_PERIOD
    self.last_text = None
    self.text_frames = []   # queued (name, values) for the chunked name
    self.text_repeat = []   # same frames, sent once more after TEXT_REPEAT
    self.text_resend_at = 0

  def __bool__(self) -> bool:
    return hasattr(self, "bus")

  # -- encoding -------------------------------------------------------------
  def _status_values(self, ins) -> dict:
    valid = bool(ins.valid)
    arrived = valid and aa_event(ins.maneuverType, ins.maneuverModifier) == DESTINATION
    flags = (FLAG_VALID if valid else 0) | (FLAG_ARRIVED if arrived else 0) | (FLAG_ROUTE_ACTIVE if valid else 0)
    dist = 0 if arrived else _clamp_u16(ins.maneuverDistance)
    return {
      "COUNTER": self.counter,
      "VERSION": VERSION,
      "FLAGS": flags,
      "MANEUVER": aa_event(ins.maneuverType, ins.maneuverModifier) if valid else UNKNOWN,
      "TURN_SIDE": turn_side(ins.maneuverModifier),
      "UNIT": UNIT_METRES,
      "DISTANCE": dist,
      "ETA_S": 0,
      "TTL_DS": TTL_DS,
    }

  def _queue_text(self, text: str) -> None:
    raw = _text_bytes(text)
    self.text_id = (self.text_id % 255) + 1
    chunks = max(1, (len(raw) + CHUNK_BYTES - 1) // CHUNK_BYTES) if raw else 0
    frames = [("OPNAV_TextCtl", {
      "COUNTER": self.counter, "TEXT_ID": self.text_id, "LENGTH": len(raw),
      "CHUNKS": chunks, "TTL_DS": TTL_DS,
    })]
    for i in range(chunks):
      part = raw[i * CHUNK_BYTES:(i + 1) * CHUNK_BYTES].ljust(CHUNK_BYTES, b"\x00")
      frames.append(("OPNAV_TextChunk", {
        "TEXT_ID": self.text_id, "CHUNK_INDEX": i,
        **{f"DATA_{j}": part[j] for j in range(CHUNK_BYTES)},
      }))
    self.text_frames = frames

  # -- per-frame update -----------------------------------------------------
  def update(self, ins, frame: int) -> list[CanData]:
    if not self:
      return []

    # A road/instruction change is only worth announcing once, but repeat it once so a
    # single dropped chunk frame cannot leave a half-name on the cluster.
    text = ins.maneuverPrimaryText if ins.valid else ""
    if text != self.last_text:
      self.last_text = text
      self._queue_text(text)
      self.text_repeat = list(self.text_frames)
      self.text_resend_at = frame + TEXT_REPEAT
    elif self.text_repeat and not self.text_frames and frame >= self.text_resend_at:
      # one repeat pass: a single dropped chunk frame must not leave half a name
      self.text_frames = list(self.text_repeat)
      self.text_repeat = []

    # Text goes out before the status. Otherwise the aapi receives a status that names
    # the new maneuver while the road chunks are still in flight, and posts one
    # instruction carrying the previous road.
    if self.text_frames:
      name, values = self.text_frames.pop(0)
      return [self._msg(name, values)]

    if frame - self.last_status_frame >= STATUS_PERIOD:
      self.last_status_frame = frame
      msg = self._msg("OPNAV_Status", self._status_values(ins))
      self.counter = (self.counter + 1) % 256
      return [msg]

    return []

  def _msg(self, name: str, values: dict) -> CanData:
    addr, dat, bus = self.packer.make_can_msg(name, self.bus, values)
    return CanData(addr, dat, bus)


def run_self_test() -> int:
  ok = True

  def check(name, cond):
    nonlocal ok
    if not cond:
      print(f"[fail] {name}")
      ok = False

  check("turn left", (aa_event("turn", "left"), turn_side("left")) == (TURN, LEFT))
  check("turn right", (aa_event("turn", "right"), turn_side("right")) == (TURN, RIGHT))
  check("slight", aa_event("turn", "slight right") == SLIGHT_TURN)
  check("sharp", aa_event("turn", "sharp left") == SHARP_TURN)
  check("uturn", aa_event("turn", "uturn") == U_TURN)
  check("arrive", aa_event("arrive", "") == DESTINATION)
  check("depart", aa_event("depart", "") == DEPART)
  check("straight", aa_event("continue", "straight") == STRAIGHT)
  check("roundabout", aa_event("roundabout", "right") == ROUNDABOUT_ENTER)
  check("roundabout exit", aa_event("exit roundabout", "left") == ROUNDABOUT_EXIT)
  check("merge", aa_event("merge", "left") == MERGE)
  check("fork", aa_event("fork", "right") == FORK)
  check("ramps", (aa_event("on ramp", "right"), aa_event("off ramp", "right")) == (ON_RAMP, OFF_RAMP))
  check("end of road is a turn", aa_event("end of road", "left") == TURN)
  check("side none", turn_side("straight") == UNSPECIFIED)
  check("text truncation", len(_text_bytes("x" * 100)) == MAX_TEXT)
  check("utf8 boundary", _text_bytes("é" * 40).decode("utf-8") is not None)

  print("navcan self-test:", "ok" if ok else "FAILED")
  return 0 if ok else 1


if __name__ == "__main__":
  raise SystemExit(run_self_test())
