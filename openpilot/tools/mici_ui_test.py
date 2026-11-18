#!/usr/bin/env python3
"""
Local mici onroad UI test mode.

Renders the real mici onroad widgets against either a synthetic scene or a recorded
route, with every indicator forced on at once, so one screen shows: both turn signal
icons, the full status icon column (settings/network/usb/chestnut/body/mic), set speed
and speed, lane lines, path, steer path, up to two lead vehicles, the planned stop
point and the confidence ball.

Usage (from the repo root):
  .venv/bin/python openpilot/tools/mici_ui_test.py                       # synthetic, animated
  .venv/bin/python openpilot/tools/mici_ui_test.py --route <rlog|qlog>   # replay a real route
  .venv/bin/python openpilot/tools/mici_ui_test.py --seconds 10

Notes
- Import raylib_nb (via gui_app), never the old `pyray` package: both ship GLFW and loading
  them together segfaults.
- Nothing here touches the car: ui_state is the local singleton, stubbed per frame.
"""

import argparse
import os
import sys
import time
from types import SimpleNamespace as NS

import numpy as np

# capture before anything (imports included) can chdir: gui_app ends up running from .venv/bin
START_CWD = os.getcwd()

from openpilot.common.basedir import BASEDIR

# cereal/opendbc live under openpilot/, the way the device and tools expect them
sys.path.insert(0, os.path.join(BASEDIR, "openpilot"))

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

import raylib_nb as rl

from cereal import log
from openpilot.selfdrive.ui.ui_state import ChestnutState, ui_state
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.confidence_ball import ConfidenceBall
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.mici.onroad.status_icons import StatusIconColumn
from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
from openpilot.selfdrive.ui.mici.onroad.turn_signals import TurnSignals

REPLAY_SERVICES = ("modelV2", "radarState", "carState", "controlsState", "selfdriveState",
                   "carParams", "extrinsicsCalibration", "longitudinalPlan", "deviceState",
                   "carBodyState", "carOutput", "liveTorqueParameters")
REQUIRED = {"modelV2", "radarState", "carState", "carParams", "extrinsicsCalibration"}


class FakeSubMaster(dict):
  """The bits of messaging.SubMaster the widgets use."""

  def __init__(self, msgs, frame=10000):
    super().__init__(msgs)
    class _Flags(dict):
      # widgets probe optional services (gpsLocationExternal, liveCalibration, ...)
      def __missing__(self, k):
        return False

    self.recv_frame = _Flags(dict.fromkeys(msgs, frame))
    self.valid = _Flags(dict.fromkeys(msgs, True))
    self.alive = _Flags(dict.fromkeys(msgs, True))
    self.updated = _Flags(dict.fromkeys(msgs, True))
    self.frame = frame

  def __getattr__(self, name):
    try:
      return self[name]
    except KeyError:
      raise AttributeError(name) from None


def _polyline(pts):
  return NS(x=[p[0] for p in pts], y=[p[1] for p in pts], z=[p[2] for p in pts])


def write_png(path: str, rgb: np.ndarray) -> None:
  """Minimal dependency-free PNG writer (the venv has no imaging library)."""
  import struct
  import zlib

  h, w, _ = rgb.shape
  raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

  def chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

  png = (b"\x89PNG\r\n\x1a\n"
         + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
         + chunk(b"IDAT", zlib.compress(raw, 6))
         + chunk(b"IEND", b""))
  with open(path, "wb") as f:
    f.write(png)


def open_window(width: int = 0, height: int = 0) -> None:
  """Open the window at the mici panel size (536x240, auto-scaled) unless overridden."""
  if width > 0 and height > 0:
    gui_app._width, gui_app._height = width, height
    gui_app._scaled_width = int(width * gui_app._scale)
    gui_app._scaled_height = int(height * gui_app._scale)
    gui_app._scaled_width += gui_app._scaled_width % 2
    gui_app._scaled_height += gui_app._scaled_height % 2
  gui_app.init_window("mici onroad test", fps=60)


def camera_transform(w: float, h: float) -> np.ndarray:
  """A plausible mici narrow-camera homography: car space (x forward, y left, z up) -> screen.

  Mirrors what AugmentedRoadView._calc_frame_matrix derives from intrinsics @ calibration: a
  pinhole (focal from the window width, principal point at centre) with a small downward pitch.
  The camera height is not in this 3x3 - the renderers pass z + _path_offset_z (extrinsics
  height) as the third coordinate, which is what puts the road below the horizon.
  """
  fx = fy = 1.15 * w
  cx, cy = w / 2.0, h / 2.0
  th = 0.06
  s, c = np.sin(th), np.cos(th)
  K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
  # camera-from-car rotation for a camera pitched down by th (y points down in image space)
  R = np.array([[0.0, 1.0, 0.0],
                [-s, 0.0, c],
                [c, 0.0, s]], dtype=np.float32)
  return K @ R


class OnroadTestView(Widget):
  def __init__(self, route: str | None, frame_limit: int, shots_dir: str | None = None,
               shot_every: int = 30, with_home_bar: bool = False, chestnut: str = "active"):
    super().__init__()
    self._model = ModelRenderer()
    self._hud = HudRenderer()
    self._ball = ConfidenceBall()
    self._icons = StatusIconColumn()
    self._turn_signals = TurnSignals()
    # --with-home-bar only: the usb/chestnut/body/mic icons live in the home screen's status bar,
    # they are not part of the driving view
    self._home = MiciHomeLayout() if with_home_bar else None

    self._t0 = time.monotonic()
    self._chestnut = ChestnutState[chestnut.upper()]
    self._shots_dir = shots_dir
    self._shot_every = max(1, shot_every)
    self._frames = 0
    self._replay = self._open_replay(route, frame_limit) if route else None
    self._last_sm = None

  @staticmethod
  def _open_replay(path, frame_limit):
    from openpilot.tools.lib.logreader import _LogFileReader, LogReader

    # a raw local file is not a route id: LogReader would try to resolve it as one
    reader = _LogFileReader(path) if os.path.isfile(path) else LogReader(path)

    latest, n = {}, 0
    for msg in reader:
      which = msg.which()
      if which in REPLAY_SERVICES:
        latest[which] = getattr(msg, which)
      if which == "modelV2":
        n += 1
        if (frame_limit and n > frame_limit) or n > 1 and REQUIRED <= latest.keys():
          yield dict(latest)
        if frame_limit and n > frame_limit:
          return

  def _synthetic(self, t: float):
    xs = np.linspace(2.0, 120.0, 24)
    def lane(off):
      return _polyline([(x, off + 0.002 * x * x, 0.0) for x in xs])
    bob = 0.15 * np.sin(t * 0.7)
    lead_d = 22.0 + 3.0 * np.sin(t * 0.5)

    model = NS(
      frameId=int(t * 20),
      position=_polyline([(x, 0.0, 0.0) for x in xs]),
      velocity=_polyline([(15.0, 0.0, 0.0) for _ in xs]),
      acceleration=_polyline([(0.1 * np.sin(t), 0.0, 0.0) for _ in xs]),
      orientationRate=_polyline([(0.0, 0.0, 0.0) for _ in xs]),
      laneLines=[lane(-1.8), lane(-1.75), lane(1.75), lane(1.8)],
      laneLineProbs=[0.95, 0.9, 0.9, 0.95],
      roadEdges=[lane(-2.2), lane(2.2)],
      roadEdgeStds=[0.1, 0.1],
      leadsV3=[],
      meta=NS(laneChangeState=0,
              disengagePredictions=NS(brakeDisengageProbs=[0.02], steerOverrideProbs=[0.01])),
      path=NS(polyline=_polyline([(x, bob * (x / 120.0), 0.0) for x in xs])),
    )
    return {
      "modelV2": model,
      "radarState": NS(leadOne=NS(present=True, radar=True, dRel=lead_d, yRel=0.05, vRel=-1.0),
                       leadTwo=NS(present=True, radar=False, dRel=lead_d + 26.0, yRel=-0.4, vRel=-0.5)),
      "carState": NS(vEgo=15.0 + 2.0 * np.sin(t * 0.3), vEgoCluster=0.0, vCruiseCluster=0.0,
                     steeringAngleDeg=8.0 * np.sin(t * 0.25), leftBlinker=True, rightBlinker=True,
                     standstill=False),
      "carBodyState": NS(blinkerLampValid=True, leftBlinkerLamp=True, rightBlinkerLamp=True),
      "controlsState": NS(deprecated=NS(vCruise=31.0),  # ~70 mph
                          lateralControlState=NS(which=lambda: "curvatureState"),
                          curvature=0.012 * np.sin(t * 0.4), desiredCurvature=0.02 * np.sin(t * 0.4)),
      "carControl": NS(latActive=True),
      "vehicleParameters": NS(roll=0.01),
      "selfdriveState": NS(enabled=True, experimentalMode=True),
      "carOutput": NS(actuatorsOutput=NS(torque=0.25 * np.sin(t * 1.3))),
      "carParams": NS(openpilotLongitudinalControl=True),
      "extrinsicsCalibration": NS(height=[1.22]),
      "longitudinalPlan": NS(longitudinalPlanSource=log.LongitudinalPlan.LongitudinalPlanSource.cruise),
      "deviceState": NS(chestnutPresent=True, networkType=0, networkStrength=NS(raw=4), networkMetered=False),
      "gpsLocationExternal": NS(hasFix=True, speed=15.0, bearingDeg=(t * 12.0) % 360.0),
      "liveTorqueParameters": NS(latAccelFactorFiltered=1.0, latAccelOffsetFiltered=0.0,
                                 frictionCoefficientFiltered=0.1),
    }

  @staticmethod
  def _force_all_indicators(msgs):
    cs = msgs["carState"]
    msgs["carState"] = NS(vEgo=cs.vEgo, vEgoCluster=0.0, vCruiseCluster=0.0,
                          steeringAngleDeg=getattr(cs, "steeringAngleDeg", 0.0),
                          leftBlinker=True, rightBlinker=True, standstill=getattr(cs, "standstill", False))
    msgs["carBodyState"] = NS(blinkerLampValid=True, leftBlinkerLamp=True, rightBlinkerLamp=True)
    msgs["selfdriveState"] = NS(enabled=True, experimentalMode=True)
    msgs["carParams"] = NS(openpilotLongitudinalControl=True)
    msgs["deviceState"] = NS(chestnutPresent=True, networkType=0, networkStrength=NS(raw=4), networkMetered=False)
    cs_lat = getattr(msgs.get("controlsState"), "lateralControlState", None)
    if cs_lat is None:
      msgs["controlsState"] = NS(deprecated=NS(vCruise=25.0),
                                 lateralControlState=NS(which=lambda: "curvatureState"),
                                 curvature=0.0, desiredCurvature=0.0)
    msgs.setdefault("carControl", NS(latActive=True))
    msgs.setdefault("vehicleParameters", NS(roll=0.0))
    msgs.setdefault("gpsLocationExternal", NS(hasFix=True, speed=15.0, bearingDeg=45.0))
    msgs.setdefault("carOutput", NS(actuatorsOutput=NS(torque=0.0)))
    msgs.setdefault("longitudinalPlan", NS(longitudinalPlanSource=log.LongitudinalPlan.LongitudinalPlanSource.cruise))

  def _update_state(self):
    t = time.monotonic() - self._t0
    if self._replay is not None:
      try:
        msgs = dict(next(self._replay))
      except StopIteration:
        gui_app.request_close()
        msgs = None
      if msgs is not None:
        self._force_all_indicators(msgs)
        self._last_sm = FakeSubMaster(msgs)
    else:
      self._last_sm = FakeSubMaster(self._synthetic(t))

    self._frames += 1
    sm = self._last_sm
    ui_state.sm = sm
    ui_state.started = True
    ui_state.status = 2  # ENGAGED
    ui_state.is_metric = False
    ui_state.always_on_dm = False
    ui_state.navigation_enabled = False
    ui_state.nav_destination_set = False
    ui_state.usb_connected = True
    ui_state.usb_unknown = False
    ui_state.chestnut_compiled = True
    ui_state.chestnut_present = True
    ui_state.chestnut_state = self._chestnut
    ui_state.is_body = True
    ui_state.recording_audio = True
    ui_state._started_frame = 0
    ui_state.CP = NS(maxLateralAccel=3.0, steerRatio=15.0, wheelbase=2.7)

    w, h = self.rect.width or gui_app.width, self.rect.height or gui_app.height
    self._model.set_transform(camera_transform(w, h))
    self._hud.set_can_draw_top_icons(True)
    self._icons.set_alert_present(False)

  def _render(self, rect):
    w, h = rect.width, rect.height
    self._model.set_rect(rl.Rectangle(0, 0, w, h))
    self._ball.set_rect(rl.Rectangle(0, 0, w, h))
    self._turn_signals.set_rect(rl.Rectangle(0, 0, w, h))
    self._icons.set_rect(rl.Rectangle(w - SIDE_PANEL_WIDTH, 0, SIDE_PANEL_WIDTH, h))

    self._model.render(rl.Rectangle(0, 0, w, h))
    self._hud.render(rl.Rectangle(0, 0, w, h))
    self._ball.render(rl.Rectangle(0, 0, w, h))
    self._icons.render(rl.Rectangle(w - SIDE_PANEL_WIDTH, 0, SIDE_PANEL_WIDTH, h))
    self._turn_signals.render(rl.Rectangle(0, 0, w, h))

    if self._home is not None:
      # opt-in: every home status icon on, in a strip along the bottom
      bar_h = 56
      for icon in self._home._status_bar_layout.widgets:
        icon.set_visible(True)
      self._home._status_bar_layout.render(rl.Rectangle(0, h - bar_h, w, bar_h))

    if self._shots_dir and self._frames % self._shot_every == 0:
      try:
        screen = rl.cpu.screen()
        rgb = np.ascontiguousarray(np.flipud(screen[:, :, :3]))
        os.makedirs(self._shots_dir, exist_ok=True)
        write_png(os.path.join(self._shots_dir, f"frame_{self._frames:05d}.png"), rgb)
      except Exception as e:  # capture is a convenience, never fail the run over it
        print(f"shot failed: {type(e).__name__}: {e}")


def main():
  ap = argparse.ArgumentParser(description="mici onroad UI test mode",
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--route", help="rlog/qlog to replay instead of the synthetic scene")
  ap.add_argument("--frames", type=int, default=0, help="stop after N model frames (replay only)")
  ap.add_argument("--seconds", type=float, default=0.0, help="stop after N seconds (0 = until closed)")
  ap.add_argument("--shots", help="write PNG frames into this directory")
  ap.add_argument("--shot-every", type=int, default=30, help="save every Nth frame (default 30)")
  ap.add_argument("--width", type=int, default=0, help="override window width (default: mici 536)")
  ap.add_argument("--height", type=int, default=0, help="override window height (default: mici 240)")
  ap.add_argument("--chestnut", default="active",
                  choices=[s.name.lower() for s in ChestnutState],
                  help="chestnut (GPU) icon state to show (default active = green)")
  ap.add_argument("--with-home-bar", action="store_true",
                  help="also draw the home status bar (usb/chestnut/body/mic); not part of the driving view")
  args = ap.parse_args()

  open_window(args.width, args.height)
  route = os.path.join(START_CWD, args.route) if args.route and not os.path.isabs(args.route) else args.route
  shots = os.path.join(START_CWD, args.shots) if args.shots and not os.path.isabs(args.shots) else args.shots
  view = OnroadTestView(route, args.frames, shots, args.shot_every, args.with_home_bar, args.chestnut)
  gui_app.push_widget(view)

  start = time.monotonic()
  for _ in gui_app.render():
    if args.seconds and time.monotonic() - start > args.seconds:
      break

  print(f"rendered {view._frames} frames")
  return 0


if __name__ == "__main__":
  sys.exit(main())
