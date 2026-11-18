#!/usr/bin/env python3
"""
Fake message sender for the real UI (`openpilot/selfdrive/ui/ui.py`).

Publishes plausible modelV2 / carState / controlsState / radarState / ... onto the msgq bus so the
normal mici UI draws a complete onroad screen without a car, a route or a camera: both turn signals,
set speed and speed, lane lines, road edges, path, steer path, one lead car (solid / dashed ghost /
upstream's planned-stop bar), the compass heading and the experimental-mode icon.

Each service is published at the rate cereal declares for it. That matters: messaging.SubMaster
marks a service invalid when it arrives faster or slower than its declared frequency, and the UI
gates the compass (gpsLocationExternal) and the experimental icon (selfdriveState) on `valid`.

Usage (two terminals, from the repo root):
  .venv/bin/python openpilot/tools/mici_test_sender.py            # sender
  .venv/bin/python openpilot/selfdrive/ui/ui.py                   # the real UI

Options:
  --lead-style auto|solid|ghost|toggle|stop   lead look (toggle = solid/ghost/stop, 1 s each)
  --no-planned-stop                            keep the model from planning a stop
  --hz 100                                     loop rate (services keep their own rates)

No camera: camerad is not running, so the camera area stays black. That is intentional.
"""

import argparse
import math
import sys
import time

from openpilot.cereal import messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.cereal.services import _services as SERVICES
from opendbc.car.structs import car

N = 24
XS = [2.0 + (120.0 - 2.0) * i / (N - 1) for i in range(N)]
LANE_OFFSETS = (-1.8, -1.75, 1.75, 1.8)
EDGE_OFFSETS = (-2.2, 2.2)

# declared frequencies (cereal/services.py); SubMaster rejects anything outside these
RATES = {
  "carParams": 2.0,
  "modelV2": 20.0,
  "carState": 100.0,
  "controlsState": 100.0,
  "selfdriveState": 100.0,
  "deviceState": 2.0,
  "pandaStates": 2.0,
  "extrinsicsCalibration": 10.0,
  "radarState": 20.0,
  "longitudinalPlan": 20.0,
  "carOutput": 100.0,
  "carControl": 100.0,
  "vehicleParameters": 10.0,
  "gpsLocationExternal": 10.0,
  "onroadEvents": 2.0,
  "managerState": 2.0,
  "narrowRoadCameraState": 20.0,
  "wideRoadCameraState": 20.0,
}
for _svc in RATES:
  RATES[_svc] = SERVICES[_svc][1] or RATES[_svc]
# carParams declares 0.02 Hz, but SubMaster floors slow services at 1 Hz for validity
RATES["carParams"] = 1.0


class Sender:
  def __init__(self, lead_style: str, planned_stop: bool):
    self.lead_style = lead_style
    self.planned_stop = planned_stop
    self.t0 = time.monotonic()
    self.pm = messaging.PubMaster(list(RATES))

    self.cp_msg = messaging.new_message("carParams")
    cp = self.cp_msg.carParams
    cp.carFingerprint = "HONDA_CIVIC"
    cp.openpilotLongitudinalControl = True
    cp.notCar = False
    cp.steerRatio = 15.0
    cp.wheelbase = 2.7
    cp.maxLateralAccel = 3.0

    # Params wants a bare CarParams struct; the bus wants the log Event carrying one
    root_cp = car.CarParams()
    root_cp.carFingerprint = cp.carFingerprint
    root_cp.openpilotLongitudinalControl = True
    root_cp.notCar = False
    root_cp.steerRatio = cp.steerRatio
    root_cp.wheelbase = cp.wheelbase
    root_cp.maxLateralAccel = cp.maxLateralAccel
    params = Params()
    params.put("CarParamsPersistent", root_cp.to_bytes(), block=True)
    params.put_bool("IsMetric", False, block=True)
    params.put_bool("RecordAudio", True, block=True)

    self._last = dict.fromkeys(RATES, -1e9)

  # --- message builders -----------------------------------------------------

  @property
  def t(self) -> float:
    return time.monotonic() - self.t0

  def _lead_solid(self, t: float) -> bool:
    if self.lead_style == "toggle":
      return int(t) % 3 == 0
    if self.lead_style == "solid":
      return True
    if self.lead_style in ("ghost", "stop"):
      return False
    return True

  def build_model(self):
    t = self.t
    msg = messaging.new_message("modelV2")
    m = msg.modelV2
    m.frameId = int(t * 20)
    m.init("laneLines", 4)
    m.init("roadEdges", 2)

    weave = 0.15 * math.sin(t * 0.4)
    for i, off in enumerate(LANE_OFFSETS):
      bend = 0.15 * (1 if off < 0 else -1)  # gentle curve, same for all lines
      m.laneLines[i].x = XS
      m.laneLines[i].y = [off + bend * (x / 120.0) ** 2 + weave for x in XS]
      m.laneLines[i].z = [0.0] * N
    m.laneLineProbs = [0.95, 0.9, 0.9, 0.95]
    m.laneLineStds = [0.1, 0.1, 0.1, 0.1]
    for i, off in enumerate(EDGE_OFFSETS):
      m.roadEdges[i].x = XS
      m.roadEdges[i].y = [off for _ in XS]
      m.roadEdges[i].z = [0.0] * N
    m.roadEdgeStds = [0.1, 0.1]

    m.position.x = XS
    m.position.y = [weave for _ in XS]
    m.position.z = [0.0] * N

    speed = 15.0 - 2.0 * math.sin(t * 0.3)
    if self.planned_stop:
      m.velocity.x = [max(0.0, min(speed, (28.0 - x) * 0.8)) for x in XS]
    else:
      m.velocity.x = [speed] * N
    m.velocity.y = [0.0] * N
    m.velocity.z = [0.0] * N

    # keep acceleration ~0: any real swing colours the whole path polygon (the long-control
    # acceleration gradient) and dominates the screen
    m.acceleration.x = [0.005 * math.sin(t * 1.3)] * N
    m.acceleration.y = [0.0] * N
    m.acceleration.z = [0.0] * N

    m.meta.laneChangeState = log.LaneChangeState.off
    m.meta.disengagePredictions.brakeDisengageProbs = [0.02]
    m.meta.disengagePredictions.steerOverrideProbs = [0.01]
    return msg

  def build(self, svc: str):
    t = self.t
    if svc == "carParams":
      return self.cp_msg

    if svc == "modelV2":
      return self.build_model()

    if svc == "carState":
      msg = messaging.new_message("carState")
      msg.carState.vEgo = 15.0 - 2.0 * math.sin(t * 0.3)
      msg.carState.vEgoCluster = 0.0
      msg.carState.vCruiseCluster = 0.0
      msg.carState.steeringAngleDeg = 6.0 * math.sin(t * 0.25)
      msg.carState.leftBlinker = True
      msg.carState.rightBlinker = True
      msg.carState.standstill = False
      return msg

    if svc == "controlsState":
      msg = messaging.new_message("controlsState")
      msg.controlsState.deprecated.vCruise = 31.0
      msg.controlsState.curvature = 0.01 * math.sin(t * 0.4)
      msg.controlsState.desiredCurvature = 0.02 * math.sin(t * 0.4)
      lat = msg.controlsState.lateralControlState.init("curvatureState")
      lat.active = True
      lat.actualCurvature = 0.01 * math.sin(t * 0.4)
      lat.desiredCurvature = 0.02 * math.sin(t * 0.4)
      return msg

    if svc == "selfdriveState":
      msg = messaging.new_message("selfdriveState")
      msg.selfdriveState.enabled = True
      msg.selfdriveState.active = True
      msg.selfdriveState.experimentalMode = True  # drives the e2e icon in the status rail
      msg.selfdriveState.state = log.SelfdriveState.OpenpilotState.enabled
      msg.selfdriveState.engageable = True
      msg.selfdriveState.alertSize = log.SelfdriveState.AlertSize.none
      msg.selfdriveState.alertStatus = log.SelfdriveState.AlertStatus.normal
      msg.selfdriveState.alertType = ""
      return msg

    if svc == "deviceState":
      msg = messaging.new_message("deviceState")
      msg.deviceState.started = True
      msg.deviceState.deviceType = log.InitData.DeviceType.mici
      msg.deviceState.chestnutPresent = True
      msg.deviceState.networkType = log.DeviceState.NetworkType.wifi
      msg.deviceState.networkStrength = log.DeviceState.NetworkStrength.great
      return msg

    if svc == "pandaStates":
      msg = messaging.new_message("pandaStates", 1)
      msg.pandaStates[0].ignitionLine = True
      msg.pandaStates[0].pandaType = log.PandaState.PandaType.dos
      return msg

    if svc == "extrinsicsCalibration":
      msg = messaging.new_message("extrinsicsCalibration")
      msg.extrinsicsCalibration.calStatus = log.ExtrinsicsCalibration.Status.calibrated
      msg.extrinsicsCalibration.rpyCalib = [0.0, 0.0, 0.0]
      msg.extrinsicsCalibration.wideFromDeviceEuler = [0.0, 0.0, 0.0]
      msg.extrinsicsCalibration.height = [1.22]
      return msg

    if svc == "radarState":
      msg = messaging.new_message("radarState")
      show = self.lead_style != "stop" and not (self.lead_style == "toggle" and int(t) % 3 == 2)
      msg.radarState.leadOne.present = show
      msg.radarState.leadOne.radar = self._lead_solid(t)
      msg.radarState.leadOne.dRel = 22.0 + 2.0 * math.sin(t * 0.5)
      msg.radarState.leadOne.yRel = 0.05
      msg.radarState.leadOne.vRel = -1.0
      msg.radarState.leadTwo.present = False
      return msg

    if svc == "longitudinalPlan":
      msg = messaging.new_message("longitudinalPlan")
      msg.longitudinalPlan.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.cruise
      return msg

    if svc == "carOutput":
      msg = messaging.new_message("carOutput")
      msg.carOutput.actuatorsOutput.torque = 0.2 * math.sin(t * 1.3)
      return msg

    if svc == "carControl":
      msg = messaging.new_message("carControl")
      msg.carControl.latActive = True
      return msg

    if svc == "vehicleParameters":
      msg = messaging.new_message("vehicleParameters")
      msg.vehicleParameters.roll = 0.01
      return msg

    if svc == "gpsLocationExternal":
      msg = messaging.new_message("gpsLocationExternal")
      msg.gpsLocationExternal.hasFix = True
      msg.gpsLocationExternal.speed = 15.0
      msg.gpsLocationExternal.bearingDeg = (t * 12.0) % 360.0  # compass drifts N -> E -> S -> W
      return msg

    if svc == "onroadEvents":
      return messaging.new_message("onroadEvents", 0)

    if svc == "managerState":
      msg = messaging.new_message("managerState")
      msg.managerState.processes = []
      return msg

    if svc in ("narrowRoadCameraState", "wideRoadCameraState"):
      msg = messaging.new_message(svc)
      cam = getattr(msg, svc)
      cam.frameId = int(t * 20)
      cam.exposureValPercent = 50.0
      cam.sensor = "ox03c10"  # UI looks up DEVICE_CAMERAS[(deviceType, sensor)]
      return msg

    raise ValueError(svc)

  def run(self, loop_hz: int):
    rk = Ratekeeper(loop_hz)
    print(f"sender up: loop {loop_hz} Hz, lead-style={self.lead_style}, planned-stop={self.planned_stop}", flush=True)
    while True:
      t = self.t
      for svc, rate in RATES.items():
        if t - self._last[svc] >= 1.0 / rate:
          self._last[svc] = t
          msg = self.build(svc)
          # SubMaster copies this header flag into sm.valid[service]; the UI gates the calibration
          # transform (lane lines, road edges, path), the compass and the e2e icon on it
          msg.valid = True
          self.pm.send(svc, msg)
      rk.keep_time()


def main():
  ap = argparse.ArgumentParser(description="fake messages for the real mici UI")
  ap.add_argument("--lead-style", default="toggle", choices=["auto", "solid", "ghost", "toggle", "stop"])
  ap.add_argument("--planned-stop", dest="planned_stop", action="store_true", default=True)
  ap.add_argument("--no-planned-stop", dest="planned_stop", action="store_false")
  ap.add_argument("--hz", type=int, default=100, help="loop rate (default 100; services keep their own)")
  args = ap.parse_args()
  Sender(args.lead_style, args.planned_stop).run(args.hz)


if __name__ == "__main__":
  sys.exit(main())
