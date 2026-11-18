import numpy as np
import pyray as rl
import pytest
from types import SimpleNamespace as NS

from cereal import log
from openpilot.selfdrive.ui.mici.onroad import model_renderer as mr


class SM(dict):
  """SubMaster-ish: dict with attribute access, like the renderers use it."""
  def __getattr__(self, k):
    try:
      return self[k]
    except KeyError:
      raise AttributeError(k) from None


def _lead(present, radar, d_rel, y_rel):
  return NS(present=present, radar=radar, dRel=d_rel, yRel=y_rel)


def _model(prob, x, y):
  return NS(prob=prob, x=[x], y=[y])


def _sm(long_source, leads_v3, lead_one, lead_two, v_ego=10.0):
  return SM({
    "longitudinalPlan": NS(longitudinalPlanSource=long_source),
    "modelV2": NS(leadsV3=leads_v3,
                  velocity=NS(x=[0.0, 0.0, 5.0, 4.0]),
                  position=NS(x=[0.0, 1.0, 2.0, 3.0])),
    "radarState": NS(leadOne=lead_one, leadTwo=lead_two),
    "carState": NS(vEgo=v_ego),
    "updated": {"modelV2": True},
  })


@pytest.fixture
def renderer():
  # bypass __init__: it loads raylib textures, which needs a window
  r = object.__new__(mr.ModelRenderer)
  r._lead_vehicles = [mr.LeadVehicle(), mr.LeadVehicle()]
  r._stop_point = mr.StopPoint()
  r._rect = rl.Rectangle(0, 0, 1000, 500)
  r._path = NS(raw_points=np.array([[1.0, 0.0, 0.0], [10.0, 0.1, 0.0],
                                    [50.0, 0.5, 0.0], [100.0, 1.0, 0.0]], dtype=np.float32))
  r._path_offset_z = 0.0
  # road-plane projection like BaseModelRenderer's: u = y/depth, v = z/depth, w = depth
  r._car_space_transform = np.array([[0.0, 60.0, 0.0],
                                     [0.0, 0.0, 60.0],
                                     [1.0, 0.0, 0.0]], dtype=np.float32)
  r._interp_extrap = mr.ModelRenderer._interp_extrap
  r._get_lead_bar = mr.ModelRenderer._get_lead_bar.__get__(r)
  r._get_lead_icon_geometry = mr.ModelRenderer._get_lead_icon_geometry.__get__(r)
  r._update_lead_bars = mr.ModelRenderer._update_lead_bars.__get__(r)
  return r


NO_LEAD = _lead(False, False, 0.0, 0.0)
e2e = log.LongitudinalPlan.LongitudinalPlanSource.e2e

CASES = {
  "radar_single": _sm(0, [_model(0.1, 0.0, 0.0)], _lead(True, True, 20.0, 0.0), NO_LEAD),
  "radar_same_vehicle": _sm(0, [_model(0.1, 0.0, 0.0)], _lead(True, True, 20.0, 0.0), _lead(True, True, 21.0, 0.2)),
  "radar_two": _sm(0, [_model(0.1, 0.0, 0.0)], _lead(True, True, 20.0, 0.0), _lead(True, False, 45.0, 1.0)),
  "e2e_leads": _sm(e2e, [_model(0.9, 25.0, 0.5), _model(0.8, 40.0, -0.5)], NO_LEAD, NO_LEAD),
  "no_leads": _sm(0, [], NO_LEAD, NO_LEAD),
  "stopped": _sm(0, [], _lead(True, True, 5.0, 0.0), NO_LEAD, v_ego=0.0),
}


@pytest.mark.parametrize("case", CASES)
def test_update_lead_bars(renderer, case):
  """Lead tuples carry four fields in this fork (present, radar, d_rel, y_rel); the upstream
  planned-stop tail unpacked three, which crashed the mici onroad view on the first frame."""
  sm = CASES[case]
  renderer._update_lead_bars(sm)
  for lead in renderer._lead_vehicles:
    # the vehicle filter always settles on a plausible distance, drawn or not
    assert 0.0 <= lead.d_filter.x < 200.0


def test_same_vehicle_only_drawn_once(renderer):
  renderer._update_lead_bars(CASES["radar_same_vehicle"])
  assert renderer._lead_vehicles[0].icon_width != 0.0
  assert renderer._lead_vehicles[1].icon_width == 0.0
