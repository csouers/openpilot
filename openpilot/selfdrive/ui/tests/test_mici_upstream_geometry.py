"""The fork customizes lead cars, not upstream's road geometry pipeline."""
import inspect
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.mici.onroad import model_renderer

from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer as BaseModelRenderer


@pytest.mark.parametrize("method", [
  "_update_raw_points", "_update_model", "_update_path_gradient",
  "_get_path_length_idx", "_map_line_to_polygon", "_hsla_to_color",
])
def test_road_geometry_uses_upstream_base(method):
  assert getattr(ModelRenderer, method) is getattr(BaseModelRenderer, method)


def test_no_fork_steering_path_overlay():
  assert not hasattr(ModelRenderer, "_draw_steer_path")
  assert "_draw_steer_path" not in inspect.getsource(ModelRenderer._render)


def test_upstream_road_widths():
  assert ModelRenderer.LANE_LINE_WIDTHS == (0.12, 0.16, 0.16, 0.16)
  assert ModelRenderer.ROAD_EDGE_WIDTH == 0.16


@pytest.mark.parametrize("lane_probability", [0.0, 1.0])
def test_road_edges_stay_red(monkeypatch, lane_probability):
  renderer = object.__new__(ModelRenderer)
  renderer._rect = SimpleNamespace(x=0, y=0)
  renderer._lane_lines = []
  renderer._lane_line_probs = np.full(4, lane_probability)
  renderer._road_edges = [SimpleNamespace(projected_points=np.ones((4, 2)))]
  renderer._road_edge_stds = [0.1]
  renderer._visible = 1.0
  colors = []
  monkeypatch.setattr(model_renderer, "draw_polygon", lambda rect, points, color: colors.append(color))
  renderer._draw_lane_lines()
  assert len(colors) == 1
  assert (colors[0].r, colors[0].g, colors[0].b) == (255, 0, 21)
  assert colors[0].a > 0
