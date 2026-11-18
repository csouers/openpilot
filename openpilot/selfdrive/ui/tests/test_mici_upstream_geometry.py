"""The fork customizes lead cars, not upstream's road geometry pipeline."""
import inspect

import pytest

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
