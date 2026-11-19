from functools import cache
import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.ui.onroad.cameraview import CameraView as BaseCameraView, color_curve


@cache
def frame_color(enhance_driver: bool) -> tuple[np.ndarray, float]:
  """20% more contrast and the display's gamma. a driver's face brightened.

  The feed is never desaturated, engaged or not: the camera shows the road as it is.
  """
  def curve(c):
    c = np.clip((c - 0.5) * 1.2 + 0.5, 0, 1)
    c = c ** (1 / 1.28)
    if enhance_driver:
      c = np.clip((c + 0.15 - 0.5) * 0.88 + 0.5, 0, 1)
      c = (c * c * (3 - 2 * c)) ** 0.8
    return c

  return color_curve(curve), 1.0  # full saturation, always


class CameraView(BaseCameraView):
  def __init__(self, name: str, stream_type: VisionStreamType):
    super().__init__(name, stream_type)
    self._enhance_driver = stream_type == VisionStreamType.VISION_STREAM_CABIN

  def _frame_color(self) -> tuple[np.ndarray, float]:
    return frame_color(self._enhance_driver)
