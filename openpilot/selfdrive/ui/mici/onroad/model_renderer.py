import colorsys
import numpy as np
import pyray as rl
from openpilot.cereal import log, messaging
from opendbc.car.structs import car
from dataclasses import dataclass, field
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0
LEAD_BAR_LENGTH = 24.0  # px
LEAD_BAR_WIDTH = 5.4  # m (3x the original 1.8, spans the lane behind the target)
LEAD_CAR_WIDTH = 2.8  # m, width of the on-road lead car icon (incl. mirrors)
ROAD_EDGE_WIDTH = 0.08  # narrower than the 0.16 lane-line width factor
ROAD_EDGE_COLOR = rl.Color(255, 0, 21, 255)  # same red as critical alerts

# neo-era steering-angle path (selfdrive/ui/ui.c @ v0.3.5 draw_steering): curvature from the
# measured steering angle, drawn as a translucent blue line; matches the lane curve when tracking well
STEER_PATH_POINTS = 50
STEER_PATH_HALF_WIDTH = 0.15  # m, half-width of the drawn line
STEER_PATH_RATIO = 15.3  # steering ratio fallback when CP isn't up yet
STEER_PATH_WHEELBASE = 2.67  # m, fallback
STEER_PATH_SLIP = 0.0014
STEER_PATH_COLOR = rl.Color(0, 0, 255, 255)  # solid blue (was the neo-era 50% alpha, too faint on mici)

STOCK_PATH_GRADIENT = Gradient(
  start=(0.0, 1.0),
  end=(0.0, 0.0),
  colors=[rl.Color(13, 248, 122, 102), rl.Color(114, 255, 92, 89), rl.Color(114, 255, 92, 0)],
  stops=[0.0, 0.5, 1.0],
)

LANE_LINE_COLORS = {
  UIStatus.DISENGAGED: rl.Color(200, 200, 200, 255),
  UIStatus.OVERRIDE: rl.Color(255, 255, 255, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 64, 255),
}


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


class LeadVehicle:
  def __init__(self):
    self.bar = np.empty((0, 2), dtype=np.float32)
    self.d_filter = FirstOrderFilter(0.0, 0.2, 1 / gui_app.target_fps, initialized=False)
    self.y_filter = FirstOrderFilter(0.0, 0.2, 1 / gui_app.target_fps, initialized=False)
    self.fade_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self.radar = False  # radar-confirmed lead (vs vision-only)
    self.icon_anchor = np.zeros(2, dtype=np.float32)  # screen px, bottom-center of the car icon
    self.icon_width = 0.0  # screen px


class ModelRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._longitudinal_control = False
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    # on-road lead car icons: solid = radar-confirmed, dashed ghost = vision-only (hondacan.py HUD_LEAD)
    # loaded near the max draw size (~the close-lead cap), then mipmap-filtered so minification
    # to far-lead sizes stays clean instead of shimmering like plain bilinear
    self._txt_lead_car = gui_app.texture('icons_mici/lead_car/lead_car.png', 130, 78, keep_aspect_ratio=True)
    self._txt_lead_car_ghost = gui_app.texture('icons_mici/lead_car/lead_car_ghost.png', 130, 78, keep_aspect_ratio=True)
    for tex in (self._txt_lead_car, self._txt_lead_car_ghost):
      rl.gen_texture_mipmaps(tex)
      rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
    self._path_offset_z = HEIGHT_INIT[0]

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    # 3x3 car space -> rect-origin space (draw methods add rect.x/y)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._path_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)

    # Check if data is up-to-date
    if (sm.recv_frame["extrinsicsCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    extrinsics_calibration = sm['extrinsicsCalibration']
    self._path_offset_z = extrinsics_calibration.height[0] if extrinsics_calibration.height else HEIGHT_INIT[0]

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      self._transform_dirty = False

    # Draw elements
    self._draw_lane_lines()
    self._draw_path()
    self._draw_steer_path()

    if render_lead_indicator:
      self._update_leads(sm)
      self._draw_lead_indicator()
    else:
      self._lead_vehicles = [LeadVehicle(), LeadVehicle()]

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, sm):
    plan = sm['longitudinalPlan']
    if plan.longitudinalPlanSource == log.LongitudinalPlan.LongitudinalPlanSource.e2e and len(sm['modelV2'].leadsV3) > 1:
      # e2e leads come from the vision model: never radar-confirmed
      leads = [(lead.prob > 0.5, False, lead.x[0], -lead.y[0]) for lead in list(sm['modelV2'].leadsV3)[:2]]
    else:
      radar = sm['radarState']
      leads = [(lead.present, lead.radar, lead.dRel + RADAR_TO_CAMERA, lead.yRel) for lead in (radar.leadOne, radar.leadTwo)]

    # both leads can be the same vehicle
    if leads[0][0] and abs(leads[1][2] - leads[0][2]) < 3.0:
      leads[1] = (False, False, 0.0, 0.0)

    # ss, cs = sm['selfdriveState'], sm['carState']
    # braking disengages without making openpilot unavailable
    # available = ss.enabled or ss.engageable or cs.brakePressed
    available = True
    lane = self._path.raw_points
    # opacity = 0.4 if ui_state.status == UIStatus.DISENGAGED else 0.8
    for lead, (present, radar_confirmed, d_rel, y_rel) in zip(self._lead_vehicles, leads, strict=True):
      visible = available and present and d_rel < MAX_DRAW_DISTANCE and len(lane) > 0
      # snap to a new vehicle instead of sliding over
      if not visible or abs(y_rel - lead.y_filter.x) > 3.0:
        lead.d_filter.initialized = lead.y_filter.initialized = False
      # No translucency or fade: detected leads are always solid.
      lead.fade_filter.x = 1.0 if visible else 0.0
      lead.radar = radar_confirmed
      if visible:
        lead.bar = self._get_lead_bar(lane, lead.d_filter.update(d_rel), lead.y_filter.update(y_rel))
        lead.icon_anchor, lead.icon_width = self._get_lead_icon_geometry(lead.bar)
      else:
        lead.icon_width = 0.0

  def _get_lead_bar(self, lane, d_rel, y_rel):
    # bar on the road behind the lead, following the lane
    x = np.array([d_rel, d_rel - min(6.0, 0.25 * d_rel)])
    y = np.interp(x, lane[:, 0], lane[:, 1]) - np.interp(d_rel, lane[:, 0], lane[:, 1]) - y_rel
    z = np.interp(x, self._path.raw_points[:, 0], self._path.raw_points[:, 2]) + self._path_offset_z
    corners = np.vstack((np.column_stack((x, y + LEAD_BAR_WIDTH / 2, z)), np.column_stack((x, y - LEAD_BAR_WIDTH / 2, z))[::-1]))
    pts = self._car_space_transform @ corners.T
    bar = (pts[:2] / pts[2]).T

    far, near = bar[[0, 3]], bar[[1, 2]]
    length = np.linalg.norm(near.mean(axis=0) - far.mean(axis=0))
    bar[[1, 2]] = far + (near - far) * np.clip(length, 3.0, LEAD_BAR_LENGTH) / length
    return bar.astype(np.float32)

  def _get_lead_icon_geometry(self, bar):
    """Car icon anchored at the bar's far edge (the lead position on the road plane):
    bottom-center on the road, width from the perspective scale there."""
    far_center = (bar[0] + bar[3]) / 2.0
    px_per_m = np.linalg.norm(bar[0] - bar[3]) / LEAD_BAR_WIDTH
    # cap the perspective blow-up when a lead is right on top of us
    width = float(np.clip(LEAD_CAR_WIDTH * px_per_m, 0.0, self._rect.height * 0.55))
    return far_center.astype(np.float32), width

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    line_width_factor = 0.12
    for i, lane_line in enumerate(self._lane_lines):
      if i in (1, 2):
        line_width_factor = 0.16
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, line_width_factor * self._lane_line_probs[i], 0.0, max_idx
      )

    # Update road edges using raw points (narrower than lane lines)
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, ROAD_EDGE_WIDTH, 0.0, max_idx)

    # Update path using raw points
    if lead and lead.present:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, 0.9, self._path_offset_z, max_idx, allow_invert=False
    )

    self._update_path_gradient()

  def _update_path_gradient(self):
    if not self._longitudinal_control:
      return

    path_pts = self._path.projected_points + np.array([self._rect.x, self._rect.y], dtype=np.float32)
    max_len = min(len(path_pts) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = path_pts[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._path_gradient.colors = segment_colors
    self._path_gradient.stops = gradient_stops

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    alpha = np.clip(prob, 0.0, 0.7)
    if adjacent:
      _base_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED])
      color = rl.Color(_base_color.r, _base_color.g, _base_color.b, int(alpha * 255))

      # turn adjacent lls orange if torque is high
      torque = self._torque_filter.x
      high_torque = abs(torque) > 0.6
      if high_torque and (left == (torque > 0)):
        color = blend_colors(
          color,
          rl.Color(255, 115, 0, int(alpha * 255)),  # orange
          np.interp(abs(torque), [0.6, 0.8], [0.0, 1.0])
        )
    else:
      color = rl.Color(255, 255, 255, int(alpha * 255))

    # if ui_state.status == UIStatus.DISENGAGED:
    #   color = rl.Color(0, 0, 0, int(alpha * 255))

    return color

  def _draw_lane_lines(self):
    """Draw lane lines and road edges. Two closest lines should be green (lane line or road edges)."""
    offset = np.array([self._rect.x, self._rect.y], dtype=np.float32)

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
      draw_polygon(self._rect, lane_line.projected_points + offset, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      draw_polygon(self._rect, road_edge.projected_points + offset, ROAD_EDGE_COLOR)

  def _draw_path(self):
    if not self._path.projected_points.size:
      return

    path_pts = self._path.projected_points + np.array([self._rect.x, self._rect.y], dtype=np.float32)

    # if ui_state.status == UIStatus.DISENGAGED:
    #   draw_polygon(self._rect, path_pts, rl.Color(0, 0, 0, 90))
    if not self._longitudinal_control:
      draw_polygon(self._rect, path_pts, gradient=STOCK_PATH_GRADIENT)
    elif len(self._path_gradient.colors) > 1:
      draw_polygon(self._rect, path_pts, gradient=self._path_gradient)
    else:
      draw_polygon(self._rect, path_pts, rl.Color(255, 255, 255, 30))

  def _draw_steer_path(self):
    """Neo-era steering-angle path: where the car goes if the wheel stays where it is."""
    cs = ui_state.sm['carState']
    cp = ui_state.CP
    steer_ratio = cp.steerRatio if cp else STEER_PATH_RATIO
    wheel_base = cp.wheelbase if cp else STEER_PATH_WHEELBASE

    angle_rad = cs.steeringAngleDeg * np.pi / 180.0
    # carState angle is left-positive, but this car space is right-positive (see the lead bar's -y_rel)
    curvature = -angle_rad / (steer_ratio * wheel_base * (1.0 + STEER_PATH_SLIP * cs.vEgo ** 2))

    x = np.arange(STEER_PATH_POINTS, dtype=np.float32)
    y = x * np.tan(np.arcsin(np.clip(x * curvature, -0.999, 0.999)) / 2.0)
    # sit on the model's road plane: z is effectively depth in this projection, z=0 lands on the horizon
    path = self._path.raw_points
    if path.shape[0] < 2:
      return
    z = np.interp(x, path[:, 0], path[:, 2]) + self._path_offset_z
    line = np.column_stack((x, y, z))

    pts = self._map_line_to_polygon(line, STEER_PATH_HALF_WIDTH, 0.0, STEER_PATH_POINTS - 1)
    if pts.size == 0:
      return
    offset = np.array([self._rect.x, self._rect.y], dtype=np.float32)
    draw_polygon(self._rect, pts + offset, STEER_PATH_COLOR)

  def _draw_lead_indicator(self):
    # green to match the other engaged-green elements (lane lines, path)
    offset = np.array([self._rect.x, self._rect.y], dtype=np.float32)
    for lead in self._lead_vehicles:
      # bar geometry is kept for icon anchoring, but no longer drawn
      if lead.fade_filter.x <= 0.0 or lead.icon_width <= 0.0:
        continue
      # solid car when the radar confirms the lead, dashed ghost for vision-only
      # (hondacan.py HUD_LEAD: 2=solid radar-fixed, 1=dashed ghost)
      texture = self._txt_lead_car if lead.radar else self._txt_lead_car_ghost
      w = lead.icon_width
      h = w * texture.height / texture.width
      pos = lead.icon_anchor + offset
      dst = rl.Rectangle(pos[0] - w / 2.0, pos[1] - h, w, h)
      src = rl.Rectangle(0, 0, texture.width, texture.height)
      rl.draw_texture_pro(texture, src, dst, rl.Vector2(0, 0), 0.0,
                          rl.Color(255, 255, 255, round(255 * lead.fade_filter.x)))

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_height: float) -> int:
    """Get the index corresponding to the given path height"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_height)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]
    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )
