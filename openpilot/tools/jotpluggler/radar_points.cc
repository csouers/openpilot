#include "tools/jotpluggler/app.h"

#include "tools/jotpluggler/common.h"
#include "tools/jotpluggler/dbc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// Top-down view of the Tesla Bosch radar point cloud, decoded straight out of the
// rlog CAN frames. Mirrors what the car's radar interface sees: geometry from
// RadarPoint{i}_A, class / moving state / size from RadarPoint{i}_B, with class
// and moving-state names taken from the DBC VAL_ tables.

namespace {

constexpr const char *RADAR_DBC = "tesla_radar_bosch_generated";
constexpr int RADAR_SLOTS = 32;
constexpr double MAX_PAIR_AGE = 0.5;  // s; A/B frames further apart than this are not paired
constexpr float MAX_FORWARD_M = 150.0f;  // fixed forward-range axis (y), so point motion is comparable across frames

constexpr ImU32 BACKGROUND = IM_COL32(16, 20, 24, 255);
constexpr ImU32 GRID_COLOR = IM_COL32(38, 44, 52, 255);
constexpr ImU32 GRID_LABEL_COLOR = IM_COL32(120, 128, 138, 255);
constexpr ImU32 EGO_COLOR = IM_COL32(63, 185, 80, 255);
constexpr ImU32 TEXT_COLOR = IM_COL32(201, 209, 217, 255);

ImU32 class_color(int cls) {
  switch (cls) {
    case 0: return IM_COL32(176, 182, 190, 220);  // unknown
    case 1: return IM_COL32(46, 196, 255, 255);   // four wheel vehicle
    case 2: return IM_COL32(255, 159, 28, 255);   // two wheel vehicle
    case 3: return IM_COL32(255, 77, 109, 255);   // pedestrian
    case 4: return IM_COL32(255, 209, 102, 255);  // construction element
    default: return IM_COL32(200, 120, 255, 255);
  }
}

enum class MarkerShape { Circle, Square, Triangle, Diamond, Cross };

MarkerShape class_shape(int cls) {
  switch (cls) {
    case 1: return MarkerShape::Triangle;
    case 2: return MarkerShape::Diamond;
    case 3: return MarkerShape::Circle;
    case 4: return MarkerShape::Square;
    default: return MarkerShape::Cross;
  }
}

std::string pretty_class_name(const std::string &name) {
  std::string out = name;
  for (const char *prefix : {"RADAR_CLASS_", "RADAR_"}) {
    if (out.rfind(prefix, 0) == 0) {
      out = out.substr(std::string(prefix).size());
      break;
    }
  }
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::replace(out.begin(), out.end(), '_', ' ');
  return out;
}

void draw_marker(ImDrawList *draw_list, int cls, ImVec2 center, float radius) {
  const ImU32 color = class_color(cls);
  switch (class_shape(cls)) {
    case MarkerShape::Circle:
      draw_list->AddCircleFilled(center, radius, color);
      break;
    case MarkerShape::Square:
      draw_list->AddRectFilled(ImVec2(center.x - radius, center.y - radius),
                               ImVec2(center.x + radius, center.y + radius), color);
      break;
    case MarkerShape::Triangle:
      draw_list->AddTriangleFilled(ImVec2(center.x, center.y - radius - 1.0f),
                                   ImVec2(center.x - radius, center.y + radius),
                                   ImVec2(center.x + radius, center.y + radius), color);
      break;
    case MarkerShape::Diamond:
      draw_list->AddQuadFilled(ImVec2(center.x, center.y - radius - 1.0f),
                               ImVec2(center.x + radius, center.y),
                               ImVec2(center.x, center.y + radius + 1.0f),
                               ImVec2(center.x - radius, center.y), color);
      break;
    case MarkerShape::Cross:
      draw_list->AddLine(ImVec2(center.x - radius, center.y - radius),
                         ImVec2(center.x + radius, center.y + radius), color, 1.6f);
      draw_list->AddLine(ImVec2(center.x - radius, center.y + radius),
                         ImVec2(center.x + radius, center.y - radius), color, 1.6f);
      break;
  }
}

struct SignalSlot {
  int long_dist = -1;
  int lat_dist = -1;
  int long_speed = -1;
  int prob_exist = -1;
  int valid = -1;
  int tracked = -1;
  int index = -1;
  int length = -1;
  int moving_state = -1;
  int prob_class = -1;
  int cls = -1;
  int index_b = -1;
  int measured = -1;
};

struct MessageRef {
  const dbc::Message *message = nullptr;
  SignalSlot slot;
  std::vector<double> latest;
  double latest_time = 0.0;
  bool has_values = false;
};

int find_signal(const dbc::Message &message, const char *name) {
  for (size_t i = 0; i < message.signals.size(); ++i) {
    if (message.signals[i].name == name) return static_cast<int>(i);
  }
  return -1;
}

MessageRef make_message_ref(const dbc::Database &database, uint32_t address) {
  MessageRef ref;
  ref.message = database.message(address);
  if (ref.message == nullptr) return ref;
  const dbc::Message &m = *ref.message;
  ref.slot.long_dist = find_signal(m, "LongDist");
  ref.slot.lat_dist = find_signal(m, "LatDist");
  ref.slot.long_speed = find_signal(m, "LongSpeed");
  ref.slot.prob_exist = find_signal(m, "ProbExist");
  ref.slot.valid = find_signal(m, "Valid");
  ref.slot.tracked = find_signal(m, "Tracked");
  ref.slot.index = find_signal(m, "Index");
  ref.slot.length = find_signal(m, "Length");
  ref.slot.moving_state = find_signal(m, "MovingState");
  ref.slot.prob_class = find_signal(m, "ProbClass");
  ref.slot.cls = find_signal(m, "Class");
  ref.slot.index_b = find_signal(m, "Index2");
  ref.slot.measured = find_signal(m, "Meas");
  ref.latest.assign(m.signals.size(), 0.0);
  return ref;
}

double value_at(const MessageRef &ref, int index, double fallback = 0.0) {
  if (index < 0 || static_cast<size_t>(index) >= ref.latest.size()) return fallback;
  return ref.latest[static_cast<size_t>(index)];
}

void decode_into(const dbc::Message &message, const CanFrameSample &sample, MessageRef *ref) {
  const auto *raw = reinterpret_cast<const uint8_t *>(sample.data.data());
  for (size_t i = 0; i < message.signals.size(); ++i) {
    const std::optional<double> value = dbc::signalValue(message.signals[i], message, raw, sample.data.size());
    if (value.has_value()) ref->latest[i] = *value;
  }
  ref->latest_time = sample.mono_time;
  ref->has_values = true;
}

std::vector<std::string> enum_names_for(const dbc::Database &database, uint32_t address, const char *signal_name) {
  std::vector<std::string> names;
  const dbc::Message *message = database.message(address);
  if (message == nullptr) return names;
  for (const dbc::Signal &signal : message->signals) {
    if (signal.name != signal_name) continue;
    names = database.enumNames(signal);
    break;
  }
  return names;
}

struct SampleRef {
  double time = 0.0;
  uint8_t bus = 0;
  uint32_t address = 0;
  const CanFrameSample *sample = nullptr;
};

const RadarPointFrame *frame_at(const RadarPointsData &data, double time) {
  if (data.frames.empty()) return nullptr;
  auto it = std::lower_bound(data.frames.begin(), data.frames.end(), time,
                             [](const RadarPointFrame &frame, double value) { return frame.time < value; });
  if (it == data.frames.end()) return &data.frames.back();
  if (it == data.frames.begin()) return &data.frames.front();
  const RadarPointFrame &after = *it;
  const RadarPointFrame &before = *(it - 1);
  return (time - before.time) <= (after.time - time) ? &before : &after;
}

}  // namespace

RadarPointsData *ensure_radar_points(AppSession *session) {
  RouteData &route = session->route_data;
  const std::string key = util::string_format("%s|%s|%zu|%.3f|%.3f",
                                             session->route_name.c_str(),
                                             session->dbc_override.c_str(),
                                             route.can_messages.size(),
                                             route.x_min,
                                             route.x_max);
  if (session->radar_points != nullptr && session->radar_points->source_key == key) {
    return session->radar_points.get();
  }

  auto data = std::make_unique<RadarPointsData>();
  data->source_key = key;
  data->dbc_name = RADAR_DBC;

  const std::optional<dbc::Database> database = load_dbc_by_name(RADAR_DBC);
  if (!database.has_value()) {
    data->error = std::string("dbc not found: ") + RADAR_DBC;
    session->radar_points = std::move(data);
    return session->radar_points.get();
  }

  std::vector<SampleRef> samples;
  uint32_t trigger_address = 0;
  for (const CanMessageData &message : route.can_messages) {
    const dbc::Message *dbc_message = database->message(message.id.address);
    if (dbc_message == nullptr) continue;
    if (dbc_message->name.rfind("RadarPoint", 0) != 0) continue;
    if (trigger_address == 0 && dbc_message->name == "RadarPoint31_B") {
      trigger_address = message.id.address;
      data->bus = message.id.bus;
    }
    for (const CanFrameSample &sample : message.samples) {
      samples.push_back(SampleRef{sample.mono_time, message.id.bus, message.id.address, &sample});
    }
  }
  if (trigger_address == 0) {
    data->error = "no RadarPoint frames in this route";
    session->radar_points = std::move(data);
    return session->radar_points.get();
  }
  std::sort(samples.begin(), samples.end(), [](const SampleRef &a, const SampleRef &b) {
    return std::tie(a.time, a.address) < std::tie(b.time, b.address);
  });

  std::vector<MessageRef> refs;
  std::unordered_map<uint32_t, size_t> ref_by_address;
  std::array<const MessageRef *, RADAR_SLOTS> refs_a {};
  std::array<const MessageRef *, RADAR_SLOTS> refs_b {};

  refs.reserve(static_cast<size_t>(RADAR_SLOTS) * 2);
  for (int slot = 0; slot < RADAR_SLOTS; ++slot) {
    for (const char *suffix : {"_A", "_B"}) {
      const std::string want = util::string_format("RadarPoint%d%s", slot, suffix);
      for (const auto &[address, message] : database->messages()) {
        if (message.name != want) continue;
        ref_by_address.emplace(address, refs.size());
        refs.push_back(make_message_ref(*database, address));
        const MessageRef *ref = &refs.back();
        if (std::string(suffix) == "_A") {
          refs_a[static_cast<size_t>(slot)] = ref;
        } else {
          refs_b[static_cast<size_t>(slot)] = ref;
        }
        break;
      }
    }
  }

  const auto trigger_it = ref_by_address.find(trigger_address);
  if (trigger_it == ref_by_address.end()) {
    data->error = "trigger message missing after dbc decode";
    session->radar_points = std::move(data);
    return session->radar_points.get();
  }

  data->class_names = enum_names_for(*database, trigger_address, "Class");
  data->move_state_names = enum_names_for(*database, trigger_address, "MovingState");
  data->class_counts.assign(std::max<size_t>(data->class_names.size(), 8), 0);

  for (const SampleRef &sample : samples) {
    if (sample.bus != data->bus) continue;
    const auto it = ref_by_address.find(sample.address);
    if (it == ref_by_address.end()) continue;
    MessageRef &ref = refs[it->second];
    if (ref.message == nullptr) continue;
    decode_into(*ref.message, *sample.sample, &ref);
    if (sample.address != trigger_address) continue;

    RadarPointFrame frame;
    frame.time = sample.time;
    for (int slot = 0; slot < RADAR_SLOTS; ++slot) {
      const MessageRef *a = refs_a[static_cast<size_t>(slot)];
      const MessageRef *b = refs_b[static_cast<size_t>(slot)];
      if (a == nullptr || b == nullptr || !a->has_values || !b->has_values) continue;
      if (std::fabs(a->latest_time - b->latest_time) > MAX_PAIR_AGE) {
        ++data->dropped_pair_mismatch;
        continue;
      }
      if (value_at(*a, a->slot.index) != value_at(*b, b->slot.index_b)) {
        ++data->dropped_pair_mismatch;
        continue;
      }
      if (value_at(*a, a->slot.tracked) < 0.5) {
        ++data->dropped_untracked;
        continue;
      }

      RadarPoint2D point;
      point.slot = slot;
      point.d_rel = static_cast<float>(value_at(*a, a->slot.long_dist));
      point.y_rel = static_cast<float>(value_at(*a, a->slot.lat_dist));
      point.v_rel = static_cast<float>(value_at(*a, a->slot.long_speed));
      point.prob_exist = static_cast<float>(value_at(*a, a->slot.prob_exist));
      point.length = static_cast<float>(value_at(*b, b->slot.length));
      point.prob_class = static_cast<float>(value_at(*b, b->slot.prob_class));
      point.cls = static_cast<uint8_t>(value_at(*b, b->slot.cls));
      point.moving_state = static_cast<uint8_t>(value_at(*b, b->slot.moving_state));
      point.valid = value_at(*a, a->slot.valid) >= 0.5;
      point.measured = value_at(*a, a->slot.measured) >= 0.5;
      if (point.cls < data->class_counts.size()) {
        ++data->class_counts[point.cls];
      }
      frame.points.push_back(point);
    }
    if (data->frames.empty()) data->t_min = frame.time;
    data->t_max = frame.time;
    data->frames.push_back(std::move(frame));
  }

  data->ok = !data->frames.empty();
  if (!data->ok) data->error = "no tracked radar points decoded";
  session->radar_points = std::move(data);
  return session->radar_points.get();
}

void draw_radar_pane(AppSession *session, UiState *state, Pane *, int pane_index) {
  TabUiState *tab_state = app_active_tab_state(state);
  if (tab_state == nullptr) {
    ImGui::TextUnformatted("Radar unavailable");
    return;
  }
  const size_t pane_count = tab_state->camera_panes.size();
  if (tab_state->radar_panes.size() < pane_count) {
    tab_state->radar_panes.resize(pane_count, TabUiState::RadarPaneState{});
  }
  if (pane_index < 0 || static_cast<size_t>(pane_index) >= tab_state->radar_panes.size()) {
    ImGui::TextUnformatted("Radar unavailable");
    return;
  }
  TabUiState::RadarPaneState &radar_state = tab_state->radar_panes[static_cast<size_t>(pane_index)];

  const RadarPointsData *data = ensure_radar_points(session);
  if (data == nullptr || !data->ok) {
    ImGui::TextDisabled("%s", (data != nullptr && !data->error.empty()) ? data->error.c_str() : "no radar points");
    ImGui::TextDisabled("Needs %s frames on the radar bus.", RADAR_DBC);
    return;
  }

  ImGui::Checkbox("valid", &radar_state.valid_only);
  ImGui::SameLine();
  ImGui::Checkbox("counts", &radar_state.show_class_counts);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(110.0f);
  ImGui::SliderFloat("min prob", &radar_state.min_prob_exist, 0.0f, 100.0f, "%.0f%%");
  ImGui::SameLine();
  if (ImGui::SmallButton("fit")) radar_state.zoom = 1.0f;
  ImGui::SameLine();
  ImGui::TextDisabled("bus %d · %zu frames · %.0f–%.0fs", data->bus, data->frames.size(),
                      data->t_min, data->t_max);

  const ImVec2 rect_min = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 size(std::max(1.0f, avail.x), std::max(1.0f, avail.y));
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("##radar_canvas", size);
  const ImVec2 rect_max(rect_min.x + size.x, rect_min.y + size.y);
  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  draw_list->PushClipRect(rect_min, rect_max, true);
  draw_list->AddRectFilled(rect_min, rect_max, BACKGROUND);
  draw_list->AddRect(rect_min, rect_max, GRID_COLOR);

  const double now = state->has_tracker_time ? state->tracker_time : data->t_max;
  const RadarPointFrame *frame = frame_at(*data, now);
  if (frame == nullptr) {
    draw_list->PopClipRect();
    return;
  }

  auto visible = [&](const RadarPoint2D &point) {
    if (radar_state.valid_only && !point.valid) return false;
    return point.prob_exist >= radar_state.min_prob_exist;
  };

  const float max_forward = MAX_FORWARD_M;
  float max_lateral = 6.0f;
  for (const RadarPoint2D &point : frame->points) {
    if (!visible(point)) continue;
    max_lateral = std::max(max_lateral, std::fabs(point.y_rel));
  }
  max_lateral = std::max(4.0f, max_lateral * 1.15f);

  const float margin_y = 22.0f;
  const float margin_x = 52.0f;  // room for the range labels + legend
  const float usable_w = std::max(1.0f, size.x - 2.0f * margin_x);
  const float usable_h = std::max(1.0f, size.y - 2.0f * margin_y);
  const float scale = std::max(0.2f, std::min(usable_w / (2.0f * max_lateral), usable_h / max_forward))
                    * std::max(0.25f, radar_state.zoom);
  const float ego_x = rect_min.x + margin_x + usable_w * 0.5f;
  const float ego_y = rect_max.y - margin_y;
  const auto to_screen = [&](float forward, float lateral) {
    return ImVec2(ego_x + lateral * scale, ego_y - forward * scale);
  };

  for (float range = 25.0f; range <= max_forward + 0.5f; range += 25.0f) {
    const float y = ego_y - range * scale;
    if (y < rect_min.y + 2.0f) break;
    draw_list->AddLine(ImVec2(rect_min.x + margin_x * 0.35f, y), ImVec2(rect_max.x - 8.0f, y), GRID_COLOR, 1.0f);
    char label[16];
    std::snprintf(label, sizeof(label), "%dm", static_cast<int>(range));
    draw_list->AddText(ImVec2(rect_min.x + 4.0f, y - ImGui::GetTextLineHeight() * 0.5f), GRID_LABEL_COLOR, label);
  }
  draw_list->AddLine(ImVec2(ego_x, rect_min.y + 2.0f), ImVec2(ego_x, ego_y), GRID_COLOR, 1.0f);
  draw_list->AddRectFilled(ImVec2(ego_x - 5.0f, ego_y - 5.0f), ImVec2(ego_x + 5.0f, ego_y + 5.0f), EGO_COLOR);

  int shown = 0;
  const RadarPoint2D *hovered = nullptr;
  ImVec2 hover_pos(0.0f, 0.0f);
  float hover_dist = 12.0f;
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  const bool hover_canvas = ImGui::IsItemHovered();

  for (const RadarPoint2D &point : frame->points) {
    if (!visible(point)) continue;
    const ImVec2 pos = to_screen(point.d_rel, point.y_rel);
    const float radius = 2.5f + 3.5f * std::clamp(point.prob_exist / 100.0f, 0.0f, 1.0f);
    draw_marker(draw_list, point.cls, pos, radius);
    ++shown;
    if (hover_canvas) {
      const float dx = pos.x - mouse.x;
      const float dy = pos.y - mouse.y;
      const float dist = std::sqrt(dx * dx + dy * dy);
      if (dist < hover_dist) {
        hover_dist = dist;
        hovered = &point;
        hover_pos = pos;
      }
    }
  }

  if (hovered != nullptr) {
    draw_list->AddCircle(hover_pos, 10.0f, IM_COL32(240, 246, 252, 180), 0, 1.2f);
  }

  {
    // fixed legend: ego + every DBC class, so entries never disappear between frames/routes
    std::vector<std::string> labels;
    labels.push_back(radar_state.show_class_counts ? "ego" : std::string("ego"));
    for (size_t cls = 0; cls < data->class_names.size(); ++cls) {
      const int count = cls < data->class_counts.size() ? data->class_counts[cls] : 0;
      const std::string name = pretty_class_name(data->class_names[cls]);
      labels.push_back(radar_state.show_class_counts
                         ? util::string_format("%s %d", name.c_str(), count)
                         : name);
    }
    float legend_width = 0.0f;
    for (const std::string &label : labels) {
      legend_width = std::max(legend_width, ImGui::CalcTextSize(label.c_str()).x);
    }
    const float legend_x = rect_max.x - legend_width - 22.0f;
    float legend_y = rect_min.y + 4.0f;
    draw_list->AddRectFilled(ImVec2(legend_x - 4.0f, legend_y + 1.0f), ImVec2(legend_x + 4.0f, legend_y + 9.0f), EGO_COLOR);
    draw_list->AddText(ImVec2(legend_x + 10.0f, legend_y), TEXT_COLOR, "ego");
    legend_y += ImGui::GetTextLineHeight() + 2.0f;
    for (size_t cls = 0; cls < data->class_names.size(); ++cls) {
      draw_marker(draw_list, static_cast<int>(cls), ImVec2(legend_x, legend_y + 5.0f), 4.0f);
      draw_list->AddText(ImVec2(legend_x + 10.0f, legend_y), TEXT_COLOR, labels[cls + 1].c_str());
      legend_y += ImGui::GetTextLineHeight() + 2.0f;
    }
  }

  double speed_kph = 0.0;
  if (const RouteSeries *v_ego = app_find_route_series(*session, "/carState/vEgo"); v_ego != nullptr) {
    if (const std::optional<double> value = app_sample_xy_value_at_time(v_ego->times, v_ego->values, false, now);
        value.has_value()) {
      speed_kph = *value * 3.6;
    }
  }
  const std::string footer = util::string_format("t %.1fs · ego %.0f km/h · %d points",
                                                frame->time, speed_kph, shown);
  draw_list->AddText(ImVec2(rect_min.x + 4.0f, rect_max.y - ImGui::GetTextLineHeight() - 2.0f),
                     GRID_LABEL_COLOR, footer.c_str());

  draw_list->PopClipRect();

  if (hovered != nullptr) {
    ImGui::BeginTooltip();
    const std::string cls_name = hovered->cls < data->class_names.size()
      ? pretty_class_name(data->class_names[hovered->cls])
      : util::string_format("class %d", static_cast<int>(hovered->cls));
    const std::string move_name = hovered->moving_state < data->move_state_names.size()
      ? pretty_class_name(data->move_state_names[hovered->moving_state])
      : util::string_format("move %d", static_cast<int>(hovered->moving_state));
    ImGui::Text("slot %d · %s", hovered->slot, cls_name.c_str());
    ImGui::Text("dRel %.1f m · yRel %+.1f m · vRel %+.1f m/s",
                static_cast<double>(hovered->d_rel),
                static_cast<double>(hovered->y_rel),
                static_cast<double>(hovered->v_rel));
    ImGui::Text("%s · length %.2f m", move_name.c_str(), static_cast<double>(hovered->length));
    ImGui::Text("prob exist %.0f%% · prob class %.0f%%%s%s",
                static_cast<double>(hovered->prob_exist),
                static_cast<double>(hovered->prob_class),
                hovered->valid ? " · valid" : "",
                hovered->measured ? " · measured" : "");
    ImGui::EndTooltip();
  }
}
