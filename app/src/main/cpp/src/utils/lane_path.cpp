#include "adas/utils/lane_path.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace adas {
namespace {
// comma `common.realtime.DT_MDL` and `FirstOrderFilter` rc on LanePlanner.
constexpr double kDtMdl = 0.05;
constexpr double kWidthEstRc = 9.95;
constexpr double kWidthCertRc = 0.95;
constexpr double kWidthEstAlpha = kDtMdl / (kWidthEstRc + kDtMdl);    // 0.005
constexpr double kWidthCertAlpha = kDtMdl / (kWidthCertRc + kDtMdl);  // 0.05

double lerp(double x, double x0, double x1, double y0, double y1)
{
  if (x1 <= x0)
    return y0;
  if (x <= x0)
    return y0;
  if (x >= x1)
    return y1;
  const double t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

double interpY(double x, const std::vector<double>& xs, const std::vector<double>& ys)
{
  if (xs.empty() || xs.size() != ys.size())
    return 0.0;
  if (x <= xs.front())
    return ys.front();
  if (x >= xs.back())
    return ys.back();
  for (size_t i = 1; i < xs.size(); ++i) {
    if (x <= xs[i]) {
      const double x0 = xs[i - 1];
      const double x1 = xs[i];
      const double t = (x1 > x0) ? (x - x0) / (x1 - x0) : 0.0;
      return ys[i - 1] + t * (ys[i] - ys[i - 1]);
    }
  }
  return ys.back();
}

void applyCameraOffset(LanePathMsg& msg, double camera_offset_m)
{
  if (std::abs(camera_offset_m) < 1e-9)
    return;
  for (auto& p : msg.polyline)
    p.y() += camera_offset_m;
  for (auto& p : msg.plan_poly)
    p.y() += camera_offset_m;
}

double lineStd0(const adas::proto::LanePolyline& lane)
{
  if (lane.y_std_size() <= 0)
    return 0.0;
  const double s = lane.y_std(0);
  return (s > 0.0 && std::isfinite(s)) ? s : 0.0;
}

void foUpdate(double& x, double alpha, double z) { x = (1.0 - alpha) * x + alpha * z; }

}  // namespace

LanePathMsg laneLinesToPath(const adas::proto::LaneLines& ll, const LanePathConfig& cfg, LaneFusionState* state,
                            double v_ego)
{
  LanePathMsg out;
  const int64_t capture_ms = ll.capture_ts_ms() > 0 ? ll.capture_ts_ms() : ll.timestamp();
  out.timestamp_us = capture_ms * 1000;
  out.capture_ts_us = capture_ms * 1000;
  out.infer_ts_us = ll.infer_ts_ms() > 0 ? ll.infer_ts_ms() * 1000 : 0;
  out.frame_id = ll.frame_id();

  const int np = std::min(ll.plan_x_size(), ll.plan_y_size());
  if (np >= 2) {
    out.polyline.reserve(static_cast<size_t>(np));
    out.plan_poly.reserve(static_cast<size_t>(np));
    for (int i = 0; i < np; ++i) {
      const double x = ll.plan_x(i);
      const double y = ll.plan_y(i);
      out.plan_poly.push_back({x, y});
      if (x >= 1.0)
        out.polyline.push_back({x, y});
    }
  }

  const int nyaw = std::min({ll.plan_yaw_size(), ll.plan_yaw_rate_size(), np});
  if (nyaw >= 2) {
    out.plan_yaw.reserve(static_cast<size_t>(nyaw));
    out.plan_yaw_rate.reserve(static_cast<size_t>(nyaw));
    for (int i = 0; i < nyaw; ++i) {
      out.plan_yaw.push_back(ll.plan_yaw(i));
      out.plan_yaw_rate.push_back(ll.plan_yaw_rate(i));
    }
  }

  out.p_lane_blend_scale = cfg.lane_blend_scale;
  out.p_camera_offset_m = cfg.camera_offset_m;
  out.p_center_force_gain = cfg.center_force_gain;

  const int nx = ll.x_size();
  const bool have_l = nx >= 2 && ll.lanes_size() > 1 && ll.lanes(1).y_size() == nx;
  const bool have_r = nx >= 2 && ll.lanes_size() > 2 && ll.lanes(2).y_size() == nx;
  if (!have_l && !have_r) {
    applyCameraOffset(out, cfg.camera_offset_m);
    return out;
  }

  std::vector<double> ll_x(static_cast<size_t>(nx));
  std::vector<double> lll_y(static_cast<size_t>(nx), 0.0);
  std::vector<double> rll_y(static_cast<size_t>(nx), 0.0);
  for (int i = 0; i < nx; ++i) {
    ll_x[static_cast<size_t>(i)] = ll.x(i);
    if (have_l)
      lll_y[static_cast<size_t>(i)] = ll.lanes(1).y(i);
    if (have_r)
      rll_y[static_cast<size_t>(i)] = ll.lanes(2).y(i);
  }

  // parse_model: raw host probs and the model's scalar std (here y_std[0], comma laneLineStds[i]).
  double l_prob = have_l ? static_cast<double>(ll.lanes(1).prob()) : 0.0;
  double r_prob = have_r ? static_cast<double>(ll.lanes(2).prob()) : 0.0;
  const double l_std = have_l ? lineStd0(ll.lanes(1)) : 0.0;
  const double r_std = have_r ? lineStd0(ll.lanes(2)) : 0.0;

  // Reduce reliance on lanelines that are too far apart or will be in a few seconds.
  std::vector<double> width_pts(static_cast<size_t>(nx));
  for (int i = 0; i < nx; ++i)
    width_pts[static_cast<size_t>(i)] = rll_y[static_cast<size_t>(i)] - lll_y[static_cast<size_t>(i)];
  double mod = 1.0;
  for (double t_check : {0.0, 1.5, 3.0}) {
    const double width_at_t = interpY(t_check * (v_ego + 7.0), ll_x, width_pts);
    mod = std::min(mod, lerp(width_at_t, 4.0, 5.0, 1.0, 0.0));
  }
  l_prob *= mod;
  r_prob *= mod;

  // Reduce reliance on uncertain lanelines.
  l_prob *= lerp(l_std, 0.15, 0.3, 1.0, 0.0);
  r_prob *= lerp(r_std, 0.15, 0.3, 1.0, 0.0);

  LaneFusionState local;
  LaneFusionState* st = state ? state : &local;
  foUpdate(st->width_certainty, kWidthCertAlpha, l_prob * r_prob);
  const double current_lane_width = std::abs(rll_y.front() - lll_y.front());
  foUpdate(st->width_est_m, kWidthEstAlpha, current_lane_width);
  const double speed_lane_width = lerp(v_ego, 0.0, 31.0, 2.8, 3.5);
  st->lane_width_m = st->width_certainty * st->width_est_m + (1.0 - st->width_certainty) * speed_lane_width;
  out.lane_width_m = st->lane_width_m;

  const double clipped_lane_width = std::min(4.0, st->lane_width_m);
  std::vector<double> lane_path_y(static_cast<size_t>(nx));
  const double inv = 1.0 / (l_prob + r_prob + 0.0001);
  for (int i = 0; i < nx; ++i) {
    const double from_l = lll_y[static_cast<size_t>(i)] + clipped_lane_width / 2.0;
    const double from_r = rll_y[static_cast<size_t>(i)] - clipped_lane_width / 2.0;
    lane_path_y[static_cast<size_t>(i)] = (l_prob * from_l + r_prob * from_r) * inv;
  }

  const double d_prob = l_prob + r_prob - l_prob * r_prob;
  out.lane_anchored = d_prob > 1e-6;
  out.lanelines_active = out.lane_anchored;
  out.lane_offset_m = 0.5 * (lll_y.front() + rll_y.front());

  if (d_prob > 1e-6 && out.polyline.size() >= 2) {
    for (auto& pt : out.polyline) {
      const double y_lane = interpY(pt.x(), ll_x, lane_path_y);
      pt.y() = d_prob * y_lane + (1.0 - d_prob) * pt.y();
    }
  } else if (d_prob > 1e-6 && out.polyline.size() < 2) {
    out.polyline.clear();
    out.polyline.reserve(static_cast<size_t>(nx));
    for (int i = 0; i < nx; ++i) {
      if (ll_x[static_cast<size_t>(i)] >= 1.0)
        out.polyline.push_back({ll_x[static_cast<size_t>(i)], lane_path_y[static_cast<size_t>(i)]});
    }
  }

  applyCameraOffset(out, cfg.camera_offset_m);
  return out;
}

}  // namespace adas
