#include "adas/utils/topic_convert.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "adas/utils/adas_topics.h"

namespace adas {
namespace {

constexpr double kWidthFilterAlpha = 0.01;

constexpr double kCenterFitXMaxM = 30.0;

constexpr double kTurnDeadbandKappa = 5e-4;

double softLaneProb(float p, float min_p)
{
  if (!(p >= min_p))
    return 0.0;
  const double span = std::max(1e-3, 1.0 - static_cast<double>(min_p));
  return std::min(1.0, (static_cast<double>(p) - min_p) / span);
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

double medianLaneStd(const ai::flow::adas::LanePolyline& lane, const ai::flow::adas::LaneLines& ll, double range_m)
{
  std::vector<double> stds;
  const int n = std::min(lane.y_std_size(), ll.x_size());
  // Clamped from below rather than replaced by the default: a narrow configured window (say 4 m) used
  // to become the widest one, 5-40 m, the opposite of what was asked for.
  const double x_max = std::max(range_m, 10.0);
  for (int i = 0; i < n; ++i) {
    const double x = ll.x(i);
    if (x < 5.0 || x > x_max)
      continue;
    const double s = lane.y_std(i);
    if (s > 0.0 && std::isfinite(s))
      stds.push_back(s);
  }
  if (stds.empty())
    return -1.0;
  std::sort(stds.begin(), stds.end());
  return stds[stds.size() / 2];
}

double stdConfidence(double median_std, double good_m, double bad_m)
{
  if (median_std < 0.0 || bad_m <= good_m)
    return 1.0;
  if (median_std <= good_m)
    return 1.0;
  if (median_std >= bad_m)
    return 0.0;
  return (bad_m - median_std) / (bad_m - good_m);
}

double ramp(double x, double x0, double x1, double y0, double y1)
{
  if (x1 <= x0)
    return y0;
  const double t = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
  return y0 + t * (y1 - y0);
}

struct QuadFit {
  bool ok = false;
  double y0 = 0.0;
  double kappa = 0.0;
};

QuadFit fitAtZero(const std::vector<double>& xs, const std::vector<double>& ys, double x_max)
{
  QuadFit out;
  double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, t0 = 0, t1 = 0, t2 = 0;
  int n = 0;
  for (size_t i = 0; i < xs.size() && i < ys.size(); ++i) {
    const double x = xs[i];
    if (x < 0.0 || x > x_max || !std::isfinite(ys[i]))
      continue;
    const double x2 = x * x;
    s0 += 1.0;
    s1 += x;
    s2 += x2;
    s3 += x2 * x;
    s4 += x2 * x2;
    t0 += ys[i];
    t1 += x * ys[i];
    t2 += x2 * ys[i];
    ++n;
  }
  if (n < 4)
    return out;

  const double d = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2) + s2 * (s3 * s1 - s2 * s2);
  if (std::abs(d) < 1e-12)
    return out;
  const double da = t2 * (s2 * s0 - s1 * s1) - s3 * (t1 * s0 - t0 * s1) + s2 * (t1 * s1 - t0 * s2);
  const double dc = s4 * (s2 * t0 - s1 * t1) - s3 * (s3 * t0 - s1 * t2) + s2 * (s3 * t1 - s2 * t2);
  out.ok = true;
  out.y0 = dc / d;
  out.kappa = 2.0 * (da / d);
  return out;
}

}  // namespace

LanePathMsg laneLinesToPath(const ai::flow::adas::LaneLines& ll, const LanePathConfig& cfg, LaneFusionState* state)
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

  const double blend_scale = std::clamp(cfg.lane_blend_scale, 0.0, 1.0);
  const bool blend_off = blend_scale <= 1e-9 && out.polyline.size() >= 2;

  const int nx = ll.x_size();
  const bool geom_ok = nx >= 2;
  const bool have_l = geom_ok && ll.lanes_size() > 1 && ll.lanes(1).y_size() == nx;
  const bool have_r = geom_ok && ll.lanes_size() > 2 && ll.lanes(2).y_size() == nx;

  double l_prob = 0.0;
  double r_prob = 0.0;
  bool width_ok = false;
  double w_used = 0.0;
  if (have_l && have_r) {
    l_prob = softLaneProb(ll.lanes(1).prob(), cfg.min_lane_prob);
    r_prob = softLaneProb(ll.lanes(2).prob(), cfg.min_lane_prob);

    l_prob *=
        stdConfidence(medianLaneStd(ll.lanes(1), ll, cfg.lane_std_range_m), cfg.lane_std_good_m, cfg.lane_std_bad_m);
    r_prob *=
        stdConfidence(medianLaneStd(ll.lanes(2), ll, cfg.lane_std_range_m), cfg.lane_std_good_m, cfg.lane_std_bad_m);

    double w_sum = 0.0;
    int w_n = 0;
    for (int i = 0; i < nx; ++i) {
      const double x = ll.x(i);
      if (x < 5.0 || x > 40.0)
        continue;

      w_sum += std::abs(ll.lanes(2).y(i) - ll.lanes(1).y(i));
      ++w_n;
    }
    const double w_med = w_n > 0 ? w_sum / w_n : 0.0;

    w_used = w_med;
    if (state && w_med > 0.0) {
      if (!state->inited) {
        state->lane_width_m = w_med;
        state->inited = true;
      } else {
        state->lane_width_m += kWidthFilterAlpha * (w_med - state->lane_width_m);
      }
      w_used = state->lane_width_m;
    }
    width_ok = w_used > cfg.lane_width_min_m && w_used < cfg.lane_width_max_m;
    out.lane_width_m = w_used;
  }

  const bool anchored = l_prob > 0.0 && r_prob > 0.0 && width_ok;
  out.lane_anchored = anchored;

  std::vector<double> xs;
  std::vector<double> lane_ys;
  if (anchored) {
    xs.reserve(static_cast<size_t>(nx));
    lane_ys.reserve(static_cast<size_t>(nx));
    for (int i = 0; i < nx; ++i) {
      const double x = ll.x(i);
      const double yl = ll.lanes(1).y(i);
      const double yr = ll.lanes(2).y(i);

      const double w = std::clamp(std::abs(yl - yr), cfg.lane_width_min_m, cfg.lane_width_max_m);
      const double from_l = yl + 0.5 * w;
      const double from_r = yr - 0.5 * w;
      const double lane_y = (l_prob * from_l + r_prob * from_r) / (l_prob + r_prob + 1e-6);
      xs.push_back(x);
      lane_ys.push_back(lane_y);
    }
  }

  out.p_lane_blend_scale = blend_scale;
  out.p_camera_offset_m = cfg.camera_offset_m;
  out.p_center_force_gain = cfg.center_force_gain;

  double center = 0.0;
  if (anchored) {
    const QuadFit fit = fitAtZero(xs, lane_ys, kCenterFitXMaxM);

    if (fit.ok)
      out.lane_offset_m = fit.y0 - cfg.cam_y_left_m;
    if (fit.ok && cfg.center_force_gain > 0.0) {
      const double offset = out.lane_offset_m;
      const double w = std::max(w_used, 1e-3);
      double cf = cfg.center_force_gain * (cfg.center_force_typical_width_m / w) * offset;
      cf *= ramp(w, 2.6, 2.8, 0.0, 1.0);
      cf *= ramp(w, 4.0, 6.0, 1.0, 0.0);
      if (std::abs(fit.kappa) > kTurnDeadbandKappa && cf * fit.kappa > 0.0)
        cf *= cfg.center_force_turn_scale;
      center = std::clamp(cf, -cfg.center_force_max_m, cfg.center_force_max_m);
      out.center_force_m = center;
    }
  }
  const double shift = cfg.camera_offset_m + center;

  if (blend_off) {
    applyCameraOffset(out, shift);
    return out;
  }
  bool lanelines_active = true;
  if (cfg.lane_mode_hysteresis) {
    const double raw_l = ll.lanes_size() > 1 ? ll.lanes(1).prob() : 0.0;
    const double raw_r = ll.lanes_size() > 2 ? ll.lanes(2).prob() : 0.0;
    lanelines_active = state ? state->lanelines_active : true;
    if (raw_l < cfg.lane_mode_off_prob && raw_r < cfg.lane_mode_off_prob)
      lanelines_active = false;
    else if (raw_l > cfg.lane_mode_on_prob || raw_r > cfg.lane_mode_on_prob)
      lanelines_active = true;
    if (state)
      state->lanelines_active = lanelines_active;
  }
  out.lanelines_active = lanelines_active;

  const double d_prob = (anchored && lanelines_active) ? (l_prob + r_prob - l_prob * r_prob) * blend_scale : 0.0;

  if (d_prob <= 1e-6 && out.polyline.size() >= 2) {
    applyCameraOffset(out, shift);
    return out;
  }

  if (d_prob > 1e-6 && out.polyline.size() >= 2) {
    for (auto& pt : out.polyline) {
      const double y_lane = interpY(pt.x(), xs, lane_ys);
      pt.y() = d_prob * y_lane + (1.0 - d_prob) * pt.y();
    }
    applyCameraOffset(out, shift);
    return out;
  }

  if (d_prob > 1e-6 && !xs.empty()) {
    out.polyline.clear();
    out.polyline.reserve(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
      if (xs[i] >= 1.0)
        out.polyline.push_back({xs[i], lane_ys[i]});
    }
  }
  applyCameraOffset(out, shift);
  return out;
}

ChassisSample carStateToChassis(const ai::flow::adas::CarState& cs, double steer_ratio)
{
  ChassisSample s;
  s.timestamp_us = cs.timestamp() * 1000;
  s.speed_mps = cs.v_ego();
  s.yaw_rate = cs.yaw_rate();
  s.steering_angle_deg = cs.steering_angle_deg();
  s.steering_pressed = cs.steering_pressed();
  s.left_blinker = cs.left_blinker();
  s.right_blinker = cs.right_blinker();
  const double ratio = std::max(steer_ratio, 1e-3);
  s.steer_rad = (cs.steering_angle_deg() * M_PI / 180.0) / ratio;
  return s;
}

RawImuSample imuToRaw(const ai::flow::adas::IMUData& imu)
{
  RawImuSample s;
  s.timestamp_us = imu.timestamp() * 1000;
  s.ax = imu.accel_x();
  s.ay = imu.accel_y();
  s.az = imu.accel_z();
  s.gx = imu.gyro_x();
  s.gy = imu.gyro_y();
  s.gz = imu.gyro_z();
  s.valid = true;
  return s;
}

CameraOdometrySample cameraOdometryToSample(const ai::flow::adas::CameraOdometry& odom)
{
  CameraOdometrySample s;
  s.timestamp_us = odom.timestamp() * 1000;
  for (int i = 0; i < 3; ++i) {
    s.trans(i) = (i < odom.trans_size()) ? odom.trans(i) : 0.0;
    s.rot(i) = (i < odom.rot_size()) ? odom.rot(i) : 0.0;
    s.trans_std(i) = (i < odom.trans_std_size()) ? odom.trans_std(i) : 1.0;
    s.rot_std(i) = (i < odom.rot_std_size()) ? odom.rot_std(i) : 1.0;
  }
  s.valid = odom.trans_size() >= 3 && odom.rot_size() >= 3;
  return s;
}

}  // namespace adas
