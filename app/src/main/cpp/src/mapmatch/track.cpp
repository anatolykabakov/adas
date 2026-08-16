#include "adas/mapmatch/track.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace adas {
namespace mapmatch {
namespace {
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

/** Mean over ±half window in path-sampled array indices. */
std::vector<double> smoothByPath(const std::vector<double>& v, int half)
{
  if (half <= 0 || v.size() < 2)
    return v;
  std::vector<double> out(v.size(), 0.0);
  for (std::size_t i = 0; i < v.size(); ++i) {
    const std::size_t lo = i > static_cast<std::size_t>(half) ? i - half : 0;
    const std::size_t hi = std::min(v.size() - 1, i + static_cast<std::size_t>(half));
    double sum = 0.0;
    for (std::size_t k = lo; k <= hi; ++k)
      sum += v[k];
    out[i] = sum / static_cast<double>(hi - lo + 1);
  }
  return out;
}

/** Stop intervals: speed below threshold longer than stop_min_s. */
std::vector<std::pair<std::size_t, std::size_t>> findStops(const std::vector<double>& t_s, const std::vector<double>& v,
                                                           double stop_speed, double stop_min_s)
{
  std::vector<std::pair<std::size_t, std::size_t>> stops;
  std::size_t i = 0;
  while (i < v.size()) {
    if (v[i] > stop_speed) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j + 1 < v.size() && v[j + 1] <= stop_speed)
      ++j;
    if (t_s[j] - t_s[i] >= stop_min_s)
      stops.emplace_back(i, j);
    i = j + 1;
  }
  return stops;
}

/** Sensor bias measured at stops and linearly interpolated between them.
 *
 *  While stopped the vehicle does not rotate, so whatever the sensor reads is bias. A single
 *  global coefficient does not work: bias drifts slowly, so we take per-stop values and
 *  interpolate. */
std::vector<double> stopBias(const std::vector<double>& t_s, const std::vector<double>& w,
                             const std::vector<std::pair<std::size_t, std::size_t>>& stops)
{
  std::vector<double> bias(t_s.size(), 0.0);
  if (stops.empty())
    return bias;

  std::vector<double> bt, bv;
  for (const auto& [a, b] : stops) {
    double sum = 0.0;
    for (std::size_t k = a; k <= b; ++k)
      sum += w[k];
    bt.push_back(0.5 * (t_s[a] + t_s[b]));
    bv.push_back(sum / static_cast<double>(b - a + 1));
  }
  for (std::size_t i = 0; i < t_s.size(); ++i) {
    const double t = t_s[i];
    if (t <= bt.front()) {
      bias[i] = bv.front();
      continue;
    }
    if (t >= bt.back()) {
      bias[i] = bv.back();
      continue;
    }
    std::size_t k = 1;
    while (k < bt.size() && bt[k] < t)
      ++k;
    const double w0 = (t - bt[k - 1]) / std::max(1e-6, bt[k] - bt[k - 1]);
    bias[i] = bv[k - 1] + w0 * (bv[k] - bv[k - 1]);
  }
  return bias;
}

/** Linear interpolation of y(t) onto a new grid. */
std::vector<double> resample(const std::vector<double>& t_src, const std::vector<double>& y,
                             const std::vector<double>& t_dst)
{
  std::vector<double> out(t_dst.size(), 0.0);
  if (t_src.size() < 2 || y.size() != t_src.size())
    return out;
  std::size_t j = 1;
  for (std::size_t i = 0; i < t_dst.size(); ++i) {
    const double t = t_dst[i];
    while (j + 1 < t_src.size() && t_src[j] < t)
      ++j;
    const double dt = t_src[j] - t_src[j - 1];
    const double w = dt > 1e-9 ? (t - t_src[j - 1]) / dt : 0.0;
    out[i] = y[j - 1] + std::clamp(w, 0.0, 1.0) * (y[j] - y[j - 1]);
  }
  return out;
}

/** Vertical-axis yaw rate from raw IMU: “up” axis from acceleration at rest. */
std::vector<double> imuYawRate(const ImuSamples& imu, const std::vector<double>& t_s, const std::vector<double>& v,
                               double stop_speed)
{
  std::vector<double> empty;
  if (imu.empty())
    return empty;

  const std::vector<double> v_at_imu = resample(t_s, v, imu.t_s);
  double gx = 0.0, gy = 0.0, gz = 0.0;
  int n = 0;
  for (std::size_t i = 0; i < imu.t_s.size(); ++i) {
    if (i < v_at_imu.size() && v_at_imu[i] > stop_speed)
      continue;
    gx += imu.accel_x[i];
    gy += imu.accel_y[i];
    gz += imu.accel_z[i];
    ++n;
  }
  if (n < 10) {
    gx = gy = gz = 0.0;
    for (std::size_t i = 0; i < imu.t_s.size(); ++i) {
      gx += imu.accel_x[i];
      gy += imu.accel_y[i];
      gz += imu.accel_z[i];
    }
  }
  const double norm = std::sqrt(gx * gx + gy * gy + gz * gz);
  if (norm < 1e-6)
    return empty;
  gx /= norm;
  gy /= norm;
  gz /= norm;

  std::vector<double> w(imu.t_s.size(), 0.0);
  for (std::size_t i = 0; i < imu.t_s.size(); ++i)
    w[i] = imu.gyro_x[i] * gx + imu.gyro_y[i] * gy + imu.gyro_z[i] * gz;
  return w;
}

}  // namespace

double Track::totalTurnDeg() const
{
  if (theta_rad.size() < 2)
    return 0.0;
  return (theta_rad.back() - theta_rad.front()) * 180.0 / M_PI;
}

std::string Track::describe() const
{
  std::ostringstream os;
  bool first = true;
  for (const auto& m : maneuvers) {
    if (!first)
      os << " → ";
    first = false;
    if (m.isTurn()) {
      os << (m.isLeft() ? "left " : "right ") << std::llround(std::abs(m.angle_deg)) << "°";
      if (m.radius_m > 0.0)
        os << " (R" << std::llround(m.radius_m) << ")";
    } else {
      os << "straight " << std::llround(m.length_m) << " m";
    }
  }
  return os.str();
}

Track buildTrack(const std::vector<double>& t_s, const std::vector<double>& speed_mps,
                 const std::vector<double>& yaw_rate_rps, const TrackConfig& cfg, const SegmentConfig& seg,
                 const ImuSamples& imu)
{
  Track out;
  const std::size_t n = std::min({t_s.size(), speed_mps.size(), yaw_rate_rps.size()});
  if (n < 2)
    return out;

  // Prepare yaw rate: chosen source minus stop bias.
  const auto count = static_cast<std::ptrdiff_t>(n);
  std::vector<double> omega(yaw_rate_rps.begin(), yaw_rate_rps.begin() + count);
  const std::vector<double> t(t_s.begin(), t_s.begin() + count);
  const std::vector<double> v(speed_mps.begin(), speed_mps.begin() + count);

  std::vector<double> omega_imu;
  if (!imu.empty() && cfg.yaw_source != TrackConfig::YawSource::Chassis) {
    const std::vector<double> raw = imuYawRate(imu, t, v, cfg.stop_speed_mps);
    if (!raw.empty()) {
      omega_imu = resample(imu.t_s, raw, t);
      double num = 0.0, den = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        if (v[i] <= cfg.stop_speed_mps)
          continue;
        num += omega_imu[i] * omega[i];
        den += std::abs(omega_imu[i] * omega[i]);
      }
      if (den > 1e-9 && num < 0.0)
        for (auto& x : omega_imu)
          x = -x;
    }
  }

  const auto stops = findStops(t, v, cfg.stop_speed_mps, cfg.stop_min_s);
  if (cfg.rezero_yaw_at_stops && !stops.empty()) {
    const std::vector<double> bias = stopBias(t, omega, stops);
    for (std::size_t i = 0; i < n; ++i)
      omega[i] -= bias[i];
    if (!omega_imu.empty()) {
      const std::vector<double> bias_imu = stopBias(t, omega_imu, stops);
      for (std::size_t i = 0; i < n; ++i)
        omega_imu[i] -= bias_imu[i];
    }
  }

  if (!omega_imu.empty()) {
    if (cfg.yaw_source == TrackConfig::YawSource::Imu) {
      omega = omega_imu;
    } else if (cfg.yaw_source == TrackConfig::YawSource::Blend) {
      const double a = std::clamp(cfg.blend_imu_hf, 0.0, 1.0);
      double lp_esp = omega.empty() ? 0.0 : omega[0];
      double lp_imu = omega_imu[0];
      constexpr double kAlpha = 0.1;
      for (std::size_t i = 0; i < n; ++i) {
        lp_esp += kAlpha * (omega[i] - lp_esp);
        lp_imu += kAlpha * (omega_imu[i] - lp_imu);
        omega[i] = lp_esp + a * (omega_imu[i] - lp_imu) + (1.0 - a) * (omega[i] - lp_esp);
      }
    }
  }

  // 1. Integrate in time to dense path; accumulate heading only while moving.
  std::vector<double> s_dense(n, 0.0), th_dense(n, 0.0);
  double s = 0.0, th = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    const double dt = clampd(t_s[i] - t_s[i - 1], 0.0, 0.5);
    const double vi = std::max(0.0, speed_mps[i]) * cfg.speed_scale;
    if (vi >= cfg.min_speed_mps)
      th += omega[i] * cfg.yaw_rate_scale * dt;
    s += vi * dt;
    s_dense[i] = s;
    th_dense[i] = th;
  }
  if (s <= cfg.resample_m * 3.0)
    return out;

  const std::size_t m = static_cast<std::size_t>(s / cfg.resample_m) + 1;
  out.s_m.reserve(m);
  out.theta_rad.reserve(m);
  std::size_t j = 1;
  for (std::size_t k = 0; k < m; ++k) {
    const double target = static_cast<double>(k) * cfg.resample_m;
    while (j + 1 < n && s_dense[j] < target)
      ++j;
    const double s0 = s_dense[j - 1], s1 = s_dense[j];
    const double w = (s1 > s0) ? (target - s0) / (s1 - s0) : 0.0;
    out.s_m.push_back(target);
    out.theta_rad.push_back(th_dense[j - 1] + w * (th_dense[j] - th_dense[j - 1]));
  }

  out.x_m.assign(out.size(), 0.0);
  out.y_m.assign(out.size(), 0.0);
  const double th0 = out.theta_rad.front();
  for (std::size_t k = 1; k < out.size(); ++k) {
    const double a = out.theta_rad[k - 1] - th0;
    out.x_m[k] = out.x_m[k - 1] + cfg.resample_m * std::cos(a);
    out.y_m[k] = out.y_m[k - 1] + cfg.resample_m * std::sin(a);
  }
  for (auto& a : out.theta_rad)
    a -= th0;

  out.maneuvers = segmentManeuvers(out, seg);
  return out;
}

std::vector<Maneuver> segmentManeuvers(const Track& track, const SegmentConfig& cfg)
{
  std::vector<Maneuver> out;
  if (track.size() < 4)
    return out;

  const double ds = track.size() > 1 ? (track.s_m[1] - track.s_m[0]) : 1.0;
  if (ds <= 1e-6)
    return out;

  // κ(s) = dθ/ds, smoothed along path: a single spike must not become a turn.
  std::vector<double> kappa(track.size(), 0.0);
  for (std::size_t i = 1; i < track.size(); ++i)
    kappa[i] = (track.theta_rad[i] - track.theta_rad[i - 1]) / ds;
  kappa[0] = kappa.size() > 1 ? kappa[1] : 0.0;
  kappa = smoothByPath(kappa, static_cast<int>(std::lround(cfg.smooth_m / ds / 2.0)));

  const double kappa_turn = cfg.turn_radius_m > 1e-6 ? 1.0 / cfg.turn_radius_m : 0.0;

  // Intervals where curvature exceeds threshold — turn candidates.
  struct Span {
    std::size_t a = 0, b = 0;
    int sign = 0;
  };
  std::vector<Span> spans;
  for (std::size_t i = 0; i < track.size(); ++i) {
    const int sign = kappa[i] > kappa_turn ? +1 : (kappa[i] < -kappa_turn ? -1 : 0);
    if (sign == 0)
      continue;
    if (!spans.empty() && spans.back().sign == sign && (track.s_m[i] - track.s_m[spans.back().b]) <= cfg.merge_gap_m) {
      spans.back().b = i;
    } else {
      spans.push_back({i, i, sign});
    }
  }

  std::vector<Span> turns;
  for (const auto& sp : spans) {
    const double angle = (track.theta_rad[sp.b] - track.theta_rad[sp.a]) * 180.0 / M_PI;
    if (std::abs(angle) >= cfg.min_turn_deg)
      turns.push_back(sp);
  }

  auto push_straight = [&](std::size_t a, std::size_t b) {
    const double len = track.s_m[b] - track.s_m[a];
    if (len < cfg.min_straight_m)
      return;
    Maneuver mv;
    mv.kind = Maneuver::Kind::Straight;
    mv.s_start_m = track.s_m[a];
    mv.s_end_m = track.s_m[b];
    mv.length_m = len;
    out.push_back(mv);
  };

  std::size_t cursor = 0;
  for (const auto& sp : turns) {
    if (sp.a > cursor)
      push_straight(cursor, sp.a);
    Maneuver mv;
    mv.kind = Maneuver::Kind::Turn;
    mv.s_start_m = track.s_m[sp.a];
    mv.s_end_m = track.s_m[sp.b];
    mv.length_m = mv.s_end_m - mv.s_start_m;
    mv.angle_deg = (track.theta_rad[sp.b] - track.theta_rad[sp.a]) * 180.0 / M_PI;
    const double angle_rad = std::abs(mv.angle_deg) * M_PI / 180.0;
    mv.radius_m = angle_rad > 1e-6 ? mv.length_m / angle_rad : 0.0;
    out.push_back(mv);
    cursor = sp.b;
  }
  if (cursor + 1 < track.size())
    push_straight(cursor, track.size() - 1);

  return out;
}

}  // namespace mapmatch
}  // namespace adas

namespace adas {
namespace mapmatch {
YawDiagnostics analyzeYaw(const std::vector<double>& t_s, const std::vector<double>& speed_mps,
                          const std::vector<double>& yaw_rate_rps, const ImuSamples& imu, const TrackConfig& cfg)
{
  YawDiagnostics d;
  const std::size_t n = std::min({t_s.size(), speed_mps.size(), yaw_rate_rps.size()});
  if (n < 10)
    return d;
  const auto count = static_cast<std::ptrdiff_t>(n);
  const std::vector<double> t(t_s.begin(), t_s.begin() + count);
  const std::vector<double> v(speed_mps.begin(), speed_mps.begin() + count);
  const std::vector<double> w(yaw_rate_rps.begin(), yaw_rate_rps.begin() + count);

  const auto stops = findStops(t, v, cfg.stop_speed_mps, cfg.stop_min_s);
  d.n_stops = static_cast<int>(stops.size());
  const double kRad2Deg = 180.0 / M_PI;

  double sum = 0.0;
  int cnt = 0;
  for (const auto& [a, b] : stops)
    for (std::size_t k = a; k <= b; ++k) {
      sum += w[k];
      ++cnt;
    }
  d.bias_chassis_deg_s = cnt > 0 ? sum / cnt * kRad2Deg : 0.0;

  std::vector<double> wi;
  const std::vector<double> raw = imuYawRate(imu, t, v, cfg.stop_speed_mps);
  if (!raw.empty()) {
    d.has_imu = true;
    wi = resample(imu.t_s, raw, t);
    double num = 0.0;
    for (std::size_t i = 0; i < n; ++i)
      if (v[i] > cfg.stop_speed_mps)
        num += wi[i] * w[i];
    if (num < 0.0)
      for (auto& x : wi)
        x = -x;

    sum = 0.0;
    cnt = 0;
    for (const auto& [a, b] : stops)
      for (std::size_t k = a; k <= b; ++k) {
        sum += wi[k];
        ++cnt;
      }
    d.bias_imu_deg_s = cnt > 0 ? sum / cnt * kRad2Deg : 0.0;

    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      if (v[i] <= cfg.stop_speed_mps)
        continue;
      const double a1 = w[i] - d.bias_chassis_deg_s / kRad2Deg;
      const double b1 = wi[i] - d.bias_imu_deg_s / kRad2Deg;
      sxy += a1 * b1;
      sxx += a1 * a1;
      syy += b1 * b1;
    }
    d.corr_chassis_imu = (sxx > 0 && syy > 0) ? sxy / std::sqrt(sxx * syy) : 0.0;
    d.scale_chassis_imu = syy > 0 ? std::sqrt(sxx / syy) : 0.0;
  }

  auto integrate = [&](const std::vector<double>& src, double bias) {
    double th = 0.0;
    for (std::size_t i = 1; i < n; ++i) {
      const double dt = std::clamp(t[i] - t[i - 1], 0.0, 0.5);
      if (v[i] >= cfg.min_speed_mps)
        th += (src[i] - bias) * dt;
    }
    return th * kRad2Deg;
  };
  d.total_turn_chassis_deg = integrate(w, 0.0);
  if (!wi.empty())
    d.total_turn_imu_deg = integrate(wi, d.bias_imu_deg_s / kRad2Deg);
  return d;
}

}  // namespace mapmatch
}  // namespace adas
