#include "adas/lateral/flowpilot_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace adas {
namespace flowpilot {
namespace {

constexpr double kMinSpeed = 1.0;
constexpr double kPsiMax = 90.0 * M_PI / 180.0;
constexpr double kRateMax = 50.0 * M_PI / 180.0;
constexpr double kDtMdl = 0.05;
constexpr double kInvalidCost = 20000.0;

inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

}  // namespace

LateralMpc::LateralMpc(LatMpcConfig cfg) : cfg_(std::move(cfg)) { reset(); }

void LateralMpc::reset()
{
  u_.fill(0.0);
  x0_.fill(0.0);
  x0_inited_ = false;
  psi_sol_.fill(0.0);
  r_sol_.fill(0.0);
  has_sol_ = false;
}

double LateralMpc::tNode(int i) { return 10.0 * std::pow(static_cast<double>(i) / kTIdxMax, 2.0); }

void LateralMpc::buildRefs(const std::vector<Vec2>& poly_left, double v, std::array<double, N + 1>& y_ref,
                           std::array<double, N + 1>& psi_ref, std::array<double, N + 1>& r_ref)
{
  y_ref.fill(0.0);
  psi_ref.fill(0.0);
  r_ref.fill(0.0);
  if (poly_left.size() < 2)
    return;

  const size_t n = poly_left.size();
  std::vector<double> s(n, 0.0);
  std::vector<double> heading(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const double dx = poly_left[i].x() - poly_left[i - 1].x();
    const double dy = poly_left[i].y() - poly_left[i - 1].y();
    s[i] = s[i - 1] + std::hypot(dx, dy);
    heading[i] = std::atan2(dy, dx);
  }
  heading[0] = heading[1];
  for (size_t i = 1; i < n; ++i) {
    double d = heading[i] - heading[i - 1];
    while (d > M_PI)
      d -= 2.0 * M_PI;
    while (d < -M_PI)
      d += 2.0 * M_PI;
    heading[i] = heading[i - 1] + d;
  }
  const double s_end = s.back();
  if (s_end < 1e-3)
    return;

  auto sample = [&](double dist, double& y, double& psi) {
    dist = clampd(dist, 0.0, s_end);
    size_t i = 1;
    while (i + 1 < n && s[i] < dist)
      ++i;
    const double s0 = s[i - 1], s1 = s[i];
    const double a = (s1 > s0 + 1e-9) ? (dist - s0) / (s1 - s0) : 0.0;
    y = lerp(poly_left[i - 1].y(), poly_left[i].y(), a);
    psi = lerp(heading[i - 1], heading[i], a);
  };

  for (int i = 0; i <= N; ++i) {
    const double dist = std::min(v * tNode(i), s_end);
    sample(dist, y_ref[i], psi_ref[i]);
  }
  for (int i = 0; i < N; ++i) {
    const double dt = std::max(tNode(i + 1) - tNode(i), 1e-4);
    r_ref[i] = (psi_ref[i + 1] - psi_ref[i]) / dt;
  }
  r_ref[N] = r_ref[N - 1];
}

void LateralMpc::buildRefsWithOrientation(const std::vector<Vec2>& poly_left, double v,
                                          const std::vector<Vec2>& plan_left, const std::vector<double>& plan_yaw_left,
                                          const std::vector<double>& plan_yaw_rate_left,
                                          std::array<double, N + 1>& y_ref, std::array<double, N + 1>& psi_ref,
                                          std::array<double, N + 1>& r_ref)
{
  y_ref.fill(0.0);
  psi_ref.fill(0.0);
  r_ref.fill(0.0);

  const std::vector<Vec2>& y_src = !poly_left.empty() ? poly_left : plan_left;
  if (y_src.size() < 2 || plan_yaw_left.size() < 2 || plan_yaw_rate_left.size() != plan_yaw_left.size())
    return;

  {
    std::array<double, N + 1> psi_unused{}, r_unused{};
    buildRefs(y_src, v, y_ref, psi_unused, r_unused);
  }

  if (plan_yaw_left.size() >= static_cast<size_t>(N + 1) && plan_yaw_rate_left.size() >= static_cast<size_t>(N + 1)) {
    for (int i = 0; i <= N; ++i) {
      psi_ref[i] = plan_yaw_left[static_cast<size_t>(i)];
      r_ref[i] = plan_yaw_rate_left[static_cast<size_t>(i)];
    }
    return;
  }

  const std::vector<Vec2>& arc = plan_left.size() >= 2 ? plan_left : y_src;
  if (arc.size() < 2 || plan_yaw_left.size() != arc.size() || plan_yaw_rate_left.size() != arc.size())
    return;

  const size_t n = arc.size();
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const double dx = arc[i].x() - arc[i - 1].x();
    const double dy = arc[i].y() - arc[i - 1].y();
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  const double s_end = s.back();
  if (s_end < 1e-3)
    return;

  auto interp_at = [&](double dist, const std::vector<double>& vals) {
    dist = clampd(dist, 0.0, s_end);
    size_t i = 1;
    while (i + 1 < n && s[i] < dist)
      ++i;
    const double s0 = s[i - 1], s1 = s[i];
    const double a = (s1 > s0 + 1e-9) ? (dist - s0) / (s1 - s0) : 0.0;
    return lerp(vals[i - 1], vals[i], a);
  };

  for (int i = 0; i <= N; ++i) {
    const double dist = std::min(v * tNode(i), s_end);
    psi_ref[i] = interp_at(dist, plan_yaw_left);
    r_ref[i] = interp_at(dist, plan_yaw_rate_left);
  }
}

void LateralMpc::forward(const std::array<double, N>& u, const std::array<double, X_DIM>& x0, double v,
                         std::array<double, N + 1>& x, std::array<double, N + 1>& y, std::array<double, N + 1>& psi,
                         std::array<double, N + 1>& r) const
{
  x[0] = x0[0];
  y[0] = x0[1];
  psi[0] = x0[2];
  r[0] = x0[3];
  const double rr = cfg_.rotation_radius;
  for (int i = 0; i < N; ++i) {
    const double dt = std::max(tNode(i + 1) - tNode(i), 1e-4);
    const double p = psi[i];
    const double rd = r[i];
    x[i + 1] = x[i] + dt * (v * std::cos(p) - rr * std::sin(p) * rd);
    y[i + 1] = y[i] + dt * (v * std::sin(p) + rr * std::cos(p) * rd);
    psi[i + 1] = clampd(psi[i] + dt * rd, -kPsiMax, kPsiMax);
    r[i + 1] = clampd(r[i] + dt * u[i], -kRateMax, kRateMax);
  }
}

double LateralMpc::evalCost(const std::array<double, N>& u, const std::array<double, X_DIM>& x0, double v,
                            const std::array<double, N + 1>& y_ref, const std::array<double, N + 1>& psi_ref,
                            const std::array<double, N + 1>& r_ref, std::array<double, N + 1>& psi_sol,
                            std::array<double, N + 1>& r_sol) const
{
  std::array<double, N + 1> xs{}, ys{};
  forward(u, x0, v, xs, ys, psi_sol, r_sol);

  const double v_off = v + cfg_.speed_offset;
  double cost = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double ey = ys[i] - y_ref[i];
    const double eh = v_off * (psi_sol[i] - psi_ref[i]);
    const double er = v_off * (r_sol[i] - r_ref[i]);
    cost += cfg_.path_weight * ey * ey;
    cost += cfg_.heading_weight * eh * eh;
    cost += cfg_.lat_accel_weight * er * er;
    if (i < N) {
      const double jerk = v_off * u[i];
      const double srate = u[i] / (v + 0.1);
      cost += cfg_.lat_jerk_weight * jerk * jerk;
      cost += cfg_.steering_rate_weight * srate * srate;
    }
  }
  return cost;
}

double LateralMpc::interpAtTime(double t, const std::array<double, N + 1>& vals)
{
  if (t <= tNode(0))
    return vals[0];
  if (t >= tNode(N))
    return vals[N];
  for (int i = 0; i < N; ++i) {
    const double t0 = tNode(i), t1 = tNode(i + 1);
    if (t1 >= t) {
      const double a = (t - t0) / std::max(t1 - t0, 1e-9);
      return lerp(vals[i], vals[i + 1], clampd(a, 0.0, 1.0));
    }
  }
  return vals[N];
}

double LateralMpc::lagAdjustedCurvature(double v, const std::array<double, N + 1>& psi_sol,
                                        const std::array<double, N + 1>& r_sol, const std::array<double, N>& u,
                                        double dt_s) const
{
  const double delay = std::max(cfg_.steer_delay_s, dt_s);
  const double kappa0 = r_sol[0] / v;
  const double psi_delay = interpAtTime(delay, psi_sol);
  const double avg_kappa = psi_delay / (v * delay);
  double desired = 2.0 * avg_kappa - kappa0;

  const double max_kappa_rate = cfg_.max_lateral_jerk / (v * v);
  const double rate0 = clampd(u[0] / v, -max_kappa_rate, max_kappa_rate);
  (void)rate0;
  desired = clampd(desired, kappa0 - max_kappa_rate * dt_s, kappa0 + max_kappa_rate * dt_s);
  return desired;
}

LatMpcResult LateralMpc::update(double speed_mps, double yaw_rate, double Lf, const std::vector<Vec2>& polyline_ego,
                                const std::vector<Vec2>& plan_poly_device, const std::vector<double>& plan_yaw_device,
                                const std::vector<double>& plan_yaw_rate_device, double dt_s)
{
  LatMpcResult out;
  dt_s = clampd(dt_s, 0.02, 0.5);
  if (polyline_ego.size() < 4 || Lf < 1e-3) {
    out.ok = false;
    return out;
  }

  const double v = std::max(speed_mps, kMinSpeed);

  std::vector<Vec2> poly_left;
  poly_left.reserve(polyline_ego.size());
  for (const auto& p : polyline_ego) {
    if (p.x() < 0.3)
      continue;
    poly_left.emplace_back(p.x(), -p.y());
  }
  if (poly_left.size() < 4) {
    out.ok = false;
    return out;
  }

  std::array<double, N + 1> y_ref{}, psi_ref{}, r_ref{};
  const bool have_orient = plan_yaw_device.size() >= 2 && plan_yaw_rate_device.size() == plan_yaw_device.size() &&
                           (plan_poly_device.size() == plan_yaw_device.size() || plan_poly_device.empty());
  if (have_orient) {
    std::vector<Vec2> plan_left;
    const auto& src = !plan_poly_device.empty() ? plan_poly_device : polyline_ego;
    plan_left.reserve(src.size());
    std::vector<double> yaw_left, rate_left;
    yaw_left.reserve(plan_yaw_device.size());
    rate_left.reserve(plan_yaw_rate_device.size());
    const size_t n_use = std::min({src.size(), plan_yaw_device.size(), plan_yaw_rate_device.size()});
    for (size_t i = 0; i < n_use; ++i) {
      plan_left.emplace_back(src[i].x(), -src[i].y());
      yaw_left.push_back(-plan_yaw_device[i]);
      rate_left.push_back(-plan_yaw_rate_device[i]);
    }
    buildRefsWithOrientation(poly_left, v, plan_left, yaw_left, rate_left, y_ref, psi_ref, r_ref);
  } else {
    buildRefs(poly_left, v, y_ref, psi_ref, r_ref);
  }

  x0_[0] = 0.0;
  x0_[1] = 0.0;
  x0_[2] = 0.0;
  if (!x0_inited_) {
    const double ld = clampd(v * 1.2, 8.0, 25.0);
    const double kappa_seed = clampd(2.0 * y_ref[0] / (ld * ld) + psi_ref[0] / ld, -0.25, 0.25);
    x0_[3] = kappa_seed * v;
    x0_inited_ = true;
  }

  std::array<double, N + 1> psi_sol{}, r_sol{};
  double best = evalCost(u_, x0_, v, y_ref, psi_ref, r_ref, psi_sol, r_sol);
  std::array<double, N> u = u_;

  for (int it = 0; it < cfg_.gd_iters; ++it) {
    std::array<double, N> g{};
    constexpr double eps = 1e-4;
    for (int k = 0; k < N; ++k) {
      auto up = u, um = u;
      up[k] += eps;
      um[k] -= eps;
      const double cp = evalCost(up, x0_, v, y_ref, psi_ref, r_ref, psi_sol, r_sol);
      const double cm = evalCost(um, x0_, v, y_ref, psi_ref, r_ref, psi_sol, r_sol);
      g[k] = (cp - cm) / (2.0 * eps);
    }
    double step = cfg_.gd_step;
    bool improved = false;
    for (int ls = 0; ls < 8; ++ls) {
      std::array<double, N> trial = u;
      for (int k = 0; k < N; ++k)
        trial[k] = clampd(u[k] - step * g[k], -3.0, 3.0);
      const double c = evalCost(trial, x0_, v, y_ref, psi_ref, r_ref, psi_sol, r_sol);
      if (c < best - 1e-9) {
        best = c;
        u = trial;
        u_ = trial;
        improved = true;
        break;
      }
      step *= 0.5;
    }
    if (!improved)
      break;
  }

  evalCost(u_, x0_, v, y_ref, psi_ref, r_ref, psi_sol, r_sol);

  const bool nan_sol = !std::isfinite(r_sol[0]) || !std::isfinite(psi_sol[1]);
  if (nan_sol || best > kInvalidCost) {
    reset();
    x0_[3] = yaw_rate;
    x0_inited_ = true;
    out.ok = false;
    return out;
  }

  psi_sol_ = psi_sol;
  r_sol_ = r_sol;
  has_sol_ = true;

  const double desired_kappa = lagAdjustedCurvature(v, psi_sol, r_sol, u_, dt_s);
  const double max_kappa_rate = cfg_.max_lateral_jerk / (v * v);
  const double desired_rate = clampd(u_[0] / v, -max_kappa_rate, max_kappa_rate);

  x0_[3] = interpAtTime(dt_s, r_sol);

  out.ok = std::isfinite(desired_kappa);
  out.desired_curvature = desired_kappa;
  out.desired_curvature_rate = desired_rate;
  out.steer_rad_vp = std::atan(Lf * desired_kappa);
  out.cost = best;
  if (!out.ok) {
    reset();
    out.desired_curvature = 0.0;
    out.steer_rad_vp = 0.0;
  }
  return out;
}

std::optional<double> LateralMpc::curvatureAtSpeed(double speed_mps, double dt_s) const
{
  if (!has_sol_)
    return std::nullopt;
  const double v = std::max(speed_mps, kMinSpeed);
  const double k = lagAdjustedCurvature(v, psi_sol_, r_sol_, u_, clampd(dt_s, 0.02, 0.5));
  if (!std::isfinite(k))
    return std::nullopt;
  return k;
}

}  // namespace flowpilot
}  // namespace adas
