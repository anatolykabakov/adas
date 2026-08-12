#include "adas/lateral/visionpilot_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace visionpilot {
using Eigen::VectorXd;

namespace {
constexpr double K_us = 0.0015;
constexpr double kDeltaMax = 0.442;
constexpr double DS_PREVIEW = 0.5;

inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

double eval_cost(const Params& p, const std::vector<double>& delta, double cte0, double epsi0, double Lf, double ds,
                 const VectorXd& v_schedule, const VectorXd& kappa_schedule)
{
  const int N = static_cast<int>(p.N);
  const double v_avg = std::max(v_schedule[0], 0.5);
  const double v2 = v_avg * v_avg;

  double max_kappa = 0.0;
  for (int s = 0; s < N; ++s)
    max_kappa = std::max(max_kappa, std::abs(kappa_schedule[s]));

  const double curve_factor = 1.0 + 20.0 * max_kappa;
  const double cte_weight = p.cte_weight_base * curve_factor;
  const double epsi_weight = 10.0 * curve_factor;
  const double delta_weight = 45000.0;
  const double ddelta_weight = (15000.0 * clampd(v2, 9.0, 25.0));

  double cost = 0.0;
  double cte = cte0;
  double epsi = epsi0;

  for (int s = 0; s < N; s++) {
    cost += cte_weight * cte * cte;
    cost += cte_weight * p.cte_quartic_scale * cte * cte * cte * cte;
    cost += epsi_weight * epsi * epsi;

    if (s + 1 >= N)
      break;

    const double k = kappa_schedule[s];
    const double delta_ff = p.ff_scale * (std::atan(Lf * k) + (K_us * v2 * k));
    const double e = delta[s] - delta_ff;
    cost += delta_weight * e * e;

    if (s + 2 < N) {
      const double dd = delta[s + 1] - delta[s];
      cost += ddelta_weight * dd * dd;
    }

    const double kappa_cmd = std::tan(delta[s]) / Lf;
    cte = cte - std::sin(epsi) * ds;
    epsi = epsi + (k - kappa_cmd) * ds;
  }
  return cost;
}

}  // namespace

LateralPlanner::LateralPlanner(Params params) : params_(params) {}
LateralPlanner::~LateralPlanner() = default;

std::vector<double> LateralPlanner::compute_steering(const double Lf, const VectorXd& state, const VectorXd& v_schedule,
                                                     const VectorXd& kappa_schedule)
{
  const Params& p = params_;
  const std::size_t N = p.N;
  const double v_curr = v_schedule[0];

  if (std::abs(v_curr) < 0.2) {
    return std::vector<double>(N - 1, 0.0);
  }

  const double ds = std::max(0.30, (v_curr * 1.0) / static_cast<double>(N));

  const double cte_raw = state[0];
  const double epsi_raw = state[1];
  const double kappa_road = state[2];

  const double delta_ff_0 = std::atan(Lf * kappa_road);
  const double kappa_cmd_0 = std::tan(delta_ff_0) / Lf;

  const double cte = cte_raw - std::sin(epsi_raw) * ds;
  const double epsi = epsi_raw + (kappa_road - kappa_cmd_0) * ds;

  const double v2 = v_curr * v_curr;

  const double cte_gain = std::max(p.cte_gain_base / (1.0 + v2), p.cte_gain_floor);
  const double delta_fb = clampd(cte_gain * cte + p.epsi_gain * epsi, -0.25, 0.25);
  std::vector<double> delta(N - 1);
  for (size_t i = 0; i < N - 1; ++i) {
    double k = kappa_schedule[static_cast<int>(i)];
    double delta_ff = p.ff_scale * (std::atan(Lf * k) + (K_us * v2 * k));
    delta[i] = clampd(delta_ff + delta_fb, -kDeltaMax, kDeltaMax);
  }

  double best = eval_cost(p, delta, cte, epsi, Lf, ds, v_schedule, kappa_schedule);
  double step = 0.08;
  constexpr int kMaxIters = 80;
  constexpr double kFdEps = 1e-4;
  std::vector<double> grad(N - 1);

  for (int it = 0; it < kMaxIters; ++it) {
    for (size_t i = 0; i < N - 1; ++i) {
      const double d0 = delta[i];
      delta[i] = clampd(d0 + kFdEps, -kDeltaMax, kDeltaMax);
      const double cp = eval_cost(p, delta, cte, epsi, Lf, ds, v_schedule, kappa_schedule);
      delta[i] = clampd(d0 - kFdEps, -kDeltaMax, kDeltaMax);
      const double cm = eval_cost(p, delta, cte, epsi, Lf, ds, v_schedule, kappa_schedule);
      delta[i] = d0;
      grad[i] = (cp - cm) / (2.0 * kFdEps);
    }

    bool improved = false;
    auto trial = delta;
    double local_step = step;
    for (int bt = 0; bt < 8; ++bt) {
      for (size_t i = 0; i < N - 1; ++i)
        trial[i] = clampd(delta[i] - local_step * grad[i], -kDeltaMax, kDeltaMax);
      const double c = eval_cost(p, trial, cte, epsi, Lf, ds, v_schedule, kappa_schedule);
      if (c < best) {
        delta = trial;
        best = c;
        step = std::min(local_step * 1.2, 0.2);
        improved = true;
        break;
      }
      local_step *= 0.5;
    }
    if (!improved)
      break;
  }

  return delta;
}

Eigen::VectorXd build_kappa_schedule(double Lf, double epsi, double kappa, double dkappa_ds, std::size_t N)
{
  const double KAPPA_MAX = std::tan(0.436332) / Lf;
  Eigen::VectorXd kappa_schedule(static_cast<int>(N));
  const double c1 = std::tan(epsi);
  const double c2 = 0.5 * kappa;
  const double c3 = dkappa_ds / 6.0;
  double x = 0.0;
  for (int i = 0; i < static_cast<int>(N); i++) {
    double yp = c1 + 2.0 * c2 * x + 3.0 * c3 * x * x;
    double ypp = 2.0 * c2 + 6.0 * c3 * x;
    double k = ypp / std::pow(1.0 + yp * yp, 1.5);
    kappa_schedule[i] = clampd(k, -KAPPA_MAX, KAPPA_MAX);
    x += DS_PREVIEW;
  }
  return kappa_schedule;
}

double KappaRateFilter::update(double kappa, double ego_v, double dt)
{
  constexpr double KAPPA_RATE_GAIN = 1.0;
  constexpr double RATE_ALPHA = 0.6;
  constexpr double RATE_MAX = 0.15;

  double dkappa_ds = 0.0;
  if (has_prev_) {
    const double ds_cycle = std::max(1e-3, ego_v * dt);
    double raw = (kappa - prev_kappa_) / ds_cycle;
    raw = clampd(raw, -RATE_MAX, RATE_MAX);
    dkappa_filt_ = RATE_ALPHA * raw + (1.0 - RATE_ALPHA) * dkappa_filt_;
    dkappa_ds = KAPPA_RATE_GAIN * dkappa_filt_;
  }
  prev_kappa_ = kappa;
  has_prev_ = true;
  return dkappa_ds;
}

void KappaRateFilter::reset()
{
  prev_kappa_ = 0.0;
  dkappa_filt_ = 0.0;
  has_prev_ = false;
}

}  // namespace visionpilot
