#include "adas/longitudinal/long_mpc.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstdlib>

namespace adas {
namespace longitudinal {
namespace {
std::array<double, kMpcNodes> makeMpcTimes()
{
  std::array<double, kMpcNodes> t{};
  for (int i = 0; i < kMpcNodes; ++i)
    t[i] = 10.0 * std::pow(static_cast<double>(i) / kMpcN, 2);
  return t;
}

std::array<double, kModelT> makeModelTimes()
{
  std::array<double, kModelT> t{};
  for (int i = 0; i < kModelT; ++i)
    t[i] = 10.0 * std::pow(static_cast<double>(i) / (kModelT - 1), 2);
  return t;
}

// Residual layout per node i: [obstacle, x, v, a, a_change] (+ [j] for i < N) then the four soft
// constraints for i < N. Rows with zero weight are still emitted as zeros — the layout stays fixed.
constexpr int kCostRows = 6;
constexpr int kConstrRows = 4;
constexpr int kRows = kMpcNodes * kCostRows + kMpcN * kConstrRows;

bool finite(const double* v, int n)
{
  for (int i = 0; i < n; ++i)
    if (!std::isfinite(v[i]))
      return false;
  return true;
}

}  // namespace

const std::array<double, kMpcNodes>& mpcTimes()
{
  static const auto t = makeMpcTimes();
  return t;
}

const std::array<double, kModelT>& modelTimes()
{
  static const auto t = makeModelTimes();
  return t;
}

double interp(double x, const double* xp, const double* fp, int n)
{
  if (n <= 0)
    return 0.0;
  if (x <= xp[0])
    return fp[0];
  if (x >= xp[n - 1])
    return fp[n - 1];
  int i = 1;
  while (i < n - 1 && xp[i] < x)
    ++i;
  const double span = xp[i] - xp[i - 1];
  const double t = span > 1e-12 ? (x - xp[i - 1]) / span : 0.0;
  return fp[i - 1] + t * (fp[i] - fp[i - 1]);
}

double safeObstacleDistance(double v_ego, const LongMpcConfig& cfg, double t_follow)
{
  return v_ego * v_ego / (2.0 * cfg.comfort_brake) + t_follow * v_ego + cfg.stop_distance;
}

double stoppedEquivalenceFactor(double v_lead, const LongMpcConfig& cfg)
{
  return v_lead * v_lead / (2.0 * cfg.comfort_brake);
}

double desiredFollowDistance(double v_ego, double v_lead, const LongMpcConfig& cfg)
{
  return safeObstacleDistance(v_ego, cfg, cfg.t_follow) - stoppedEquivalenceFactor(v_lead, cfg);
}

LeadTrajectory extrapolateLead(double x_lead, double v_lead, double a_lead, double a_lead_tau)
{
  const auto& T = mpcTimes();
  LeadTrajectory out;
  double v = v_lead;
  double x = x_lead;
  for (int i = 0; i < kMpcNodes; ++i) {
    const double dt = i == 0 ? 0.0 : T[i] - T[i - 1];
    const double a = a_lead * std::exp(-a_lead_tau * T[i] * T[i] / 2.0);
    v = std::max(0.0, v + dt * a);
    x = x + dt * v;
    out.v[i] = v;
    out.x[i] = x;
  }
  return out;
}

LongMpc::LongMpc(LongMpcConfig cfg) : cfg_(cfg) { reset(); }

void LongMpc::reset()
{
  u_.fill(0.0);
  x_sol_.fill(0.0);
  v_sol_.fill(0.0);
  a_sol_.fill(0.0);
  j_sol_.fill(0.0);
  prev_a_.fill(0.0);
  crash_cnt_ = 0;
  iters_ = 0;
  cost_ = 0.0;
  source_ = "cruise";
}

void LongMpc::setAccelLimits(double a_min, double a_max)
{
  a_min_ = a_min;
  a_max_ = a_max;
}

void LongMpc::setCurState(double v_ego, double a_ego)
{
  // A large speed jump means the previous solution describes another situation; start over from it.
  if (std::abs(x0_v_ - v_ego) > 2.0)
    u_.fill(0.0);
  x0_v_ = v_ego;
  x0_a_ = a_ego;
}

void LongMpc::setWeights(bool prev_accel_constraint) { prev_accel_constraint_ = prev_accel_constraint; }

void LongMpc::rollout(const std::array<double, kMpcN>& u, std::array<double, kMpcNodes>& x,
                      std::array<double, kMpcNodes>& v, std::array<double, kMpcNodes>& a) const
{
  const auto& T = mpcTimes();
  x[0] = 0.0;
  v[0] = x0_v_;
  a[0] = x0_a_;
  for (int k = 0; k < kMpcN; ++k) {
    // Exact integration of constant jerk over the interval — the plant is a triple integrator.
    const double dt = T[k + 1] - T[k];
    x[k + 1] = x[k] + v[k] * dt + 0.5 * a[k] * dt * dt + u[k] * dt * dt * dt / 6.0;
    v[k + 1] = v[k] + a[k] * dt + 0.5 * u[k] * dt * dt;
    a[k + 1] = a[k] + u[k] * dt;
  }
}

double LongMpc::residuals(const std::array<double, kMpcN>& u, const Params& p, double* r, double* J) const
{
  const auto& T = mpcTimes();
  std::array<double, kMpcNodes> x{}, v{}, a{};
  rollout(u, x, v, a);

  // Sensitivities of node i to jerk k < i: the increment the interval leaves, propagated ballistically.
  // dx/du, dv/du, da/du are constants of the grid, so this is the only place they are needed.
  auto sens = [&](int i, int k, double& dx, double& dv, double& da) {
    if (k >= i) {
      dx = dv = da = 0.0;
      return;
    }
    const double d = T[k + 1] - T[k];
    const double tau = T[i] - T[k + 1];
    da = d;
    dv = 0.5 * d * d + d * tau;
    dx = d * d * d / 6.0 + 0.5 * d * d * tau + 0.5 * d * tau * tau;
  };

  const double w_obs = std::sqrt(cfg_.x_ego_obstacle_cost);
  const double w_x = std::sqrt(cfg_.x_ego_cost);
  const double w_v = std::sqrt(cfg_.v_ego_cost);
  const double w_a = std::sqrt(cfg_.a_ego_cost);
  const double w_j = std::sqrt(cfg_.j_ego_cost);
  const double a_change = prev_accel_constraint_ ? cfg_.a_change_cost : 0.0;
  const double fade_x[3] = {0.0, 1.0, 2.0};
  const double fade_y[3] = {1.0, 1.0, 0.0};
  const double z_limit = std::sqrt(cfg_.limit_cost);
  const double z_danger = std::sqrt(cfg_.danger_zone_cost);

  auto row = [&](int idx, double value, const double* dstate /* d/dx, d/dv, d/da */, int node, double dj, int jk) {
    r[idx] = value;
    if (!J)
      return;
    double* Jr = J + idx * kMpcN;
    for (int k = 0; k < kMpcN; ++k) {
      double dx, dv, da;
      sens(node, k, dx, dv, da);
      Jr[k] = dstate ? dstate[0] * dx + dstate[1] * dv + dstate[2] * da : 0.0;
      if (k == jk)
        Jr[k] += dj;
    }
  };

  // acados scales each stage's cost (and slack penalty) by its time step and the terminal one by 1:
  // on this grid the first interval is 0.07 s and the last 1.6 s, so an early jerk is cheap and a
  // late gap is dear. Without the same scaling the plan comes out far gentler than upstream's.
  auto stage_scale = [&](int i) { return std::sqrt(i < kMpcN ? T[i + 1] - T[i] : 1.0); };

  double cost = 0.0;
  int idx = 0;
  for (int i = 0; i < kMpcNodes; ++i) {
    const double sc = stage_scale(i);
    const double sd = safeObstacleDistance(v[i], cfg_, cfg_.t_follow);
    const double dsd_dv = v[i] / cfg_.comfort_brake + cfg_.t_follow;
    const double h = v[i] + 10.0;
    const double g = (p.x_obstacle[i] - x[i]) - sd;
    // obstacle term: ((x_obs − x) − safe_dist(v)) / (v + 10)
    {
      const double w = sc * w_obs;
      const double d[3] = {w * (-1.0 / h), w * ((-dsd_dv) * h - g) / (h * h), 0.0};
      row(idx++, w * g / h, d, i, 0.0, -1);
    }
    {
      const double d[3] = {sc * w_x, 0.0, 0.0};
      row(idx++, sc * w_x * x[i], d, i, 0.0, -1);
    }
    {
      const double d[3] = {0.0, sc * w_v, 0.0};
      row(idx++, sc * w_v * v[i], d, i, 0.0, -1);
    }
    {
      const double d[3] = {0.0, 0.0, sc * w_a};
      row(idx++, sc * w_a * a[i], d, i, 0.0, -1);
    }
    {
      const double w_ac = sc * std::sqrt(a_change * interp(T[i], fade_x, fade_y, 3));
      const double d[3] = {0.0, 0.0, w_ac};
      row(idx++, w_ac * (a[i] - p.prev_a[i]), d, i, 0.0, -1);
    }
    if (i < kMpcN) {
      row(idx++, sc * w_j * u[i], nullptr, i, sc * w_j, i);
    } else {
      row(idx++, 0.0, nullptr, i, 0.0, -1);
    }
  }
  for (int i = 0; i < kMpcN; ++i) {
    const double sc = stage_scale(i);
    const double zl = sc * z_limit;
    const double zd = sc * z_danger;
    const double sd = safeObstacleDistance(v[i], cfg_, cfg_.t_follow);
    const double dsd_dv = v[i] / cfg_.comfort_brake + cfg_.t_follow;
    const double h = v[i] + 10.0;
    // c0: v ≥ 0
    if (v[i] < 0.0) {
      const double d[3] = {0.0, -zl, 0.0};
      row(idx++, zl * (-v[i]), d, i, 0.0, -1);
    } else {
      row(idx++, 0.0, nullptr, i, 0.0, -1);
    }
    // c1: a − a_min ≥ 0
    if (a[i] < p.a_min) {
      const double d[3] = {0.0, 0.0, -zl};
      row(idx++, zl * (p.a_min - a[i]), d, i, 0.0, -1);
    } else {
      row(idx++, 0.0, nullptr, i, 0.0, -1);
    }
    // c2: a_max − a ≥ 0
    if (a[i] > p.a_max) {
      const double d[3] = {0.0, 0.0, zl};
      row(idx++, zl * (a[i] - p.a_max), d, i, 0.0, -1);
    } else {
      row(idx++, 0.0, nullptr, i, 0.0, -1);
    }
    // c3: ((x_obs − x) − ldf·safe_dist) / (v + 10) ≥ 0 — the danger zone
    const double g = (p.x_obstacle[i] - x[i]) - p.lead_danger_factor * sd;
    if (g < 0.0) {
      const double d[3] = {zd * (1.0 / h), zd * ((p.lead_danger_factor * dsd_dv) * h + g) / (h * h), 0.0};
      row(idx++, zd * (-g / h), d, i, 0.0, -1);
    } else {
      row(idx++, 0.0, nullptr, i, 0.0, -1);
    }
  }
  for (int k = 0; k < kRows; ++k)
    cost += r[k] * r[k];
  return cost;
}

bool LongMpc::solve(const Params& p)
{
  std::vector<double> r(kRows), J(kRows * kMpcN), r_try(kRows);
  std::array<double, kMpcN> u = u_;
  double cost = residuals(u, p, r.data(), J.data());
  if (std::getenv("ADAS_LONG_MPC_DEBUG")) {
    // Finite-difference check of the analytic Jacobian, row by row; prints the worst entry.
    std::vector<double> r2(kRows);
    double worst = 0.0;
    int wr = -1, wk = -1;
    for (int k = 0; k < kMpcN; ++k) {
      auto u2 = u;
      const double h = 1e-6;
      u2[k] += h;
      residuals(u2, p, r2.data(), nullptr);
      for (int row = 0; row < kRows; ++row) {
        const double fd = (r2[row] - r[row]) / h;
        const double err = std::abs(fd - J[row * kMpcN + k]);
        if (err > worst) {
          worst = err;
          wr = row;
          wk = k;
        }
      }
    }
    std::fprintf(stderr, "long_mpc: jacobian worst err %.3g at row %d (node %d, kind %d) col %d: J=%.4f\n", worst, wr,
                 wr < kMpcNodes * kCostRows ? wr / kCostRows : -1, wr < kMpcNodes * kCostRows ? wr % kCostRows : -1, wk,
                 wr >= 0 ? J[wr * kMpcN + wk] : 0.0);
  }
  double lambda = 1e-3;
  int it = 0;
  for (; it < cfg_.max_iterations; ++it) {
    // Normal equations with Levenberg damping.
    double H[kMpcN][kMpcN];
    double g[kMpcN];
    for (int a = 0; a < kMpcN; ++a) {
      g[a] = 0.0;
      for (int b = 0; b < kMpcN; ++b)
        H[a][b] = 0.0;
    }
    for (int row = 0; row < kRows; ++row) {
      const double* Jr = &J[row * kMpcN];
      for (int a = 0; a < kMpcN; ++a) {
        if (Jr[a] == 0.0)
          continue;
        g[a] += Jr[a] * r[row];
        for (int b = 0; b < kMpcN; ++b)
          H[a][b] += Jr[a] * Jr[b];
      }
    }
    bool improved = false;
    for (int attempt = 0; attempt < 6 && !improved; ++attempt) {
      double A[kMpcN][kMpcN];
      double rhs[kMpcN];
      for (int a = 0; a < kMpcN; ++a) {
        rhs[a] = -g[a];
        for (int b = 0; b < kMpcN; ++b)
          A[a][b] = H[a][b];
        A[a][a] += lambda * (H[a][a] + 1e-6);
      }
      // Gaussian elimination with partial pivoting on the 12×12 system.
      bool singular = false;
      for (int c = 0; c < kMpcN && !singular; ++c) {
        int piv = c;
        for (int rr = c + 1; rr < kMpcN; ++rr)
          if (std::abs(A[rr][c]) > std::abs(A[piv][c]))
            piv = rr;
        if (std::abs(A[piv][c]) < 1e-14) {
          singular = true;
          break;
        }
        if (piv != c) {
          std::swap(A[piv], A[c]);
          std::swap(rhs[piv], rhs[c]);
        }
        for (int rr = c + 1; rr < kMpcN; ++rr) {
          const double f = A[rr][c] / A[c][c];
          for (int cc = c; cc < kMpcN; ++cc)
            A[rr][cc] -= f * A[c][cc];
          rhs[rr] -= f * rhs[c];
        }
      }
      if (singular) {
        lambda *= 10.0;
        continue;
      }
      std::array<double, kMpcN> step{};
      for (int c = kMpcN - 1; c >= 0; --c) {
        double s = rhs[c];
        for (int cc = c + 1; cc < kMpcN; ++cc)
          s -= A[c][cc] * step[cc];
        step[c] = s / A[c][c];
      }
      // Backtracking along the Gauss–Newton direction. The stiff bound penalties (1e6) make the full
      // step overshoot into a constraint whose row was inactive when the step was built; halving the
      // step until the cost drops recovers in a few tries where inflating λ alone would crawl.
      double alpha = 1.0;
      for (int ls = 0; ls < 8 && !improved; ++ls, alpha *= 0.5) {
        std::array<double, kMpcN> u_try = u;
        for (int k = 0; k < kMpcN; ++k)
          u_try[k] += alpha * step[k];
        const double cost_try = residuals(u_try, p, r_try.data(), nullptr);
        if (std::isfinite(cost_try) && cost_try < cost) {
          u = u_try;
          improved = true;
          lambda = std::max(1e-6, lambda * 0.3);
          const double rel = (cost - cost_try) / std::max(cost, 1e-9);
          cost = residuals(u, p, r.data(), J.data());
          if (rel < 1e-7) {
            ++it;
            goto done;
          }
        }
      }
      if (!improved)
        lambda *= 10.0;
    }
    if (!improved)
      break;
  }
done:
  if (std::getenv("ADAS_LONG_MPC_DEBUG")) {
    std::fprintf(stderr, "long_mpc: iters=%d cost=%.4g lambda=%.2g u=", it, cost, lambda);
    for (int k = 0; k < kMpcN; ++k)
      std::fprintf(stderr, "%.3f ", u[k]);
    std::fprintf(stderr, "\n");
  }
  iters_ = it;
  if (!finite(u.data(), kMpcN)) {
    reset();
    return false;
  }
  u_ = u;
  cost_ = cost;
  rollout(u_, x_sol_, v_sol_, a_sol_);
  j_sol_ = u_;
  return true;
}

bool LongMpc::update(const std::optional<LeadTrajectory>& lead0, const std::optional<LeadTrajectory>& lead1,
                     double v_cruise, bool lead0_believed)
{
  const auto& T = mpcTimes();
  Params p;
  // upstream: params[:,0] = MIN_ACCEL, params[:,1] = max_a — the comfort deceleration only bounds the
  // cruise obstacle below; a lead that brakes hard may still be answered with up to MIN_ACCEL.
  p.a_min = cfg_.min_accel;
  p.a_max = std::min(a_max_, cfg_.max_accel);
  p.lead_danger_factor = cfg_.lead_danger_factor;
  p.prev_a = prev_a_;

  // The "cruise obstacle": where a car already at the set speed would be, kept inside what ego can
  // physically do in that time so the solver never starts outside its own bounds.
  std::array<double, kMpcNodes> cruise_obstacle{};
  {
    double x = 0.0;
    for (int i = 0; i < kMpcNodes; ++i) {
      const double v_lower = x0_v_ + T[i] * a_min_ * 1.05;
      const double v_upper = x0_v_ + T[i] * a_max_ * 1.05;
      const double v_c = std::clamp(v_cruise, v_lower, v_upper);
      const double dt = i == 0 ? 0.0 : T[i] - T[i - 1];
      x += dt * v_c;
      cruise_obstacle[i] = x + safeObstacleDistance(v_c, cfg_, cfg_.t_follow);
    }
  }
  auto obstacle_of = [&](const LeadTrajectory& l, int i) { return l.x[i] + stoppedEquivalenceFactor(l.v[i], cfg_); };

  for (int i = 0; i < kMpcNodes; ++i) {
    double best = cruise_obstacle[i];
    if (lead0)
      best = std::min(best, obstacle_of(*lead0, i));
    if (lead1)
      best = std::min(best, obstacle_of(*lead1, i));
    p.x_obstacle[i] = best;
  }
  {
    source_ = "cruise";
    double best = cruise_obstacle[0];
    if (lead1 && obstacle_of(*lead1, 0) < best) {
      best = obstacle_of(*lead1, 0);
      source_ = "lead1";
    }
    if (lead0 && obstacle_of(*lead0, 0) <= best) {
      source_ = "lead0";
    }
  }

  const bool ok = solve(p);
  if (!ok)
    return false;

  // prev_a for the next tick: this solution read 0.05 s later, the way upstream re-times it.
  std::array<double, kMpcNodes> shifted{};
  for (int i = 0; i < kMpcNodes; ++i)
    shifted[i] = T[i] + 0.05;
  for (int i = 0; i < kMpcNodes; ++i)
    prev_a_[i] = interp(shifted[i], T.data(), a_sol_.data(), kMpcNodes);

  bool crash = false;
  if (lead0 && lead0_believed) {
    for (int i = 0; i < kMpcNodes; ++i) {
      if (T[i] >= 5.0)
        break;
      if (lead0->x[i] - x_sol_[i] < cfg_.crash_distance)
        crash = true;
    }
  }
  crash_cnt_ = crash ? crash_cnt_ + 1 : 0;
  return true;
}

}  // namespace longitudinal
}  // namespace adas
