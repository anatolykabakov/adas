#include "mapmatch/fit.h"

#include <algorithm>
#include <cmath>

#include <limits>

#include <Eigen/Dense>

namespace adas {
namespace mapmatch {
namespace {

thread_local std::vector<double> g_route_x, g_route_y;

double distSqToSegment(double px, double py, double ax, double ay, double bx, double by, double* qx, double* qy)
{
  const double vx = bx - ax, vy = by - ay;
  const double wx = px - ax, wy = py - ay;
  const double vv = vx * vx + vy * vy;
  double t = vv > 1e-12 ? (wx * vx + wy * vy) / vv : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  const double dx = wx - t * vx, dy = wy - t * vy;
  if (qx)
    *qx = ax + t * vx;
  if (qy)
    *qy = ay + t * vy;
  return dx * dx + dy * dy;
}

struct Layout {
  int n_turns = 0;
  int n_straights = 0;
  int n_blocks = 0;
  double block_m = 300.0;
  int size() const { return 5 + n_turns + n_straights + n_blocks; }
  int turn(int j) const { return 5 + j; }
  int straight(int i) const { return 5 + n_turns + i; }
  int drift(int b) const { return 5 + n_turns + n_straights + b; }
};

struct SampleTag {
  int turns_before = 0;
  int straight_id = -1;
};

std::vector<SampleTag> tagSamples(const Track& track, const std::vector<std::size_t>& idx, const Layout& lay)
{
  std::vector<SampleTag> tags(idx.size());
  (void)lay;
  for (std::size_t k = 0; k < idx.size(); ++k) {
    const double s = track.s_m[idx[k]];
    int turns_before = 0;
    int straight_id = -1;
    int t = 0, st = 0;
    for (const auto& mv : track.maneuvers) {
      if (mv.isTurn()) {
        if (s >= mv.s_end_m)
          ++turns_before;
        ++t;
      } else {
        if (s >= mv.s_start_m && s < mv.s_end_m)
          straight_id = st;
        ++st;
      }
    }
    tags[k] = {turns_before, straight_id};
  }
  return tags;
}

void rebuild(const Track& track, const Eigen::VectorXd& p, const Layout& lay, const std::vector<SampleTag>& tags,
             const std::vector<std::size_t>& idx, std::vector<double>& xs, std::vector<double>& ys)
{
  const double x0 = p[0], y0 = p[1], th0 = p[2];
  const double s_v = p[3], s_w = p[4];

  xs.assign(idx.size(), 0.0);
  ys.assign(idx.size(), 0.0);

  double x = x0, y = y0;
  double s_prev = 0.0;
  for (std::size_t k = 0; k < idx.size(); ++k) {
    const std::size_t i = idx[k];
    const double s_here = track.s_m[i];
    double th = th0 + s_w * track.theta_rad[i];
    for (int j = 0; j < tags[k].turns_before; ++j)
      th += p[lay.turn(j)] * M_PI / 180.0;
    double drifted = 0.0;
    for (int b = 0; b < lay.n_blocks; ++b) {
      const double b0 = b * lay.block_m;
      if (s_here <= b0)
        break;
      const double in_block = std::min(s_here - b0, lay.block_m);
      drifted += p[lay.drift(b)] * in_block / 100.0;
    }
    th += drifted * M_PI / 180.0;

    double stretch = s_v;
    if (tags[k].straight_id >= 0)
      stretch *= (1.0 + p[lay.straight(tags[k].straight_id)]);
    const double step = (s_here - s_prev) * stretch;
    s_prev = s_here;

    if (k > 0) {
      x += step * std::cos(th);
      y += step * std::sin(th);
    }
    xs[k] = x;
    ys[k] = y;
  }
}

void sampleByFraction(const std::vector<double>& xs, const std::vector<double>& ys, int n, std::vector<double>& ox,
                      std::vector<double>& oy)
{
  ox.clear();
  oy.clear();
  if (xs.size() < 2)
    return;
  std::vector<double> s(xs.size(), 0.0);
  for (std::size_t i = 1; i < xs.size(); ++i)
    s[i] = s[i - 1] + std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
  const double total = s.back();
  if (total < 1e-6)
    return;
  for (int k = 0; k < n; ++k) {
    const double target = total * static_cast<double>(k) / static_cast<double>(n - 1);
    const auto it = std::lower_bound(s.begin(), s.end(), target);
    const std::size_t i = std::min(xs.size() - 1, static_cast<std::size_t>(std::distance(s.begin(), it)));
    const std::size_t j = i > 0 ? i - 1 : 0;
    const double seg = s[i] - s[j];
    const double w = seg > 1e-9 ? (target - s[j]) / seg : 0.0;
    ox.push_back(xs[j] + w * (xs[i] - xs[j]));
    oy.push_back(ys[j] + w * (ys[i] - ys[j]));
  }
}

bool nearestOnRoute(const std::vector<double>& rx, const std::vector<double>& ry, double x, double y,
                    std::size_t& cursor, std::size_t window, double& nx, double& ny, double& dist)
{
  if (rx.size() < 2)
    return false;
  const std::size_t last = rx.size() - 1;
  const std::size_t from = cursor;
  const std::size_t to = std::min(last, cursor + window);
  double best = std::numeric_limits<double>::max();
  std::size_t best_i = from;
  double bx = 0.0, by = 0.0;
  for (std::size_t i = from; i < to; ++i) {
    double qx = 0.0, qy = 0.0;
    const double d2 = distSqToSegment(x, y, rx[i], ry[i], rx[i + 1], ry[i + 1], &qx, &qy);
    if (d2 < best) {
      best = d2;
      best_i = i;
      bx = qx;
      by = qy;
    }
  }
  if (best == std::numeric_limits<double>::max())
    return false;
  cursor = best_i;
  nx = bx;
  ny = by;
  dist = std::sqrt(best);
  return true;
}

}  // namespace

FitResult fitTrackToRoute(const RoadMap& map, const Track& track, const std::vector<std::uint32_t>& dir_edges,
                          const FitConfig& cfg)
{
  FitResult res;
  if (dir_edges.empty() || track.size() < 4)
    return res;

  std::vector<double> rx, ry, exs, eys;
  for (const std::uint32_t de : dir_edges) {
    map.edgePolyline(de >> 1, exs, eys);
    if ((de & 1u) != 0u) {
      std::reverse(exs.begin(), exs.end());
      std::reverse(eys.begin(), eys.end());
    }
    for (std::size_t i = (rx.empty() ? 0 : 1); i < exs.size(); ++i) {
      rx.push_back(exs[i]);
      ry.push_back(eys[i]);
    }
  }
  if (rx.size() < 2)
    return res;

  constexpr int kSamples = 200;
  std::vector<double> px, py, qx, qy;
  sampleByFraction(rx, ry, kSamples, px, py);
  sampleByFraction(track.x_m, track.y_m, kSamples, qx, qy);
  if (px.size() != qx.size() || px.empty())
    return res;

  double pcx = 0.0, pcy = 0.0, qcx = 0.0, qcy = 0.0;
  for (std::size_t i = 0; i < px.size(); ++i) {
    pcx += px[i];
    pcy += py[i];
    qcx += qx[i];
    qcy += qy[i];
  }
  const double n = static_cast<double>(px.size());
  pcx /= n;
  pcy /= n;
  qcx /= n;
  qcy /= n;

  double num = 0.0, den = 0.0;
  for (std::size_t i = 0; i < px.size(); ++i) {
    const double ax = qx[i] - qcx, ay = qy[i] - qcy;
    const double bx = px[i] - pcx, by = py[i] - pcy;
    num += ax * by - ay * bx;
    den += ax * bx + ay * by;
  }
  const double theta = std::atan2(num, den);
  const double c = std::cos(theta), s = std::sin(theta);
  const double x0 = pcx - (c * qcx - s * qcy);
  const double y0 = pcy - (s * qcx + c * qcy);

  g_route_x = rx;
  g_route_y = ry;
  FitResult r = fitTrack(map, track, x0, y0, theta, cfg);
  g_route_x.clear();
  g_route_y.clear();
  return r;
}

FitResult fitTrack(const RoadMap& map, const Track& track, double x0_m, double y0_m, double heading_rad,
                   const FitConfig& cfg)
{
  FitResult res;
  if (map.empty() || track.size() < 4)
    return res;

  const double ds = track.s_m[1] - track.s_m[0];
  const int stride = std::max(1, static_cast<int>(std::lround(cfg.sample_m / std::max(ds, 1e-6))));
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < track.size(); i += static_cast<std::size_t>(stride))
    idx.push_back(i);
  if (idx.size() < 6)
    return res;

  Layout lay;
  for (const auto& mv : track.maneuvers)
    (mv.isTurn() ? lay.n_turns : lay.n_straights)++;
  lay.block_m = std::max(50.0, cfg.drift_block_m);
  lay.n_blocks = static_cast<int>(std::ceil(track.lengthM() / lay.block_m));
  const auto tags = tagSamples(track, idx, lay);

  const int np = lay.size();
  Eigen::VectorXd p = Eigen::VectorXd::Zero(np);
  p[0] = x0_m;
  p[1] = y0_m;
  p[2] = heading_rad;
  p[3] = 1.0;
  p[4] = 1.0;

  const int n_geo = 2 * static_cast<int>(idx.size());
  const int n_reg = 2 + lay.n_turns + lay.n_straights + lay.n_blocks;
  const int nr = n_geo + n_reg;

  std::vector<double> xs, ys;
  double sigma_road = cfg.sigma_road_m;
  double search_m = 250.0;

  const bool on_route = !g_route_x.empty();
  const std::size_t window = 400;

  auto residuals = [&](const Eigen::VectorXd& q, Eigen::VectorXd& r) {
    const double geo_w = 1.0 / (sigma_road * std::sqrt(static_cast<double>(idx.size())));
    rebuild(track, q, lay, tags, idx, xs, ys);
    r.setZero(nr);
    std::size_t cursor = 0;
    for (std::size_t k = 0; k < idx.size(); ++k) {
      double nx = 0.0, ny = 0.0, d = -1.0;
      std::uint32_t ei = 0xFFFFFFFFu;
      const bool found = on_route ? nearestOnRoute(g_route_x, g_route_y, xs[k], ys[k], cursor, window, nx, ny, d) :
                                    map.nearestPoint(xs[k], ys[k], nx, ny, d, ei, search_m);
      if (!found) {
        continue;
      }
      double wx = nx - xs[k];
      double wy = ny - ys[k];
      const double huber = std::max(cfg.max_residual_m, 2.0 * sigma_road);
      if (d > huber) {
        const double scale = std::sqrt(huber / d);
        wx *= scale;
        wy *= scale;
      }
      r[2 * k] = wx * geo_w;
      r[2 * k + 1] = wy * geo_w;
    }
    int j = n_geo;
    r[j++] = (q[3] - 1.0) / cfg.sigma_speed_scale;
    r[j++] = (q[4] - 1.0) / cfg.sigma_yaw_scale;
    for (int t = 0; t < lay.n_turns; ++t)
      r[j++] = q[lay.turn(t)] / cfg.sigma_turn_deg;
    for (int s = 0; s < lay.n_straights; ++s)
      r[j++] = q[lay.straight(s)] / cfg.sigma_straight_rel;
    for (int b = 0; b < lay.n_blocks; ++b)
      r[j++] = q[lay.drift(b)] / cfg.sigma_drift_deg_per_100m;
  };

  Eigen::VectorXd r(nr), r_try(nr);
  Eigen::MatrixXd J(nr, np);
  int it = 0;

  const int steps = std::max(1, cfg.anneal_steps);
  for (int step = 0; step < steps; ++step) {
    const double f =
        std::pow(std::max(1.0, cfg.anneal_start_scale), 1.0 - static_cast<double>(step) / std::max(1, steps - 1));
    sigma_road = cfg.sigma_road_m * f;
    search_m = std::max(250.0, 20.0 * sigma_road);
    residuals(p, r);
    double cost = r.squaredNorm();
    double lambda = 1e-3;
    for (int inner = 0; inner < cfg.iterations; ++inner) {
      ++it;
      for (int c = 0; c < np; ++c) {
        double scale = 1e-2;
        if (c < 2)
          scale = 0.5;
        else if (c == 2)
          scale = 1e-3;
        else if (c < 5)
          scale = 1e-3;
        else if (c >= lay.drift(0))
          scale = 1e-2;
        Eigen::VectorXd q = p;
        q[c] += scale;
        residuals(q, r_try);
        J.col(c) = (r_try - r) / scale;
      }

      Eigen::MatrixXd H = J.transpose() * J;
      const Eigen::VectorXd g = -J.transpose() * r;
      bool improved = false;
      for (int tries = 0; tries < 6 && !improved; ++tries) {
        Eigen::MatrixXd Hl = H;
        Hl.diagonal().array() += lambda * (H.diagonal().array() + 1e-9);
        const Eigen::VectorXd dp = Hl.ldlt().solve(g);
        Eigen::VectorXd q = p + dp;
        q[3] = std::clamp(q[3], 0.8, 1.25);
        q[4] = std::clamp(q[4], 0.8, 1.25);
        residuals(q, r_try);
        const double c_try = r_try.squaredNorm();
        if (c_try < cost) {
          p = q;
          r = r_try;
          cost = c_try;
          lambda = std::max(1e-6, lambda * 0.5);
          improved = true;
        } else {
          lambda *= 4.0;
        }
      }
      if (!improved)
        break;
    }
  }

  sigma_road = cfg.sigma_road_m;
  rebuild(track, p, lay, tags, idx, xs, ys);
  std::vector<double> dists;
  dists.reserve(idx.size());
  {
    std::size_t cursor = 0;
    for (std::size_t k = 0; k < idx.size(); ++k) {
      double nx = 0.0, ny = 0.0, d = -1.0;
      std::uint32_t ei = 0xFFFFFFFFu;
      const bool found = on_route ? nearestOnRoute(g_route_x, g_route_y, xs[k], ys[k], cursor, window, nx, ny, d) :
                                    map.nearestPoint(xs[k], ys[k], nx, ny, d, ei, 250.0);
      if (found)
        dists.push_back(d);
    }
  }
  if (dists.empty())
    return res;
  std::vector<double> sorted = dists;
  std::sort(sorted.begin(), sorted.end());

  res.ok = true;
  res.iterations = it;
  res.x0_m = p[0];
  res.y0_m = p[1];
  res.heading_rad = p[2];
  res.speed_scale = p[3];
  res.yaw_rate_scale = p[4];
  for (int t = 0; t < lay.n_turns; ++t)
    res.turn_corr_deg.push_back(p[lay.turn(t)]);
  for (int s = 0; s < lay.n_straights; ++s)
    res.straight_corr.push_back(p[lay.straight(s)]);
  for (int b = 0; b < lay.n_blocks; ++b)
    res.drift_deg_per_100m.push_back(p[lay.drift(b)]);

  double sum2 = 0.0;
  for (const double d : dists)
    sum2 += d * d;
  res.rms_m = std::sqrt(sum2 / static_cast<double>(dists.size()));
  res.median_m = sorted[sorted.size() / 2];
  res.p95_m = sorted[std::min(sorted.size() - 1, static_cast<std::size_t>(0.95 * sorted.size()))];

  double deform = 0.0;
  deform += std::pow((p[3] - 1.0) / cfg.sigma_speed_scale, 2);
  deform += std::pow((p[4] - 1.0) / cfg.sigma_yaw_scale, 2);
  for (const double t : res.turn_corr_deg)
    deform += std::pow(t / cfg.sigma_turn_deg, 2);
  for (const double s : res.straight_corr)
    deform += std::pow(s / cfg.sigma_straight_rel, 2);
  for (const double d : res.drift_deg_per_100m)
    deform += std::pow(d / cfg.sigma_drift_deg_per_100m, 2);
  res.deform_cost = std::sqrt(deform / std::max(1, n_reg));
  res.score = res.rms_m + cfg.sigma_road_m * res.deform_cost;

  std::vector<std::size_t> all(track.size());
  for (std::size_t i = 0; i < track.size(); ++i)
    all[i] = i;
  const auto tags_all = tagSamples(track, all, lay);
  rebuild(track, p, lay, tags_all, all, res.x_m, res.y_m);
  return res;
}

}  // namespace mapmatch
}  // namespace adas
