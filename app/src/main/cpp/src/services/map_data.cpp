#include "adas/services/map_data.h"
#include "adas/utils/proto_convert.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>

#include "adas/mapmatch/dir_edge.h"
#include "adas/utils/logger.h"
#include "adas/utils/math_utils.h"

namespace adas {
namespace services {
namespace {
constexpr double kDeg = M_PI / 180.0;

bool fileExists(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  return in.good();
}

std::vector<std::string> mapPathCandidates(const std::string& path)
{
  if (!path.empty() && path.front() == '/')
    return {path};
  return {
      path,           "/sdcard/adas_maps/" + path, "/storage/emulated/0/adas_maps/" + path,
      "maps/" + path, "../maps/" + path,           "../../../../maps/" + path,
  };
}

}  // namespace

MapData::MapData(Config config) : config_(std::move(config)) {}

bool MapData::loadMap()
{
  if (config_.map_path.empty()) {
    LOGI("MapData: map_path empty — service idle");
    return false;
  }
  for (const auto& candidate : mapPathCandidates(config_.map_path)) {
    if (!fileExists(candidate))
      continue;
    if (map_.load(candidate)) {
      map_resolved_path_ = candidate;
      return true;
    }
    LOGE("MapData: %s exists but did not load", candidate.c_str());
  }
  LOGE("MapData: map '%s' not found — service idle. On the phone this means the asset was not "
       "unpacked (is `map.path` the asset's name?); elsewhere, put it in maps/ or /sdcard/adas_maps/",
       config_.map_path.c_str());
  return false;
}

void MapData::configure()
{
  map_loaded_ = loadMap();

  subscribe<adas::proto::GPSData>(topics::kGpsData, [this](const adas::proto::GPSData& m) { onGpsData(m); });
  subscribe<adas::proto::LocalizationPose>(topics::kLocalizationPose,
                                           [this](const adas::proto::LocalizationPose& m) { onPose(m); });

  const auto period_ms = static_cast<uint64_t>(std::max(1.0, 1000.0 / std::max(0.1, config_.update_hz)));
  scheduleTimer(
      period_ms, [this]() { onTick(); }, "route");

  LOGI("MapData → %s  map=%s (%zu edges) %.1f Hz horizon=%.0fm step=%.1fm window=%.0fm "
       "kappa_th=%.4f a_lat=%.1f",
       topics::kMapLocal, map_loaded_ ? map_resolved_path_.c_str() : "none", map_.edgeCount(), config_.update_hz,
       config_.route.horizon_m, config_.route.step_m, config_.route.window_m, config_.route.turn_kappa,
       config_.route.max_lat_acc);
}

void MapData::reset()
{
  have_anchor_ = false;
  anchor_has_pose_ = false;
  have_pose_ = false;
  have_yaw_ = false;
  last_fix_us_ = 0;
  last_gap_m_ = 0.0;
  last_local_map_us_ = 0;
  ticks_ = 0;
  matched_ticks_ = 0;
  route_ = mapmatch::RouteAhead{};
}

void MapData::onGpsData(const adas::proto::GPSData& payload)
{
  if (!map_loaded_)
    return;
  const double lat = payload.latitude();
  const double lon = payload.longitude();
  if (!std::isfinite(lat) || !std::isfinite(lon) || (std::abs(lat) < 1e-9 && std::abs(lon) < 1e-9))
    return;

  const auto [mx, my] = map_.frame().toLocal(lat, lon);

  if (have_anchor_ && have_pose_) {
    const double pred_x = anchor_map_x_ + (pose_x_ - anchor_pose_x_);
    const double pred_y = anchor_map_y_ + (pose_y_ - anchor_pose_y_);
    last_gap_m_ = std::hypot(pred_x - mx, pred_y - my);
    if (last_gap_m_ > config_.max_pose_gap_m)
      LOGW("MapData: pose drifted %.0f m from GPS between fixes — re-anchoring", last_gap_m_);
  }

  anchor_map_x_ = mx;
  anchor_map_y_ = my;
  anchor_pose_x_ = pose_x_;
  anchor_pose_y_ = pose_y_;
  have_anchor_ = true;
  anchor_has_pose_ = have_pose_;
  last_lat_ = lat;
  last_lon_ = lon;
  last_fix_us_ = payload.timestamp() * 1000;

  gps_speed_mps_ = payload.speed();
  gps_bearing_deg_ = payload.bearing();
  gps_course_valid_ = std::isfinite(gps_bearing_deg_) && gps_speed_mps_ > 2.0;
}

void MapData::onPose(const adas::proto::LocalizationPose& payload)
{
  pose_x_ = payload.x();
  pose_y_ = payload.y();
  pose_yaw_ = payload.yaw();
  pose_v_ = payload.v();

  if (have_anchor_ && !anchor_has_pose_) {
    anchor_pose_x_ = pose_x_;
    anchor_pose_y_ = pose_y_;
    anchor_has_pose_ = true;
  }
  have_pose_ = true;
}

bool MapData::currentPosition(double& x, double& y, double& yaw) const
{
  if (!have_anchor_)
    return false;

  if (have_pose_) {
    x = anchor_map_x_ + (pose_x_ - anchor_pose_x_);
    y = anchor_map_y_ + (pose_y_ - anchor_pose_y_);
    yaw = pose_yaw_;
    return true;
  }

  x = anchor_map_x_;
  y = anchor_map_y_;
  if (!gps_course_valid_)
    return false;
  yaw = yawEnuFromBearingDeg(gps_bearing_deg_);
  return true;
}

void MapData::fillLocalMap(adas::proto::MapLocalState& out, double x, double y)
{
  const double r = config_.local_map_radius_m;
  out.set_has_local_map(true);
  out.set_local_map_radius_m(static_cast<float>(r));

  std::vector<double> xs, ys;
  for (const std::uint32_t ei : map_.edgesInBBox(x - r, y - r, x + r, y + r)) {
    map_.edgePolyline(ei, xs, ys);
    if (xs.size() < 2)
      continue;
    auto* e = out.add_local_edges();
    e->set_name(map_.edgeName(ei));
    for (std::size_t k = 0; k < xs.size(); ++k) {
      e->add_x(static_cast<float>(xs[k]));
      e->add_y(static_cast<float>(ys[k]));
    }
  }
}

void MapData::onTick()
{
  if (!map_loaded_)
    return;

  const auto t_us = static_cast<int64_t>(now());

  MapLocalInputs in;
  in.timestamp_us = t_us;
  in.pose_gps_gap_m = last_gap_m_;

  double x = 0.0, y = 0.0, yaw = 0.0;
  const bool have_pos = currentPosition(x, y, yaw);
  const double speed = have_pose_ ? pose_v_ : gps_speed_mps_;

  const bool fix_fresh = last_fix_us_ != 0 && (t_us - last_fix_us_) < static_cast<int64_t>(config_.max_fix_age_s * 1e6);

  if (!have_pos || !fix_fresh) {
    publish(topics::kMapLocal, createMapLocal(in));
    ++ticks_;
    return;
  }

  if (speed >= config_.min_speed_mps) {
    last_good_yaw_ = yaw;
    have_yaw_ = true;
  } else if (have_yaw_) {
    yaw = last_good_yaw_;
  }

  const auto t0 = now();
  route_ = mapmatch::buildRouteAhead(map_, x, y, yaw, config_.route);
  const auto build_us = now() - t0;

  in.positioned = true;
  in.lat = last_lat_;
  in.lon = last_lon_;
  in.map_x = x;
  in.map_y = y;
  in.yaw = yaw;
  in.speed_mps = speed;
  in.build_ms = build_us / 1000.0;
  in.route_step_m = config_.route.step_m;
  in.route = &route_;

  adas::proto::MapLocalState msg = createMapLocal(in);

  if (route_.matched) {
    ++matched_ticks_;
    const auto period_us = static_cast<int64_t>(config_.local_map_period_s * 1e6);
    if (period_us <= 0 || last_local_map_us_ == 0 || (t_us - last_local_map_us_) >= period_us) {
      fillLocalMap(msg, x, y);
      last_local_map_us_ = t_us;
    }
  }

  publish(topics::kMapLocal, msg);

  if (++ticks_ % 600 == 0) {
    LOGI("MapData: %llu ticks, matched %.0f%%, last match %.1f m on '%s', build %.2f ms",
         static_cast<unsigned long long>(ticks_),
         100.0 * static_cast<double>(matched_ticks_) / static_cast<double>(ticks_), route_.match_dist_m,
         route_.road_name.c_str(), build_us / 1000.0);
  }
}

}  // namespace services
}  // namespace adas
