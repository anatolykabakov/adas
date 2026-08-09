#include "adas/services/map_data.h"

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

/** Where a relative `map_path` may live.
 *
 *  On the phone it normally never gets here: the map ships as an APK asset, Java unpacks it when the
 *  `map_data` node is on and `nativeStart` passes the absolute path. The candidates below are the fallbacks
 *  that keep the service usable without that — the host build running from the repo, and a map pushed by
 *  hand (`adb push maps/Moscow.osm.admap /sdcard/adas_maps/`) when testing a map the APK does not carry. */
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

  subscribe<ai::flow::adas::ZMQMessage>(topics::kGpsData,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onGpsData(m); });
  subscribe<ai::flow::adas::ZMQMessage>(topics::kLocalizationPose,
                                        [this](const ai::flow::adas::ZMQMessage& m) { onPose(m); });

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

void MapData::onGpsData(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_gps_data() || !map_loaded_)
    return;
  const auto& g = msg.gps_data();
  const double lat = g.latitude();
  const double lon = g.longitude();
  if (!std::isfinite(lat) || !std::isfinite(lon) || (std::abs(lat) < 1e-9 && std::abs(lon) < 1e-9))
    return;

  const auto [mx, my] = map_.frame().toLocal(lat, lon);

  // Measure how far the pose had drifted since the previous fix before moving the anchor onto this one.
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
  // If the localizer has not spoken yet there is no pose to anchor against, and the zeros stored above are
  // not a reading — they are the absence of one. `onPose` re-anchors on the first real pose; without that,
  // a fix arriving before the localizer starts would offset every later position by the pose at that moment.
  anchor_has_pose_ = have_pose_;
  last_lat_ = lat;
  last_lon_ = lon;
  last_fix_us_ = g.timestamp() * 1000;

  gps_speed_mps_ = g.speed();
  gps_bearing_deg_ = g.bearing();
  // Bearing is only meaningful while moving, and a stationary phone reports exactly 0 rather than nothing.
  gps_course_valid_ = std::isfinite(gps_bearing_deg_) && gps_speed_mps_ > 2.0;
}

void MapData::onPose(const ai::flow::adas::ZMQMessage& msg)
{
  if (!msg.has_localization_pose())
    return;
  const auto& p = msg.localization_pose();
  pose_x_ = p.x();
  pose_y_ = p.y();
  pose_yaw_ = p.yaw();
  pose_v_ = p.v();

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

  // No localizer: the fix is all there is, and heading has to come from GPS course.
  x = anchor_map_x_;
  y = anchor_map_y_;
  if (!gps_course_valid_)
    return false;
  yaw = yawEnuFromBearingDeg(gps_bearing_deg_);
  return true;
}

void MapData::fillLocalMap(ai::flow::adas::MapLocalState& out, double x, double y)
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
  ai::flow::adas::ZMQMessage zmq;
  zmq.set_timestamp(t_us / 1000);
  zmq.set_topic(topics::kMapLocal);
  auto* m = zmq.mutable_map_local();
  m->set_timestamp(t_us / 1000);
  m->set_map_loaded(true);
  m->set_pose_gps_gap_m(static_cast<float>(last_gap_m_));

  double x = 0.0, y = 0.0, yaw = 0.0;
  const bool have_pos = currentPosition(x, y, yaw);
  const double speed = have_pose_ ? pose_v_ : gps_speed_mps_;

  // A fix that stopped arriving means the position is dead reckoning off an anchor of unknown age. Publish
  // the unmatched message anyway — a gap in the log is indistinguishable from the service being dead.
  const bool fix_fresh = last_fix_us_ != 0 && (t_us - last_fix_us_) < static_cast<int64_t>(config_.max_fix_age_s * 1e6);

  if (!have_pos || !fix_fresh) {
    publish(topics::kMapLocal, zmq);
    ++ticks_;
    return;
  }

  // Heading is the one input a stopped car cannot supply: yaw rate integrates noise and GPS course is
  // meaningless. Hold the last good one rather than letting the route flip to the opposite carriageway.
  if (speed >= config_.min_speed_mps) {
    last_good_yaw_ = yaw;
    have_yaw_ = true;
  } else if (have_yaw_) {
    yaw = last_good_yaw_;
  }

  const auto t0 = now();
  route_ = mapmatch::buildRouteAhead(map_, x, y, yaw, config_.route);
  const auto build_us = now() - t0;

  m->set_lat(last_lat_);
  m->set_lon(last_lon_);
  m->set_map_x(x);
  m->set_map_y(y);
  m->set_yaw(yaw);
  m->set_speed_mps(static_cast<float>(speed));
  m->set_matched(route_.matched);
  m->set_build_ms(static_cast<float>(build_us / 1000.0));

  if (route_.matched) {
    ++matched_ticks_;
    m->set_match_dist_m(static_cast<float>(route_.match_dist_m));
    m->set_heading_delta_deg(static_cast<float>(route_.heading_delta_rad / kDeg));
    m->set_road_name(route_.road_name);
    m->set_route_step_m(static_cast<float>(config_.route.step_m));
    m->set_route_length_m(static_cast<float>(route_.length_m));
    m->set_route_node_spacing_m(static_cast<float>(route_.node_spacing_m));

    m->mutable_route_x()->Reserve(static_cast<int>(route_.x_m_pts.size()));
    m->mutable_route_y()->Reserve(static_cast<int>(route_.y_m_pts.size()));
    m->mutable_route_kappa()->Reserve(static_cast<int>(route_.kappa.size()));
    for (std::size_t i = 0; i < route_.x_m_pts.size(); ++i) {
      m->add_route_x(static_cast<float>(route_.x_m_pts[i]));
      m->add_route_y(static_cast<float>(route_.y_m_pts[i]));
      m->add_route_kappa(static_cast<float>(i < route_.kappa.size() ? route_.kappa[i] : 0.0));
    }

    for (const auto& t : route_.turns) {
      auto* s = m->add_turns();
      s->set_start_m(static_cast<float>(t.start_m));
      s->set_end_m(static_cast<float>(t.end_m));
      s->set_kappa(static_cast<float>(t.kappa));
      s->set_speed_mps(static_cast<float>(t.speed_mps));
      s->set_sign(t.sign);
    }

    // The `liveMapData` summary: the section we are inside, and the first one starting ahead of us.
    for (const auto& t : route_.turns) {
      if (t.start_m <= 0.5 && t.end_m > 0.0) {
        m->set_turn_speed_valid(true);
        m->set_turn_speed_mps(static_cast<float>(t.speed_mps));
        m->set_turn_speed_end_m(static_cast<float>(t.end_m));
        m->set_turn_speed_sign(t.sign);
        break;
      }
    }
    for (const auto& t : route_.turns) {
      if (t.start_m > 0.5) {
        m->set_next_turn_valid(true);
        m->set_next_turn_speed_mps(static_cast<float>(t.speed_mps));
        m->set_next_turn_distance_m(static_cast<float>(t.start_m));
        break;
      }
    }

    const auto period_us = static_cast<int64_t>(config_.local_map_period_s * 1e6);
    if (period_us <= 0 || last_local_map_us_ == 0 || (t_us - last_local_map_us_) >= period_us) {
      fillLocalMap(*m, x, y);
      last_local_map_us_ = t_us;
    }
  }

  publish(topics::kMapLocal, zmq);

  if (++ticks_ % 600 == 0) {
    LOGI("MapData: %llu ticks, matched %.0f%%, last match %.1f m on '%s', build %.2f ms",
         static_cast<unsigned long long>(ticks_),
         100.0 * static_cast<double>(matched_ticks_) / static_cast<double>(ticks_), route_.match_dist_m,
         route_.road_name.c_str(), build_us / 1000.0);
  }
}

}  // namespace services
}  // namespace adas
