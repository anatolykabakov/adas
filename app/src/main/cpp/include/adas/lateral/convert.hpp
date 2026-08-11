#pragma once

#include <cmath>

#include "adas/lateral/types.hpp"
#include "adas/utils/adas_topics.h"

namespace adas {
namespace lateral {

/** Сообщения middleware -> вход планера. Сюда же уезжает сдвиг опоры на смещение камеры: планер
 *  получает готовую опору в ego-кадре и про камеру не знает. */
inline Input inputFromMessages(const LanePathMsg& path, double speed_mps, double yaw_rate, bool have_chassis,
                               double frame_dt_s, double cam_y_left_m, const VehicleParams& vehicle)
{
  Input in;
  in.speed_mps = speed_mps;
  in.yaw_rate = yaw_rate;
  in.have_chassis = have_chassis;
  in.frame_dt_s = frame_dt_s;
  in.lane_anchored = path.lane_anchored;
  in.lane_width_m = path.lane_width_m;
  in.lane_offset_m = path.lane_offset_m;
  in.vehicle = vehicle;

  in.polyline_ego = path.polyline;
  in.plan_poly = path.plan_poly;
  in.plan_yaw = path.plan_yaw;
  in.plan_yaw_rate = path.plan_yaw_rate;
  if (std::abs(cam_y_left_m) > 1e-12) {
    for (auto& p : in.polyline_ego)
      p.y() -= cam_y_left_m;
    for (auto& p : in.plan_poly)
      p.y() -= cam_y_left_m;
  }
  return in;
}

/** Выход планера -> публикуемая структура сервиса. */
inline void applyToOutput(const Output& src, LaneKeepOutput& dst)
{
  dst.status = src.status;
  dst.controller = src.controller;
  dst.steer_rad = src.steer_rad;
  dst.steer_norm = src.steer_norm;
  dst.curvature = src.curvature;
  dst.cte_m = src.cte_m;
  dst.epsi_rad = src.epsi_rad;
  dst.lookahead_m = src.lookahead_m;
  dst.target_x = src.target_x;
  dst.target_y = src.target_y;
  dst.has_target = src.has_target;

  dst.dbg.speed_mps = src.dbg.speed_mps;
  dst.dbg.n_points = src.dbg.n_points;
  dst.dbg.pp_steer_raw_rad = src.dbg.pp_steer_raw_rad;
  dst.dbg.mpc_kappa_path = src.dbg.mpc_kappa_path;
  dst.dbg.mpc_kappa_yaw = src.dbg.mpc_kappa_yaw;
  dst.dbg.mpc_kappa_used = src.dbg.mpc_kappa_used;
  dst.dbg.mpc_dkappa_ds = src.dbg.mpc_dkappa_ds;
  dst.dbg.mpc_delta_vp_rad = src.dbg.mpc_delta_vp_rad;
  dst.dbg.mpc_delta_clamped_rad = src.dbg.mpc_delta_clamped_rad;
  dst.dbg.mpc_max_steer_rad = src.dbg.mpc_max_steer_rad;
  dst.dbg.max_steer_rad = src.dbg.max_steer_rad;
}

}  // namespace lateral
}  // namespace adas
