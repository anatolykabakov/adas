#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>

#include "adas/adas_app.h"
#include "adas/mapmatch/fit.h"
#include "adas/mapmatch/geo.h"
#include "adas/mapmatch/road_map.h"
#include "adas/mapmatch/road_route.h"
#include "adas/mapmatch/search.h"
#include "adas/mapmatch/track.h"
#include "adas/mapmatch/window_search.h"
#include "adas/lateral/visionpilot_mpc.hpp"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/values.h"

namespace py = pybind11;

PYBIND11_MODULE(core, m)
{
  m.doc() = "";

  m.def("set_mpc_cost_weights", &visionpilot::set_cost_weights, py::arg("cte_weight_base"),
        py::arg("cte_quartic_scale"));

  m.def("set_mpc_warm_start_gains", &visionpilot::set_warm_start_gains, py::arg("epsi_gain"), py::arg("ff_scale"));

  m.def("set_mpc_cte_gain_base", &visionpilot::set_cte_gain_base, py::arg("base"));
  m.def("set_mpc_cte_gain_floor", &visionpilot::set_cte_gain_floor, py::arg("floor"));

  // The MQB torque limiter, exposed because a replay has no panda in the loop and the command that reaches
  // the rack is the limited one. Comparing our unlimited torque against upstream's `actuatorsOutput` is not
  // a comparison — the rate limit alone turns a requested 300 into a median 187 applied. Exposed rather
  // than reimplemented in the harness: a second copy of the asymmetric up/down logic would be a second
  // thing to keep correct.
  m.def("apply_driver_steer_torque_limits", &volkswagen::applyDriverSteerTorqueLimits, py::arg("apply_torque"),
        py::arg("driver_torque"), py::arg("apply_steer_last"));
  m.attr("STEER_MAX") = volkswagen::CarControllerParams::STEER_MAX;
  m.attr("STEER_STEP") = volkswagen::CarControllerParams::STEER_STEP;

  py::module_ mm = m.def_submodule("mapmatch");

  py::class_<adas::mapmatch::WindowSearchConfig>(mm, "WindowSearchConfig")
      .def(py::init<>())
      .def_readwrite("window_m", &adas::mapmatch::WindowSearchConfig::window_m)
      .def_readwrite("tol_deg", &adas::mapmatch::WindowSearchConfig::tol_deg)
      .def_readwrite("clip_deg", &adas::mapmatch::WindowSearchConfig::clip_deg)
      .def_readwrite("beam", &adas::mapmatch::WindowSearchConfig::beam)
      .def_readwrite("per_edge", &adas::mapmatch::WindowSearchConfig::per_edge)
      .def_readwrite("cell_m", &adas::mapmatch::WindowSearchConfig::cell_m)
      .def_readwrite("per_cell", &adas::mapmatch::WindowSearchConfig::per_cell)
      .def_readwrite("defer_deg", &adas::mapmatch::WindowSearchConfig::defer_deg)
      .def_readwrite("defer_beam", &adas::mapmatch::WindowSearchConfig::defer_beam)
      .def_readwrite("max_expand", &adas::mapmatch::WindowSearchConfig::max_expand)
      .def_readwrite("verbose", &adas::mapmatch::WindowSearchConfig::verbose);

  py::class_<adas::mapmatch::WindowRoute>(mm, "WindowRoute")
      .def_readonly("dir_edges", &adas::mapmatch::WindowRoute::dir_edges)
      .def_readonly("cost", &adas::mapmatch::WindowRoute::cost);

  mm.def("search_by_windows", &adas::mapmatch::searchByWindows, py::arg("road_map"), py::arg("window_deg"),
         py::arg("config") = adas::mapmatch::WindowSearchConfig{});

  py::class_<adas::mapmatch::LocalFrame>(mm, "LocalFrame")
      .def(py::init([](double lat0, double lon0) {
             return adas::mapmatch::LocalFrame{lat0, lon0};
           }),
           py::arg("lat0_deg") = 55.7539, py::arg("lon0_deg") = 37.6208)
      .def_readonly("lat0_deg", &adas::mapmatch::LocalFrame::lat0_deg)
      .def_readonly("lon0_deg", &adas::mapmatch::LocalFrame::lon0_deg)
      .def("to_local", &adas::mapmatch::LocalFrame::toLocal, py::arg("lat_deg"), py::arg("lon_deg"))
      .def("to_geo", &adas::mapmatch::LocalFrame::toGeo, py::arg("east_m"), py::arg("north_m"))
      .def(
          "to_local_many",
          [](const adas::mapmatch::LocalFrame& self, const std::vector<double>& lat, const std::vector<double>& lon) {
            std::vector<double> e, n;
            self.toLocalMany(lat, lon, e, n);
            return std::make_pair(std::move(e), std::move(n));
          },
          py::arg("lat_deg"), py::arg("lon_deg"));

  py::class_<adas::mapmatch::TrackConfig> track_cfg(mm, "TrackConfig");
  py::enum_<adas::mapmatch::TrackConfig::YawSource>(track_cfg, "YawSource")
      .value("Chassis", adas::mapmatch::TrackConfig::YawSource::Chassis)
      .value("Imu", adas::mapmatch::TrackConfig::YawSource::Imu)
      .value("Blend", adas::mapmatch::TrackConfig::YawSource::Blend);
  track_cfg.def(py::init<>())
      .def_readwrite("yaw_source", &adas::mapmatch::TrackConfig::yaw_source)
      .def_readwrite("rezero_yaw_at_stops", &adas::mapmatch::TrackConfig::rezero_yaw_at_stops)
      .def_readwrite("stop_speed_mps", &adas::mapmatch::TrackConfig::stop_speed_mps)
      .def_readwrite("stop_min_s", &adas::mapmatch::TrackConfig::stop_min_s)
      .def_readwrite("blend_imu_hf", &adas::mapmatch::TrackConfig::blend_imu_hf)
      .def_readwrite("min_speed_mps", &adas::mapmatch::TrackConfig::min_speed_mps)
      .def_readwrite("resample_m", &adas::mapmatch::TrackConfig::resample_m)
      .def_readwrite("speed_scale", &adas::mapmatch::TrackConfig::speed_scale)
      .def_readwrite("yaw_rate_scale", &adas::mapmatch::TrackConfig::yaw_rate_scale);

  py::class_<adas::mapmatch::SegmentConfig>(mm, "SegmentConfig")
      .def(py::init<>())
      .def_readwrite("turn_radius_m", &adas::mapmatch::SegmentConfig::turn_radius_m)
      .def_readwrite("min_turn_deg", &adas::mapmatch::SegmentConfig::min_turn_deg)
      .def_readwrite("merge_gap_m", &adas::mapmatch::SegmentConfig::merge_gap_m)
      .def_readwrite("smooth_m", &adas::mapmatch::SegmentConfig::smooth_m)
      .def_readwrite("min_straight_m", &adas::mapmatch::SegmentConfig::min_straight_m);

  py::class_<adas::mapmatch::Maneuver> maneuver(mm, "Maneuver");
  py::enum_<adas::mapmatch::Maneuver::Kind>(maneuver, "Kind")
      .value("Straight", adas::mapmatch::Maneuver::Kind::Straight)
      .value("Turn", adas::mapmatch::Maneuver::Kind::Turn);
  maneuver.def_readonly("kind", &adas::mapmatch::Maneuver::kind)
      .def_readonly("length_m", &adas::mapmatch::Maneuver::length_m)
      .def_readonly("angle_deg", &adas::mapmatch::Maneuver::angle_deg)
      .def_readonly("s_start_m", &adas::mapmatch::Maneuver::s_start_m)
      .def_readonly("s_end_m", &adas::mapmatch::Maneuver::s_end_m)
      .def_readonly("radius_m", &adas::mapmatch::Maneuver::radius_m)
      .def_property_readonly("is_turn", &adas::mapmatch::Maneuver::isTurn)
      .def_property_readonly("is_left", &adas::mapmatch::Maneuver::isLeft);

  py::class_<adas::mapmatch::Track>(mm, "Track")
      .def_readonly("s_m", &adas::mapmatch::Track::s_m)
      .def_readonly("x_m", &adas::mapmatch::Track::x_m)
      .def_readonly("y_m", &adas::mapmatch::Track::y_m)
      .def_readonly("theta_rad", &adas::mapmatch::Track::theta_rad)
      .def_readonly("maneuvers", &adas::mapmatch::Track::maneuvers)
      .def_property_readonly("length_m", &adas::mapmatch::Track::lengthM)
      .def_property_readonly("total_turn_deg", &adas::mapmatch::Track::totalTurnDeg)
      .def("describe", &adas::mapmatch::Track::describe);

  py::class_<adas::mapmatch::ImuSamples>(mm, "ImuSamples")
      .def(py::init<>())
      .def_readwrite("t_s", &adas::mapmatch::ImuSamples::t_s)
      .def_readwrite("gyro_x", &adas::mapmatch::ImuSamples::gyro_x)
      .def_readwrite("gyro_y", &adas::mapmatch::ImuSamples::gyro_y)
      .def_readwrite("gyro_z", &adas::mapmatch::ImuSamples::gyro_z)
      .def_readwrite("accel_x", &adas::mapmatch::ImuSamples::accel_x)
      .def_readwrite("accel_y", &adas::mapmatch::ImuSamples::accel_y)
      .def_readwrite("accel_z", &adas::mapmatch::ImuSamples::accel_z);

  py::class_<adas::mapmatch::YawDiagnostics>(mm, "YawDiagnostics")
      .def_readonly("has_imu", &adas::mapmatch::YawDiagnostics::has_imu)
      .def_readonly("bias_chassis_deg_s", &adas::mapmatch::YawDiagnostics::bias_chassis_deg_s)
      .def_readonly("bias_imu_deg_s", &adas::mapmatch::YawDiagnostics::bias_imu_deg_s)
      .def_readonly("corr_chassis_imu", &adas::mapmatch::YawDiagnostics::corr_chassis_imu)
      .def_readonly("scale_chassis_imu", &adas::mapmatch::YawDiagnostics::scale_chassis_imu)
      .def_readonly("n_stops", &adas::mapmatch::YawDiagnostics::n_stops)
      .def_readonly("total_turn_chassis_deg", &adas::mapmatch::YawDiagnostics::total_turn_chassis_deg)
      .def_readonly("total_turn_imu_deg", &adas::mapmatch::YawDiagnostics::total_turn_imu_deg);

  mm.def("analyze_yaw", &adas::mapmatch::analyzeYaw, py::arg("t_s"), py::arg("speed_mps"), py::arg("yaw_rate_rps"),
         py::arg("imu") = adas::mapmatch::ImuSamples{}, py::arg("config") = adas::mapmatch::TrackConfig{});

  mm.def("build_track", &adas::mapmatch::buildTrack, py::arg("t_s"), py::arg("speed_mps"), py::arg("yaw_rate_rps"),
         py::arg("config") = adas::mapmatch::TrackConfig{}, py::arg("segment") = adas::mapmatch::SegmentConfig{},
         py::arg("imu") = adas::mapmatch::ImuSamples{});
  mm.def("segment_maneuvers", &adas::mapmatch::segmentManeuvers, py::arg("track"),
         py::arg("config") = adas::mapmatch::SegmentConfig{});

  py::class_<adas::mapmatch::RoadEdge>(mm, "RoadEdge")
      .def_readonly("node_a", &adas::mapmatch::RoadEdge::node_a)
      .def_readonly("node_b", &adas::mapmatch::RoadEdge::node_b)
      .def_readonly("point_count", &adas::mapmatch::RoadEdge::point_count)
      .def_readonly("length_m", &adas::mapmatch::RoadEdge::length_m)
      .def_readonly("oneway", &adas::mapmatch::RoadEdge::oneway);

  py::class_<adas::mapmatch::RoadMap>(mm, "RoadMap")
      .def(py::init<>())
      .def("load", &adas::mapmatch::RoadMap::load, py::arg("path"))
      .def_property_readonly("node_count", &adas::mapmatch::RoadMap::nodeCount)
      .def_property_readonly("edge_count", &adas::mapmatch::RoadMap::edgeCount)
      .def_property_readonly("frame", &adas::mapmatch::RoadMap::frame)
      .def("edge", &adas::mapmatch::RoadMap::edge, py::arg("i"))
      .def("edge_name", &adas::mapmatch::RoadMap::edgeName, py::arg("i"))
      .def("edges_in_bbox", &adas::mapmatch::RoadMap::edgesInBBox, py::arg("x0"), py::arg("y0"), py::arg("x1"),
           py::arg("y1"))
      .def(
          "edge_polyline",
          [](const adas::mapmatch::RoadMap& self, std::uint32_t i) {
            std::vector<double> xs, ys;
            self.edgePolyline(i, xs, ys);
            return std::make_pair(std::move(xs), std::move(ys));
          },
          py::arg("i"))
      .def(
          "polylines_in_bbox",
          [](const adas::mapmatch::RoadMap& self, double x0, double y0, double x1, double y1) {
            std::vector<double> xs, ys, exs, eys;
            for (const std::uint32_t ei : self.edgesInBBox(x0, y0, x1, y1)) {
              self.edgePolyline(ei, exs, eys);
              xs.insert(xs.end(), exs.begin(), exs.end());
              ys.insert(ys.end(), eys.begin(), eys.end());
              xs.push_back(std::nan(""));
              ys.push_back(std::nan(""));
            }
            return std::make_pair(std::move(xs), std::move(ys));
          },
          py::arg("x0"), py::arg("y0"), py::arg("x1"), py::arg("y1"))
      .def(
          "nearest_edge",
          [](const adas::mapmatch::RoadMap& self, double x, double y, double search_m) {
            double d = -1.0;
            const std::uint32_t ei = self.nearestEdge(x, y, &d, search_m);
            return std::make_tuple(ei, d, ei == 0xFFFFFFFFu ? std::string{} : self.edgeName(ei));
          },
          py::arg("x"), py::arg("y"), py::arg("search_m") = 200.0);

  // Route ahead and its curvature — the same code the on-device `map_data` service runs, exposed so a
  // recorded run can be replayed through it offline (`app/src/main/scripts/bag_map_data.py`). Runs that were
  // driven before the service existed have GPS in the bag, so they can be analysed too.
  py::class_<adas::mapmatch::RouteConfig>(mm, "RouteConfig")
      .def(py::init<>())
      .def_readwrite("horizon_m", &adas::mapmatch::RouteConfig::horizon_m)
      .def_readwrite("match_search_m", &adas::mapmatch::RouteConfig::match_search_m)
      .def_readwrite("max_match_dist_m", &adas::mapmatch::RouteConfig::max_match_dist_m)
      .def_readwrite("max_match_heading_deg", &adas::mapmatch::RouteConfig::max_match_heading_deg)
      .def_readwrite("heading_weight_m_per_rad", &adas::mapmatch::RouteConfig::heading_weight_m_per_rad)
      .def_readwrite("step_m", &adas::mapmatch::RouteConfig::step_m)
      .def_readwrite("window_m", &adas::mapmatch::RouteConfig::window_m)
      .def_readwrite("turn_kappa", &adas::mapmatch::RouteConfig::turn_kappa)
      .def_readwrite("max_lat_acc", &adas::mapmatch::RouteConfig::max_lat_acc)
      .def_readwrite("min_section_m", &adas::mapmatch::RouteConfig::min_section_m)
      .def_readwrite("max_curv_deviation", &adas::mapmatch::RouteConfig::max_curv_deviation)
      .def_readwrite("max_curv_split_arc_deg", &adas::mapmatch::RouteConfig::max_curv_split_arc_deg)
      .def_readwrite("straight_max_deg", &adas::mapmatch::RouteConfig::straight_max_deg);

  py::class_<adas::mapmatch::TurnSection>(mm, "TurnSection")
      .def_readonly("start_m", &adas::mapmatch::TurnSection::start_m)
      .def_readonly("end_m", &adas::mapmatch::TurnSection::end_m)
      .def_readonly("kappa", &adas::mapmatch::TurnSection::kappa)
      .def_readonly("speed_mps", &adas::mapmatch::TurnSection::speed_mps)
      .def_readonly("sign", &adas::mapmatch::TurnSection::sign)
      .def("__repr__", [](const adas::mapmatch::TurnSection& t) {
        return "<TurnSection " + std::to_string(static_cast<int>(t.start_m)) + ".." +
               std::to_string(static_cast<int>(t.end_m)) +
               " m, R=" + std::to_string(static_cast<int>(t.kappa > 1e-9 ? 1.0 / t.kappa : 0.0)) +
               " m, v=" + std::to_string(static_cast<int>(t.speed_mps * 3.6)) + " km/h, " +
               (t.sign > 0 ? "left>" : "right>");
      });

  py::class_<adas::mapmatch::RouteAhead>(mm, "RouteAhead")
      .def_readonly("matched", &adas::mapmatch::RouteAhead::matched)
      .def_readonly("match_dist_m", &adas::mapmatch::RouteAhead::match_dist_m)
      .def_readonly("heading_delta_rad", &adas::mapmatch::RouteAhead::heading_delta_rad)
      .def_readonly("road_name", &adas::mapmatch::RouteAhead::road_name)
      .def_readonly("x_m", &adas::mapmatch::RouteAhead::x_m)
      .def_readonly("y_m", &adas::mapmatch::RouteAhead::y_m)
      .def_readonly("dir_edges", &adas::mapmatch::RouteAhead::dir_edges)
      .def_readonly("s_m", &adas::mapmatch::RouteAhead::s_m)
      .def_readonly("x", &adas::mapmatch::RouteAhead::x_m_pts)
      .def_readonly("y", &adas::mapmatch::RouteAhead::y_m_pts)
      .def_readonly("kappa", &adas::mapmatch::RouteAhead::kappa)
      .def_readonly("turns", &adas::mapmatch::RouteAhead::turns)
      .def_readonly("length_m", &adas::mapmatch::RouteAhead::length_m)
      .def_readonly("node_spacing_m", &adas::mapmatch::RouteAhead::node_spacing_m);

  mm.def("build_route_ahead", &adas::mapmatch::buildRouteAhead, py::arg("map"), py::arg("x"), py::arg("y"),
         py::arg("yaw_rad"), py::arg("cfg") = adas::mapmatch::RouteConfig{});

  mm.def(
      "curvature_along",
      [](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& query_s,
         double window_m) { return adas::mapmatch::curvatureAlong(x, y, query_s, window_m); },
      py::arg("x"), py::arg("y"), py::arg("query_s"), py::arg("window_m") = 25.0);

  py::class_<adas::mapmatch::FitConfig>(mm, "FitConfig")
      .def(py::init<>())
      .def_readwrite("sample_m", &adas::mapmatch::FitConfig::sample_m)
      .def_readwrite("sigma_road_m", &adas::mapmatch::FitConfig::sigma_road_m)
      .def_readwrite("sigma_speed_scale", &adas::mapmatch::FitConfig::sigma_speed_scale)
      .def_readwrite("sigma_yaw_scale", &adas::mapmatch::FitConfig::sigma_yaw_scale)
      .def_readwrite("sigma_turn_deg", &adas::mapmatch::FitConfig::sigma_turn_deg)
      .def_readwrite("sigma_straight_rel", &adas::mapmatch::FitConfig::sigma_straight_rel)
      .def_readwrite("drift_block_m", &adas::mapmatch::FitConfig::drift_block_m)
      .def_readwrite("sigma_drift_deg_per_100m", &adas::mapmatch::FitConfig::sigma_drift_deg_per_100m)
      .def_readwrite("max_residual_m", &adas::mapmatch::FitConfig::max_residual_m)
      .def_readwrite("iterations", &adas::mapmatch::FitConfig::iterations)
      .def_readwrite("anneal_steps", &adas::mapmatch::FitConfig::anneal_steps)
      .def_readwrite("anneal_start_scale", &adas::mapmatch::FitConfig::anneal_start_scale);

  py::class_<adas::mapmatch::FitResult>(mm, "FitResult")
      .def_readonly("ok", &adas::mapmatch::FitResult::ok)
      .def_readonly("x0_m", &adas::mapmatch::FitResult::x0_m)
      .def_readonly("y0_m", &adas::mapmatch::FitResult::y0_m)
      .def_readonly("heading_rad", &adas::mapmatch::FitResult::heading_rad)
      .def_readonly("speed_scale", &adas::mapmatch::FitResult::speed_scale)
      .def_readonly("yaw_rate_scale", &adas::mapmatch::FitResult::yaw_rate_scale)
      .def_readonly("turn_corr_deg", &adas::mapmatch::FitResult::turn_corr_deg)
      .def_readonly("straight_corr", &adas::mapmatch::FitResult::straight_corr)
      .def_readonly("drift_deg_per_100m", &adas::mapmatch::FitResult::drift_deg_per_100m)
      .def_readonly("x_m", &adas::mapmatch::FitResult::x_m)
      .def_readonly("y_m", &adas::mapmatch::FitResult::y_m)
      .def_readonly("rms_m", &adas::mapmatch::FitResult::rms_m)
      .def_readonly("median_m", &adas::mapmatch::FitResult::median_m)
      .def_readonly("p95_m", &adas::mapmatch::FitResult::p95_m)
      .def_readonly("deform_cost", &adas::mapmatch::FitResult::deform_cost)
      .def_readonly("score", &adas::mapmatch::FitResult::score)
      .def_readonly("iterations", &adas::mapmatch::FitResult::iterations);

  py::class_<adas::mapmatch::SearchConfig>(mm, "SearchConfig")
      .def(py::init<>())
      .def_readwrite("turn_tol_deg", &adas::mapmatch::SearchConfig::turn_tol_deg)
      .def_readwrite("dist_tol_rel", &adas::mapmatch::SearchConfig::dist_tol_rel)
      .def_readwrite("dist_tol_abs_m", &adas::mapmatch::SearchConfig::dist_tol_abs_m)
      .def_readwrite("straight_max_deg", &adas::mapmatch::SearchConfig::straight_max_deg)
      .def_readwrite("beam_width", &adas::mapmatch::SearchConfig::beam_width)
      .def_readwrite("max_candidates", &adas::mapmatch::SearchConfig::max_candidates)
      .def_readwrite("min_seed_edge_m", &adas::mapmatch::SearchConfig::min_seed_edge_m)
      .def_readwrite("max_overshoot_rel", &adas::mapmatch::SearchConfig::max_overshoot_rel)
      .def_readwrite("dist_tol_min_m", &adas::mapmatch::SearchConfig::dist_tol_min_m)
      .def_readwrite("check_first_straight", &adas::mapmatch::SearchConfig::check_first_straight)
      .def_readwrite("turn_max_len_m", &adas::mapmatch::SearchConfig::turn_max_len_m)
      .def_readwrite("turn_start_min_deg", &adas::mapmatch::SearchConfig::turn_start_min_deg)
      .def_readwrite("verbose", &adas::mapmatch::SearchConfig::verbose);

  py::class_<adas::mapmatch::RouteCandidate>(mm, "RouteCandidate")
      .def_readonly("dir_edges", &adas::mapmatch::RouteCandidate::dir_edges)
      .def_readonly("cost", &adas::mapmatch::RouteCandidate::cost)
      .def_readonly("length_m", &adas::mapmatch::RouteCandidate::length_m)
      .def_readonly("start_x_m", &adas::mapmatch::RouteCandidate::start_x_m)
      .def_readonly("start_y_m", &adas::mapmatch::RouteCandidate::start_y_m)
      .def_readonly("start_heading_rad", &adas::mapmatch::RouteCandidate::start_heading_rad)
      .def_readonly("matched_turns", &adas::mapmatch::RouteCandidate::matched_turns);

  mm.def("search_routes", &adas::mapmatch::searchRoutes, py::arg("road_map"), py::arg("track"),
         py::arg("config") = adas::mapmatch::SearchConfig{});

  mm.def("fit_track", &adas::mapmatch::fitTrack, py::arg("road_map"), py::arg("track"), py::arg("x0_m"),
         py::arg("y0_m"), py::arg("heading_rad"), py::arg("config") = adas::mapmatch::FitConfig{});

  mm.def("fit_track_to_route", &adas::mapmatch::fitTrackToRoute, py::arg("road_map"), py::arg("track"),
         py::arg("dir_edges"), py::arg("config") = adas::mapmatch::FitConfig{});

  py::class_<adas::Vec2>(m, "Vec2")
      .def(py::init<>())
      .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
      .def_property(
          "x", [](const adas::Vec2& v) { return v.x(); }, [](adas::Vec2& v, double val) { v.x() = val; })
      .def_property(
          "y", [](const adas::Vec2& v) { return v.y(); }, [](adas::Vec2& v, double val) { v.y() = val; });

  py::class_<adas::ChassisSample>(m, "ChassisSample")
      .def(py::init<>())
      .def_readwrite("timestamp_us", &adas::ChassisSample::timestamp_us)
      .def_readwrite("speed_mps", &adas::ChassisSample::speed_mps)
      .def_readwrite("steer_rad", &adas::ChassisSample::steer_rad)
      .def_readwrite("yaw_rate", &adas::ChassisSample::yaw_rate);

  py::class_<adas::LanePathMsg>(m, "LanePathMsg")
      .def(py::init<>())
      .def_readwrite("timestamp_us", &adas::LanePathMsg::timestamp_us)
      .def_readwrite("frame_id", &adas::LanePathMsg::frame_id)
      .def_readwrite("polyline", &adas::LanePathMsg::polyline)
      .def_readwrite("plan_poly", &adas::LanePathMsg::plan_poly)
      .def_readwrite("plan_yaw", &adas::LanePathMsg::plan_yaw)
      .def_readwrite("plan_yaw_rate", &adas::LanePathMsg::plan_yaw_rate);

  py::class_<adas::GpsSample>(m, "GpsSample")
      .def(py::init<>())
      .def_readwrite("timestamp_us", &adas::GpsSample::timestamp_us)
      .def_readwrite("x", &adas::GpsSample::x)
      .def_readwrite("y", &adas::GpsSample::y)
      .def_readwrite("speed_mps", &adas::GpsSample::speed_mps)
      .def_readwrite("bearing_deg", &adas::GpsSample::bearing_deg)
      .def_readwrite("yaw_enu", &adas::GpsSample::yaw_enu)
      .def_readwrite("vx", &adas::GpsSample::vx)
      .def_readwrite("vy", &adas::GpsSample::vy)
      .def_readwrite("course_valid", &adas::GpsSample::course_valid)
      .def_readwrite("valid", &adas::GpsSample::valid);

  py::class_<adas::ImuSample>(m, "ImuSample")
      .def(py::init<>())
      .def_readwrite("timestamp_us", &adas::ImuSample::timestamp_us)
      .def_readwrite("yaw_rate", &adas::ImuSample::yaw_rate)
      .def_readwrite("valid", &adas::ImuSample::valid);

  py::class_<adas::LocalizationPose>(m, "LocalizationPose")
      .def(py::init<>())
      .def_readonly("timestamp_us", &adas::LocalizationPose::timestamp_us)
      .def_readonly("x", &adas::LocalizationPose::x)
      .def_readonly("y", &adas::LocalizationPose::y)
      .def_readonly("yaw", &adas::LocalizationPose::yaw)
      .def_readonly("v", &adas::LocalizationPose::v)
      .def_readonly("yaw_rate", &adas::LocalizationPose::yaw_rate)
      .def_readonly("odom_x", &adas::LocalizationPose::odom_x)
      .def_readonly("odom_y", &adas::LocalizationPose::odom_y)
      .def_readonly("ekf_x", &adas::LocalizationPose::ekf_x)
      .def_readonly("ekf_y", &adas::LocalizationPose::ekf_y);

  py::class_<adas::LaneKeepOutput>(m, "LaneKeepOutput")
      .def_readonly("timestamp_us", &adas::LaneKeepOutput::timestamp_us)
      .def_readonly("capture_ts_us", &adas::LaneKeepOutput::capture_ts_us)
      .def_readonly("vision_ts_us", &adas::LaneKeepOutput::vision_ts_us)
      .def_readonly("chassis_ts_us", &adas::LaneKeepOutput::chassis_ts_us)
      .def_readonly("publish_ts_us", &adas::LaneKeepOutput::publish_ts_us)
      .def_readonly("steer_rad", &adas::LaneKeepOutput::steer_rad)
      .def_readonly("steer_norm", &adas::LaneKeepOutput::steer_norm)
      .def_readonly("desired_swa_deg", &adas::LaneKeepOutput::desired_swa_deg)
      .def_readonly("actual_swa_deg", &adas::LaneKeepOutput::actual_swa_deg)
      .def_readonly("angle_error_deg", &adas::LaneKeepOutput::angle_error_deg)
      .def_readonly("lookahead_m", &adas::LaneKeepOutput::lookahead_m)
      .def_readonly("target_x", &adas::LaneKeepOutput::target_x)
      .def_readonly("target_y", &adas::LaneKeepOutput::target_y)
      .def_readonly("has_target", &adas::LaneKeepOutput::has_target)
      .def_readonly("curvature", &adas::LaneKeepOutput::curvature)
      .def_readonly("cte_m", &adas::LaneKeepOutput::cte_m)
      .def_readonly("epsi_rad", &adas::LaneKeepOutput::epsi_rad)
      .def_readonly("status", &adas::LaneKeepOutput::status)
      .def_readonly("controller", &adas::LaneKeepOutput::controller);

  // The actuation command: what CarController would have been handed. `torque_cnm` is signed cNm before the
  // MQB rate/driver limiter, `enabled` is the app's own lateral gate.
  py::class_<ai::flow::adas::SteerCommand>(m, "SteerCommand")
      .def_property_readonly("torque_cnm", &ai::flow::adas::SteerCommand::torque_cnm)
      .def_property_readonly("enabled", &ai::flow::adas::SteerCommand::enabled)
      .def_property_readonly("capture_ts_ms", &ai::flow::adas::SteerCommand::capture_ts_ms)
      .def_property_readonly("vision_ts_ms", &ai::flow::adas::SteerCommand::vision_ts_ms)
      .def_property_readonly("chassis_ts_ms", &ai::flow::adas::SteerCommand::chassis_ts_ms)
      .def_property_readonly("publish_ts_ms", &ai::flow::adas::SteerCommand::publish_ts_ms);

  py::class_<ai::flow::adas::SafetyWarnState>(m, "SafetyWarnState")
      .def_property_readonly("timestamp", &ai::flow::adas::SafetyWarnState::timestamp)
      .def_property_readonly("accel_ms2", &ai::flow::adas::SafetyWarnState::accel_ms2)
      .def_property_readonly("cte_m", &ai::flow::adas::SafetyWarnState::cte_m)
      .def_property_readonly("cte_rate_ms", &ai::flow::adas::SafetyWarnState::cte_rate_ms)
      .def_property_readonly("epsi_rad", &ai::flow::adas::SafetyWarnState::epsi_rad)
      .def_property_readonly("kappa", &ai::flow::adas::SafetyWarnState::kappa)
      .def_property_readonly("lateral_valid", &ai::flow::adas::SafetyWarnState::lateral_valid)
      .def_property_readonly("v_ego", &ai::flow::adas::SafetyWarnState::v_ego)
      .def_property_readonly("lead_d", &ai::flow::adas::SafetyWarnState::lead_d)
      .def_property_readonly("lead_v", &ai::flow::adas::SafetyWarnState::lead_v)
      .def_property_readonly("lead_prob", &ai::flow::adas::SafetyWarnState::lead_prob)
      .def_property_readonly("has_lead", &ai::flow::adas::SafetyWarnState::has_lead)
      .def_property_readonly("ttc_s", &ai::flow::adas::SafetyWarnState::ttc_s)
      .def_property_readonly("a_req_ms2", &ai::flow::adas::SafetyWarnState::a_req_ms2)
      .def_property_readonly("threat_valid", &ai::flow::adas::SafetyWarnState::threat_valid)
      .def_property_readonly("driver_steering", &ai::flow::adas::SafetyWarnState::driver_steering)
      .def_property_readonly("lane_anchored", &ai::flow::adas::SafetyWarnState::lane_anchored)
      .def_property_readonly("fcw", &ai::flow::adas::SafetyWarnState::fcw)
      .def_property_readonly("aeb", &ai::flow::adas::SafetyWarnState::aeb)
      .def_property_readonly("lldw", &ai::flow::adas::SafetyWarnState::lldw)
      .def_property_readonly("rldw", &ai::flow::adas::SafetyWarnState::rldw)
      .def_property_readonly("status", &ai::flow::adas::SafetyWarnState::status);

  py::class_<adas::CameraCalibrationState>(m, "CameraCalibrationState")
      .def_readonly("timestamp_us", &adas::CameraCalibrationState::timestamp_us)
      .def_readonly("roll_deg", &adas::CameraCalibrationState::roll_deg)
      .def_readonly("pitch_deg", &adas::CameraCalibrationState::pitch_deg)
      .def_readonly("yaw_deg", &adas::CameraCalibrationState::yaw_deg)
      .def_readonly("camera_height_m", &adas::CameraCalibrationState::camera_height_m)
      .def_readonly("calibration_success", &adas::CameraCalibrationState::calibration_success)
      .def_readonly("n_updates", &adas::CameraCalibrationState::n_updates)
      .def_readonly("vp_u", &adas::CameraCalibrationState::vp_u)
      .def_readonly("vp_v", &adas::CameraCalibrationState::vp_v)
      .def_readonly("has_vp", &adas::CameraCalibrationState::has_vp)
      .def_readonly("cal_percent", &adas::CameraCalibrationState::cal_percent);

  py::class_<AdasApp, std::shared_ptr<AdasApp>>(m, "AdasApp")
      .def(py::init([](double wheelbase, double pitch0_deg, double yaw0_deg, double camera_height,
                       int camera_calib_history_len, double gps_noise_pos, double gps_update_interval,
                       bool topic_convert) {
             auto app =
                 std::make_shared<AdasApp>(AdasApp::Mode::Simulated, wheelbase, pitch0_deg, yaw0_deg, camera_height,
                                           camera_calib_history_len, gps_noise_pos, gps_update_interval, topic_convert);
             app->start();
             return app;
           }),
           py::arg("wheelbase") = 2.636, py::arg("pitch0_deg") = 0.0, py::arg("yaw0_deg") = 0.0,
           py::arg("camera_height") = 1.22, py::arg("camera_calib_history_len") = 50, py::arg("gps_noise_pos") = 0.5,
           py::arg("gps_update_interval") = 0.2, py::arg("topic_convert") = false)
      .def("start", &AdasApp::start)
      .def("stop", &AdasApp::stop)
      .def("step", &AdasApp::step, py::arg("timestamp_us"))
      .def("reset_localization", &AdasApp::resetLocalization, py::arg("x") = 0.0, py::arg("y") = 0.0,
           py::arg("yaw") = 0.0, py::arg("v") = 0.0, py::arg("yaw_rate") = 0.0)
      .def("set_camera_intrinsics", &AdasApp::setCameraIntrinsics, py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"))
      .def("set_camera_estimate", &AdasApp::setCameraEstimate, py::arg("pitch_deg"), py::arg("yaw_deg"))
      .def("set_camera_height", &AdasApp::setCameraHeight, py::arg("height_m"))
      .def("reset_camera_calib", &AdasApp::resetCameraCalib)
      .def("set_lane_keep_pp", &AdasApp::setLaneKeepPp, py::arg("k_dd"), py::arg("ld_min"), py::arg("ld_max"),
           py::arg("shift"))
      .def("set_lane_keep_pp_ld_curv_gain", &AdasApp::setLaneKeepPpLdCurvGain, py::arg("gain"))
      .def("set_lane_keep_max_steer_deg", &AdasApp::setLaneKeepMaxSteerDeg, py::arg("max_steer_deg"))
      .def("set_lane_keep_controller", &AdasApp::setLaneKeepController, py::arg("controller"))
      .def("set_lane_keep_mpc_kappa_yaw_blend", &AdasApp::setLaneKeepMpcKappaYawBlend, py::arg("alpha"),
           py::arg("min_speed") = 3.0)
      .def("set_lane_keep_mpc_ema_alphas", &AdasApp::setLaneKeepMpcEmaAlphas, py::arg("kappa_alpha"),
           py::arg("epsi_alpha"), py::arg("cte_alpha"))
      .def("set_lane_keep_steer_slew_limit_deg", &AdasApp::setLaneKeepSteerSlewLimitDeg, py::arg("deg"))
      .def("set_lane_keep_vehicle_model", &AdasApp::setLaneKeepVehicleModel, py::arg("on"),
           py::arg("tire_stiffness_factor") = 0.64)
      .def("set_lane_keep_fp_steer_delay_s", &AdasApp::setLaneKeepFpSteerDelayS, py::arg("seconds"))
      .def("set_lane_keep_fp_steering_rate_weight", &AdasApp::setLaneKeepFpSteeringRateWeight, py::arg("weight"))
      .def("set_lane_keep_cam_y_left_m", &AdasApp::setLaneKeepCamYLeftM, py::arg("m"))
      .def("set_lane_keep_pid_gains", &AdasApp::setLaneKeepPidGains, py::arg("kp"), py::arg("ki"), py::arg("kf"))
      .def("set_lane_keep_recompute_setpoint", &AdasApp::setLaneKeepRecomputeSetpoint, py::arg("on"))
      .def("set_param", static_cast<bool (AdasApp::*)(const std::string&, double)>(&AdasApp::setParam), py::arg("name"),
           py::arg("value"))
      .def("set_param_str", static_cast<bool (AdasApp::*)(const std::string&, const std::string&)>(&AdasApp::setParam),
           py::arg("name"), py::arg("value"))
      .def("get_param", &AdasApp::getParam, py::arg("name"))
      .def("update_params", &AdasApp::updateParams, py::arg("params"))
      .def(
          "publish_chassis",
          // `steering_angle_deg` and `steering_pressed` are not decoration: the angle PID closes its loop on
          // the measured steering-wheel angle and hands over to the driver on `steering_pressed`. A replay
          // that leaves them at zero exercises the planner and silently bypasses the controller.
          [](AdasApp& self, int64_t timestamp_us, double speed_mps, double steer_rad, double yaw_rate,
             double steering_angle_deg, bool steering_pressed) {
            adas::ChassisSample c;
            c.timestamp_us = timestamp_us;
            c.speed_mps = speed_mps;
            c.steer_rad = steer_rad;
            c.yaw_rate = yaw_rate;
            c.steering_angle_deg = steering_angle_deg;
            c.steering_pressed = steering_pressed;
            self.publishChassis(c);
          },
          py::arg("timestamp_us"), py::arg("speed_mps"), py::arg("steer_rad") = 0.0, py::arg("yaw_rate") = 0.0,
          py::arg("steering_angle_deg") = 0.0, py::arg("steering_pressed") = false)
      .def(
          "publish_lanes",
          [](AdasApp& self, int64_t timestamp_us, const std::vector<std::pair<double, double>>& poly, int frame_id,
             const std::vector<std::pair<double, double>>& plan_poly, const std::vector<double>& plan_yaw,
             const std::vector<double>& plan_yaw_rate, bool lane_anchored) {
            adas::LanePathMsg msg;
            msg.timestamp_us = timestamp_us;
            msg.frame_id = frame_id;
            msg.lane_anchored = lane_anchored;
            msg.polyline.reserve(poly.size());
            for (const auto& p : poly)
              msg.polyline.push_back({p.first, p.second});
            msg.plan_poly.reserve(plan_poly.size());
            for (const auto& p : plan_poly)
              msg.plan_poly.push_back({p.first, p.second});
            msg.plan_yaw = plan_yaw;
            msg.plan_yaw_rate = plan_yaw_rate;
            self.publishLanes(msg);
          },
          py::arg("timestamp_us"), py::arg("polyline_ego"), py::arg("frame_id") = 0,
          py::arg("plan_poly") = std::vector<std::pair<double, double>>{}, py::arg("plan_yaw") = std::vector<double>{},
          py::arg("plan_yaw_rate") = std::vector<double>{}, py::arg("lane_anchored") = false)
      .def(
          "publish_lane_lines",
          [](AdasApp& self, py::bytes serialized) {
            ai::flow::adas::LaneLines ll;
            const std::string data = serialized;
            if (!ll.ParseFromString(data))
              throw std::invalid_argument("publish_lane_lines: not a LaneLines message");
            self.publishLaneLines(ll);
          },
          py::arg("lane_lines_bytes"))
      .def(
          "publish_lane_uv",
          [](AdasApp& self, int64_t timestamp_us, const std::vector<std::pair<double, double>>& left,
             const std::vector<std::pair<double, double>>& right) {
            adas::LaneUvMsg msg;
            msg.timestamp_us = timestamp_us;
            msg.left_uv.reserve(left.size());
            msg.right_uv.reserve(right.size());
            for (const auto& p : left)
              msg.left_uv.push_back({p.first, p.second});
            for (const auto& p : right)
              msg.right_uv.push_back({p.first, p.second});
            self.publishLaneUv(msg);
          },
          py::arg("timestamp_us"), py::arg("left_uv"), py::arg("right_uv"))
      .def(
          "publish_gps",
          [](AdasApp& self, int64_t timestamp_us, double x, double y, double speed_mps, double bearing_deg,
             double yaw_enu, double vx, double vy, bool course_valid, bool valid) {
            adas::GpsSample g;
            g.timestamp_us = timestamp_us;
            g.x = x;
            g.y = y;
            g.speed_mps = speed_mps;
            g.bearing_deg = bearing_deg;
            g.yaw_enu = yaw_enu;
            g.vx = vx;
            g.vy = vy;
            g.course_valid = course_valid;
            g.valid = valid;
            self.publishGps(g);
          },
          py::arg("timestamp_us"), py::arg("x"), py::arg("y"), py::arg("speed_mps") = 0.0, py::arg("bearing_deg") = 0.0,
          py::arg("yaw_enu") = 0.0, py::arg("vx") = 0.0, py::arg("vy") = 0.0, py::arg("course_valid") = false,
          py::arg("valid") = true)
      .def(
          "publish_imu",
          [](AdasApp& self, int64_t timestamp_us, double yaw_rate, bool valid) {
            adas::ImuSample imu;
            imu.timestamp_us = timestamp_us;
            imu.yaw_rate = yaw_rate;
            imu.valid = valid;
            self.publishImu(imu);
          },
          py::arg("timestamp_us"), py::arg("yaw_rate"), py::arg("valid") = true)
      .def("pop_messages", [](AdasApp& self) {
        py::list out;
        for (auto& msg : self.popMessages()) {
          std::visit([&](auto&& v) { out.append(py::cast(std::move(v))); }, msg);
        }
        return out;
      });

  m.attr("PyAdasApp") = m.attr("AdasApp");
}
