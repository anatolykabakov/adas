#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>

#include "adas/adas_app.h"
#include "adas/mapmatch/road_map.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/values.h"

namespace py = pybind11;

PYBIND11_MODULE(core, m)
{
  m.doc() = "";

  m.def("apply_driver_steer_torque_limits", &volkswagen::applyDriverSteerTorqueLimits, py::arg("apply_torque"),
        py::arg("driver_torque"), py::arg("apply_steer_last"));
  m.attr("STEER_MAX") = volkswagen::CarControllerParams::STEER_MAX;
  m.attr("STEER_STEP") = volkswagen::CarControllerParams::STEER_STEP;

  py::module_ mm = m.def_submodule("mapmatch");

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

  py::class_<adas::Vec2>(m, "Vec2")
      .def(py::init<>())
      .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
      .def_property(
          "x", [](const adas::Vec2& v) { return v.x(); }, [](adas::Vec2& v, double val) { v.x() = val; })
      .def_property("y", [](const adas::Vec2& v) { return v.y(); }, [](adas::Vec2& v, double val) { v.y() = val; });

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
      .def_readonly("ekf_y", &adas::LocalizationPose::ekf_y)
      // Road bank and the learned parameters. Without them a replay cannot show which numbers the
      // controller drove with — and under `use_learned_params` it drives with these, not with the config
      // constants.
      .def_readonly("road_roll_deg", &adas::LocalizationPose::road_roll_deg)
      .def_readonly("road_roll_std_deg", &adas::LocalizationPose::road_roll_std_deg)
      .def_readonly("road_roll_valid", &adas::LocalizationPose::road_roll_valid)
      .def_readonly("learned_stiffness_factor", &adas::LocalizationPose::learned_stiffness_factor)
      .def_readonly("learned_steer_ratio", &adas::LocalizationPose::learned_steer_ratio)
      .def_readonly("learned_angle_offset_deg", &adas::LocalizationPose::learned_angle_offset_deg)
      .def_readonly("learned_stiffness_std", &adas::LocalizationPose::learned_stiffness_std)
      .def_readonly("learned_steer_ratio_std", &adas::LocalizationPose::learned_steer_ratio_std)
      .def_readonly("learned_params_valid", &adas::LocalizationPose::learned_params_valid)
      .def_readonly("learned_sample_count", &adas::LocalizationPose::learned_sample_count);

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

  py::class_<adas::proto::SteerCommand>(m, "SteerCommand")
      .def_property_readonly("torque_cnm", &adas::proto::SteerCommand::torque_cnm)
      .def_property_readonly("enabled", &adas::proto::SteerCommand::enabled)
      .def_property_readonly("capture_ts_ms", &adas::proto::SteerCommand::capture_ts_ms)
      .def_property_readonly("vision_ts_ms", &adas::proto::SteerCommand::vision_ts_ms)
      .def_property_readonly("chassis_ts_ms", &adas::proto::SteerCommand::chassis_ts_ms)
      .def_property_readonly("publish_ts_ms", &adas::proto::SteerCommand::publish_ts_ms);

  py::class_<adas::proto::SafetyWarnState>(m, "SafetyWarnState")
      .def_property_readonly("timestamp", &adas::proto::SafetyWarnState::timestamp)
      .def_property_readonly("accel_ms2", &adas::proto::SafetyWarnState::accel_ms2)
      .def_property_readonly("cte_m", &adas::proto::SafetyWarnState::cte_m)
      .def_property_readonly("cte_rate_ms", &adas::proto::SafetyWarnState::cte_rate_ms)
      .def_property_readonly("epsi_rad", &adas::proto::SafetyWarnState::epsi_rad)
      .def_property_readonly("kappa", &adas::proto::SafetyWarnState::kappa)
      .def_property_readonly("lateral_valid", &adas::proto::SafetyWarnState::lateral_valid)
      .def_property_readonly("v_ego", &adas::proto::SafetyWarnState::v_ego)
      .def_property_readonly("lead_d", &adas::proto::SafetyWarnState::lead_d)
      .def_property_readonly("lead_v", &adas::proto::SafetyWarnState::lead_v)
      .def_property_readonly("lead_prob", &adas::proto::SafetyWarnState::lead_prob)
      .def_property_readonly("has_lead", &adas::proto::SafetyWarnState::has_lead)
      .def_property_readonly("ttc_s", &adas::proto::SafetyWarnState::ttc_s)
      .def_property_readonly("a_req_ms2", &adas::proto::SafetyWarnState::a_req_ms2)
      .def_property_readonly("threat_valid", &adas::proto::SafetyWarnState::threat_valid)
      .def_property_readonly("driver_steering", &adas::proto::SafetyWarnState::driver_steering)
      .def_property_readonly("lane_anchored", &adas::proto::SafetyWarnState::lane_anchored)
      .def_property_readonly("fcw", &adas::proto::SafetyWarnState::fcw)
      .def_property_readonly("aeb", &adas::proto::SafetyWarnState::aeb)
      .def_property_readonly("lldw", &adas::proto::SafetyWarnState::lldw)
      .def_property_readonly("rldw", &adas::proto::SafetyWarnState::rldw)
      .def_property_readonly("status", &adas::proto::SafetyWarnState::status);

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
      .def(
          "set_lane_keep_pp",
          [](AdasApp& a, double k_dd, double ld_min, double ld_max, double shift) {
            a.setParam("pp_k_dd", k_dd);
            a.setParam("pp_ld_min", ld_min);
            a.setParam("pp_ld_max", ld_max);
            a.setParam("pp_shift", shift);
          },
          py::arg("k_dd"), py::arg("ld_min"), py::arg("ld_max"), py::arg("shift"))
      // Single-knob names map straight onto the parameter registry: the Python API keeps its named,
      // discoverable methods, and C++ keeps one entry point (setParam) instead of a wrapper per knob.
      .def(
          "set_lane_keep_pp_ld_curv_gain", [](AdasApp& a, double gain) { a.setParam("pp_ld_curv_gain", gain); },
          py::arg("gain"))
      .def(
          "set_lane_keep_max_steer_deg", [](AdasApp& a, double deg) { a.setParam("max_steer_deg", deg); },
          py::arg("max_steer_deg"))
      .def(
          "set_lane_keep_controller",
          [](AdasApp& a, const std::string& controller) { a.setParam("lane_keep_controller", controller); },
          py::arg("controller"))
      .def(
          "set_lane_keep_steer_slew_limit_deg", [](AdasApp& a, double deg) { a.setParam("steer_slew_limit_deg", deg); },
          py::arg("deg"))
      .def(
          "set_lane_keep_tire_stiffness", [](AdasApp& a, double f) { a.setParam("tire_stiffness_factor", f); },
          py::arg("tire_stiffness_factor"))
      .def(
          "set_lane_keep_vehicle_model",
          [](AdasApp& a, bool use, double f) {
            a.setParam("lat_use_vehicle_model", use);
            a.setParam("tire_stiffness_factor", f);
          },
          py::arg("use_vehicle_model"), py::arg("tire_stiffness_factor"))
      .def(
          "set_lane_keep_fp_steer_delay_s", [](AdasApp& a, double s) { a.setParam("fp_steer_delay_s", s); },
          py::arg("seconds"))
      .def(
          "set_lane_keep_fp_steering_rate_weight",
          [](AdasApp& a, double w) { a.setParam("fp_steering_rate_weight", w); }, py::arg("weight"))
      .def(
          "set_lane_keep_cam_y_left_m", [](AdasApp& a, double m) { a.setParam("cam_y_left_m", m); }, py::arg("m"))
      .def(
          "set_lane_keep_pid_gains",
          [](AdasApp& a, double kp, double ki, double kf) {
            a.setParam("pid_kp", kp);
            a.setParam("pid_ki", ki);
            a.setParam("pid_kf", kf);
          },
          py::arg("kp"), py::arg("ki"), py::arg("kf"))
      .def("set_param", static_cast<bool (AdasApp::*)(const std::string&, double)>(&AdasApp::setParam), py::arg("name"),
           py::arg("value"))
      .def("set_param_str", static_cast<bool (AdasApp::*)(const std::string&, const std::string&)>(&AdasApp::setParam),
           py::arg("name"), py::arg("value"))
      .def("get_param", &AdasApp::getParam, py::arg("name"))
      .def("update_params", &AdasApp::updateParams, py::arg("params"))
      .def(
          "publish_chassis",
          [](AdasApp& self, int64_t timestamp_us, double speed_mps, double steer_rad, double yaw_rate,
             double steering_angle_deg, bool steering_pressed) {
            adas::ChassisSample c;
            c.timestamp_us = timestamp_us;
            c.speed_mps = speed_mps;
            c.steer_rad = steer_rad;
            c.yaw_rate = yaw_rate;
            c.steering_angle_deg = steering_angle_deg;
            c.steering_pressed = steering_pressed;
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kVehicleState, adas::carStateFromChassis(c));
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
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kVisionPathIn, adas::createLanePath(msg));
          },
          py::arg("timestamp_us"), py::arg("polyline_ego"), py::arg("frame_id") = 0,
          py::arg("plan_poly") = std::vector<std::pair<double, double>>{}, py::arg("plan_yaw") = std::vector<double>{},
          py::arg("plan_yaw_rate") = std::vector<double>{}, py::arg("lane_anchored") = false)
      .def(
          "publish_lane_lines",
          [](AdasApp& self, py::bytes serialized) {
            adas::proto::LaneLines ll;
            const std::string data = serialized;
            if (!ll.ParseFromString(data))
              throw std::invalid_argument("publish_lane_lines: not a LaneLines message");
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kVisionLanes, ll);
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
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kCalibLaneUv, msg);
          },
          py::arg("timestamp_us"), py::arg("left_uv"), py::arg("right_uv"))
      .def(
          "publish_gps",
          [](AdasApp& self, int64_t timestamp_us, double x, double y, double speed_mps, double bearing_deg,
             double yaw_enu, double vx, double vy, bool course_valid, bool valid, double accuracy_m, int satellites) {
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
            // The reported accuracy decides whether the filter takes the position and with what noise.
            // Without it a replay measured a different pipeline than the car runs: the receiver on drive
            // 2026_08_13_23_01_56 reported 10 m.
            g.accuracy_m = accuracy_m;
            g.satellites = satellites;
            g.valid = valid;
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kGpsLocation, g);
          },
          py::arg("timestamp_us"), py::arg("x"), py::arg("y"), py::arg("speed_mps") = 0.0, py::arg("bearing_deg") = 0.0,
          py::arg("yaw_enu") = 0.0, py::arg("vx") = 0.0, py::arg("vy") = 0.0, py::arg("course_valid") = false,
          py::arg("valid") = true, py::arg("accuracy_m") = 0.0, py::arg("satellites") = 0)
      .def(
          "publish_imu",
          [](AdasApp& self, int64_t timestamp_us, double yaw_rate, bool valid) {
            adas::ImuSample imu;
            imu.timestamp_us = timestamp_us;
            imu.yaw_rate = yaw_rate;
            imu.valid = valid;
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kImuYaw, imu);
          },
          py::arg("timestamp_us"), py::arg("yaw_rate"), py::arg("valid") = true)
      .def(
          // The receiver's fix as it comes: the localization service projects latitude and longitude
          // itself. `publish_gps` puts already-projected metres on the same topic and nothing subscribes
          // to those — an offline run was dead reckoning, never receiving GNSS at all.
          "publish_gps_location",
          [](AdasApp& self, py::bytes serialized) {
            adas::proto::GPSLocation gps;
            const std::string data = serialized;
            if (!gps.ParseFromString(data))
              throw std::invalid_argument("publish_gps_location: not a GPSLocation message");
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kGpsLocation, gps);
          },
          py::arg("gps_bytes"))
      .def(
          // Raw IMU: reaches `ImuCalibrator`, and therefore yields the road bank. `publish_imu` carries
          // an already-calibrated yaw rate, from which the calibrator cannot fix an orientation.
          "publish_imu_data",
          [](AdasApp& self, py::bytes serialized) {
            adas::proto::IMUData imu;
            const std::string data = serialized;
            if (!imu.ParseFromString(data))
              throw std::invalid_argument("publish_imu_data: not an IMUData message");
            if (auto mw = self.getMiddleware())
              mw->publish(adas::topics::kImu, imu);
          },
          py::arg("imu_bytes"))
      .def("pop_messages", [](AdasApp& self) {
        py::list out;
        for (auto& msg : self.popMessages()) {
          std::visit([&](auto&& v) { out.append(py::cast(std::move(v))); }, msg);
        }
        return out;
      });

  m.attr("PyAdasApp") = m.attr("AdasApp");
}
