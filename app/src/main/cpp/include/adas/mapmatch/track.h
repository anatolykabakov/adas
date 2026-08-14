#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace adas {
namespace mapmatch {
struct ImuSamples {
  std::vector<double> t_s;
  std::vector<double> gyro_x, gyro_y, gyro_z;
  std::vector<double> accel_x, accel_y, accel_z;

  bool empty() const { return t_s.size() < 10; }
};

struct TrackConfig {
  double min_speed_mps = 0.5;  ///< Samples below this speed are dropped [m/s]: at rest the heading is undefined.
  enum class YawSource { Chassis = 0, Imu = 1, Blend = 2 };
  YawSource yaw_source = YawSource::Chassis;  ///< Which yaw rate builds the track: CAN, phone gyro, or a blend.
  bool rezero_yaw_at_stops = false;           ///< Re-zero the gyro bias at every stop, which is where it is observable.
  double stop_speed_mps = 0.2;                ///< Speed below which the car counts as stopped [m/s].
  double stop_min_s = 1.0;                    ///< How long that must hold before it counts as a stop [s].
  double blend_imu_hf = 0.5;                  ///< Crossover for the blend: gyro above it, CAN below.
  double resample_m = 2.0;                    ///< Track is resampled to this spacing [m].
  double speed_scale = 1.0;                   ///< Correction on wheel speed; the 1.2 % scale error lives here.
  double yaw_rate_scale = 1.0;                ///< Correction on the yaw rate.
};

struct Maneuver {
  enum class Kind : std::uint8_t { Straight, Turn };

  Kind kind = Kind::Straight;
  double length_m = 0.0;
  double angle_deg = 0.0;
  double s_start_m = 0.0;
  double s_end_m = 0.0;
  double radius_m = 0.0;

  bool isTurn() const { return kind == Kind::Turn; }
  bool isLeft() const { return isTurn() && angle_deg > 0.0; }
};

struct Track {
  std::vector<double> s_m;
  std::vector<double> x_m;
  std::vector<double> y_m;
  std::vector<double> theta_rad;
  std::vector<Maneuver> maneuvers;

  std::size_t size() const { return s_m.size(); }
  double lengthM() const { return s_m.empty() ? 0.0 : s_m.back(); }
  double totalTurnDeg() const;
  std::string describe() const;
};

struct SegmentConfig {
  double turn_radius_m = 80.0;
  double min_turn_deg = 25.0;
  double merge_gap_m = 25.0;
  double smooth_m = 10.0;
  double min_straight_m = 5.0;
};

Track buildTrack(const std::vector<double>& t_s, const std::vector<double>& speed_mps,
                 const std::vector<double>& yaw_rate_rps, const TrackConfig& cfg = {}, const SegmentConfig& seg = {},
                 const ImuSamples& imu = {});

struct YawDiagnostics {
  bool has_imu = false;
  double bias_chassis_deg_s = 0.0;
  double bias_imu_deg_s = 0.0;
  double corr_chassis_imu = 0.0;
  double scale_chassis_imu = 0.0;
  int n_stops = 0;
  double total_turn_chassis_deg = 0.0;
  double total_turn_imu_deg = 0.0;
};

YawDiagnostics analyzeYaw(const std::vector<double>& t_s, const std::vector<double>& speed_mps,
                          const std::vector<double>& yaw_rate_rps, const ImuSamples& imu, const TrackConfig& cfg = {});

std::vector<Maneuver> segmentManeuvers(const Track& track, const SegmentConfig& cfg = {});

}  // namespace mapmatch
}  // namespace adas
