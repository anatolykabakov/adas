#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "adas/utils/adas_topics.h"
#include "adas/utils/math_utils.h"

namespace adas {
/**
 * \brief Camera pitch and yaw from the direction the camera-odometry translation points.
 *
 * \details When the car drives straight, the translation vector points along the optical axis of a
 * perfectly aligned camera; whatever angle it makes instead is the mounting error. Independent of lane
 * markings, which is why it is kept alongside the vanishing-point estimate rather than instead of it.
 */
class PoseCalibrator {
public:
  enum Status { Uncalibrated = 0, Calibrated = 1, Invalid = 2, Recalibrating = 3 };

  struct LastSampleDebug {
    bool odom_valid = false;
    bool accepted = false;
    bool gate_speed = false;
    bool gate_yaw_rate = false;
    bool gate_rpy_certain = false;
    double v_ego = 0.0;
    double odom_trans_x = 0.0;
    double odom_trans_y = 0.0;
    double odom_trans_z = 0.0;
    double odom_rot_z = 0.0;
    double odom_angle_std = 0.0;
    double observed_pitch_deg = 0.0;
    double observed_yaw_deg = 0.0;
    const char* reject_reason = "";
  };

  explicit PoseCalibrator(double pitch0_deg = 0.0, double yaw0_deg = 0.0, double height_m = 1.22);

  /**
   * \brief Restart the estimate from a seed.
   *
   * \param[in] pitch0_deg Seed pitch [deg], negative looking down.
   * \param[in] yaw0_deg Seed yaw [deg], positive to the left.
   * \param[in] valid_blocks How much of the seed to trust: 0 starts from nothing, the full count treats
   * the seed as converged, which is how a saved calibration survives a restart.
   */
  void reset(double pitch0_deg, double yaw0_deg, int valid_blocks = 0);
  /// \param[in] v_ego_mps Ego speed [m/s]; samples below the working range are ignored.
  void setVEgo(double v_ego_mps);
  /// \param[in] height_m Camera height above the road [m]; carried through, not estimated.
  void setHeight(double height_m) { height_m_ = height_m; }

  /**
   * \brief Feed one camera-odometry sample.
   *
   * \param[in] odom Translation, rotation and their standard deviations.
   * \return True when the sample entered the estimate; false when it was too noisy or too slow.
   */
  bool handleCamOdom(const CameraOdometrySample& odom);

  /// Current estimate [deg]: pitch negative looking down, yaw positive to the left.
  double pitchDeg() const;
  double yawDeg() const;
  double rollDeg() const { return 0.0; }
  double heightM() const { return height_m_; }
  Status status() const { return status_; }
  int validBlocks() const { return valid_blocks_; }
  /// Convergence, 0 to 100, as shown to the driver.
  int calPercent() const;
  /// True once the estimate may be used for projection.
  bool calibrated() const { return status_ == Calibrated; }

  /// Roll, pitch, yaw [rad] after smoothing — what a consumer should project with.
  Vec3 smoothRpy() const;
  Vec3 calibSpread() const { return calib_spread_; }
  double oldRpyWeight() const { return old_rpy_weight_; }
  int blockIdx() const { return block_idx_; }
  int sampleInBlock() const { return idx_; }
  double vEgo() const { return v_ego_; }
  const LastSampleDebug& lastSample() const { return last_sample_; }

  static const char* statusName(Status s);

private:
  void updateStatus();
  std::vector<int> validIdxs() const;

  static constexpr double kMinSpeed = 15.0 * 0.44704;
  static constexpr double kMaxVelAngleStd = 0.25 * M_PI / 180.0;
  static constexpr double kMaxYawRate = 2.0 * M_PI / 180.0;
  static constexpr int kSmoothCycles = 10;
  static constexpr int kBlockSize = 100;
  static constexpr int kInputsNeeded = 5;
  static constexpr int kInputsWanted = 50;
  static constexpr double kMaxSpread = 2.0 * M_PI / 180.0;

  double height_m_ = 1.22;
  double v_ego_ = 0.0;
  Vec3 rpy_ = Vec3::Zero();
  std::array<Vec3, kInputsWanted> rpys_{};
  Vec3 old_rpy_ = Vec3::Zero();
  Vec3 calib_spread_ = Vec3::Zero();
  double old_rpy_weight_ = 0.0;
  int valid_blocks_ = 0;
  int idx_ = 0;
  int block_idx_ = 0;
  Status status_ = Uncalibrated;
  LastSampleDebug last_sample_{};
};

}  // namespace adas
