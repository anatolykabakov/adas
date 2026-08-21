#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "adas/utils/math_utils.h"

namespace adas {
struct ImageLine {
  double m = 0.0;
  double c = 0.0;
};

/// \return Intersection of two image lines [px], or nothing when near-parallel.
std::optional<Vec2> getIntersection(const ImageLine& a, const ImageLine& b);

/// \return (pitch, yaw) [rad] implied by a vanishing point at (u_i, v_i) with the given intrinsics.
Vec2 getPitchYawFromVp(double u_i, double v_i, double fx, double fy, double cx, double cy);

/// Fit v(u) to lane points [px]. \return Nothing when residuals exceed the threshold.
std::optional<ImageLine> fitLineVOfU(const std::vector<Vec2>& uv, double mean_residuals_thresh = 25.0);

/** Camera pitch and yaw from where the lane lines converge in the image. */
class VanishingPointCalibrator {
public:
  /// \param[in] history_len Estimates averaged; \p pitch0_deg / \p yaw0_deg mount prior [deg].
  explicit VanishingPointCalibrator(int history_len = 50, double pitch0_deg = 0.0, double yaw0_deg = 0.0);

  /// Back to the prior; history dropped.
  void reset();
  /// Overwrite the current estimate [deg].
  void setEstimate(double pitch_deg, double yaw_deg);

  /// One update from two fitted lane lines. \return True when a vanishing point was accepted.
  bool updateFromLines(const ImageLine& left, const ImageLine& right, double fx, double fy, double cx, double cy);

  /// Same update from raw lane points [px].
  bool updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv, double fx, double fy,
                    double cx, double cy);

  /// \return Current pitch estimate [deg].
  double pitchDeg() const { return pitch_deg_; }
  /// \return Current yaw estimate [deg].
  double yawDeg() const { return yaw_deg_; }
  /// \return True after at least one accepted update.
  bool success() const { return success_; }
  /// \return Accepted updates so far.
  int nUpdates() const { return n_updates_; }
  /// \return True when the last frame yielded a vanishing point.
  bool hasVp() const { return has_vp_; }
  /// \return Last vanishing point u [px].
  double vpU() const { return vp_u_; }
  /// \return Last vanishing point v [px].
  double vpV() const { return vp_v_; }
  /// \return Estimates currently in the averaging window.
  int historySize() const { return static_cast<int>(history_.size()); }

private:
  bool addToHistory(double pitch_rad, double yaw_rad);

  int history_len_ = 50;
  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;
  bool success_ = false;
  int n_updates_ = 0;
  bool has_vp_ = false;
  double vp_u_ = 0.0;
  double vp_v_ = 0.0;
  std::vector<Vec2> history_;
};

}  // namespace adas
