#pragma once

#include <algorithm>
#include <cmath>

namespace adas {
namespace lateral {
/**
 * \brief Vehicle parameters with an online estimate: steering ratio, tyre stiffness, angle zero.
 *
 * \details Both services need these, for different reasons: the planner so its solver knows what
 * curvature turns into, the controller so it can compute a setpoint and a command. The class is shared
 * precisely so that "learned overrides configured" is written once — two copies, once drifted apart,
 * would give the planner and the controller different ideas about the same car.
 *
 * \note No control law lives here.
 */
class LearnedVehicle {
public:
  struct Config {
    double steer_ratio = 15.7;            ///< Configured steering ratio, used until the learner is trusted.
    double steer_sign = -1.0;             ///< Which way a positive command turns the wheel: +1 or -1.
    double max_steer_deg = 8.0;           ///< Ceiling on the commanded road-wheel angle [deg].
    double tire_stiffness_factor = 0.64;  ///< Scale on the reference tyre stiffness; 1.0 is the reference car.
  };

  LearnedVehicle() = default;
  explicit LearnedVehicle(Config cfg)
    : cfg_(cfg)
    , steer_ratio_(std::max(cfg.steer_ratio, 1e-3))
    , steer_sign_(cfg.steer_sign < 0.0 ? -1.0 : 1.0)
    , max_steer_rad_(cfg.max_steer_deg * M_PI / 180.0)
  {
  }

  /** \brief Apply the online estimate.
   *
   *  \param[in] valid The estimator's own gate. While it is false the configured constants apply, so
   *  losing validity rolls the parameters back rather than freezing them at their last learned value. */
  void setLearnedParams(bool valid, double stiffness, double ratio, double offset_deg)
  {
    learned_valid_ = valid && stiffness > 0.05 && ratio > 1.0;
    if (learned_valid_) {
      learned_stiffness_ = stiffness;
      learned_steer_ratio_ = ratio;
      learned_angle_offset_deg_ = offset_deg;
    }
  }

  /// True while the learner's estimate is in force rather than the configured constants.
  bool usingLearnedParams() const { return learned_valid_; }
  double effectiveStiffnessFactor() const
  {
    return usingLearnedParams() ? learned_stiffness_ : cfg_.tire_stiffness_factor;
  }
  /// Ratio actually used this tick: learned when valid, configured otherwise.
  double effectiveSteerRatio() const { return usingLearnedParams() ? learned_steer_ratio_ : steer_ratio_; }
  double effectiveAngleOffsetDeg() const { return usingLearnedParams() ? learned_angle_offset_deg_ : 0.0; }

  /// The configured ratio, ignoring the learned one: this is what converts a measured steering-wheel
  /// angle into a road-wheel angle.
  /// The configured ratio, never the learned one.
  double steerRatio() const { return steer_ratio_; }
  /// Which way a positive command turns this car: +1 or -1.
  double steerSign() const { return steer_sign_; }
  /// Ceiling on the commanded road-wheel angle [rad].
  double maxSteerRad() const { return max_steer_rad_; }

  /// \param[in] deg Ceiling on the road-wheel angle [deg].
  void setMaxSteerDeg(double deg) { max_steer_rad_ = deg * M_PI / 180.0; }
  /// \param[in] ratio Configured steering ratio; floored so it can never divide by zero.
  void setSteerRatio(double ratio) { steer_ratio_ = std::max(ratio, 1e-3); }
  /// \param[in] sign Any negative value means -1; anything else means +1.
  void setSteerSign(double sign) { steer_sign_ = (sign < 0.0) ? -1.0 : 1.0; }
  /// \param[in] f Scale on the reference tyre stiffness; 1.0 is the reference car.
  void setTireStiffnessFactor(double f) { cfg_.tire_stiffness_factor = f; }

private:
  Config cfg_{};
  double steer_ratio_ = 15.7;
  double steer_sign_ = -1.0;
  double max_steer_rad_ = 8.0 * M_PI / 180.0;

  bool learned_valid_ = false;
  double learned_stiffness_ = 0.64;
  double learned_steer_ratio_ = 15.7;
  double learned_angle_offset_deg_ = 0.0;
};

}  // namespace lateral
}  // namespace adas
