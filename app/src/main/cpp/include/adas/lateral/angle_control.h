#pragma once

#include <algorithm>
#include <cmath>

#include "adas/lateral/learned_vehicle.h"
#include "adas/lateral/slew_guard.h"
#include "adas/lateral/types.h"
#include "adas/utils/adas_topics.h"
#include "adas/utils/lat_control_pid.h"

namespace adas {
namespace lateral {
/** The angle loop: road-wheel angle to steering-wheel angle to normalised torque. */
class AngleControl {
public:
  struct Config {
    double pid_kp = 0.6;            ///< Angle PID: proportional gain.
    double pid_ki = 0.2;            ///< Angle PID: integral gain, per second.
    double pid_kf = 6e-5;           ///< Angle PID: feedforward gain on `swa * (v^2 + v0^2)`.
    double pid_ff_floor_mps = 0.0;  ///< Speed floor in the feedforward [m/s], so it does not vanish at low speed.
    /** \brief Integrator rate [Hz].
     *
     *  A constant, as upstream's `PIDController(rate=100)` is: taking the measured interval instead made
     *  the command depend on the very first tick, and it buys about 1 cNm RMS. */
    double pid_rate_hz = 100.0;  ///< Tick rate the integral is scaled by [Hz].

    double steer_ratio = 15.7;            ///< Steering-wheel to road-wheel ratio.
    double steer_sign = -1.0;             ///< Which way a positive command turns the wheel: +1 or -1.
    double max_steer_deg = 8.0;           ///< Ceiling on the commanded road-wheel angle [deg].
    double tire_stiffness_factor = 0.64;  ///< Scale on the reference tyre stiffness; 1.0 is the reference car.
  };

  /// \param[in] cfg PID gains, vehicle geometry and limits.
  explicit AngleControl(Config cfg)
    : cfg_(cfg)
    , pid_(cfg.pid_kp, cfg.pid_ki, cfg.pid_kf, cfg.pid_rate_hz, cfg.pid_ff_floor_mps)
    , veh_({cfg.steer_ratio, cfg.steer_sign, cfg.max_steer_deg, cfg.tire_stiffness_factor})
  {
  }

  /// Full reset: integrator, slew state and setpoint.
  void reset()
  {
    pid_.reset();
    slew_.reset();
    desired_swa_deg_ = 0.0;
  }

  /// No command: setpoint to zero and the integrator reset, so it cannot wind up against nothing.
  void clearSetpoint()
  {
    desired_swa_deg_ = 0.0;
    desired_swa_no_offset_deg_ = 0.0;
    pid_.reset();
  }

  /** \brief Setpoint from a road-wheel angle.
   *
   *  The learned angle zero is *added*: the estimator solved `delta = (SWA - offset) / ratio`, and this is
   *  the same relation run backwards. */
  void setSetpointFromSteer(double steer_rad)
  {
    desired_swa_no_offset_deg_ = veh_.steerSign() * (steer_rad * 180.0 / M_PI) * effectiveSteerRatio();
    desired_swa_deg_ = desired_swa_no_offset_deg_ + effectiveAngleOffsetDeg();
  }

  /// Rate limit applied to the command before the PID sees it.
  void setSlewConfig(const SlewGuard::Config& c) { slew_.setConfig(c); }
  /**
   * \brief Rate-limit the requested angle in place.
   * \param[in,out] steer_rad Requested road-wheel angle [rad]; clipped to the allowed step.
   * \param[in] speed_mps Ego speed [m/s]; the allowance narrows with speed.
   * \param[in] frame_dt_s Time since the previous request [s].
   * \param[in] commanded False while not steering, so the guard tracks instead of limiting.
   * \return True when the request was actually clipped — recorded, since a permanently clipped command
   */
  bool applySlew(double& steer_rad, double speed_mps, double frame_dt_s, bool commanded)
  {
    return slew_.apply(steer_rad, speed_mps, frame_dt_s, commanded);
  }

  /**
   * \brief One control step against the current chassis frame.
   * \param[in] active Whether the gates allow steering; false releases the loop.
   * \param[in] ch Chassis: measured steering angle, speed, and whether the driver is on the wheel.
   * \return Command and PID internals.
   */
  LatControlPid::Result update(bool active, const ChassisSample& ch)
  {
    return pid_.update(active, desired_swa_deg_, ch.steering_angle_deg, ch.speed_mps, ch.steering_pressed,
                       desired_swa_no_offset_deg_);
  }

  /// Hand the paramsd estimate over; used only while \p valid is true.
  void setLearnedParams(bool valid, double stiffness, double ratio, double offset_deg)
  {
    veh_.setLearnedParams(valid, stiffness, ratio, offset_deg);
  }

  /// \return True when the effective* values come from the learner rather than the config.
  bool usingLearnedParams() const { return veh_.usingLearnedParams(); }
  /// \return Stiffness factor in force.
  double effectiveStiffnessFactor() const { return veh_.effectiveStiffnessFactor(); }
  /// \return Steering ratio in force.
  double effectiveSteerRatio() const { return veh_.effectiveSteerRatio(); }
  /// \return Steering-zero offset in force [deg].
  double effectiveAngleOffsetDeg() const { return veh_.effectiveAngleOffsetDeg(); }

  /// The configured ratio, ignoring the learned one: converts a measured steering-wheel angle into a
  /// road-wheel angle.
  double steerRatio() const { return veh_.steerRatio(); }
  /// Current setpoint [deg], including the learned angle zero.
  double desiredSwaDeg() const { return desired_swa_deg_; }
  /// \return Road-wheel angle ceiling [rad].
  double maxSteerRad() const { return veh_.maxSteerRad(); }

  /// Set the road-wheel angle ceiling [deg].
  void setMaxSteerDeg(double deg) { veh_.setMaxSteerDeg(deg); }
  /// Set the steering ratio.
  void setSteerRatio(double ratio) { veh_.setSteerRatio(ratio); }
  /// Set the wheel-direction sign: +1 or -1.
  void setSteerSign(double sign) { veh_.setSteerSign(sign); }
  /// Gains as in \ref PidController.
  void setPidGains(double kp, double ki, double kf) { pid_.setGains(kp, ki, kf); }
  /// Set the tyre-stiffness factor.
  void setTireStiffnessFactor(double f) { veh_.setTireStiffnessFactor(f); }

private:
  Config cfg_;
  LatControlPid pid_;
  SlewGuard slew_;
  LearnedVehicle veh_;

  double desired_swa_deg_ = 0.0;
  double desired_swa_no_offset_deg_ = 0.0;
};

}  // namespace lateral
}  // namespace adas
