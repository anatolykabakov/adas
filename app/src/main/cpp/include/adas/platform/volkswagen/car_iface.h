#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "messages.pb.h"
#include "adas/utils/can_parser.h"
#include "adas/utils/speed_filter.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/mqb_car_state_decoder.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"
#include "adas/platform/volkswagen/values.h"

namespace volkswagen {
/** The car interface: the only place that knows the brand and the CAN layout. */
class CarIface {
public:
  struct Config {
    std::string dbc_path;                      ///< CAN database the decoder parses.
    adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  };

  /// \param[in] config DBC path and filter settings.
  explicit CarIface(Config config);

  /// Load the DBC. Without it chassis decoding is off, while sending frames keeps working.
  void init();

  /// `CI.update`: frames into a chassis state. \return True when it changed and is worth publishing.
  bool update(const adas::proto::CANData& msg, int64_t now_ms);

  /// `CI.apply`: a controller command into frames for the platform.
  std::vector<can_frame> apply(const CarControl& cc, const CarStateView& cs) { return car_controller_.update(cc, cs); }

  /// \return Decoded state.
  const adas::proto::CarState& carState() const { return decoder_.state(); }
  /// \return Brand-neutral view.
  CarStateView carStateView() const { return decoder_.toCarStateView(); }

  /// This car's torque ceiling [cNm]: the controller computes a normalised command, the limit lives here.
  int maxTorqueCNm() const { return CarControllerParams::STEER_MAX; }
  /// Whether the panda's safety mode is the one under which this car accepts torque.
  bool safetyModelOk(uint32_t safety_mode) const { return safety_mode == MqbSafetyConstants::kVolkswagen; }
  /// Whether the car lets torque through right now: TSK, EPS and the assist status are brand knowledge.
  bool actuationAllowed(bool controls_allowed) const
  {
    return lateralActuationAllowed(controls_allowed, /*lat_always_on=*/true, carStateView());
  }

  /// \return Last cruise status.
  int lastTskStatus() const { return decoder_.lastTskStatus(); }
  /// \return EPS HCA status byte.
  uint8_t epsHcaStatus() const { return decoder_.epsHcaStatus(); }
  /// \return Torque last put on the bus [unit].
  int applySteerLast() const { return car_controller_.applySteerLast(); }

private:
  Config config_;
  std::unique_ptr<DBSParser> dbc_;
  MqbCarStateDecoder decoder_;
  CarController car_controller_;
};

}  // namespace volkswagen
