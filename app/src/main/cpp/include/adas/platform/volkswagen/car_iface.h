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
/**
 * \brief The car interface: the only place that knows the brand and the CAN layout.
 *
 * \details This is upstream's `CarInterfaceBase`, and it is a field rather than a process there too:
 * `Controls.__init__` holds `self.CI`, which owns `self.CS` (the decoder) and `self.CC` (the frame
 * packer), and `controlsd` calls `CI.update(can_strs)` and `CI.apply(CC)`. We have three services
 * around it: planner, controller, platform.
 *
 * The control law is not part of this: it receives a chassis frame and a plan, while addresses,
 * signals and frame counters are known only here. Neither is the hardware — frames arrive and leave
 * by value, so the interface is testable against a recording without a panda.
 */
class CarIface {
public:
  struct Config {
    std::string dbc_path;
    adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  };

  explicit CarIface(Config config);

  /// Load the DBC. Without it chassis decoding is off, while sending frames keeps working.
  void init();

  /// `CI.update`: frames into a chassis state. \return True when it changed and is worth publishing.
  bool update(const adas::proto::CANData& msg, int64_t now_ms);

  /// `CI.apply`: a controller command into frames for the platform.
  std::vector<can_frame> apply(const CarControl& cc, const CarStateView& cs) { return car_controller_.update(cc, cs); }

  const adas::proto::CarState& carState() const { return decoder_.state(); }
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

  int lastTskStatus() const { return decoder_.lastTskStatus(); }
  uint8_t epsHcaStatus() const { return decoder_.epsHcaStatus(); }
  int applySteerLast() const { return car_controller_.applySteerLast(); }

private:
  Config config_;
  std::unique_ptr<DBSParser> dbc_;
  MqbCarStateDecoder decoder_;
  CarController car_controller_;
};

}  // namespace volkswagen
