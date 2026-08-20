#pragma once

#include <cstdint>
#include <optional>

#include "car_state.pb.h"
#include "adas/panda/can_frame.h"
#include "adas/utils/can_parser.h"
#include "adas/utils/speed_filter.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/values.h"

namespace volkswagen {
/// \return True for CAN ids the MQB port decodes.
bool isAllowedMqbRxAddress(uint32_t address);

/// \return True when the TSK status means the stock cruise is engaged.
inline bool cruiseEngagedFromTsk(int tsk_status) { return tsk_status == 3 || tsk_status == 4 || tsk_status == 5; }

/// \return True when the stock cruise is at least available.
inline bool cruiseAvailableFromTsk(int tsk_status) { return cruiseEngagedFromTsk(tsk_status) || tsk_status == 2; }

/** `Getriebe_11.GE_Fahrstufe` per `vw_mqb_2010.dbc`: 5 P, 6 R, 7 N, 8 D, 9 S, 10 E, 13/14 T. Only reverse
 *  matters here, and 0 means the frame has not been seen yet rather than a gear. */
inline bool gearIsReverse(int gear) { return gear == 6; }

/** Decodes MQB CAN frames into CarState through the DBC. */
class MqbCarStateDecoder {
public:
  /// \param[in] dbc Parsed DBC; null leaves the decoder inert.
  explicit MqbCarStateDecoder(DBSParser* dbc = nullptr) : dbc_(dbc) {}

  /// Attach the DBC after construction.
  void setDbc(DBSParser* dbc) { dbc_ = dbc; }

  /** Wheel-speed filtering and the wheel-radius correction — see `SpeedFilter`. */
  void setSpeedFilterConfig(const adas::SpeedFilter::Config& cfg) { speed_filter_.setConfig(cfg); }
  /// \return The wheel-speed filter.
  const adas::SpeedFilter& speedFilter() const { return speed_filter_; }

  /// Decode one frame into the state.
  void updateFromFrame(const can_frame& frame, int64_t now_ms);

  /// \return True when a decoded value changed since the last consume.
  bool dirty() const { return dirty_; }
  /// \return The dirty flag, clearing it.
  bool consumeDirty()
  {
    const bool d = dirty_;
    dirty_ = false;
    return d;
  }

  /// \return The decoded state.
  adas::proto::CarState& state() { return state_; }
  /// Const overload.
  const adas::proto::CarState& state() const { return state_; }

  /// \return EPS HCA status byte from LH_EPS_03.
  uint8_t epsHcaStatus() const { return eps_hca_status_; }
  /// \return Stock LDW payload, mirrored into our HUD frame.
  const LdwStockValues& ldwStock() const { return ldw_stock_; }
  /// \return Stock cruise-button payload, base for our presses.
  const GraStockValues& graStock() const { return gra_stock_; }
  /// \return Last TSK (cruise) status seen.
  int lastTskStatus() const { return last_tsk_status_; }

  /// \return The brand-neutral view of the decoded state.
  CarStateView toCarStateView() const;

private:
  DBSParser* dbc_ = nullptr;
  adas::proto::CarState state_;
  bool dirty_ = false;
  adas::SpeedFilter speed_filter_{};
  int64_t prev_v_ts_ms_ = 0;
  bool brake_esp_ = false;
  bool brake_motor_ = false;
  uint8_t eps_hca_status_ = 0;
  LdwStockValues ldw_stock_;
  GraStockValues gra_stock_;
  int last_tsk_status_ = -1;
};

}  // namespace volkswagen
