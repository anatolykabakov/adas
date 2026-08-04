#pragma once

#include <cstdint>
#include <optional>

#include "car_state.pb.h"
#include "panda/can_frame.h"
#include "utils/can_parser.h"
#include "volkswagen/carcontroller.h"
#include "volkswagen/values.h"

namespace volkswagen {

bool isAllowedMqbRxAddress(uint32_t address);

inline bool cruiseEngagedFromTsk(int tsk_status) { return tsk_status == 3 || tsk_status == 4 || tsk_status == 5; }

inline bool cruiseAvailableFromTsk(int tsk_status) { return cruiseEngagedFromTsk(tsk_status) || tsk_status == 2; }

class MqbCarStateDecoder {
public:
  explicit MqbCarStateDecoder(DBSParser* dbc = nullptr) : dbc_(dbc) {}

  void setDbc(DBSParser* dbc) { dbc_ = dbc; }

  void updateFromFrame(const can_frame& frame);

  bool dirty() const { return dirty_; }
  bool consumeDirty()
  {
    const bool d = dirty_;
    dirty_ = false;
    return d;
  }

  ai::flow::adas::CarState& state() { return state_; }
  const ai::flow::adas::CarState& state() const { return state_; }

  uint8_t epsHcaStatus() const { return eps_hca_status_; }
  const LdwStockValues& ldwStock() const { return ldw_stock_; }
  const GraStockValues& graStock() const { return gra_stock_; }
  int lastTskStatus() const { return last_tsk_status_; }

  CarStateView toCarStateView() const;

private:
  DBSParser* dbc_ = nullptr;
  ai::flow::adas::CarState state_;
  bool dirty_ = false;
  double prev_v_ego_ = 0.0;
  int64_t prev_v_ts_ms_ = 0;
  bool brake_esp_ = false;
  bool brake_motor_ = false;
  uint8_t eps_hca_status_ = 0;
  LdwStockValues ldw_stock_;
  GraStockValues gra_stock_;
  int last_tsk_status_ = -1;
};

}  // namespace volkswagen
