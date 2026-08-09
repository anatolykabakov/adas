#include "volkswagen/panda_safety_supervisor.h"

#include <algorithm>
#include <tuple>

#include "utils/logger.h"
#include "volkswagen/values.h"

namespace volkswagen {

void PandaSafetySupervisor::scheduleSafetyRetry(int64_t now_ms)
{
  safety_fail_streak_ = std::min(safety_fail_streak_ + 1, 16);
  const int shift = std::min(safety_fail_streak_ - 1, 3);
  const int64_t delay = std::min(C::kSafetyRetryMaxMs, C::kSafetyRetryBaseMs << shift);
  next_safety_attempt_ms_ = now_ms + delay;
}

void PandaSafetySupervisor::clearSafetyRetry()
{
  safety_fail_streak_ = 0;
  next_safety_attempt_ms_ = 0;
}

bool PandaSafetySupervisor::updateIgnitionSticky(bool ignition_hw, uint32_t voltage_mv, int64_t now_ms)
{
  const bool voltage_on = voltage_mv >= C::kIgnVoltageOnMv;
  const bool voltage_off = voltage_mv < C::kIgnVoltageOffMv;
  if (ignition_hw || voltage_on) {
    ignition_sticky_ = true;
    ignition_low_since_ms_ = 0;
  } else if (voltage_off) {
    if (ignition_low_since_ms_ == 0) {
      ignition_low_since_ms_ = now_ms;
    } else if ((now_ms - ignition_low_since_ms_) >= C::kIgnOffDebounceMs) {
      ignition_sticky_ = false;
    }
  } else {
    ignition_low_since_ms_ = 0;
  }
  return ignition_sticky_;
}

std::optional<health_t> PandaSafetySupervisor::tick(Panda& panda, health_t health, int64_t now_ms,
                                                    const SafetyLogContext& log)
{
  health_t s = health;

  const bool ignition_hw = (health.ignition_line_pkt != 0) || (health.ignition_can_pkt != 0);
  const bool ignition = updateIgnitionSticky(ignition_hw, health.voltage_pkt, now_ms);

  constexpr uint16_t want = C::kVolkswagen;
  constexpr uint16_t param = C::kParamStock;

  if (!initialized_) {
    initialized_ = true;
    if (auto vers = panda.get_packets_versions()) {
      const uint8_t hv = std::get<0>(*vers);
      const uint8_t cv = std::get<1>(*vers);
      const uint8_t chv = std::get<2>(*vers);
      LOGI("Panda packet versions health=%u can=%u can_health=%u (host expects health=%u can_health=%u)", hv, cv, chv,
           HEALTH_PACKET_VERSION, CAN_HEALTH_PACKET_VERSION);
      // v11 is not a mismatch: `Panda::get_state` reads the v11 layout when the firmware speaks it, and
      // that is the version dragonpilot's panda declares. Reporting it as "fields will be wrong" sent
      // whoever debugged the assist gate looking in the wrong place.
      if (hv != HEALTH_PACKET_VERSION && hv != HEALTH_PACKET_VERSION_V11) {
        LOGE("Panda health packet version %u is neither %u nor %u — fields will be wrong", hv,
             HEALTH_PACKET_VERSION, HEALTH_PACKET_VERSION_V11);
      }
    }
    LOGI("Panda init V=%u ign_hw=%d ign=%d harness=%u safety_now=%u faults=0x%x", health.voltage_pkt,
         ignition_hw ? 1 : 0, ignition ? 1 : 0, health.car_harness_status_pkt, health.safety_mode_pkt,
         health.faults_pkt);

    panda.set_power_saving(false);

    if (s.safety_mode_pkt != C::kNoOutput) {
      panda.set_safety_model(C::kNoOutput, 0);
    }
    panda.set_alternative_experience(alt_exp_, 0);
    alt_exp_configured_ = true;
    LOGI("Panda alt_exp=%u set before VW safety (disengage_on_gas=%d alka=%d)", alt_exp_,
         (alt_exp_ & C::kAltExpDisableDisengageOnGas) ? 1 : 0, (alt_exp_ & C::kAltExpAlka) ? 1 : 0);

    if (ignition) {
      panda.set_safety_model(want, param);
      if (auto after = panda.get_state()) {
        LOGI("Panda initial safety want=%u got=%u harness=%u allowed=%d hb_lost=%d psave=%d alt=%u faults=0x%x", want,
             after->safety_mode_pkt, after->car_harness_status_pkt, after->controls_allowed_pkt ? 1 : 0,
             after->heartbeat_lost_pkt ? 1 : 0, after->power_save_enabled_pkt ? 1 : 0,
             after->alternative_experience_pkt, after->faults_pkt);
        safety_configured_ = (after->safety_mode_pkt == want);
        if (!safety_configured_) {
          scheduleSafetyRetry(now_ms);
          LOGW("Panda initial safety DID NOT STICK — next retry in %lld ms",
               static_cast<long long>(next_safety_attempt_ms_ - now_ms));
        } else {
          clearSafetyRetry();
        }
        s = *after;
      }
    }
  }

  const bool power_save_desired = !ignition;
  if (s.power_save_enabled_pkt != static_cast<uint8_t>(power_save_desired)) {
    panda.set_power_saving(power_save_desired);
    LOGW("Panda power_save %d → %d", s.power_save_enabled_pkt ? 1 : 0, power_save_desired ? 1 : 0);
  }

  if (ignition) {
    if (s.safety_mode_pkt != want) {
      const bool can_retry = (now_ms >= next_safety_attempt_ms_);
      if (can_retry) {
        if (!alt_exp_configured_ || s.safety_mode_pkt == C::kNoOutput || s.safety_mode_pkt == 0) {
          if (s.safety_mode_pkt != C::kNoOutput) {
            panda.set_safety_model(C::kNoOutput, 0);
          }
          panda.set_alternative_experience(alt_exp_, 0);
          alt_exp_configured_ = true;
        }
        LOGW("Panda safety drifted to %u — re-set %u (harness=%u faults=0x%x)", s.safety_mode_pkt, want,
             s.car_harness_status_pkt, s.faults_pkt);
        panda.set_safety_model(want, param);
        if (auto after = panda.get_state()) {
          safety_configured_ = (after->safety_mode_pkt == want);
          if (!safety_configured_) {
            scheduleSafetyRetry(now_ms);
            LOGW("Panda safety DID NOT STICK (want %u, still %u) — backoff %lld ms (streak=%d)", want,
                 after->safety_mode_pkt, static_cast<long long>(next_safety_attempt_ms_ - now_ms), safety_fail_streak_);
          } else {
            clearSafetyRetry();
          }
          s = *after;
        } else {
          scheduleSafetyRetry(now_ms);
        }
      }
    } else {
      safety_configured_ = true;
      clearSafetyRetry();
    }
  } else {
    if (s.safety_mode_pkt != C::kNoOutput) {
      LOGW("Panda ignition off (debounced) → NOOUTPUT");
      panda.set_safety_model(C::kNoOutput, 0);
      alt_exp_configured_ = false;
    }
    safety_configured_ = false;
    clearSafetyRetry();
  }

  const bool hb_engaged = (s.safety_mode_pkt == want);
  panda.send_heartbeat(hb_engaged);

  const bool allowed_now = s.controls_allowed_pkt != 0;
  if (allowed_now != last_controls_allowed_) {
    LOGW("controls_allowed %d → %d | safety=%u TSK=%d cruise_eng=%d brake=%d gas=%d V=%u hb_eng=%d",
         last_controls_allowed_ ? 1 : 0, allowed_now ? 1 : 0, s.safety_mode_pkt, log.last_tsk_status,
         log.cruise_engaged ? 1 : 0, log.brake_pressed ? 1 : 0, log.gas_pressed ? 1 : 0, s.voltage_pkt,
         hb_engaged ? 1 : 0);
  }

  last_safety_mode_ = s.safety_mode_pkt;
  last_controls_allowed_ = allowed_now;
  last_ignition_ = ignition;

  static int safety_log_div = 0;
  if (++safety_log_div >= 50) {
    safety_log_div = 0;
    LOGI("Panda health: safety=%u param=%u allowed=%d hb_lost=%d psave=%d V=%u ign_hw=%d ign=%d harness=%u "
         "faults=0x%x configured=%d hb_eng=%d TSK=%d eps_hca=%u(%s) apply=%d cmd=%d lat=%d ldw=%d",
         s.safety_mode_pkt, s.safety_param_pkt, s.controls_allowed_pkt ? 1 : 0, s.heartbeat_lost_pkt ? 1 : 0,
         s.power_save_enabled_pkt ? 1 : 0, s.voltage_pkt, ignition_hw ? 1 : 0, ignition ? 1 : 0,
         s.car_harness_status_pkt, s.faults_pkt, safety_configured_ ? 1 : 0, hb_engaged ? 1 : 0, log.last_tsk_status,
         log.eps_hca_status, epsHcaStatusName(log.eps_hca_status), log.apply_steer_last, log.hca_cmd_steer,
         log.lat_cmd_active ? 1 : 0, log.ldw_valid ? 1 : 0);
  }

  return s;
}

}  // namespace volkswagen
