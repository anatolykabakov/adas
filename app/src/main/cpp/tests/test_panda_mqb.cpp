#include <cstddef>
#include <cstring>

#include <gtest/gtest.h>

#include "panda/health.h"
#include "volkswagen/mqb_car_state_decoder.h"
#include "volkswagen/panda_safety_supervisor.h"

using volkswagen::cruiseAvailableFromTsk;
using volkswagen::cruiseEngagedFromTsk;
using volkswagen::isAllowedMqbRxAddress;
using volkswagen::PandaSafetySupervisor;

TEST(MqbDecoder, AllowlistContainsCoreIds)
{
  EXPECT_TRUE(isAllowedMqbRxAddress(0x086));
  EXPECT_TRUE(isAllowedMqbRxAddress(0x09F));
  EXPECT_TRUE(isAllowedMqbRxAddress(0x120));
  EXPECT_TRUE(isAllowedMqbRxAddress(0x126));
  EXPECT_FALSE(isAllowedMqbRxAddress(0x001));
}

TEST(MqbDecoder, TskCruiseMapping)
{
  EXPECT_FALSE(cruiseEngagedFromTsk(2));
  EXPECT_TRUE(cruiseAvailableFromTsk(2));
  EXPECT_TRUE(cruiseEngagedFromTsk(3));
  EXPECT_TRUE(cruiseEngagedFromTsk(4));
  EXPECT_TRUE(cruiseEngagedFromTsk(5));
  EXPECT_FALSE(cruiseEngagedFromTsk(0));
  EXPECT_FALSE(cruiseAvailableFromTsk(0));
}

TEST(PandaSafety, IgnitionStickyHysteresisAndDebounce)
{
  PandaSafetySupervisor s;

  EXPECT_TRUE(s.updateIgnitionSticky(false, 12000, 0));

  EXPECT_TRUE(s.updateIgnitionSticky(false, 11000, 1000));

  EXPECT_TRUE(s.updateIgnitionSticky(false, 10000, 2000));
  EXPECT_TRUE(s.updateIgnitionSticky(false, 10000, 4000));
  EXPECT_FALSE(s.updateIgnitionSticky(false, 10000, 5500));

  EXPECT_TRUE(s.updateIgnitionSticky(true, 10000, 6000));
}

TEST(PandaSafety, ConstantsMatchContract)
{
  using C = volkswagen::MqbSafetyConstants;
  EXPECT_EQ(C::kVolkswagen, 15);
  EXPECT_EQ(C::kNoOutput, 19);
  EXPECT_EQ(C::kIgnVoltageOnMv, 11500u);
  EXPECT_EQ(C::kIgnVoltageOffMv, 10500u);
  EXPECT_EQ(C::kIgnOffDebounceMs, 3000);
  EXPECT_EQ(C::kSafetyRetryBaseMs, 1000);
  EXPECT_EQ(C::kSafetyRetryMaxMs, 10000);
}

TEST(PandaHealth, PacketLayoutMatchesFirmwareV16)
{
  EXPECT_EQ(HEALTH_PACKET_VERSION, 16);
  EXPECT_EQ(offsetof(health_t, faults_pkt), 28u);
  EXPECT_EQ(offsetof(health_t, car_harness_status_pkt), 35u);
  EXPECT_EQ(offsetof(health_t, safety_mode_pkt), 36u);
  EXPECT_EQ(offsetof(health_t, power_save_enabled_pkt), 40u);
  EXPECT_EQ(sizeof(health_t), 58u);
}

TEST(PandaHealth, PacketLayoutMatchesFirmwareV11)
{
  EXPECT_EQ(HEALTH_PACKET_VERSION_V11, 11);
  EXPECT_EQ(offsetof(health_v11_t, faults_pkt), 32u);
  EXPECT_EQ(offsetof(health_v11_t, ignition_line_pkt), 36u);
  EXPECT_EQ(offsetof(health_v11_t, car_harness_status_pkt), 40u);
  EXPECT_EQ(offsetof(health_v11_t, safety_mode_pkt), 41u);
  EXPECT_EQ(sizeof(health_v11_t), 57u);
}

TEST(PandaHealth, V11ConversionRecoversIgnitionAndSafetyMode)
{
  health_v11_t v{};
  v.uptime_pkt = 53;
  v.voltage_pkt = 12601;
  v.rx_buffer_overflow_pkt = 67912;
  v.ignition_line_pkt = 1;
  v.controls_allowed_pkt = 1;
  v.car_harness_status_pkt = 1;
  v.safety_mode_pkt = 19;
  v.alternative_experience_pkt = 1;
  v.safety_rx_checks_invalid_pkt = 3;

  const health_t h = healthFromV11(v);
  EXPECT_EQ(h.uptime_pkt, 53u);
  EXPECT_EQ(h.voltage_pkt, 12601u);
  EXPECT_EQ(h.rx_buffer_overflow_pkt, 67912u);
  EXPECT_EQ(h.ignition_line_pkt, 1);
  EXPECT_EQ(h.controls_allowed_pkt, 1);
  EXPECT_EQ(h.car_harness_status_pkt, 1);
  EXPECT_EQ(h.safety_mode_pkt, 19);
  EXPECT_EQ(h.alternative_experience_pkt, 1);
  EXPECT_EQ(h.safety_rx_checks_invalid_pkt, 3);

  EXPECT_EQ(h.spi_checksum_error_count_pkt, 0);
  EXPECT_EQ(h.sbu1_voltage_mV, 0);
  EXPECT_EQ(h.sbu2_voltage_mV, 0);
  EXPECT_EQ(h.som_reset_triggered, 0);
}

TEST(PandaHealth, V11ReadAsV16MisparsesExactlyAsInTheRun)
{
  health_v11_t v{};
  v.ignition_line_pkt = 1;
  v.controls_allowed_pkt = 1;
  v.car_harness_status_pkt = 1;
  v.safety_mode_pkt = 19;
  v.alternative_experience_pkt = 1;

  health_t wrong{};
  static_assert(sizeof(health_v11_t) < sizeof(health_t), "v11 shorter than v16");
  std::memcpy(&wrong, &v, sizeof(v));

  EXPECT_EQ(wrong.ignition_line_pkt, 0);
  EXPECT_EQ(wrong.safety_mode_pkt, 1);
  EXPECT_EQ(wrong.safety_param_pkt, 256);
  EXPECT_NE(wrong.heartbeat_lost_pkt, 0);
  EXPECT_NE(wrong.power_save_enabled_pkt, 0);
  EXPECT_FLOAT_EQ(wrong.interrupt_load_pkt, 2.3509887e-38f);
}
