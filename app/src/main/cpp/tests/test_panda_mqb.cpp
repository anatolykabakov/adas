#include <cstddef>
#include <cstring>

#include <gtest/gtest.h>

#include "adas/panda/health.h"
#include "adas/platform/volkswagen/mqb_car_state_decoder.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"

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

// Always-on lateral. Both halves of it are one number and one gate, and the number is not a guess: it is
// the `alternativeExperience` dragonpilot ran on this car, read out of its own `pandaStates`.
TEST(AlwaysOnLateral, TheAlternativeExperienceMatchesWhatWorkedOnThisFirmware)
{
  using C = volkswagen::MqbSafetyConstants;
  EXPECT_EQ(C::kAltExpDisableDisengageOnGas, 1);
  EXPECT_EQ(C::kAltExpAlka, 16);
  // 17 is the value observed in their logs, over every route, with `safety_tx_blocked` never incrementing
  // and torque applied in 96.3 % of the frames where `controls_allowed` was false.
  EXPECT_EQ(C::kAltExpDisableDisengageOnGas | C::kAltExpAlka, 17);
}

TEST(AlwaysOnLateral, TheSupervisorDefaultsToTheConservativeValue)
{
  volkswagen::PandaSafetySupervisor s;
  EXPECT_EQ(s.alternativeExperience(), volkswagen::MqbSafetyConstants::kAltExpDisableDisengageOnGas) << "the ALKA bit "
                                                                                                        "must not "
                                                                                                        "appear "
                                                                                                        "without the "
                                                                                                        "flag that "
                                                                                                        "also opens "
                                                                                                        "the gate";
  s.setAlternativeExperience(17);
  EXPECT_EQ(s.alternativeExperience(), 17);
}

// `Getriebe_11.GE_Fahrstufe` per vw_mqb_2010.dbc: 5 P, 6 R, 7 N, 8 D, 9 S, 10 E, 13/14 T.
TEST(AlwaysOnLateral, ReverseIsGearSix)
{
  using volkswagen::gearIsReverse;
  EXPECT_TRUE(gearIsReverse(6));
  for (int g : {0, 5, 7, 8, 9, 10, 13, 14})
    EXPECT_FALSE(gearIsReverse(g)) << "gear " << g;
  EXPECT_FALSE(gearIsReverse(0)) << "0 is 'frame not seen yet', not a gear";
}

// The gate keys on the main switch, not on engagement — that distinction is the whole feature. On the run
// that prompted this work `TSK_Status` was 2 (main on, not engaged) for 70.7 % of the time.
TEST(AlwaysOnLateral, TheMainSwitchIsWhatTheGateNeeds)
{
  using volkswagen::cruiseAvailableFromTsk;
  using volkswagen::cruiseEngagedFromTsk;
  EXPECT_TRUE(cruiseAvailableFromTsk(2));
  EXPECT_FALSE(cruiseEngagedFromTsk(2)) << "TSK_Status 2 is exactly the 70.7 % where we used to give up";
}

// The decision that puts torque on the rack, as a truth table. Every row is a case that occurs on the road,
// and two of them are the ones this feature exists for.
TEST(AlwaysOnLateral, TheGateTruthTable)
{
  using volkswagen::CarStateView;
  using volkswagen::lateralActuationAllowed;

  CarStateView cs;
  cs.cruiseAvailable = true;
  cs.gearReverse = false;
  cs.gearKnown = true;

  // Off: exactly the old behaviour, whatever the car state says.
  EXPECT_TRUE(lateralActuationAllowed(/*controls_allowed=*/true, /*always_on=*/false, cs));
  EXPECT_FALSE(lateralActuationAllowed(false, false, cs)) << "this is the 70.7 % we used to give up";

  // On: the main switch is enough. Below ~30 km/h and on brake the stock cruise disengages and
  // `controls_allowed` goes false — that is the whole point.
  EXPECT_TRUE(lateralActuationAllowed(false, true, cs));

  // Reverse is a hard no even with the switch on.
  cs.gearReverse = true;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs));
  EXPECT_TRUE(lateralActuationAllowed(true, true, cs)) << "if the panda already allows control, honour it";
  cs.gearReverse = false;

  // Main switch off — the driver has not asked for any assistance at all.
  cs.cruiseAvailable = false;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs));
  cs.cruiseAvailable = true;

  // Gear not seen yet: Getriebe_11 reads as 0, so "not reverse" is formally true but substantially
  // unknown — between app start and the first frame the car may be in reverse.
  cs.gearKnown = false;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs)) << "unknown gear must not open lateral actuation";
  EXPECT_TRUE(lateralActuationAllowed(true, true, cs)) << "panda already allows control — honour it";
  cs.gearKnown = true;

  // And the superset property the bag analysis relies on: whenever `controls_allowed` holds, so does this.
  for (bool avail : {false, true})
    for (bool rev : {false, true})
      for (bool on : {false, true}) {
        CarStateView c;
        c.cruiseAvailable = avail;
        c.gearReverse = rev;
        c.gearKnown = true;
        EXPECT_TRUE(lateralActuationAllowed(true, on, c)) << "controls_allowed must imply actuation-allowed — "
                                                             "`vis.bag_io.lateral_actuation_on` ORs on it";
      }
}
