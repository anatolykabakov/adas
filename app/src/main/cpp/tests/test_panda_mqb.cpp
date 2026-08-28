#include <cstddef>
#include <cstring>

#include <gtest/gtest.h>

#include "adas/panda/health.h"
#include "adas/platform/car_platform.h"
#include "adas/platform/volkswagen/carcontroller.h"
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

/**
 * A re-seated panda is a board we have never configured, and the supervisor must treat it as one — while
 * keeping what belongs to the car. Both halves matter: forgetting too little leaves the board without the
 * ALKA bit, forgetting too much re-arms the ignition debounce and costs three seconds of actuation.
 */
TEST(PandaSafety, ReseatForgetsTheBoardAndKeepsTheCar)
{
  using C = volkswagen::MqbSafetyConstants;
  PandaSafetySupervisor s;
  s.setAlternativeExperience(C::kAltExpDisableDisengageOnGas | C::kAltExpAlka);
  ASSERT_TRUE(s.updateIgnitionSticky(true, 12000, 0)) << "the car is on before the phone slept";

  s.resetBoardState();

  // Board state: forgotten, so the next tick re-inits and nothing may actuate until it has.
  EXPECT_EQ(s.lastSafetyMode(), C::kNoOutput);
  EXPECT_FALSE(s.lastControlsAllowed());
  EXPECT_FALSE(s.safetyConfigured());
  // Our own intent is not board state — it is what we are about to write to the new board.
  EXPECT_EQ(s.alternativeExperience(), C::kAltExpDisableDisengageOnGas | C::kAltExpAlka);
  // Ignition belongs to the car: the debounce is not re-armed, so a low reading right after the reseat
  // starts the timer rather than declaring the car off.
  EXPECT_TRUE(s.updateIgnitionSticky(false, 10000, 100));
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

// the `alternativeExperience` dragonpilot ran on this car, read out of its own `pandaStates`.
TEST(AlwaysOnLateral, TheAlternativeExperienceMatchesWhatWorkedOnThisFirmware)
{
  using C = volkswagen::MqbSafetyConstants;
  EXPECT_EQ(C::kAltExpDisableDisengageOnGas, 1);
  EXPECT_EQ(C::kAltExpAlka, 16);
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

// that prompted this work `TSK_Status` was 2 (main on, not engaged) for 70.7 % of the time.
TEST(AlwaysOnLateral, TheMainSwitchIsWhatTheGateNeeds)
{
  using volkswagen::cruiseAvailableFromTsk;
  using volkswagen::cruiseEngagedFromTsk;
  EXPECT_TRUE(cruiseAvailableFromTsk(2));
  EXPECT_FALSE(cruiseEngagedFromTsk(2)) << "TSK_Status 2 is exactly the 70.7 % where we used to give up";
}

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
  EXPECT_TRUE(lateralActuationAllowed(true, false, cs));
  EXPECT_FALSE(lateralActuationAllowed(false, false, cs)) << "this is the 70.7 % we used to give up";

  // `controls_allowed` goes false — that is the whole point.
  EXPECT_TRUE(lateralActuationAllowed(false, true, cs));

  cs.gearReverse = true;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs));
  EXPECT_TRUE(lateralActuationAllowed(true, true, cs)) << "if the panda already allows control, honour it";
  cs.gearReverse = false;

  cs.cruiseAvailable = false;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs));
  cs.cruiseAvailable = true;

  cs.gearKnown = false;
  EXPECT_FALSE(lateralActuationAllowed(false, true, cs)) << "unknown gear must not open lateral actuation";
  EXPECT_TRUE(lateralActuationAllowed(true, true, cs)) << "panda already allows control — honour it";
  cs.gearKnown = true;

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

/** The car is chosen by name, and an unknown name must not fall back to a car we happen to have:
 *  guessing the make wrong is guessing the CAN layout wrong, and a frame built for the wrong layout
 *  still means something on the bus it is sent to. */
TEST(CarPlatformFactory, KnownNameBuildsAndUnknownRefuses)
{
  adas::platform::CarPlatformOptions opts;

  for (const std::string& name : adas::platform::knownCarPlatforms()) {
    auto car = adas::platform::makeCarPlatform(name, opts);
    ASSERT_NE(car, nullptr) << name << " is advertised as known";
    EXPECT_NE(car->dbcAssetName(), nullptr) << name << " must say which CAN database decodes it";
  }

  EXPECT_EQ(adas::platform::makeCarPlatform("", opts), nullptr);
  EXPECT_EQ(adas::platform::makeCarPlatform("honda_civic", opts), nullptr);
  EXPECT_EQ(adas::platform::makeCarPlatform("vw_golf_7", opts), nullptr) << "a near miss is still a miss";
}

/**
 * Whatever the brand, a freshly seated descriptor must leave the car unable to actuate: the safety model
 * lives on the board, and a board we have not configured has not got it.
 */
TEST(CarPlatformFactory, ReseatingLeavesEveryCarUnableToActuate)
{
  for (const std::string& name : adas::platform::knownCarPlatforms()) {
    auto car = adas::platform::makeCarPlatform(name, {});
    ASSERT_NE(car, nullptr) << name;
    car->configureSafety();
    car->resetPandaState();
    EXPECT_FALSE(car->safetyModelOk()) << name << " thinks an unconfigured board is in the right mode";
    EXPECT_FALSE(car->lateralActuationAllowed()) << name << " would steer through an unconfigured board";
  }
}

/** The steering envelope reaches the controller through the neutral interface, not through the brand. */
TEST(CarPlatformFactory, SteerLimitsComeThroughTheInterface)
{
  auto car = adas::platform::makeCarPlatform("vw_golf_7_mqb", {});
  ASSERT_NE(car, nullptr);
  const adas::platform::SteerLimits lim = car->steerLimits();
  EXPECT_EQ(lim.maxTorqueCNm, 300);
  EXPECT_EQ(lim.stepFrames, 2);
  EXPECT_GT(lim.deltaDownPerStep, lim.deltaUpPerStep) << "releasing the rack is always safe, so it may be faster";
  EXPECT_GT(lim.driverAllowanceCNm, 0);
}

// --- the acceleration frames ---------------------------------------------------------------------------

namespace {
int bits(const can_frame& f, int start, int length)
{
  int v = 0;
  for (int i = 0; i < length; ++i) {
    const int bit = start + i;
    const uint8_t byte = static_cast<uint8_t>(f.dat[bit / 8]);
    v |= ((byte >> (bit % 8)) & 1) << i;
  }
  return v;
}
}  // namespace

TEST(MqbAccFrames, Acc06CarriesTheRequestWithACountedChecksum)
{
  volkswagen::AccControl acc;
  acc.enabled = true;
  acc.accel_ms2 = -1.0f;
  acc.acc_type = 1;
  acc.acc_status = 3;
  uint8_t counter = 5;
  const auto f = volkswagen::create_acc_06(0, acc, &counter);
  EXPECT_EQ(0x122, f.address);
  ASSERT_EQ(8u, f.dat.size());
  EXPECT_EQ(5, bits(f, 8, 4));
  EXPECT_EQ(6, counter);
  // ACC_Sollbeschleunigung_02: (−1.0 + 7.22) / 0.005 = 1244
  EXPECT_EQ(1244, bits(f, 24, 11));
  EXPECT_EQ(80, bits(f, 40, 8));  // 4.0 m/s³ / 0.05
  EXPECT_EQ(1, bits(f, 58, 2));
  EXPECT_EQ(3, bits(f, 60, 3));
  const uint8_t crc = volkswagen::volkswagen_mqb_checksum(0x122, reinterpret_cast<const uint8_t*>(f.dat.data()), 8);
  EXPECT_EQ(crc, static_cast<uint8_t>(f.dat[0]));

  // The panda decodes the same field the same way: ((B4 & 7) << 8 | B3) * 5 − 7220 = −1000 milli-m/s².
  const uint8_t b3 = static_cast<uint8_t>(f.dat[3]);
  const uint8_t b4 = static_cast<uint8_t>(f.dat[4]);
  EXPECT_EQ(-1000, ((((b4 & 0x07) << 8) | b3) * 5) - 7220);
}

TEST(MqbAccFrames, InactiveFramesSayNothingAsked)
{
  volkswagen::AccControl acc;  // enabled = false
  uint8_t c6 = 0, c7 = 0;
  const auto f6 = volkswagen::create_acc_06(0, acc, &c6);
  const auto f7 = volkswagen::create_acc_07(0, acc, &c7);
  // 3.01 m/s² → (3.01 + 7.22) / 0.005 = 2046 in both frames, gradients zero, stop distance 20.46.
  EXPECT_EQ(2046, bits(f6, 24, 11));
  EXPECT_EQ(0, bits(f6, 40, 8));
  EXPECT_EQ(2046, bits(f7, 53, 11));
  EXPECT_EQ(2046, bits(f7, 12, 11));
  // ACC_Folgebeschl must always read 3.02: (3.02 + 4.6) / 0.03 = 254 — the panda checks exactly this.
  EXPECT_EQ(254, bits(f7, 32, 8));
  EXPECT_EQ((254 * 30) - 4600, 3020);
}

TEST(MqbAccFrames, Acc07EncodesStoppingAndHold)
{
  volkswagen::AccControl acc;
  acc.enabled = true;
  acc.accel_ms2 = -0.5f;
  acc.stopping = true;
  uint8_t c = 0;
  auto f = volkswagen::create_acc_07(0, acc, &c);
  EXPECT_EQ(0x12E, f.address);
  EXPECT_EQ(75, bits(f, 12, 11));  // 0.75 m stop distance
  EXPECT_EQ(1, bits(f, 23, 1));    // ACC_Anhalten
  EXPECT_EQ(1, bits(f, 28, 3));    // hold request
  acc.stopping = false;
  acc.starting = true;
  f = volkswagen::create_acc_07(0, acc, &c);
  EXPECT_EQ(4, bits(f, 28, 3));    // hold release
  EXPECT_EQ(1, bits(f, 31, 1));    // ACC_Anfahren
  const uint8_t crc = volkswagen::volkswagen_mqb_checksum(0x12E, reinterpret_cast<const uint8_t*>(f.dat.data()), 8);
  EXPECT_EQ(crc, static_cast<uint8_t>(f.dat[0]));
}

TEST(MqbAccFrames, TheCarControllerSendsAccOnlyWhenLongControlIsOn)
{
  volkswagen::CarController cc;
  volkswagen::CarControl req;
  volkswagen::CarStateView cs;
  cs.epsHcaStatus = 3;
  cs.vEgo = 20.f;
  req.longActive = true;
  req.actuators.accelMs2 = 0.5f;
  auto frames = cc.update(req, cs);
  for (const auto& f : frames)
    EXPECT_NE(0x122, f.address) << "ACC_06 without the switch would fight the stock ACC";

  volkswagen::CarController on;
  on.setLongControlEnabled(true);
  frames = on.update(req, cs);
  bool saw06 = false, saw07 = false, saw02 = false;
  for (const auto& f : frames) {
    saw06 |= f.address == 0x122;
    saw07 |= f.address == 0x12E;
    saw02 |= f.address == 0x30C;
  }
  EXPECT_TRUE(saw06 && saw07 && saw02);
  EXPECT_NEAR(0.5f, on.applyAccelLast(), 1e-6);
}

TEST(MqbAccFrames, TheLongGateNeedsControlsAllowedAndNoBrake)
{
  volkswagen::CarStateView cs;
  cs.cruiseAvailable = true;
  cs.gearKnown = true;
  EXPECT_TRUE(volkswagen::longitudinalActuationAllowed(true, cs));
  EXPECT_FALSE(volkswagen::longitudinalActuationAllowed(false, cs)) << "no always-on longitudinal";
  cs.brakePressed = true;
  EXPECT_FALSE(volkswagen::longitudinalActuationAllowed(true, cs));
  cs.brakePressed = false;
  cs.gearReverse = true;
  EXPECT_FALSE(volkswagen::longitudinalActuationAllowed(true, cs));
}

TEST(MqbAccFrames, TheSafetyParamFollowsTheSwitch)
{
  using C = volkswagen::MqbSafetyConstants;
  EXPECT_EQ(1, C::kParamLongControl);
  auto lateral_only = adas::platform::makeCarPlatform("vw_golf_7_mqb", {});
  ASSERT_TRUE(lateral_only);
  EXPECT_FALSE(lateral_only->longLimits().supported);
  adas::platform::CarPlatformOptions o;
  o.long_control_enabled = true;
  auto with_long = adas::platform::makeCarPlatform("vw_golf_7_mqb", o);
  ASSERT_TRUE(with_long);
  EXPECT_TRUE(with_long->longLimits().supported);
  EXPECT_FLOAT_EQ(2.0f, with_long->longLimits().accelMaxMs2);
  EXPECT_FLOAT_EQ(-3.5f, with_long->longLimits().accelMinMs2);
  EXPECT_FALSE(with_long->longitudinalActuationAllowed()) << "no panda report yet: nothing may go out";
}

TEST(CarPlatformFactory, EveryMqbVariantCarriesItsOwnGeometry)
{
  auto golf = adas::platform::makeCarPlatform("vw_golf_7_mqb", {});
  auto passat = adas::platform::makeCarPlatform("vw_passat_b8_mqb", {});
  ASSERT_TRUE(golf && passat);
  EXPECT_STREQ("vw_passat_b8_mqb", passat->name());
  EXPECT_STREQ("vw_mqb_2010.dbc", passat->dbcAssetName()) << "one CAN layout for the whole platform";
  EXPECT_NEAR(2.636, golf->defaults().wheelbase_m, 1e-9);
  EXPECT_NEAR(2.79, passat->defaults().wheelbase_m, 1e-9);
  EXPECT_GT(passat->defaults().mass_kg, golf->defaults().mass_kg);
  EXPECT_FALSE(adas::platform::makeCarPlatform("vw_golf_8", {})) << "a Golf 8 is not MQB; guessing it would guess the bus";
}
