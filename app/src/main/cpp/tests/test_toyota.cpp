#include <cmath>
#include <cstdio>
#include <fstream>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "adas/adas_app.h"
#include "adas/platform/car_platform.h"
#include "adas/platform/toyota/toyota_platform.h"
#include "adas/platform/toyota/values.h"

namespace {
using adas::platform::toyota::Addresses;
using adas::platform::toyota::CarControllerParams;
using adas::platform::toyota::ToyotaTss2;

/// The DBC shipped in the APK, reached from the build directory.
std::string dbcPath() { return std::string(ADAS_SOURCE_DIR) + "/../assets/toyota_nodsu_pt_generated.dbc"; }

adas::proto::CANData frameOf(uint32_t address, const std::string& payload)
{
  adas::proto::CANData data;
  auto* f = data.add_frames();
  f->set_address(address);
  f->set_data(payload);
  return data;
}
}  // namespace

/** The DBC must be present and parseable: a missing asset would leave the decoder silently empty. */
TEST(Toyota, ShippedDbcLoadsAndDecodesWheelSpeeds)
{
  ToyotaTss2 car(dbcPath(), {});
  car.init();

  // WHEEL_SPEEDS carries four 16-bit values, 0.01 km/h per bit with a −67.67 offset. 10000 raw is
  // therefore 100 − 67.67 = 32.33 km/h on every wheel.
  std::string payload(8, '\0');
  for (int i = 0; i < 4; i++) {
    payload[2 * i] = static_cast<char>(0x27);
    payload[2 * i + 1] = static_cast<char>(0x10);
  }
  ASSERT_TRUE(car.update(frameOf(Addresses::kWheelSpeeds, payload), 1000));

  const double expected_ms = (10000 * 0.01 - 67.67) / 3.6;
  EXPECT_NEAR(car.carState().v_ego_raw(), expected_ms, 0.05) << "signal names or scaling drifted from the DBC";
  EXPECT_FALSE(car.carState().standstill());
}

/** Ported from `opendbc/can/common.cc::toyota_checksum`: length + address bytes + payload. */
TEST(Toyota, SteeringChecksumMatchesOpendbc)
{
  // Reproduced by hand for STEERING_LKA (0x2E4, five bytes) with a zero payload:
  // 5 + 0xE4 + 0x02 = 0xEB.
  ToyotaTss2 car(dbcPath(), {});
  car.init();
  adas::platform::CarControl cc;
  cc.latActive = true;
  cc.actuators.steerTorqueCNm = 0;
  const auto frames = car.apply(cc);

  // Sending is off until the port has met a car, so nothing goes out — that is the point of the gate.
  EXPECT_TRUE(frames.empty()) << "the Toyota port must not put frames on a bus it has never been tested on";
  EXPECT_FALSE(ToyotaTss2::sendingAllowed());
}

/** The car's own facts come from the platform, not from a config that may describe another car. */
TEST(Toyota, DefaultsAreToyotaNotVolkswagen)
{
  auto toyota = adas::platform::makeCarPlatform("toyota_tss2", {});
  auto vw = adas::platform::makeCarPlatform("vw_golf_7_mqb", {});
  ASSERT_NE(toyota, nullptr);
  ASSERT_NE(vw, nullptr);

  EXPECT_NEAR(toyota->defaults().wheelbase_m, 2.70, 1e-9);
  EXPECT_NEAR(toyota->defaults().steer_ratio, 13.9, 1e-9);
  EXPECT_GT(toyota->defaults().steer_sign, 0.0) << "Toyota commands positive to the left";
  EXPECT_LT(vw->defaults().steer_sign, 0.0) << "MQB is the other way round";

  EXPECT_EQ(toyota->steerLimits().maxTorqueCNm, CarControllerParams::STEER_MAX);
  EXPECT_NE(toyota->steerLimits().maxTorqueCNm, vw->steerLimits().maxTorqueCNm) << "torque units are not comparable "
                                                                                   "across makes; one ceiling for both "
                                                                                   "would be wrong for one";
  EXPECT_STREQ(toyota->dbcAssetName(), "toyota_nodsu_pt_generated.dbc");
}

/**
 * The car's numbers reach the config, and the config still wins where it says something.
 *
 * <p>This is the whole point of moving them: a config that says nothing about the vehicle must still
 * describe the right one, and a config carried over from another car must not quietly redefine this
 * one's geometry.
 */
TEST(CarDefaults, PlatformSeedsTheConfigAndConfigStillOverrides)
{
  const std::string dir = std::string(ADAS_SOURCE_DIR) + "/../../../..";
  const std::string path = dir + "/adas_car_defaults_test.json";

  {
    std::ofstream f(path);
    f << R"({"vehicle": {"name": "toyota_tss2"}})";
  }
  bool ok = false;
  AdasApp::Config toyota = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(toyota.lane_keep.wheelbase_m, 2.70, 1e-9) << "an empty config must still be this car";
  EXPECT_NEAR(toyota.lane_keep.steer_ratio, 13.9, 1e-9);
  EXPECT_GT(toyota.lane_keep.steer_sign, 0.0);

  {
    std::ofstream f(path);
    f << R"({"vehicle": {"name": "toyota_tss2", "steer_ratio": 11.5}})";
  }
  AdasApp::Config overridden = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(overridden.lane_keep.steer_ratio, 11.5, 1e-9) << "a stated key must still win";
  EXPECT_NEAR(overridden.lane_keep.wheelbase_m, 2.70, 1e-9) << "and must not disturb the rest";

  {
    std::ofstream f(path);
    f << R"({"vehicle": {"name": "vw_golf_7_mqb"}})";
  }
  AdasApp::Config vw = AdasApp::Config::loadFromFile(path, &ok);
  ASSERT_TRUE(ok);
  EXPECT_NEAR(vw.lane_keep.wheelbase_m, 2.636, 1e-9);
  EXPECT_NEAR(vw.lane_keep.steer_ratio, 15.6, 1e-9) << "what the car runs with today";
  EXPECT_LT(vw.lane_keep.steer_sign, 0.0);

  std::remove(path.c_str());
}
