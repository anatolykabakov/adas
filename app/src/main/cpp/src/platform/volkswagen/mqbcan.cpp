#include "adas/platform/volkswagen/mqbcan.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace volkswagen {
namespace {
constexpr long MSG_HCA_01 = 0x126;
constexpr long MSG_LDW_02 = 0x397;
constexpr long MSG_ACC_06 = 0x122;
constexpr long MSG_ACC_07 = 0x12E;
constexpr long MSG_ACC_02 = 0x30C;

// The CRC secret is a per-message table indexed by the 4-bit counter (opendbc's volkswagen_mqb_checksum).
// HCA_01 and ACC_02 happen to use a constant; the two acceleration frames do not.
constexpr uint8_t HCA_01_SECRET[16] = {0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA,
                                       0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA};
constexpr uint8_t ACC_02_SECRET[16] = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
                                       0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
constexpr uint8_t ACC_06_SECRET[16] = {0x37, 0x7D, 0xF3, 0xA9, 0x18, 0x46, 0x6D, 0x4D,
                                       0x3D, 0x71, 0x92, 0x9C, 0xE5, 0x32, 0x10, 0xB9};
constexpr uint8_t ACC_07_SECRET[16] = {0xF8, 0xE5, 0x97, 0xC9, 0xD6, 0x07, 0x47, 0x21,
                                       0x66, 0xDD, 0xCF, 0x6F, 0xA1, 0x94, 0x74, 0x63};

const uint8_t* crc8_lut_8h2f()
{
  static uint8_t lut[256];
  static bool init = false;
  if (!init) {
    for (int i = 0; i < 256; ++i) {
      uint8_t crc = static_cast<uint8_t>(i);
      for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x2F) : static_cast<uint8_t>(crc << 1);
      }
      lut[i] = crc;
    }
    init = true;
  }
  return lut;
}

const uint8_t* secretFor(long address)
{
  switch (address) {
    case MSG_HCA_01:
      return HCA_01_SECRET;
    case MSG_ACC_02:
      return ACC_02_SECRET;
    case MSG_ACC_06:
      return ACC_06_SECRET;
    case MSG_ACC_07:
      return ACC_07_SECRET;
    default:
      return nullptr;
  }
}

void set_bits(uint8_t* data, int value, int start, int length)
{
  for (int bit_index = start; bit_index < start + length; ++bit_index) {
    const int byte_index = bit_index / 8;
    const int bit_offset = bit_index % 8;
    if ((value >> (bit_index - start)) & 1) {
      data[byte_index] |= static_cast<uint8_t>(1u << bit_offset);
    } else {
      data[byte_index] &= static_cast<uint8_t>(~(1u << bit_offset));
    }
  }
}

/// DBC physical value → raw field, rounded and clamped to the field width.
int phys(float value, float scale, float offset, int bits)
{
  const int raw = static_cast<int>(std::lround((value - offset) / scale));
  return std::clamp(raw, 0, (1 << bits) - 1);
}

can_frame make_frame(long address, int bus, const uint8_t* data, size_t len)
{
  can_frame f;
  f.address = address;
  f.src = bus;
  f.busTime = 0;
  f.dat.assign(reinterpret_cast<const char*>(data), len);
  return f;
}

uint8_t take_counter(uint8_t* counter)
{
  const uint8_t cnt = counter ? (*counter & 0x0F) : 0;
  if (counter)
    *counter = static_cast<uint8_t>((cnt + 1) & 0x0F);
  return cnt;
}

}  // namespace

uint8_t volkswagen_mqb_checksum(long address, const uint8_t* data, size_t len)
{
  const uint8_t* lut = crc8_lut_8h2f();
  uint8_t crc = 0xFF;
  for (size_t i = 1; i < len; ++i) {
    crc ^= data[i];
    crc = lut[crc];
  }
  const uint8_t counter = data[1] & 0x0F;
  if (const uint8_t* secret = secretFor(address))
    crc ^= secret[counter];
  crc = lut[crc];
  return static_cast<uint8_t>(crc ^ 0xFF);
}

can_frame create_steering_control(int bus, int apply_steer, bool lkas_enabled, uint8_t* counter)
{
  apply_steer = std::clamp(apply_steer, -CarControllerParams::STEER_MAX, CarControllerParams::STEER_MAX);
  uint8_t data[8] = {};
  set_bits(data, take_counter(counter), 8, 4);
  set_bits(data, 0x3, 12, 4);
  set_bits(data, std::abs(apply_steer), 16, 14);
  set_bits(data, lkas_enabled ? 1 : 0, 30, 1);
  set_bits(data, apply_steer < 0 ? 1 : 0, 31, 1);
  set_bits(data, 1, 32, 1);
  set_bits(data, lkas_enabled ? 0 : 1, 33, 1);
  set_bits(data, lkas_enabled ? 1 : 0, 34, 1);
  set_bits(data, 0xFE, 40, 8);
  set_bits(data, 0x07, 48, 8);
  data[0] = volkswagen_mqb_checksum(MSG_HCA_01, data, 8);
  return make_frame(MSG_HCA_01, bus, data, 8);
}

can_frame create_lka_hud_control(int bus, const LdwStockValues& ldw_stock, bool enabled, bool steering_pressed,
                                 int hud_alert, const HudControl& hud)
{
  uint8_t data[8] = {};
  if (ldw_stock.valid) {
    std::memcpy(data, ldw_stock.data, 8);
  }
  set_bits(data, (enabled && steering_pressed) ? 1 : 0, 61, 1);
  set_bits(data, (enabled && !steering_pressed) ? 1 : 0, 62, 1);
  const int left = hud.leftLaneDepart ? 3 : (1 + (hud.leftLaneVisible ? 1 : 0));
  const int right = hud.rightLaneDepart ? 3 : (1 + (hud.rightLaneVisible ? 1 : 0));
  set_bits(data, left, 38, 2);
  set_bits(data, right, 36, 2);
  set_bits(data, hud_alert & 0xF, 16, 4);
  return make_frame(MSG_LDW_02, bus, data, 8);
}

can_frame create_acc_06(int bus, const AccControl& acc, uint8_t* counter)
{
  using P = CarControllerParams;
  uint8_t data[8] = {};
  set_bits(data, take_counter(counter), 8, 4);
  // ACC_zul_Regelabw_unten @16|6 (0.024) and _oben @35|5 (0.0625): the tolerance band, 0.2 m/s² both ways.
  set_bits(data, phys(0.2f, 0.024f, 0.f, 6), 16, 6);
  set_bits(data, acc.enabled ? 1 : 0, 22, 2);  // ACC_StartStopp_Info
  // ACC_Sollbeschleunigung_02 @24|11 (0.005, −7.22): the request, or 3.01 for "nothing asked".
  const float accel = acc.enabled ? std::clamp(acc.accel_ms2, P::ACCEL_MIN, P::ACCEL_MAX) : P::ACCEL_INACTIVE;
  set_bits(data, phys(accel, 0.005f, -7.22f, 11), 24, 11);
  set_bits(data, phys(0.2f, 0.0625f, 0.f, 5), 35, 5);
  // Gradients @40|8 and @48|8 (0.05): how fast the request may move, m/s³; zero when inactive.
  const int grad = acc.enabled ? phys(P::ACC_JERK_LIMIT, 0.05f, 0.f, 8) : 0;
  set_bits(data, grad, 40, 8);
  set_bits(data, grad, 48, 8);
  set_bits(data, acc.starting ? 1 : 0, 56, 1);  // ACC_Anfahren
  set_bits(data, acc.stopping ? 1 : 0, 57, 1);  // ACC_Anhalten
  set_bits(data, acc.acc_type & 0x3, 58, 2);    // ACC_Typ
  set_bits(data, acc.acc_status & 0x7, 60, 3);  // ACC_Status_ACC
  data[0] = volkswagen_mqb_checksum(MSG_ACC_06, data, 8);
  return make_frame(MSG_ACC_06, bus, data, 8);
}

can_frame create_acc_07(int bus, const AccControl& acc, uint8_t* counter)
{
  using P = CarControllerParams;
  uint8_t data[8] = {};
  set_bits(data, take_counter(counter), 8, 4);
  // ACC_Anhalteweg @12|11 (0.01): stop distance; 20.46 is one step past the field's range = inactive.
  set_bits(data, phys(acc.stopping ? 0.75f : 20.46f, 0.01f, 0.f, 11), 12, 11);
  set_bits(data, acc.stopping ? 1 : 0, 23, 1);           // ACC_Anhalten
  set_bits(data, acc.enabled ? 2 : 0, 26, 2);            // ACC_Freilauf_Info
  // ACC_Anforderung_HMS @28|3: 0 none, 1 hold request, 3 hold standby, 4 hold release.
  const int hold = acc.starting ? 4 : (acc.esp_hold ? 3 : (acc.stopping ? 1 : 0));
  set_bits(data, hold, 28, 3);
  set_bits(data, acc.starting ? 1 : 0, 31, 1);           // ACC_Anfahren
  // ACC_Folgebeschl @32|8 (0.03, −4.6): always the inactive 3.02 — the panda refuses anything else.
  set_bits(data, phys(3.02f, 0.03f, -4.6f, 8), 32, 8);
  const float accel = acc.enabled ? std::clamp(acc.accel_ms2, P::ACCEL_MIN, P::ACCEL_MAX) : P::ACCEL_INACTIVE;
  set_bits(data, phys(accel, 0.005f, -7.22f, 11), 53, 11);  // ACC_Sollbeschleunigung_02
  data[0] = volkswagen_mqb_checksum(MSG_ACC_07, data, 8);
  return make_frame(MSG_ACC_07, bus, data, 8);
}

can_frame create_acc_02(int bus, const AccHud& hud, uint8_t* counter)
{
  uint8_t data[8] = {};
  set_bits(data, take_counter(counter), 8, 4);
  // ACC_Wunschgeschw_02 @12|10 (0.32 km/h); 327.36 is the "no speed" value the cluster leaves blank.
  const float kph = hud.set_speed_kph < 250.f ? hud.set_speed_kph : 327.36f;
  set_bits(data, phys(kph, 0.32f, 0.f, 10), 12, 10);
  set_bits(data, std::clamp(hud.lead_distance, 0, 1023), 24, 10);  // ACC_Abstandsindex
  set_bits(data, std::clamp(hud.time_gap, 0, 7), 37, 3);           // ACC_Gesetzte_Zeitluecke
  set_bits(data, 3, 44, 2);                                        // ACC_Display_Prio
  set_bits(data, hud.acc_status & 0x7, 61, 3);                     // ACC_Status_Anzeige
  data[0] = volkswagen_mqb_checksum(MSG_ACC_02, data, 8);
  return make_frame(MSG_ACC_02, bus, data, 8);
}

}  // namespace volkswagen
