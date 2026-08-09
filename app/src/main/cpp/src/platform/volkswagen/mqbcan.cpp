#include "adas/platform/volkswagen/mqbcan.h"

#include <algorithm>
#include <cstring>

namespace volkswagen {
namespace {

constexpr long MSG_HCA_01 = 0x126;
constexpr long MSG_LDW_02 = 0x397;
constexpr long MSG_GRA_ACC_01 = 0x12B;

constexpr uint8_t HCA_01_SECRET[16] = {0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA,
                                       0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA};

constexpr uint8_t GRA_ACC_01_SECRET[16] = {0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB,
                                           0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B};

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

uint8_t volkswagen_mqb_checksum_hca(const uint8_t* data, size_t len)
{
  const uint8_t* lut = crc8_lut_8h2f();
  uint8_t crc = 0xFF;
  for (size_t i = 1; i < len; ++i) {
    crc ^= data[i];
    crc = lut[crc];
  }
  const uint8_t counter = data[1] & 0x0F;
  crc ^= HCA_01_SECRET[counter];
  crc = lut[crc];
  return static_cast<uint8_t>(crc ^ 0xFF);
}

uint8_t volkswagen_mqb_checksum_gra(const uint8_t* data, size_t len)
{
  const uint8_t* lut = crc8_lut_8h2f();
  uint8_t crc = 0xFF;
  for (size_t i = 1; i < len; ++i) {
    crc ^= data[i];
    crc = lut[crc];
  }
  const uint8_t counter = data[1] & 0x0F;
  crc ^= GRA_ACC_01_SECRET[counter];
  crc = lut[crc];
  return static_cast<uint8_t>(crc ^ 0xFF);
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

can_frame make_frame(long address, int bus, const uint8_t* data, size_t len)
{
  can_frame f;
  f.address = address;
  f.src = bus;
  f.busTime = 0;
  f.dat.assign(reinterpret_cast<const char*>(data), len);
  return f;
}

}  // namespace

can_frame create_steering_control(int bus, int apply_steer, bool lkas_enabled, uint8_t* counter)
{
  apply_steer = std::clamp(apply_steer, -CarControllerParams::STEER_MAX, CarControllerParams::STEER_MAX);
  uint8_t data[8] = {};
  const uint8_t cnt = counter ? (*counter & 0x0F) : 0;
  set_bits(data, cnt, 8, 4);
  set_bits(data, 0x3, 12, 4);
  set_bits(data, std::abs(apply_steer), 16, 14);
  set_bits(data, lkas_enabled ? 1 : 0, 30, 1);
  set_bits(data, apply_steer < 0 ? 1 : 0, 31, 1);
  set_bits(data, 1, 32, 1);
  set_bits(data, lkas_enabled ? 0 : 1, 33, 1);
  set_bits(data, lkas_enabled ? 1 : 0, 34, 1);
  set_bits(data, 0xFE, 40, 8);
  set_bits(data, 0x07, 48, 8);
  data[0] = volkswagen_mqb_checksum_hca(data, 8);
  if (counter) {
    *counter = static_cast<uint8_t>((cnt + 1) & 0x0F);
  }
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

can_frame create_acc_buttons_control(int bus, const GraStockValues& gra_stock, const CruiseButtonCmd& cmd)
{
  uint8_t data[8] = {};
  if (gra_stock.valid) {
    std::memcpy(data, gra_stock.data, 8);
  }
  // Clear tip / cancel / resume / set — keep Hauptschalter, Codierung, Typ*, etc.
  set_bits(data, 0, 13, 1);  // GRA_Abbrechen
  set_bits(data, 0, 16, 1);  // GRA_Tip_Setzen
  set_bits(data, 0, 17, 1);  // GRA_Tip_Hoch
  set_bits(data, 0, 18, 1);  // GRA_Tip_Runter
  set_bits(data, 0, 19, 1);  // GRA_Tip_Wiederaufnahme

  const uint8_t cnt = static_cast<uint8_t>(((data[1] & 0x0F) + 1) & 0x0F);
  set_bits(data, cnt, 8, 4);

  if (cmd.cancel)
    set_bits(data, 1, 13, 1);
  if (cmd.set)
    set_bits(data, 1, 16, 1);
  if (cmd.tip_up)
    set_bits(data, 1, 17, 1);
  if (cmd.tip_down)
    set_bits(data, 1, 18, 1);
  if (cmd.resume)
    set_bits(data, 1, 19, 1);

  data[0] = volkswagen_mqb_checksum_gra(data, 8);
  return make_frame(MSG_GRA_ACC_01, bus, data, 8);
}

}  // namespace volkswagen
