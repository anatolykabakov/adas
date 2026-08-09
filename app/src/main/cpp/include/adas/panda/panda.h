#pragma once

#include <cstdint>
#include <ctime>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <vector>

#include "adas/panda/health.h"
#include "adas/panda/can_declarations.h"
#include "adas/panda/can.h"
#include "adas/panda/panda_comms.h"
#include "adas/panda/can_frame.h"

#define USB_TX_SOFT_LIMIT (0x100U)
#define USBPACKET_MAX_SIZE (0x40)

#define RECV_SIZE (0x4000U)

#define CAN_REJECTED_BUS_OFFSET 0xC0U
#define CAN_RETURNED_BUS_OFFSET 0x80U

namespace cereal::PandaState {
enum class PandaType : uint16_t {
  UNKNOWN,
  WHITE_PANDA,
  GREY_PANDA,
  BLACK_PANDA,
  PEDAL,
  UNO,
  DOS,
  RED_PANDA,
  RED_PANDA_V2,
  TRES,
};
}
struct __attribute__((packed)) can_header {
  uint8_t reserved : 1;
  uint8_t bus : 3;
  uint8_t data_len_code : 4;
  uint8_t rejected : 1;
  uint8_t returned : 1;
  uint8_t extended : 1;
  uint32_t addr : 29;
  uint8_t checksum : 8;
};

class Panda {
public:
  std::unique_ptr<PandaCommsHandle> handle;
  Panda(std::string serial = "", uint32_t bus_offset = 0);
  Panda(int fd, uint32_t bus_offset = 0);

  cereal::PandaState::PandaType hw_type = cereal::PandaState::PandaType::UNKNOWN;

  uint8_t health_version = HEALTH_PACKET_VERSION;
  bool has_rtc = false;
  const uint32_t bus_offset;

  bool connected();
  bool comms_healthy();
  std::string hw_serial();

  static std::vector<std::string> list();

  cereal::PandaState::PandaType get_hw_type();

  void set_safety_model(uint16_t safety_model, uint16_t safety_param = 0U);
  void set_alternative_experience(uint16_t alternative_experience, uint16_t safety_param_sp = 0U);
  void set_rtc(struct tm sys_time);
  struct tm get_rtc();
  void set_fan_speed(uint16_t fan_speed);
  uint16_t get_fan_speed();
  void set_ir_pwr(uint16_t ir_pwr);
  std::optional<health_t> get_state();
  std::optional<can_health_t> get_can_state(uint16_t can_number);
  void set_loopback(bool loopback);
  std::optional<std::vector<uint8_t>> get_firmware_version();
  std::optional<std::string> get_serial();
  std::optional<std::tuple<uint8_t, uint8_t, uint8_t>> get_packets_versions();
  void detect_health_version();
  void set_power_saving(bool power_saving);
  void enable_deepsleep();
  void set_heartbeat_disabled();
  void send_heartbeat(bool engaged, bool engaged_mads = false);
  void set_can_speed_kbps(uint16_t bus, uint16_t speed);
  void set_can_fd_auto(uint16_t bus, bool enabled);
  void set_data_speed_kbps(uint16_t bus, uint16_t speed);
  void set_canfd_non_iso(uint16_t bus, bool non_iso);
  void can_send(const std::vector<can_frame>& can_data_list);
  bool can_receive(std::vector<can_frame>& out_vec);
  void can_reset_communications();

protected:
  uint8_t receive_buffer[RECV_SIZE + sizeof(can_header) + 64];
  uint32_t receive_buffer_size = 0;

  Panda(uint32_t bus_offset) : bus_offset(bus_offset) {}
  void pack_can_buffer(const std::vector<can_frame>& can_data_list, std::function<void(uint8_t*, size_t)> write_func);
  bool unpack_can_buffer(uint8_t* data, uint32_t& size, std::vector<can_frame>& out_vec);
  uint8_t calculate_checksum(uint8_t* data, uint32_t len);
};
