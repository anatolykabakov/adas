#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "messages.pb.h"
#include "adas/utils/can_parser.h"
#include "adas/utils/speed_filter.h"
#include "adas/platform/volkswagen/carcontroller.h"
#include "adas/platform/volkswagen/mqb_car_state_decoder.h"
#include "adas/platform/volkswagen/panda_safety_supervisor.h"
#include "adas/platform/volkswagen/values.h"

namespace volkswagen {
/**
 * \brief Интерфейс машины: единственное место, знающее марку и CAN.
 *
 * Это `CarInterfaceBase` апстрима: у них он тоже не процесс, а поле контроллера — `Controls.__init__`
 * держит `self.CI`, внутри которого `self.CS` (декодер) и `self.CC` (упаковщик кадров), а
 * `controlsd` зовёт `CI.update(can_strs)` и `CI.apply(CC)`. Сервисов три: планер, контроллер, платформа.
 *
 * Закон управления сюда не входит: он получает шасси и план, а про адреса, сигналы и счётчики кадров
 * знает только этот класс. Железа здесь тоже нет — кадры приходят и уходят значениями, поэтому
 * интерфейс проверяется на записи без панды.
 */
class CarIface {
public:
  struct Config {
    std::string dbc_path;
    adas::SpeedFilter::Config speed_filter{};  ///< Wheel-speed filter settings.
  };

  explicit CarIface(Config config);

  /// Загрузить DBC. Без него разбор шасси выключен, отправка кадров продолжает работать.
  void init();

  /// `CI.update`: кадры в шасси. true, если шасси изменилось и его стоит опубликовать.
  bool update(const adas::proto::CANData& msg, int64_t now_ms);

  /// `CI.apply`: команда контроллера в кадры для платформы.
  std::vector<can_frame> apply(const CarControl& cc, const CarStateView& cs) { return car_controller_.update(cc, cs); }

  const adas::proto::CarState& carState() const { return decoder_.state(); }
  CarStateView carStateView() const { return decoder_.toCarStateView(); }

  /// Предел момента этой машины: контроллер считает нормированную команду, границу знает интерфейс.
  int maxTorqueCNm() const { return CarControllerParams::STEER_MAX; }
  /// Тот ли режим безопасности в панде, при котором эта машина принимает момент.
  bool safetyModelOk(uint32_t safety_mode) const { return safety_mode == MqbSafetyConstants::kVolkswagen; }
  /// Пропускает ли машина момент прямо сейчас: TSK, EPS и статус помощи — знание марки.
  bool actuationAllowed(bool controls_allowed) const
  {
    return lateralActuationAllowed(controls_allowed, /*lat_always_on=*/true, carStateView());
  }

  int lastTskStatus() const { return decoder_.lastTskStatus(); }
  uint8_t epsHcaStatus() const { return decoder_.epsHcaStatus(); }
  int applySteerLast() const { return car_controller_.applySteerLast(); }

private:
  Config config_;
  std::unique_ptr<DBSParser> dbc_;
  MqbCarStateDecoder decoder_;
  CarController car_controller_;
};

}  // namespace volkswagen
