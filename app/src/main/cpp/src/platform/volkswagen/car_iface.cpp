#include "adas/platform/volkswagen/car_iface.h"

#include <utility>

#include "adas/utils/logger.h"

namespace volkswagen {
CarIface::CarIface(Config config) : config_(std::move(config)) {}

void CarIface::init()
{
  if (config_.dbc_path.empty()) {
    LOGW("CarIface: пути к DBC нет — разбор шасси выключен");
  } else {
    dbc_ = std::make_unique<DBSParser>(config_.dbc_path);
    decoder_.setDbc(dbc_.get());
    LOGI("CarIface: DBC %s (%zu сообщений)", config_.dbc_path.c_str(), dbc_->getAllMessages().size());
  }
  decoder_.setSpeedFilterConfig(config_.speed_filter);
}

bool CarIface::update(const adas::proto::CANData& msg, int64_t now_ms)
{
  for (const auto& f : msg.frames()) {
    can_frame cf;
    cf.address = f.address();
    cf.dat = f.data();
    cf.busTime = f.bus_time();
    cf.src = f.src();
    decoder_.updateFromFrame(cf, now_ms);
  }
  if (!decoder_.consumeDirty())
    return false;
  decoder_.state().set_timestamp(now_ms);
  return true;
}

}  // namespace volkswagen
