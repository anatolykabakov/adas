#include "adas/platform/car_platform.h"

#include "adas/platform/volkswagen/vw_platform.h"
#include "adas/utils/logger.h"

namespace adas {
namespace platform {

std::vector<std::string> knownCarPlatforms() { return {"vw_golf_7_mqb", "volkswagen"}; }

std::unique_ptr<CarPlatform> makeCarPlatform(const std::string& name, const CarPlatformOptions& opts)
{
  // No default. A car we do not recognise is a CAN layout we do not know, and sending a frame built for
  // the wrong layout is not a degraded mode — it is an unrelated message on a bus that will act on it.
  if (name == "vw_golf_7_mqb" || name == "volkswagen") {
    return std::make_unique<volkswagen::VolkswagenMqb>(opts.dbc_path, opts.speed_filter, opts.cruise_buttons_enabled,
                                                       opts.cruise_tip_cooldown_ms);
  }
  LOGE("makeCarPlatform: unknown vehicle name '%s'", name.c_str());
  return nullptr;
}

}  // namespace platform
}  // namespace adas
