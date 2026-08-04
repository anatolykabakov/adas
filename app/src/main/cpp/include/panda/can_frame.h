#pragma once

#include <cstdint>
#include <string>

struct can_frame {
  long address;
  std::string dat;
  long busTime;
  long src;
};
