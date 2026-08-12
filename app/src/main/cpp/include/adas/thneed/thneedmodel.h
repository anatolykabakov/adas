#pragma once

#include <cstdint>
#include <string>

#include "adas/thneed/runmodel.h"
#include "adas/thneed/thneed.h"

class ThneedModel : public RunModel {
public:
  ThneedModel(uint8_t* modelData, float* _output, size_t _output_size, int runtime, bool use_tf8 = false,
              cl_context context = NULL);
  void* getCLBuffer(const std::string name);
  void execute();

private:
  Thneed* thneed = NULL;
  bool recorded;
  float* output;
};
