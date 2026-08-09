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
  // flowpilot had `PubMaster pm({"modelV2", "cameraOdometry"})` here, the only reason this header
  // pulled in cereal. It is unused in `ThneedModel::execute()` — they publish from their own JNI — so it
  // is removed along with the include; the output goes to Java and our pipeline publishes it.
  bool recorded;
  float* output;
};
