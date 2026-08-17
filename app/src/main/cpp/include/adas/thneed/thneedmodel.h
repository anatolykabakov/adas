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

  /**
   * \brief Save the recorded run, already compiled by this GPU.
   *
   * \details The point is `binaries = true`: kernel sources are portable, but building them on every
   * start costs time, and the Adreno compiler on the phone knows its own hardware better than the one
   * that
   * prepared it does not. Call after at least one `execute`, otherwise there is nothing to write.
   *
   * \param[in] path Where to write.
   * \param[in] binaries true — binaries for this device, false — kernel sources.
   * \return false if writing failed or no run has been recorded yet.
   */
  bool saveTo(const char* path, bool binaries) { return recorded && thneed != NULL && thneed->save(path, binaries); }

private:
  Thneed* thneed = NULL;
  bool recorded;
  float* output;
};
