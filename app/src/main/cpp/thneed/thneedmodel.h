#pragma once

#include <cstdint>
#include <string>

#include "runmodel.h"
#include "thneed.h"

class ThneedModel : public RunModel {
public:
  ThneedModel(uint8_t* modelData, float *_output, size_t _output_size, int runtime, bool use_tf8 = false, cl_context context = NULL);
  void *getCLBuffer(const std::string name);
  void execute();
private:
  Thneed *thneed = NULL;
  // Здесь у flowpilot стоял `PubMaster pm({"modelV2", "cameraOdometry"})` — единственная причина, по
  // которой заголовок тянул cereal. В `ThneedModel::execute()` он не используется: публикуют они из
  // своего JNI. Убран вместе с инклюдом; выход отдаётся в Java, публикацией занимается наш конвейер.
  bool recorded;
  float *output;
};
