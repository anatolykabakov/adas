#pragma once

#include <cstdint>

namespace adas {
constexpr int kModelW = 512;
constexpr int kModelH = 256;

void warp_yuv_to_frame6(const uint8_t* y, const uint8_t* u, const uint8_t* v, int width, int height,
                        const float m_model_to_cam[9], float* out6);

}  // namespace adas
