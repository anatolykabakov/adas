#include "adas/utils/yuv_warp.h"

#include <algorithm>
#include <cmath>

namespace adas {
namespace {
void mul3(const float a[9], const float b[9], float r[9])
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j] + a[i * 3 + 1] * b[1 * 3 + j] + a[i * 3 + 2] * b[2 * 3 + j];
    }
  }
}

void transform_scale_buffer(const float m[9], float s, float out[9])
{
  const float inv_s = 1.0f / s;
  const float transform_out[9] = {inv_s, 0, 0.5f, 0, inv_s, 0.5f, 0, 0, 1};
  const float transform_in[9] = {s, 0, -0.5f * s, 0, s, -0.5f * s, 0, 0, 1};
  float tmp[9];
  mul3(m, transform_out, tmp);
  mul3(transform_in, tmp, out);
}

inline float sample_bilinear_u8(const uint8_t* px, int w, int h, float sx, float sy)
{
  if (sx < 0.f)
    sx = 0.f;
  else if (sx > w - 1)
    sx = static_cast<float>(w - 1);
  if (sy < 0.f)
    sy = 0.f;
  else if (sy > h - 1)
    sy = static_cast<float>(h - 1);

  const int x0 = static_cast<int>(std::floor(sx));
  const int y0 = static_cast<int>(std::floor(sy));
  const int x1 = std::min(x0 + 1, w - 1);
  const int y1 = std::min(y0 + 1, h - 1);
  const float fx = sx - x0;
  const float fy = sy - y0;
  const float v00 = px[y0 * w + x0];
  const float v10 = px[y0 * w + x1];
  const float v01 = px[y1 * w + x0];
  const float v11 = px[y1 * w + x1];
  const float top = v00 + (v10 - v00) * fx;
  const float bot = v01 + (v11 - v01) * fx;
  return top + (bot - top) * fy;
}

inline float sample_proj(const uint8_t* px, int w, int h, const float m[9], int x, int y)
{
  const float X = m[0] * x + m[1] * y + m[2];
  const float Y = m[3] * x + m[4] * y + m[5];
  const float W = m[6] * x + m[7] * y + m[8];
  if (std::fabs(W) < 1e-8f)
    return 0.f;
  return sample_bilinear_u8(px, w, h, X / W, Y / W);
}

}  // namespace

void warp_yuv_to_frame6(const uint8_t* y, const uint8_t* u, const uint8_t* v, int width, int height,
                        const float m_model_to_cam[9], float* out6)
{
  const int ww = kModelW / 2;
  const int hh = kModelH / 2;
  const int plane = ww * hh;
  const int uv_w = width / 2;
  const int uv_h = height / 2;

  float m_uv[9];
  transform_scale_buffer(m_model_to_cam, 0.5f, m_uv);

  for (int j = 0; j < hh; ++j) {
    for (int i = 0; i < ww; ++i) {
      const int idx = j * ww + i;
      const int mx0 = 2 * i;
      const int my0 = 2 * j;
      out6[0 * plane + idx] = sample_proj(y, width, height, m_model_to_cam, mx0, my0);
      out6[1 * plane + idx] = sample_proj(y, width, height, m_model_to_cam, mx0, my0 + 1);
      out6[2 * plane + idx] = sample_proj(y, width, height, m_model_to_cam, mx0 + 1, my0);
      out6[3 * plane + idx] = sample_proj(y, width, height, m_model_to_cam, mx0 + 1, my0 + 1);
      out6[4 * plane + idx] = sample_proj(u, uv_w, uv_h, m_uv, i, j);
      out6[5 * plane + idx] = sample_proj(v, uv_w, uv_h, m_uv, i, j);
    }
  }
}

}  // namespace adas
