#include "adas/utils/vanishing_point_calib.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace adas {

std::optional<Vec2> getIntersection(const ImageLine& a, const ImageLine& b)
{
  if (std::abs(a.m - b.m) < 1e-9)
    return std::nullopt;
  const double u = (b.c - a.c) / (a.m - b.m);
  const double v = a.m * u + a.c;
  return Vec2(u, v);
}

Vec2 getPitchYawFromVp(double u_i, double v_i, double fx, double fy, double cx, double cy)
{
  const Vec3 r3 = Vec3((u_i - cx) / fx, (v_i - cy) / fy, 1.0).normalized();
  const double yaw = -std::atan2(r3.x(), r3.z());
  const double pitch = std::asin(std::clamp(r3.y(), -1.0, 1.0));
  return Vec2(pitch, yaw);
}

std::optional<ImageLine> fitLineVOfU(const std::vector<Vec2>& uv, double mean_residuals_thresh)
{
  std::vector<Vec2> pts;
  pts.reserve(uv.size());
  for (const auto& p : uv) {
    if (p.allFinite())
      pts.push_back(p);
  }
  if (pts.size() < 8)
    return std::nullopt;

  const Eigen::Index n = static_cast<Eigen::Index>(pts.size());
  Eigen::MatrixXd A(n, 2);
  Eigen::VectorXd b(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    A(i, 0) = pts[static_cast<size_t>(i)].x();
    A(i, 1) = 1.0;
    b(i) = pts[static_cast<size_t>(i)].y();
  }

  const Eigen::Vector2d mc = A.colPivHouseholderQr().solve(b);
  if (!mc.allFinite())
    return std::nullopt;

  const double m = mc(0);
  const double c = mc(1);
  const double rss = (b - A * mc).squaredNorm();
  if (rss / static_cast<double>(n) > mean_residuals_thresh)
    return std::nullopt;
  return ImageLine{m, c};
}

VanishingPointCalibrator::VanishingPointCalibrator(int history_len, double pitch0_deg, double yaw0_deg)
  : history_len_(history_len), pitch_deg_(pitch0_deg), yaw_deg_(yaw0_deg)
{
}

void VanishingPointCalibrator::reset()
{
  history_.clear();
  success_ = false;
  has_vp_ = false;
  n_updates_ = 0;
}

void VanishingPointCalibrator::setEstimate(double pitch_deg, double yaw_deg)
{
  pitch_deg_ = pitch_deg;
  yaw_deg_ = yaw_deg;
}

bool VanishingPointCalibrator::addToHistory(double pitch_rad, double yaw_rad)
{
  history_.emplace_back(pitch_rad, yaw_rad);
  if (static_cast<int>(history_.size()) <= history_len_)
    return false;

  Vec2 sum = Vec2::Zero();
  for (const auto& py : history_)
    sum += py;
  const Vec2 mean = sum / static_cast<double>(history_.size());
  pitch_deg_ = mean.x() * 180.0 / M_PI;
  yaw_deg_ = mean.y() * 180.0 / M_PI;
  success_ = true;
  ++n_updates_;
  history_.clear();
  return true;
}

bool VanishingPointCalibrator::updateFromLines(const ImageLine& left, const ImageLine& right, double fx, double fy,
                                               double cx, double cy)
{
  auto vp = getIntersection(left, right);
  if (!vp)
    return false;
  const double u_i = vp->x();
  const double v_i = vp->y();
  if (!(-0.5 * cx <= u_i && u_i <= 2.5 * cx && -0.5 * cy <= v_i && v_i <= 2.5 * cy)) {
    return false;
  }
  const Vec2 pitch_yaw = getPitchYawFromVp(u_i, v_i, fx, fy, cx, cy);
  const double pitch = pitch_yaw.x();
  const double yaw = pitch_yaw.y();
  if (std::abs(pitch * 180.0 / M_PI) > 25.0 || std::abs(yaw * 180.0 / M_PI) > 15.0) {
    return false;
  }
  if (v_i > cy + 0.05 * std::max(fy, 1.0))
    return false;
  if (std::abs(v_i - cy) > 0.45 * std::max(fy, 1.0))
    return false;

  has_vp_ = true;
  vp_u_ = u_i;
  vp_v_ = v_i;
  return addToHistory(pitch, yaw);
}

bool VanishingPointCalibrator::updateFromUv(const std::vector<Vec2>& left_uv, const std::vector<Vec2>& right_uv,
                                            double fx, double fy, double cx, double cy)
{
  auto left = fitLineVOfU(left_uv);
  auto right = fitLineVOfU(right_uv);
  if (!left || !right)
    return false;
  if (left->m * right->m >= 0.0)
    return false;
  return updateFromLines(*left, *right, fx, fy, cx, cy);
}

}  // namespace adas
