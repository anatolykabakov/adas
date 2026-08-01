#include "utils/pose_calibrator.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace adas {
namespace {

constexpr double kPitchLo = -0.09074112085129739;
constexpr double kPitchHi = 0.17;
constexpr double kYawLo = -0.06912048084718224;
constexpr double kYawHi = 0.06912048084718235;

bool finite3(const Vec3& a) { return a.allFinite(); }

Vec3 sanityClip(Vec3 rpy)
{
  if (!finite3(rpy))
    return Vec3::Zero();
  rpy.y() = std::clamp(rpy.y(), kPitchLo - 0.005, kPitchHi + 0.005);
  rpy.z() = std::clamp(rpy.z(), kYawLo - 0.005, kYawHi + 0.005);
  return rpy;
}

bool isValid(const Vec3& rpy)
{
  return kPitchLo < rpy.y() && rpy.y() < kPitchHi && kYawLo < rpy.z() && rpy.z() < kYawHi;
}

Mat3 rotFromEuler(const Vec3& eu)
{
  return (Eigen::AngleAxisd(eu.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(eu.y(), Vec3::UnitY()) *
          Eigen::AngleAxisd(eu.x(), Vec3::UnitX()))
      .toRotationMatrix();
}

Vec3 eulerFromRot(const Mat3& R)
{
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  double roll = 0.0, yaw = 0.0;
  if (std::abs(std::cos(pitch)) > 1e-6) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  return Vec3(roll, pitch, yaw);
}

}  // namespace

PoseCalibrator::PoseCalibrator(double pitch0_deg, double yaw0_deg, double height_m) : height_m_(height_m)
{
  reset(pitch0_deg, yaw0_deg, 0);
}

void PoseCalibrator::reset(double pitch0_deg, double yaw0_deg, int valid_blocks)
{
  rpy_ = Vec3(0.0, pitch0_deg * M_PI / 180.0, yaw0_deg * M_PI / 180.0);
  valid_blocks_ = std::max(0, valid_blocks);
  for (auto& row : rpys_)
    row = rpy_;
  idx_ = 0;
  block_idx_ = 0;
  old_rpy_ = Vec3::Zero();
  old_rpy_weight_ = 0.0;
  calib_spread_ = Vec3::Zero();
  status_ = Uncalibrated;
  last_sample_ = {};
  updateStatus();
}

void PoseCalibrator::setVEgo(double v_ego_mps) { v_ego_ = v_ego_mps; }

Vec3 PoseCalibrator::smoothRpy() const
{
  if (old_rpy_weight_ > 0.0)
    return old_rpy_weight_ * old_rpy_ + (1.0 - old_rpy_weight_) * rpy_;
  return rpy_;
}

double PoseCalibrator::pitchDeg() const { return smoothRpy().y() * 180.0 / M_PI; }
double PoseCalibrator::yawDeg() const { return smoothRpy().z() * 180.0 / M_PI; }

const char* PoseCalibrator::statusName(Status s)
{
  switch (s) {
    case Calibrated:
      return "Calibrated";
    case Invalid:
      return "Invalid";
    case Recalibrating:
      return "Recalibrating";
    case Uncalibrated:
    default:
      return "Uncalibrated";
  }
}

int PoseCalibrator::calPercent() const
{
  return std::min(100, (valid_blocks_ * kBlockSize + idx_) * 100 / (kInputsNeeded * kBlockSize));
}

bool PoseCalibrator::handleCamOdom(const CameraOdometrySample& odom)
{
  LastSampleDebug dbg;
  dbg.odom_valid = odom.valid;
  dbg.v_ego = v_ego_;
  dbg.odom_trans_x = odom.trans.x();
  dbg.odom_trans_y = odom.trans.y();
  dbg.odom_trans_z = odom.trans.z();
  dbg.odom_rot_z = odom.rot.z();
  dbg.odom_angle_std = (odom.trans.x() > 1e-3) ? std::atan2(odom.trans_std.y(), odom.trans.x()) : 1e9;

  if (!odom.valid) {
    dbg.reject_reason = "no_odom";
    last_sample_ = dbg;
    return false;
  }
  old_rpy_weight_ = std::max(0.0, old_rpy_weight_ - 1.0 / kSmoothCycles);

  const double ego = (v_ego_ > 0.5) ? v_ego_ : odom.trans.x();
  dbg.v_ego = ego;
  dbg.gate_speed = ego > kMinSpeed && odom.trans.x() > kMinSpeed;
  dbg.gate_yaw_rate = std::abs(odom.rot.z()) < kMaxYawRate;
  const bool rpy_certain = dbg.odom_angle_std < kMaxVelAngleStd;
  dbg.gate_rpy_certain = rpy_certain || valid_blocks_ < kInputsNeeded;
  const bool straight_and_fast = dbg.gate_speed && dbg.gate_yaw_rate;
  if (!(straight_and_fast && dbg.gate_rpy_certain)) {
    if (!dbg.gate_speed)
      dbg.reject_reason = "slow";
    else if (!dbg.gate_yaw_rate)
      dbg.reject_reason = "yaw_rate";
    else
      dbg.reject_reason = "uncertain";
    last_sample_ = dbg;
    return false;
  }

  const Vec3 observed(0.0, -std::atan2(odom.trans.z(), odom.trans.x()), std::atan2(odom.trans.y(), odom.trans.x()));
  dbg.observed_pitch_deg = observed.y() * 180.0 / M_PI;
  dbg.observed_yaw_deg = observed.z() * 180.0 / M_PI;
  const Vec3 new_rpy = sanityClip(eulerFromRot(rotFromEuler(smoothRpy()) * rotFromEuler(observed)));

  const double w_idx = static_cast<double>(idx_);
  const double w_new = static_cast<double>(kBlockSize - idx_);
  rpys_[static_cast<size_t>(block_idx_)] =
      (w_idx * rpys_[static_cast<size_t>(block_idx_)] + w_new * new_rpy) / kBlockSize;

  idx_ = (idx_ + 1) % kBlockSize;
  if (idx_ == 0) {
    block_idx_ += 1;
    valid_blocks_ = std::max(block_idx_, valid_blocks_);
    block_idx_ = block_idx_ % kInputsWanted;
  }
  updateStatus();
  dbg.accepted = true;
  dbg.reject_reason = "";
  last_sample_ = dbg;
  return true;
}

std::vector<int> PoseCalibrator::validIdxs() const
{
  std::vector<int> out;
  for (int i = 0; i < block_idx_; ++i)
    out.push_back(i);
  for (int i = std::min(valid_blocks_, block_idx_ + 1); i < valid_blocks_; ++i)
    out.push_back(i);
  return out;
}

void PoseCalibrator::updateStatus()
{
  const auto idxs = validIdxs();
  if (!idxs.empty()) {
    Vec3 sum = Vec3::Zero();
    Vec3 mx = Vec3::Constant(-1e9);
    Vec3 mn = Vec3::Constant(1e9);
    for (int vi : idxs) {
      const Vec3& v = rpys_[static_cast<size_t>(vi)];
      sum += v;
      mx = mx.cwiseMax(v);
      mn = mn.cwiseMin(v);
    }
    rpy_ = sum / static_cast<double>(idxs.size());
    calib_spread_ = (mx - mn).cwiseAbs();
  } else {
    calib_spread_ = Vec3::Zero();
  }

  if (valid_blocks_ < kInputsNeeded) {
    if (status_ != Recalibrating)
      status_ = Uncalibrated;
  } else if (isValid(rpy_)) {
    status_ = Calibrated;
  } else {
    status_ = Invalid;
  }

  const double spread = calib_spread_.maxCoeff();
  if (spread > kMaxSpread && status_ == Calibrated) {
    const int prev = (block_idx_ - 1 + kInputsWanted) % kInputsWanted;
    const Vec3 from = rpy_;
    rpy_ = rpys_[static_cast<size_t>(prev)];
    valid_blocks_ = 1;
    for (auto& row : rpys_)
      row = rpy_;
    idx_ = 0;
    block_idx_ = 0;
    old_rpy_ = from;
    old_rpy_weight_ = 1.0;
    status_ = Recalibrating;
  }
}

}  // namespace adas
