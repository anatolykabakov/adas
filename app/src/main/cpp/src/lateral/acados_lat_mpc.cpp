#include "adas/lateral/acados_lat_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_lat.h"
}

namespace adas {
namespace flowpilot {
namespace {

constexpr int kN = LAT_N;
constexpr int kNx = LAT_NX;
constexpr int kNp = LAT_NP;
constexpr int kCostDim = 5;
constexpr int kCostDimE = 3;
constexpr double kSpeedOffset = 10.0;
constexpr double kMinSpeed = 0.1;

double tIdx(int i) { return 10.0 * std::pow(static_cast<double>(i) / 32.0, 2.0); }

}  // namespace

struct AcadosLatMpc::Impl {
  lat_solver_capsule* capsule = nullptr;
  ocp_nlp_config* cfg = nullptr;
  ocp_nlp_dims* dims = nullptr;
  ocp_nlp_in* in = nullptr;
  ocp_nlp_out* out = nullptr;
  bool ok = false;

  Impl()
  {
    capsule = lat_acados_create_capsule();
    if (capsule == nullptr)
      return;
    if (lat_acados_create(capsule) != 0) {
      lat_acados_free_capsule(capsule);
      capsule = nullptr;
      return;
    }
    cfg = lat_acados_get_nlp_config(capsule);
    dims = lat_acados_get_nlp_dims(capsule);
    in = lat_acados_get_nlp_in(capsule);
    out = lat_acados_get_nlp_out(capsule);
    ok = cfg != nullptr && dims != nullptr && in != nullptr && out != nullptr;
  }

  ~Impl()
  {
    if (capsule != nullptr) {
      lat_acados_free(capsule);
      lat_acados_free_capsule(capsule);
    }
  }
};

AcadosLatMpc::AcadosLatMpc() : impl_(std::make_unique<Impl>()) {}
AcadosLatMpc::~AcadosLatMpc() = default;

bool AcadosLatMpc::available() const { return impl_ && impl_->ok; }
int AcadosLatMpc::horizonNodes() { return kN; }
double AcadosLatMpc::nodeTime(int i) { return tIdx(i); }

void AcadosLatMpc::setWeights(const Weights& w)
{
  if (!available())
    return;
  std::array<double, kCostDim * kCostDim> W{};
  const std::array<double, kCostDim> diag = {w.path, w.heading, w.lat_accel, w.lat_jerk, w.steering_rate};
  for (int i = 0; i < kCostDim; ++i)
    W[i * kCostDim + i] = diag[i];
  for (int i = 0; i < kN; ++i)
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, i, "W", W.data());

  std::array<double, kCostDimE * kCostDimE> We{};
  for (int i = 0; i < kCostDimE; ++i)
    We[i * kCostDimE + i] = diag[i];
  ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, kN, "W", We.data());
}

void AcadosLatMpc::reset()
{
  if (!available())
    return;
  std::array<double, kNx> zero_x{};
  std::array<double, kNp> zero_p{};
  std::array<double, kCostDim> zero_y{};
  for (int i = 0; i <= kN; ++i) {
    ocp_nlp_out_set(impl_->cfg, impl_->dims, impl_->out, i, "x", zero_x.data());
    lat_acados_update_params(impl_->capsule, i, zero_p.data(), kNp);
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, i, "yref", zero_y.data());
  }
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, 0, "lbx", zero_x.data());
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, 0, "ubx", zero_x.data());
  lat_acados_solve(impl_->capsule);
}

AcadosLatMpc::Result AcadosLatMpc::solve(double v_ego, double rotation_radius, double yaw_rate,
                                         const std::vector<double>& y_ref, const std::vector<double>& psi_ref,
                                         const std::vector<double>& r_ref, double steer_delay_s)
{
  Result res;
  if (!available() || static_cast<int>(y_ref.size()) < kN + 1 || static_cast<int>(psi_ref.size()) < kN + 1 ||
      static_cast<int>(r_ref.size()) < kN + 1)
    return res;

  const double v = std::max(v_ego, kMinSpeed);
  const double v_off = v + kSpeedOffset;

  std::array<double, kNx> x0 = {0.0, 0.0, 0.0, yaw_rate};
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, 0, "ubx", x0.data());

  const std::array<double, kNp> p = {v, rotation_radius};
  for (int i = 0; i <= kN; ++i) {
    lat_acados_update_params(impl_->capsule, i, const_cast<double*>(p.data()), kNp);
    const std::array<double, kCostDim> yref = {y_ref[i], v_off * psi_ref[i], v_off * r_ref[i], 0.0, 0.0};
    if (i < kN)
      ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, i, "yref", const_cast<double*>(yref.data()));
    else
      ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, kN, "yref", const_cast<double*>(yref.data()));
  }

  res.status = lat_acados_solve(impl_->capsule);
  ocp_nlp_get(impl_->cfg, lat_acados_get_nlp_solver(impl_->capsule), "cost_value", &res.cost);

  res.psi_sol.assign(kN + 1, 0.0);
  res.r_sol.assign(kN + 1, 0.0);
  std::array<double, kNx> x{};
  for (int i = 0; i <= kN; ++i) {
    ocp_nlp_out_get(impl_->cfg, impl_->dims, impl_->out, i, "x", x.data());
    res.psi_sol[i] = x[2];
    res.r_sol[i] = x[3];
  }
  double u0 = 0.0;
  ocp_nlp_out_get(impl_->cfg, impl_->dims, impl_->out, 0, "u", &u0);

  const double delay = std::max(steer_delay_s, 1e-3);
  const double kappa0 = res.r_sol[0] / v;
  double psi_delay = res.psi_sol[kN];
  for (int i = 0; i < kN; ++i) {
    if (tIdx(i + 1) >= delay) {
      const double t0 = tIdx(i), t1 = tIdx(i + 1);
      const double a = (delay - t0) / std::max(t1 - t0, 1e-9);
      psi_delay = res.psi_sol[i] + a * (res.psi_sol[i + 1] - res.psi_sol[i]);
      break;
    }
  }
  const double avg_kappa = psi_delay / (v * delay);
  res.desired_curvature = 2.0 * avg_kappa - kappa0;
  res.desired_curvature_rate = u0 / v;
  res.ok = std::isfinite(res.desired_curvature) && (res.status == 0 || res.status == 2);
  return res;
}

}  // namespace flowpilot
}  // namespace adas
