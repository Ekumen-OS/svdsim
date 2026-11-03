// Copyright 2026 Ekumen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "svdsim_server/models/mnc_tire_model.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace svdsim_server {

MNCTireModel::MNCTireModel(const TireModelParams& params)
    : longitudinal_params_(params.longitudinal),
      lateral_params_(params.lateral),
      effective_wheel_radius_(params.effective_wheel_radius),
      combined_slip_{params.combined_slip} {}

MNCTireModel::TireForces MNCTireModel::compute_forces(double vx, double vy, double w, double Fz) const {
  const auto v_wheel = effective_wheel_radius_ * w;

  const auto slip_normalization_den = std::max(std::abs(vx), std::abs(v_wheel));

  if (negligible(slip_normalization_den)) {
    return {
        0.0, -std::copysign(
                 Fz * lateral_params_.Dn, vy)};  // No slip, so no longitudinal force, and lateral force at zero slip
  }

  const auto slip_velocity_x = v_wheel - vx;

  const auto v_approx = std::abs(vx) + std::abs(vy);  // cheap module calculation

  const auto s = std::abs(slip_velocity_x / slip_normalization_den);                       // Longitudinal slip ratio
  const auto alpha = negligible(v_approx) ? 0.0 : std::atan2(std::abs(vy), std::abs(vx));  // Slip angle [rad]

  auto [Fx, Fy] = combined_slip_                               //
                      ? compute_forces_combined(s, alpha, Fz)  //
                      : compute_forces_pure(s, alpha, Fz);     //

  if ((Fx < 0.0) || (Fy < 0.0)) {
    std::cerr << "Warning: Negative tire force computed. This may indicate an issue with the Pacejka parameters or "
                 "slip conditions."
              << std::endl;
    std::cerr << "Computed forces: Fx = " << Fx << " N, Fy = " << Fy << " N" << std::endl;
    std::cerr << "Inputs: vx = " << vx << " m/s, vy = " << vy << " m/s, w = " << w << " rad/s, Fz = " << Fz << " N"
              << std::endl;
  }

  Fx = std::copysign(Fx, slip_velocity_x);
  Fy = std::copysign(Fy, -vy);

  return {Fx, Fy};
}

MNCTireModel::TireForces MNCTireModel::compute_forces_pure(double s, double alpha, double Fz) const {
  const auto alpha_deg = alpha * 180.0 / M_PI;
  const auto F0x = evaluate_pacejka(longitudinal_params_, s, Fz);
  const auto F0y = evaluate_pacejka(lateral_params_, alpha_deg, Fz);

  return {F0x, F0y};
}

MNCTireModel::TireForces MNCTireModel::compute_forces_combined(double s, double alpha, double Fz) const {
  // Modified Nicolas-Comstock (MNC) combined slip tire model
  //
  // Reference:
  // Brach, R. M., and Brach, R. M. (2000). "Modeling Combined Braking and Steering
  // Tire Forces." SAE Technical Paper 2000-01-0357.
  // DOI: 10.4271/2000-01-0357
  // https://www.researchgate.net/publication/267196202_Modeling_Combined_Braking_and_Steering_Tire_Forces
  //
  // The MNC model combines pure longitudinal and lateral forces under simultaneous slip conditions.
  // The formulation has a removable singularity at zero slip, so we calculate numerator and denominator
  // separately and handle the zero slip case explicitly.

  // For MNC model, we need pure forces: F0x at (s, alpha=0) and F0y at (s=0, alpha)
  const auto alpha_deg = alpha * 180.0 / M_PI;
  const auto F0x = evaluate_pacejka(longitudinal_params_, s, Fz);
  const auto F0y = evaluate_pacejka(lateral_params_, alpha_deg, Fz);

  // if either is zero, the combined slip is undefined.
  if (negligible(F0x) || negligible(F0y)) {
    return {F0x, F0y};
  }

  // Tire stiffness coefficients at origin (dF/dx at x=0)
  const auto Cs = evaluate_stiffness(longitudinal_params_, Fz);  // Longitudinal stiffness
  const auto Ca =
      evaluate_stiffness(lateral_params_, Fz) * 180.0 / M_PI;  // Cornering stiffness. Convert the lateral stiffness
                                                               // from per degree to per radian for
                                                               // consistency with the original MNC formulation

  // Guard against singularities in trigonometric functions
  const auto cos_alpha = std::cos(alpha);
  const auto sin_alpha = std::sin(alpha);
  // When |alpha| approaches 90 deg, tan(alpha) approaches infinity, use large value with appropriate sign
  const auto tan_alpha = negligible(cos_alpha) ? std::copysign(1e6, sin_alpha) : std::tan(alpha);

  // Common base denominator term (used in both Fx and Fy denominators)
  const auto base_denom = std::sqrt(pw2(s * F0y) + pw2(F0x * tan_alpha));

  // MNC longitudinal force. Notice I removed a (1 - s) factor from the second term
  // in the square root, because otherwise the limit \alpha -> 0 would not yield the
  // pure slip longitudinal force, which is a requirement of the model.
  const auto Fx_combined_num = F0x * F0y * std::sqrt(pw2(s * Ca) + pw2(cos_alpha * F0x));
  const auto Fx_combined_den = base_denom * Ca;

  // MNC lateral force
  const auto Fy_combined_num = F0x * F0y * std::sqrt(pw2((1 - s) * cos_alpha * F0y) + pw2(sin_alpha * Cs));
  const auto Fy_combined_den = base_denom * Cs * cos_alpha;

  // Handle singularities: return zero forces when denominators are negligible
  const auto Fx = negligible(Fx_combined_den) ? 0.0 : Fx_combined_num / Fx_combined_den;
  const auto Fy = negligible(Fy_combined_den) ? 0.0 : Fy_combined_num / Fy_combined_den;

  return {Fx, Fy};
}

double MNCTireModel::evaluate_pacejka(const PacejkaParams& params, double x, double vertical_load) const {
  // Pacejka Magic Formula:
  // F = D * sin(C * atan(B * x - E * (B * x - atan(B * x))))
  //
  // where:
  // - D = Dn * Fz (peak factor scaled by vertical load)
  // - B = stiffness factor
  // - C = shape factor
  // - E = curvature factor
  // - x = slip ratio or slip angle

  const auto D = params.Dn * vertical_load;
  const auto B = params.B;
  const auto C = params.C;
  const auto E = params.E;

  const auto Bx = B * x;
  const auto atan_Bx = std::atan(Bx);
  const auto phi = C * std::atan(Bx - E * (Bx - atan_Bx));

  return D * std::sin(phi);
}

double MNCTireModel::evaluate_stiffness(const PacejkaParams& params, double vertical_load) const {
  const auto D = params.Dn * vertical_load;
  const auto B = params.B;
  const auto C = params.C;

  return B * C * D;
}

}  // namespace svdsim_server
