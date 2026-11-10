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

#ifndef SVDSIM_SERVER_MODELS_MNC_TIRE_MODEL_HPP_
#define SVDSIM_SERVER_MODELS_MNC_TIRE_MODEL_HPP_

#include <cmath>
#include <optional>
#include <utility>

#include "svdsim_server/models/tire_model_interface.hpp"

namespace svdsim_server {

/**
 * @brief Pacejka tire model parameters
 */
struct PacejkaParams {
  double Dn;  ///< Normalized peak factor (dimensionless)
  double B;   ///< Stiffness factor (1/rad or 1/slip ratio)
  double C;   ///< Shape factor (dimensionless)
  double E;   ///< Curvature factor (dimensionless)
};

/**
 * @brief Parameters for tire model
 *
 * Contains Pacejka parameters for both longitudinal and lateral tire forces,
 * plus effective wheel radius and combined slip configuration.
 */
struct TireModelParams {
  PacejkaParams longitudinal;     ///< Longitudinal (slip ratio) Pacejka parameters
  PacejkaParams lateral;          ///< Lateral (slip angle) Pacejka parameters for a slip in degrees
  double effective_wheel_radius;  ///< Effective wheel radius [m]
  bool combined_slip;             ///< Whether to use combined slip effects
};

/**
 * @brief Tire model using Pacejka Magic Formula with Modified Nicolas-Comstock combined slip
 *
 * Implements the Pacejka Magic Formula tire model for computing longitudinal and lateral
 * tire forces based on wheel velocities and vertical load. When combined slip is enabled,
 * uses the Modified Nicolas-Comstock (MNC) model to account for interaction between
 * longitudinal and lateral slip.
 *
 * The MNC model combines pure longitudinal and lateral forces under simultaneous slip
 * conditions, providing more realistic tire behavior during combined braking and steering.
 *
 * Reference:
 * Brach, R. M., and Brach, R. M. (2000). "Modeling Combined Braking and Steering
 * Tire Forces." SAE Technical Paper 2000-01-0357.
 */
class MNCTireModel : public TireModelInterface {
 public:
  /**
   * @brief Construct a tire model with specified parameters
   *
   * @param params Tire model parameters including Pacejka coefficients, effective wheel radius,
   *               and combined slip flag.
   */
  explicit MNCTireModel(const TireModelParams& params);

  /**
   * @brief Compute tire forces given wheel velocities and vertical load
   *
   * Computes tire forces using either pure slip (Pacejka Magic Formula only) or combined slip
   * (Modified Nicolas-Comstock model) depending on the configuration.
   *
   * In pure slip mode, longitudinal and lateral forces are computed independently.
   * In combined slip mode, the MNC model accounts for interaction between slip ratio and
   * slip angle, providing more realistic force coupling during combined braking and steering.
   *
   * @param vx Longitudinal velocity at wheel contact patch [m/s]
   * @param vy Lateral velocity at wheel contact patch [m/s]
   * @param w Wheel angular velocity [rad/s]
   * @param Fz Vertical tire load [N]
   * @return TireForces Longitudinal (F_x) and lateral (F_y) tire forces [N]
   */
  [[nodiscard]] TireForces compute_forces(double vx, double vy, double w, double Fz) const override;

 private:
  PacejkaParams longitudinal_params_;  ///< Longitudinal Pacejka parameters
  PacejkaParams lateral_params_;       ///< Lateral Pacejka parameters
  double effective_wheel_radius_;      ///< Effective wheel radius [m]
  bool combined_slip_;                 ///< Whether to use combined slip effects

  /**
   * @brief Compute pure slip tire forces (no combined slip effects)
   * @param s Longitudinal slip ratio
   * @param alpha Slip angle [rad]
   * @param Fz Vertical tire load [N]
   */
  [[nodiscard]] TireForces compute_forces_pure(double s, double alpha, double Fz) const;

  /**
   * @brief Compute combined slip tire forces using Modified Nicolas-Comstock model
   *
   * Implements the MNC model from Brach & Brach (2000) to compute tire forces under
   * simultaneous longitudinal and lateral slip conditions. The model combines pure slip
   * forces using a coupling formulation that accounts for the interaction between slip
   * ratio and slip angle.
   *
   * Reference: Brach, R. M., and Brach, R. M. (2000). "Modeling Combined Braking and
   * Steering Tire Forces." SAE Technical Paper 2000-01-0357.
   *
   * @param s Longitudinal slip ratio
   * @param alpha Slip angle [rad]
   * @param Fz Vertical tire load [N]
   * @return TireForces Combined longitudinal (F_x) and lateral (F_y) forces [N]
   */
  [[nodiscard]] TireForces compute_forces_combined(double s, double alpha, double Fz) const;

  /**
   * @brief Evaluate Pacejka Magic Formula for given slip input
   * @param params Pacejka parameters (B, C, D, E)
   * @param x Slip ratio (longitudinal) or slip angle in degrees (lateral)
   * @param vertical_load Vertical load on tire [N]
   * @return Tire force [N]
   */
  [[nodiscard]] double evaluate_pacejka(const PacejkaParams& params, double x, double vertical_load) const;

  /**
   * @brief Compute tire stiffness coefficient at origin (dF/dx at x=0)
   * @param params Pacejka parameters
   * @param vertical_load Vertical load on tire [N]
   * @return Stiffness coefficient [N/rad] or [N]
   */
  [[nodiscard]] double evaluate_stiffness(const PacejkaParams& params, double vertical_load) const;

  // Performance-critical helper functions
  static constexpr bool negligible(double v) { return std::abs(v) < 1e-6; }
  static constexpr double pw2(double x) { return x * x; }
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_MODELS_MNC_TIRE_MODEL_HPP_
