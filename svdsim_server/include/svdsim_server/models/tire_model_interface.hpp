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

#ifndef SVDSIM_SERVER_MODELS_TIRE_MODEL_INTERFACE_HPP_
#define SVDSIM_SERVER_MODELS_TIRE_MODEL_INTERFACE_HPP_

namespace svdsim_server {

/**
 * @brief Abstract interface for tire models
 *
 * Defines the common interface that all tire models must implement.
 * Tire models compute longitudinal and lateral tire forces based on
 * wheel velocities and vertical load.
 *
 * Implementations include:
 * - MNCTireModel: Pacejka Magic Formula with optional combined slip (MNC)
 */
class TireModelInterface {
 public:
  /**
   * @brief Tire force output structure
   */
  struct TireForces {
    double F_x;  ///< Longitudinal tire force [N]
    double F_y;  ///< Lateral tire force [N]
  };

  virtual ~TireModelInterface() = default;

  /**
   * @brief Compute tire forces given wheel velocities and vertical load
   *
   * Computes longitudinal and lateral tire forces based on the contact patch
   * velocities, wheel angular velocity, and vertical load on the tire.
   *
   * @param vx Longitudinal velocity at wheel contact patch [m/s]
   * @param vy Lateral velocity at wheel contact patch [m/s]
   * @param w Wheel angular velocity [rad/s]
   * @param Fz Vertical tire load [N]
   * @return TireForces Longitudinal (F_x) and lateral (F_y) tire forces [N]
   */
  [[nodiscard]] virtual TireForces compute_forces(double vx, double vy, double w, double Fz) const = 0;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_MODELS_TIRE_MODEL_INTERFACE_HPP_
