// Copyright 2025 Ekumen
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

#ifndef MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_MODEL_INTERFACE_HPP_
#define MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_MODEL_INTERFACE_HPP_

// Non-ROS dependencies
#include <eigen3/Eigen/Core>

// Project headers
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

/**
 * @brief Abstract interface for single-track vehicle models
 *
 * Defines the common interface that all single-track vehicle models must implement.
 * Single-track models represent the vehicle as a bicycle with front and rear axles,
 * with the reference point located at the rear axle center.
 *
 * Implementations include:
 * - SingleTrackKinematicModel: Kinematic model without tire slip
 * - SingleTrackDriftModel: Dynamic model with tire slip and drift dynamics
 */
class SingleTrackModelInterface {
 public:
  virtual ~SingleTrackModelInterface() = default;

  /**
   * @brief Initialize the model state with a given initial pose
   *
   * Resets the internal state vector to correspond to the specified pose.
   * The reference point is at the rear axle center.
   *
   * @param initial_pose Initial 2D pose (SE2) of the vehicle base_link (at rear axle)
   */
  virtual void reset(const Sophus::SE2d& initial_pose) = 0;

  /**
   * @brief Integrate the model forward in time
   *
   * Updates the internal state by numerically integrating the dynamics equations.
   * Uses RK4 numerical integration. Saturation effects (steering/velocity clamping,
   * angle normalization) are applied after each integration step.
   *
   * @param input Control input (steering velocity [rad/s], acceleration [m/s²])
   * @param dt Time step for each integration step [s]
   * @param num_steps Number of RK4 steps to perform (default: 1)
   */
  virtual void integrate(const ControlInput& input, double dt, int num_steps = 1) = 0;

  /**
   * @brief Get the current nominal vehicle state
   *
   * Returns the complete vehicle state including pose, velocities, accelerations,
   * and tire forces (model-dependent).
   *
   * @return VehicleStateSummary Complete vehicle state representation
   */
  [[nodiscard]] virtual VehicleStateSummary state_summary() const = 0;

  /**
   * @brief Get the current SE2 pose of the vehicle
   *
   * Extracts the 2D position and yaw angle from the internal state.
   * The pose represents the vehicle at the rear axle center.
   *
   * @return Sophus::SE2d Current SE2 pose
   */
  [[nodiscard]] virtual Sophus::SE2d pose() const = 0;

  /**
   * @brief Velocity of the vehicle according to a virtual encoder at the rear wheels
   *
   * Returns the longitudinal velocity measured at the rear axle. This corresponds
   * to what would be measured by a wheel encoder on the rear wheels.
   *
   * @return double Current velocity in x-direction [m/s]
   */
  [[nodiscard]] virtual double virtual_encoder_velocity() const = 0;

  /**
   * @brief Get the current steering angle
   *
   * Returns the current steering angle of the front wheels.
   *
   * @return double Current steering angle of front wheels [rad]
   */
  [[nodiscard]] virtual double steering() const = 0;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_MODEL_INTERFACE_HPP_
