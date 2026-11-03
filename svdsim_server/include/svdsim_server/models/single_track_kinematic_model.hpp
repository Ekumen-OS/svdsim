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

#ifndef MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_KINEMATIC_MODEL_HPP_
#define MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_KINEMATIC_MODEL_HPP_

// Non-ROS dependencies
#include <eigen3/Eigen/Core>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/models/single_track_model_interface.hpp"

namespace svdsim_server {

/**
 * @brief Parameters for single-track kinematic vehicle model
 */
struct SingleTrackKinematicParams {
  double wheelbase;               ///< Distance between front and rear axles [m]
  double rear_track_width;        ///< Distance between left and right rear wheels [m]
  double effective_wheel_radius;  ///< Effective radius of rear wheels [m]
  double mass;                    ///< Vehicle mass [kg]
};

/**
 * @brief Single-track kinematic vehicle model
 *
 * State vector (7D):
 * - x0: x-position in global coordinate system [m]
 * - x1: y-position in global coordinate system [m]
 * - x2: steering angle of front wheels [rad]
 * - x3: velocity in x-direction [m/s]
 * - x4: yaw angle [rad]
 * - x5: left rear wheel angular position [rad]
 * - x6: right rear wheel angular position [rad]
 *
 * Input vector (2D):
 * - u0: steering velocity [rad/s]
 * - u1: longitudinal acceleration [m/s²]
 *
 * **This model's reference point is located at the rear axle center**.
 */
class SingleTrackKinematicModel : public SingleTrackModelInterface {
 public:
  /**
   * @brief Construct a new Single Track Kinematic Model
   *
   * @param params Model parameters defining physical constraints and limits
   */
  explicit SingleTrackKinematicModel(const SingleTrackKinematicParams& params);

  /**
   * @brief Initialize the model state with a given initial pose
   *
   * @param initial_pose Initial 2D pose (SE2) of the vehicle base_link (at rear axle)
   */
  void reset(const Sophus::SE2d& initial_pose) override;

  /**
   * @brief Integrate the single-track kinematic model forward in time
   *
   * Uses RK4 numerical integration. Saturation effects (steering/velocity clamping,
   * angle normalization) are applied after each integration step, not just at the end.
   *
   * @param input Control input (steering velocity [rad/s], acceleration [m/s²])
   * @param dt Time step for each integration step [s]
   * @param num_steps Number of RK4 steps to perform (default: 1)
   */
  void integrate(const ControlInput& input, double dt, int num_steps = 1) override;

  /**
   * @brief Get the current nominal vehicle state
   *
   * Returns cached state from last integration. Lateral tire forces are zero (kinematic).
   * Longitudinal tire forces computed as F = m*a/4 per wheel.
   *
   * @return const VehicleStateSummary& Complete vehicle state representation
   */
  [[nodiscard]] VehicleStateSummary state_summary() const override;

  /**
   * @brief Get the current SE2 pose of the vehicle
   *
   * Extracts the 2D position and yaw angle from the internal state.
   *
   * @return Sophus::SE2d Current SE2 pose
   */
  [[nodiscard]] Sophus::SE2d pose() const override;

  /**
   * @brief Velocity of the vehicle according to a virtual encoder at the rear wheels.
   *
   * @return double Current velocity in x-direction [m/s]
   */
  [[nodiscard]] double virtual_encoder_velocity() const override;

  /**
   * @brief Get the current steering angle
   *
   * @return double Current steering angle of front wheels [rad]
   */
  [[nodiscard]] double steering() const override;

 protected:
  /// @brief State vector type (7D)
  using State = Eigen::Matrix<double, 7, 1>;

  SingleTrackKinematicParams params_;
  State state_;                ///< Internal state vector
  ControlInput latest_input_;  ///< Latest control input applied

  int32_t rear_left_wheel_cycles_ = 0.0;  ///< Cumulative full rotations of front wheel
  int32_t rear_right_wheel_cycles_ = 0;   ///< Cumulative full rotations of rear wheel

  /**
   * @brief Compute state derivative for the single-track kinematic model
   *
   * Implements single-track kinematic equations with differential rear wheel velocities.
   * Input is constrained using SteeringConstraints and AccelerationConstraints.
   *
   * @param state Current 7D state vector [x, y, steering, v, yaw, θ_L, θ_R]
   * @param input Control input [steering_vel, accel]
   * @return State 7D state derivative vector (dx/dt)
   */
  State dynamic_function(const State& state, const ControlInput& input) const;

  /**
   * @brief Apply saturation and normalization to internal state
   *
   * Clamps steering/velocity to limits and normalizes angles to [-pi, pi].
   */
  void apply_saturation_effects();
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_KINEMATIC_MODEL_HPP_
