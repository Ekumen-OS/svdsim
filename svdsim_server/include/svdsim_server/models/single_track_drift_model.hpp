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

#ifndef MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_DRIFT_MODEL_HPP_
#define MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_DRIFT_MODEL_HPP_

// Non-ROS dependencies
#include <memory>

#include <eigen3/Eigen/Core>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/models/single_track_model_interface.hpp"
#include "svdsim_server/models/tire_model_interface.hpp"

namespace svdsim_server {

/**
 * @brief Parameters for single-track drift vehicle model
 */
struct SingleTrackDriftParams {
  double wheelbase;                        ///< Distance between front and rear axles [m]
  double rear_track_width;                 ///< Distance between left and right rear wheels [m]
  double total_mass;                       ///< Total (sprung + unsprung) vehicle mass [kg]
  double yaw_inertia;                      ///< Yaw moment of inertia [kg·m²]
  double center_of_mass_forward_distance;  ///< Distance from center of gravity to rear axle [m]
  double center_of_mass_height;            ///< Height of center of mass relative to ground [m]
  double roll_center_height;               ///< Height of roll center relative to ground [m]
  double effective_wheel_radius;           ///< Effective radius of rear wheels [m]
  double wheel_inertia;  ///< Combined rotational inertia of two physical wheels (front or rear pair) [kg·m²]
  double engine_torque_split = 0.5;  ///< Fraction of engine torque applied to front wheels
  double brake_torque_split = 0.5;   ///< Fraction of braking torque applied to front wheels
};

/**
 * @brief Single-track drift vehicle model
 *
 * State vector (12D):
 * - x, y: global position [m]
 * - steering: front wheel steering angle [rad]
 * - velocity: longitudinal velocity at vehicle center [m/s]
 * - yaw: orientation angle [rad]
 * - yaw_rate: angular velocity [rad/s]
 * - slip_angle: vehicle slip angle [rad]
 * - front_wheel_w: front wheel angular speed [rad/s]
 * - rear_wheel_w: rear wheel angular speed [rad/s]
 * - rear_left_wheel_pos, rear_right_wheel_pos: wheel angular positions [rad]
 * - accel: longitudinal acceleration at vehicle center [m/s²]
 *
 * Input vector (2D):
 * - u0: steering velocity [rad/s]
 * - u1: longitudinal acceleration [m/s²]
 *
 * **This model's reference point is located at the rear axle center**.
 */
class SingleTrackDriftModel : public SingleTrackModelInterface {
 public:
  /**
   * @brief Construct a new Single Track Drift Model
   *
   * @param params Model parameters defining physical constraints and limits
   * @param tire_model Tire model for computing tire forces
   */
  explicit SingleTrackDriftModel(const SingleTrackDriftParams& params, std::unique_ptr<TireModelInterface> tire_model);

  /**
   * @brief Initialize the model state with a given initial pose
   *
   * @param initial_pose Initial 2D pose (SE2) of the vehicle base_link (at rear axle)
   */
  void reset(const Sophus::SE2d& initial_pose) override;

  /**
   * @brief Integrate the single-track drift model forward in time
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
   * Returns cached state from last integration including tire forces from drift dynamics.
   *
   * @return VehicleStateSummary Complete vehicle state representation
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
   * @brief Get the current velocity of the vehicle in body frame at base_link
   *
   * Computes velocity at base_link from COG velocity using rigid body transformation.
   *
   * @return Eigen::Vector2d Current velocity vector [m/s]
   */
  [[nodiscard]] Eigen::Vector2d velocity() const;

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
  /// @brief State vector type
  using State = Eigen::Matrix<double, 12, 1>;

  // State vector indices
  static constexpr int STATE_X = 0;                      ///< x-position in global coordinate system [m]
  static constexpr int STATE_Y = 1;                      ///< y-position in global coordinate system [m]
  static constexpr int STATE_STEERING = 2;               ///< steering angle of front wheels [rad]
  static constexpr int STATE_VELOCITY = 3;               ///< velocity at vehicle center [m/s]
  static constexpr int STATE_YAW = 4;                    ///< yaw angle [rad]
  static constexpr int STATE_YAW_RATE = 5;               ///< yaw rate [rad/s]
  static constexpr int STATE_SLIP_ANGLE = 6;             ///< slip angle at vehicle center [rad]
  static constexpr int STATE_FRONT_WHEEL_W = 7;          ///< front wheel angular speed [rad/s]
  static constexpr int STATE_REAR_WHEEL_W = 8;           ///< rear wheel angular speed [rad/s]
  static constexpr int STATE_REAR_LEFT_WHEEL_POS = 9;    ///< left wheel angular position [rad]
  static constexpr int STATE_REAR_RIGHT_WHEEL_POS = 10;  ///< right wheel angular position [rad]
  static constexpr int STATE_ACCEL = 11;                 ///< longitudinal acceleration at vehicle center [m/s²]
  static constexpr int STATE_SIZE = 12;                  ///< Total number of state variables

  // Control input indices
  static constexpr int INPUT_STEERING_VEL = 0;  ///< steering angle velocity [rad/s]
  static constexpr int INPUT_ACCEL = 1;         ///< longitudinal acceleration [m/s²]
  static constexpr int INPUT_SIZE = 2;          ///< Total number of control inputs

  double gravity_acc_ = 9.81;
  const double vmin_ = 0.15;  // speed at which to start blending
  const double vmax_ = 0.30;  // speed at which to finish blending

  SingleTrackDriftParams params_;                   ///< Model parameters
  Sophus::SE2d vehicle_pose_in_cog_;                ///< Pose of the center of gravity relative to the vehicle frame
  State state_;                                     ///< Internal state vector
  ControlInput latest_input_;                       ///< Latest control input applied
  std::unique_ptr<TireModelInterface> tire_model_;  ///< Tire model for force calculations

  int32_t rear_left_wheel_cycles_ = 0;   ///< Cumulative full rotations of left rear wheel
  int32_t rear_right_wheel_cycles_ = 0;  ///< Cumulative full rotations of right rear wheel

  /**
   * @brief Compute state derivative for the single-track drift model
   *
   * Implements single-track drift dynamics with tire slip modeling.
   *
   * @param state Current state vector
   * @param input Control input [steering_vel, accel]
   * @param dt Time step for integration
   * @return State State derivative vector (dx/dt)
   */
  State dynamic_function(const State& state, const ControlInput& input, const double dt) const;

  /**
   * @brief Apply saturation and normalization to internal state
   *
   * Clamps steering/velocity to limits and normalizes angles to [-pi, pi].
   */
  void apply_saturation_effects();

  /**
   * @brief Compute kinematic single-track dynamics (low-speed model)
   *
   * Used for velocities below vmin_ or blended with dynamic model between vmin_ and vmax_.
   *
   * @param state Current state vector
   * @param input Control input [steering_vel, accel]
   * @param dt Time step for integration
   * @return State State derivative vector (dx/dt)
   */
  SingleTrackDriftModel::State dynamic_function_ks(const State& state, const ControlInput& input, const double dt)
      const;

  /**
   * @brief Compute dynamic single-track drift model (high-speed model)
   *
   * Uses tire force model with combined slip. Active for velocities above vmax_ or blended
   * with kinematic model between vmin_ and vmax_.
   *
   * @param state Current state vector
   * @param input Control input [steering_vel, accel]
   * @param dt Time step for integration
   * @return State State derivative vector (dx/dt)
   */
  SingleTrackDriftModel::State dynamic_function_std(const State& state, const ControlInput& input, const double dt)
      const;

  /**
   * @brief Compute angular velocities of left and right rear wheels
   *
   * @param longitudinal_velocity Longitudinal velocity at vehicle center [m/s]
   * @param yaw_rate Yaw rate of the vehicle [rad/s]
   * @return std::pair<double, double> Angular velocities [d_phi_left, d_phi_right] [rad/s]
   */
  std::pair<double, double> computeRearWheelAngularVelocities(double longitudinal_velocity, double yaw_rate) const;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_SINGLE_TRACK_DRIFT_MODEL_HPP_
