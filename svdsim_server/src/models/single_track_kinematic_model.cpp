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

#include "svdsim_server/models/single_track_kinematic_model.hpp"

#include <cmath>

#include <sophus/so2.hpp>

#include "svdsim_server/models/runge_kutta.hpp"

namespace svdsim_server {

SingleTrackKinematicModel::SingleTrackKinematicModel(const SingleTrackKinematicParams& params)
    : params_(params), state_(State::Zero()), latest_input_(ControlInput::Zero()) {}

void SingleTrackKinematicModel::reset(const Sophus::SE2d& initial_pose) {
  // Extract 2D position and yaw from SE2 pose
  const auto translation = initial_pose.translation();
  const double yaw = initial_pose.so2().log();

  state_(0) = translation.x();  // x-position
  state_(1) = translation.y();  // y-position
  state_(2) = 0.0;              // steering angle (reset to zero)
  state_(3) = 0.0;              // velocity (reset to zero)
  state_(4) = yaw;              // yaw angle
  state_(5) = 0.0;              // left rear wheel angle (reset to zero)
  state_(6) = 0.0;              // right rear wheel angle (reset to zero)

  rear_left_wheel_cycles_ = 0;
  rear_right_wheel_cycles_ = 0;
}

void SingleTrackKinematicModel::integrate(const ControlInput& input, double dt, int num_steps) {
  // Create dynamics function compatible with rk4_step
  auto dfun = [this](const State& state, const ControlInput& input) -> State {
    return this->dynamic_function(state, input);
  };

  // Perform RK4 integration with non-linear effects applied after each step
  for (int i = 0; i < num_steps; ++i) {
    state_ = rk4_step<State, ControlInput>(dfun, state_, input, dt);
    apply_saturation_effects();
  }

  latest_input_ = input;
}

VehicleStateSummary SingleTrackKinematicModel::state_summary() const {
  VehicleStateSummary state_summary;

  // Construct SE2 pose from 2D position and yaw
  state_summary.base_link_pose = Sophus::SE2d(Sophus::SO2d(state_(4)), Eigen::Vector2d(state_(0), state_(1)));

  // Vehicle frame velocities (assuming planar motion, only longitudinal velocity)
  state_summary.vehicle_velocity = Eigen::Vector2d(state_(3), 0.0);

  // Calculate yaw rate for angular velocity (scalar in 2D)
  const double yaw_rate = (state_(3) / params_.wheelbase) * std::tan(state_(2));
  state_summary.vehicle_angular_velocity = yaw_rate;

  // Steering angle
  state_summary.steering_angle = state_(2);

  // Rear wheel angles
  state_summary.rear_left_wheel_angle = state_(5);
  state_summary.rear_left_wheel_cycles = rear_left_wheel_cycles_;
  state_summary.rear_right_wheel_angle = state_(6);
  state_summary.rear_right_wheel_cycles = rear_right_wheel_cycles_;

  return state_summary;
}

Sophus::SE2d SingleTrackKinematicModel::pose() const {
  // Extract x, y position and yaw angle from state
  const double x = state_(0);
  const double y = state_(1);
  const double yaw = state_(4);

  return Sophus::SE2d(Sophus::SO2d(yaw), Eigen::Vector2d(x, y));
}

double SingleTrackKinematicModel::virtual_encoder_velocity() const {
  return state_(3);
}

double SingleTrackKinematicModel::steering() const {
  return state_(2);
}

SingleTrackKinematicModel::State SingleTrackKinematicModel::dynamic_function(
    const State& state,
    const ControlInput& input) const {
  // State vector components:
  // x0 = x-position in a global coordinate system
  // x1 = y-position in a global coordinate system
  // x2 = steering angle of front wheels
  // x3 = velocity in x-direction
  // x4 = yaw angle
  // x5 = left rear wheel angular position
  // x6 = right rear wheel angular position

  // Inputs:
  // u0 = steering angle velocity of front wheels
  // u1 = longitudinal acceleration

  // Calculate the constrained steering velocity and acceleration
  const auto steering_velocity = input(0);
  const auto acceleration = input(1);

  // Calculate yaw rate
  const double yaw_rate = (state(3) / params_.wheelbase) * std::tan(state(2));

  // Calculate linear velocities of left and right rear wheels
  // v_left = v - yaw_rate * track_width / 2
  // v_right = v + yaw_rate * track_width / 2
  const double v_left = state(3) - yaw_rate * params_.rear_track_width / 2.0;
  const double v_right = state(3) + yaw_rate * params_.rear_track_width / 2.0;

  // Calculate angular velocities of rear wheels: omega = v / r
  const double d_phi_left = v_left / params_.effective_wheel_radius;
  const double d_phi_right = v_right / params_.effective_wheel_radius;

  // calculate the state derivatives x' = f(x, u)
  auto f = State{
      state(3) * std::cos(state(4)),  // dx/dt
      state(3) * std::sin(state(4)),  // dy/dt
      steering_velocity,              // d(steering angle)/dt
      acceleration,                   // d(velocity)/dt
      yaw_rate,                       // d(yaw angle)/dt
      d_phi_left,                     // d(left wheel angle)/dt
      d_phi_right                     // d(right wheel angle)/dt
  };

  return f;
}

void SingleTrackKinematicModel::apply_saturation_effects() {
  // Normalize yaw angle using Sophus SO2
  // SO2 automatically handles angle wrapping to principal range [-π, π)
  Sophus::SO2d yaw_rotation(state_(4));
  state_(4) = yaw_rotation.log();

  // bring wheels back the [0, 2pi) range and count full rotations for odometry
  auto front_integer_cycles = static_cast<int32_t>(std::trunc(state_(5) / (2.0 * M_PI)));
  rear_left_wheel_cycles_ += front_integer_cycles;
  state_(5) -= front_integer_cycles * 2.0 * M_PI;

  auto rear_integer_cycles = static_cast<int32_t>(std::trunc(state_(6) / (2.0 * M_PI)));
  rear_right_wheel_cycles_ += rear_integer_cycles;
  state_(6) -= rear_integer_cycles * 2.0 * M_PI;
}

}  // namespace svdsim_server
