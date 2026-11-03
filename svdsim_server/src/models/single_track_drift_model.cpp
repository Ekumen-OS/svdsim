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

#include "svdsim_server/models/single_track_drift_model.hpp"

#include <cmath>
#include <iostream>

#include <sophus/so2.hpp>

#include "svdsim_server/models/runge_kutta.hpp"

namespace svdsim_server {

SingleTrackDriftModel::SingleTrackDriftModel(
    const SingleTrackDriftParams& params,
    std::unique_ptr<TireModelInterface> tire_model)
    : params_(params),
      vehicle_pose_in_cog_{
          Sophus::SE2d(Sophus::SO2d(0.0), Eigen::Vector2d(params.center_of_mass_forward_distance, 0.0)).inverse()},
      state_(State::Zero()),
      latest_input_(ControlInput::Zero(INPUT_SIZE)),
      tire_model_(std::move(tire_model)) {
  if (!tire_model_) {
    throw std::invalid_argument("SingleTrackDriftModel requires a valid MNCTireModel");
  }
}

void SingleTrackDriftModel::reset(const Sophus::SE2d& initial_pose) {
  // initial_pose is the base_link (rear axle) pose
  // Transform to center of gravity pose for state initialization
  const Sophus::SE2d cog_pose = initial_pose * vehicle_pose_in_cog_.inverse();

  const Eigen::Vector2d translation = cog_pose.translation();
  const double yaw = cog_pose.so2().log();

  state_ = State::Zero();
  state_(STATE_X) = translation.x();
  state_(STATE_Y) = translation.y();
  state_(STATE_STEERING) = 0.0;
  state_(STATE_VELOCITY) = 0.0;
  state_(STATE_YAW) = yaw;
  state_(STATE_YAW_RATE) = 0.0;
  state_(STATE_SLIP_ANGLE) = 0.0;
  state_(STATE_FRONT_WHEEL_W) = 0.0;
  state_(STATE_REAR_WHEEL_W) = 0.0;
  state_(STATE_REAR_LEFT_WHEEL_POS) = 0.0;
  state_(STATE_REAR_RIGHT_WHEEL_POS) = 0.0;
  state_(STATE_ACCEL) = 0.0;

  rear_left_wheel_cycles_ = 0;
  rear_right_wheel_cycles_ = 0;
}

void SingleTrackDriftModel::integrate(const ControlInput& u, double dt, int num_steps) {
  // Create dynamics function compatible with rk4_step
  auto dfun = [this, dt](const State& s, const ControlInput& u) -> State { return this->dynamic_function(s, u, dt); };

  // Perform RK4 integration with non-linear effects applied after each step
  for (int i = 0; i < num_steps; ++i) {
    state_ = rk4_step<State, ControlInput>(dfun, state_, u, dt);
    apply_saturation_effects();
  }

  latest_input_ = u;
}

VehicleStateSummary SingleTrackDriftModel::state_summary() const {
  VehicleStateSummary state_summary;

  // Get base_link pose and velocity
  state_summary.base_link_pose = pose();
  state_summary.vehicle_velocity = velocity();

  // Angular velocity from yaw rate (scalar in 2D)
  state_summary.vehicle_angular_velocity = state_(STATE_YAW_RATE);

  // Steering angle
  state_summary.steering_angle = state_(STATE_STEERING);

  // Rear wheel angles
  state_summary.rear_left_wheel_angle = state_(STATE_REAR_LEFT_WHEEL_POS);
  state_summary.rear_left_wheel_cycles = rear_left_wheel_cycles_;
  state_summary.rear_right_wheel_angle = state_(STATE_REAR_RIGHT_WHEEL_POS);
  state_summary.rear_right_wheel_cycles = rear_right_wheel_cycles_;

  return state_summary;
}

Sophus::SE2d SingleTrackDriftModel::pose() const {
  // State contains pose at center of gravity, transform to base_link
  Sophus::SE2d cog_pose_in_world(Sophus::SO2d(state_(STATE_YAW)), Eigen::Vector2d(state_(STATE_X), state_(STATE_Y)));
  return cog_pose_in_world * vehicle_pose_in_cog_;
}

Eigen::Vector2d SingleTrackDriftModel::velocity() const {
  // Velocity at center of gravity in body frame
  const Eigen::Vector2d v_cog(                                      //
      state_(STATE_VELOCITY) * std::cos(state_(STATE_SLIP_ANGLE)),  //
      state_(STATE_VELOCITY) * std::sin(state_(STATE_SLIP_ANGLE))   //
  );                                                                //

  // Use the offset from vehicle_pose_in_cog_ transformation
  const Eigen::Vector2d r_cog_to_base = vehicle_pose_in_cog_.translation();

  // Transform velocity: v_base = v_cog + omega x r_cog_to_base
  // In 2D: omega x r = [-omega * r_y, omega * r_x]
  const Eigen::Vector2d omega_cross_r(              //
      -state_(STATE_YAW_RATE) * r_cog_to_base.y(),  //
      state_(STATE_YAW_RATE) * r_cog_to_base.x()    //
  );                                                //

  return v_cog + omega_cross_r;
}

double SingleTrackDriftModel::virtual_encoder_velocity() const {
  return state_(STATE_REAR_WHEEL_W) * params_.effective_wheel_radius;
}

double SingleTrackDriftModel::steering() const {
  return state_(STATE_STEERING);
}

SingleTrackDriftModel::State
SingleTrackDriftModel::dynamic_function(const State& s, const ControlInput& u, const double dt) const {
  // calculate both the kinematic and standard dynamic functions.
  // then blend them based on the current speed using a smooth transition function.

  const auto forward_speed = std::abs(s(STATE_VELOCITY));

  if (forward_speed <= vmin_) {
    return dynamic_function_ks(s, u, dt);
  } else {  // if (forward_speed >= vmax_) {
    return dynamic_function_std(s, u, dt);
  }

  // in the transition region, blend the two models using a raised cosine function.
  // This function smooth f and df, but the second derivative is discontinuous at the endpoints.
  // If this were an issue we would to use a sigmoid, which unlike common road I don't use because
  // if vb, va and vmin are not properly chosen there's a discontinuity in the blend values at vmin.
  const auto delta = (forward_speed - vmin_) / (vmax_ - vmin_);
  const auto w_std = 0.5 * (1 - std::cos(M_PI * delta));
  const auto w_ks = 1.0 - w_std;

  return w_ks * dynamic_function_ks(s, u, dt) + w_std * dynamic_function_std(s, u, dt);
}

void SingleTrackDriftModel::apply_saturation_effects() {
  // Normalize yaw angle using Sophus SO2
  // SO2 automatically handles angle wrapping to principal range [-π, π)
  Sophus::SO2d yaw_rotation(state_(STATE_YAW));
  state_(STATE_YAW) = yaw_rotation.log();

  if (std::abs(state_(STATE_SLIP_ANGLE)) > M_PI / 2.0) {
    state_(STATE_VELOCITY) = -state_(STATE_VELOCITY);
    state_(STATE_SLIP_ANGLE) += M_PI;  // flip slip angle to opposite direction
  }

  // normalize beta to [-pi, pi) as well to prevent numerical issues in the dynamic function
  Sophus::SO2d slip_rotation(state_(STATE_SLIP_ANGLE));
  state_(STATE_SLIP_ANGLE) = slip_rotation.log();

  // bring wheels back the [0, 2pi) range and count full rotations for odometry
  auto left_wheel_cycles = static_cast<int32_t>(std::trunc(state_(STATE_REAR_LEFT_WHEEL_POS) / (2.0 * M_PI)));
  rear_left_wheel_cycles_ += left_wheel_cycles;
  state_(STATE_REAR_LEFT_WHEEL_POS) -= left_wheel_cycles * 2.0 * M_PI;

  auto right_wheel_cycles = static_cast<int32_t>(std::trunc(state_(STATE_REAR_RIGHT_WHEEL_POS) / (2.0 * M_PI)));
  rear_right_wheel_cycles_ += right_wheel_cycles;
  state_(STATE_REAR_RIGHT_WHEEL_POS) -= right_wheel_cycles * 2.0 * M_PI;
}

SingleTrackDriftModel::State
SingleTrackDriftModel::dynamic_function_ks(const State& s, const ControlInput& u, const double dt) const {
  const auto lf = params_.wheelbase - params_.center_of_mass_forward_distance;
  const auto lr = params_.center_of_mass_forward_distance;
  const auto lwb = params_.wheelbase;

  // front and rear tire speeds
  const auto u_wf =
      s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE)) * std::cos(s(STATE_STEERING))                                //
      + (s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) + lf * s(STATE_YAW_RATE)) * std::sin(s(STATE_STEERING));  //
  const auto u_wr = s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE));                                               //

  auto squared = [](double x) { return x * x; };

  const auto beta = std::atan(std::tan(s(STATE_STEERING)) * lr / lwb);

  const auto d_yaw = (s(STATE_VELOCITY) * std::cos(beta) * std::tan(s(STATE_STEERING)) / lwb) *
                     (std::cos(s(STATE_SLIP_ANGLE)) > 0.0 ? 1.0 : -1.0);

  // derivative of slip angle and yaw rate
  const auto d_beta_ks =
      (lr * u(INPUT_STEERING_VEL)) /
      (lwb * squared(std::cos(s(STATE_STEERING))) * (1 + squared(squared(std::tan(s(STATE_STEERING))) * lr / lwb)));

  const auto dd_psi_ks =
      1 / lwb *
      (u(INPUT_ACCEL) * std::cos(s(STATE_SLIP_ANGLE)) * std::tan(s(STATE_STEERING))                   //
       - s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) * d_beta_ks * std::tan(s(STATE_STEERING))  //
       + s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE)) * u(INPUT_STEERING_VEL) /
             squared(std::cos(s(STATE_STEERING)))  //
      );                                           //

  // derivative of angular speeds
  const double omega_tau = 0.02;  // time constant for wheel speed dynamics
  const auto d_omega_f_ks = (1 / omega_tau) * (u_wf / params_.effective_wheel_radius - s(STATE_FRONT_WHEEL_W));
  const auto d_omega_r_ks = (1 / omega_tau) * (u_wr / params_.effective_wheel_radius - s(STATE_REAR_WHEEL_W));

  const auto [d_phi_left, d_phi_right] =
      computeRearWheelAngularVelocities(s(STATE_REAR_WHEEL_W) * params_.effective_wheel_radius, s(STATE_YAW_RATE));

  // Acceleration along vehicle X-axis: d(v*cos(slip_angle))/dt
  const auto accel_x =
      u(INPUT_ACCEL) * std::cos(s(STATE_SLIP_ANGLE)) - s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) * d_beta_ks;

  return State{
      s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE) + s(STATE_YAW)),  // dx/dt
      s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE) + s(STATE_YAW)),  // dy/dt
      u(INPUT_STEERING_VEL),                                             // d(steering angle)/dt
      accel_x,                                                           // d(velocity)/dt
      d_yaw,                                                             // d(yaw angle)/dt
      dd_psi_ks,                                                         // d(yaw rate)/dt
      d_beta_ks,                                                         // d(slip angle)/dt
      d_omega_f_ks,                                                      // d(front wheel angular speed)/dt
      d_omega_r_ks,                                                      // d(rear wheel angular speed)/dt
      d_phi_left,                                                        // d(left wheel position)/dt
      d_phi_right,                                                       // d(right wheel position)/dt
      (accel_x - s(STATE_ACCEL)) / dt  // d(accel)/dt (tracks X-axis accel with tau=dt)
  };
}

SingleTrackDriftModel::State
SingleTrackDriftModel::dynamic_function_std(const State& s, const ControlInput& u, const double dt) const {
  const auto lf = params_.wheelbase - params_.center_of_mass_forward_distance;
  const auto lr = params_.center_of_mass_forward_distance;
  const auto lwb = params_.wheelbase;
  const auto m = params_.total_mass;
  const auto I = params_.yaw_inertia;

  const auto h_s = params_.center_of_mass_height - params_.roll_center_height;

  // vertical tire forces using actual vehicle acceleration (from state)
  const auto F_zf_dyn = m * (1. * -s(STATE_ACCEL) * h_s + gravity_acc_ * lr) / lwb;

  const auto weight = m * gravity_acc_;
  const auto F_zf = std::clamp(F_zf_dyn, 0.2 * weight, 0.8 * weight);
  const auto F_zr = weight - F_zf;  // ensure total weight is correct for tire model

  // Compute tire forces using MNCTireModel (combined slip forces)
  // Front tire: velocity at front wheel in body frame, then rotate to wheel frame
  const Eigen::Vector2d velocity_front_body(                                        //
      s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE)),                            //
      s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) + lf * s(STATE_YAW_RATE));  //
  const Sophus::SO2d body_to_wheel_rotation(-s(STATE_STEERING));
  const Eigen::Vector2d velocity_front_wheel = body_to_wheel_rotation * velocity_front_body;

  const auto front_tire_forces = tire_model_->compute_forces(  //
      std::max(velocity_front_wheel.x(), 0.0),                 //
      velocity_front_wheel.y(),                                //
      std::max(s(STATE_FRONT_WHEEL_W), 0.0),                   //
      F_zf);                                                   //

  // Rear tire: velocity at rear wheel in body frame (no rotation needed - rear wheel aligned with body)
  const auto vx_rear = s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE));
  const auto vy_rear = s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) - lr * s(STATE_YAW_RATE);

  const auto rear_tire_forces = tire_model_->compute_forces(  //
      std::max(vx_rear, 0.0),                                 //
      vy_rear,                                                //
      std::max(s(STATE_REAR_WHEEL_W), 0.0),                   //
      F_zr);                                                  //

  // Extract forces in tire frame
  const auto F_xf = front_tire_forces.F_x;
  const auto F_yf = front_tire_forces.F_y;
  const auto F_xr = rear_tire_forces.F_x;
  const auto F_yr = rear_tire_forces.F_y;

  // convert acceleration u to brake and engine torque
  const auto T_B = (u(INPUT_ACCEL) < 0.0) ? m * params_.effective_wheel_radius * u(INPUT_ACCEL) : 0.0;
  const auto T_E = (u(INPUT_ACCEL) > 0.0) ? m * params_.effective_wheel_radius * u(INPUT_ACCEL) : 0.0;

  const auto d_v =                                                               //
      (1.0 / m) * (-F_yf * std::sin(s(STATE_STEERING) - s(STATE_SLIP_ANGLE))     //
                   + F_yr * std::sin(s(STATE_SLIP_ANGLE))                        //
                   + F_xr * std::cos(s(STATE_SLIP_ANGLE))                        //
                   + F_xf * std::cos(s(STATE_STEERING) - s(STATE_SLIP_ANGLE)));  //

  const auto dd_psi =                                           //
      (1.0 / I) * (F_yf * std::cos(s(STATE_STEERING)) * lf      //
                   - F_yr * lr                                  //
                   + F_xf * std::sin(s(STATE_STEERING)) * lf);  //

  const auto d_beta =
      -s(STATE_YAW_RATE)                                                                                 //
      + (1.0 / (m * s(STATE_VELOCITY))) * (F_yf * std::cos(s(STATE_STEERING) - s(STATE_SLIP_ANGLE))      //
                                           + F_yr * std::cos(s(STATE_SLIP_ANGLE))                        //
                                           - F_xr * std::sin(s(STATE_SLIP_ANGLE))                        //
                                           + F_xf * std::sin(s(STATE_STEERING) - s(STATE_SLIP_ANGLE)));  //

  // Wheel dynamics.
  // wheels cannot spin backwards, so we lower-bound the angular speed at zero.
  auto d_omega_f = 0.0;
  if (s(STATE_FRONT_WHEEL_W) > 0.0) {
    d_omega_f = (-params_.effective_wheel_radius * F_xf  //
                 + params_.brake_torque_split * T_B      //
                 + params_.engine_torque_split * T_E     //
                 ) *                                     //
                (1.0 / params_.wheel_inertia);           //
  } else {
    // make the speed go to zero
    d_omega_f = -s(STATE_FRONT_WHEEL_W) / dt;
  }

  auto d_omega_r = 0.0;
  if (s(STATE_REAR_WHEEL_W) > 0.0) {
    d_omega_r = (-params_.effective_wheel_radius * F_xr       //
                 + (1.0 - params_.brake_torque_split) * T_B   //
                 + (1.0 - params_.engine_torque_split) * T_E  //
                 ) *                                          //
                (1.0 / params_.wheel_inertia);                //

  } else {
    // make the speed go to zero
    d_omega_r = -s(STATE_REAR_WHEEL_W) / dt;
  }

  const auto [d_phi_left, d_phi_right] =
      computeRearWheelAngularVelocities(s(STATE_REAR_WHEEL_W) * params_.effective_wheel_radius, s(STATE_YAW_RATE));

  // Acceleration along vehicle X-axis: d(v*cos(slip_angle))/dt
  const auto accel_x = d_v * std::cos(s(STATE_SLIP_ANGLE)) - s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE)) * d_beta;

  return State{
      s(STATE_VELOCITY) * std::cos(s(STATE_SLIP_ANGLE) + s(STATE_YAW)),  // dx/dt
      s(STATE_VELOCITY) * std::sin(s(STATE_SLIP_ANGLE) + s(STATE_YAW)),  // dy/dt
      u(INPUT_STEERING_VEL),                                             // d(steering angle)/dt
      d_v,                                                               // d(velocity)/dt
      s(STATE_YAW_RATE),                                                 // d(yaw angle)/dt
      dd_psi,                                                            // d(yaw rate)/dt
      d_beta,                                                            // d(slip angle)/dt
      d_omega_f,                                                         // d(front wheel angular speed)/dt
      d_omega_r,                                                         // d(rear wheel angular speed)/dt
      d_phi_left,                                                        // d(left wheel position)/dt
      d_phi_right,                                                       // d(right wheel position)/dt
      (accel_x - s(STATE_ACCEL)) / dt  // d(accel)/dt (tracks X-axis accel with tau=dt)
  };
}

std::pair<double, double> SingleTrackDriftModel::computeRearWheelAngularVelocities(
    double longitudinal_velocity,
    double yaw_rate) const {
  // Calculate the linear velocities at the left and right rear wheels
  const double half_track = params_.rear_track_width / 2.0;
  const double omega_left = longitudinal_velocity - half_track * yaw_rate;
  const double omega_right = longitudinal_velocity + half_track * yaw_rate;

  // Convert linear velocities to angular velocities
  const double d_phi_left = omega_left / params_.effective_wheel_radius;
  const double d_phi_right = omega_right / params_.effective_wheel_radius;

  return {d_phi_left, d_phi_right};
}

}  // namespace svdsim_server
