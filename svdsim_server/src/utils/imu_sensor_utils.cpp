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

#include "svdsim_server/utils/imu_sensor_utils.hpp"

#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>

namespace svdsim_server {

TrajectoryDerivatives compute_derivatives_from_trajectory(
    const Sophus::SE2d& pose_t0,
    const Sophus::SE2d& pose_t1,
    const Sophus::SE2d& pose_t2,
    double dt) {
  TrajectoryDerivatives result;

  // Extract positions (2D)
  const Eigen::Vector2d pos_t0 = pose_t0.translation();
  const Eigen::Vector2d pos_t1 = pose_t1.translation();
  const Eigen::Vector2d pos_t2 = pose_t2.translation();

  // Extract orientations using SO2 rotations
  const Sophus::SO2d R_t0 = pose_t0.so2();
  const Sophus::SO2d R_t1 = pose_t1.so2();
  const Sophus::SO2d R_t2 = pose_t2.so2();

  // Compute linear velocities using backward differences
  // v(t0) = (pos(t0) - pos(t1)) / dt
  // v(t-1) = (pos(t-1) - pos(t-2)) / dt
  const Eigen::Vector2d vel_t0_2d = (pos_t0 - pos_t1) / dt;
  const Eigen::Vector2d vel_t1_2d = (pos_t1 - pos_t2) / dt;

  // Compute linear acceleration using backward difference of velocities
  // a(t0) = (v(t0) - v(t-1)) / dt
  const Eigen::Vector2d accel_2d = (vel_t0_2d - vel_t1_2d) / dt;
  result.linear_acceleration << accel_2d.x(), accel_2d.y(), 0.0;

  // Compute angular velocities using SO2 group operations (handles wrap-around correctly)
  // Relative rotation from t1 to t0: R_delta = R_t0 * R_t1^-1
  // Angular displacement: delta_theta = log(R_delta)
  const Sophus::SO2d R_delta_01 = R_t0 * R_t1.inverse();
  const double delta_theta_01 = R_delta_01.log();
  const double omega_z_t0 = delta_theta_01 / dt;

  // Relative rotation from t2 to t1
  const Sophus::SO2d R_delta_12 = R_t1 * R_t2.inverse();
  const double delta_theta_12 = R_delta_12.log();
  const double omega_z_t1 = delta_theta_12 / dt;

  // Construct angular velocity vector (only Z component is non-zero for planar motion)
  result.angular_velocity << 0.0, 0.0, omega_z_t0;

  // Compute angular acceleration using backward difference of angular velocities
  // alpha_z(t0) = (omega_z(t0) - omega_z(t-1)) / dt
  const double alpha_z = (omega_z_t0 - omega_z_t1) / dt;
  result.angular_acceleration << 0.0, 0.0, alpha_z;

  return result;
}

ImuSamplingInputs prepare_imu_sampling_inputs(
    const Sophus::SE2d& pose_t0,
    const Sophus::SE2d& pose_t1,
    const Sophus::SE2d& pose_t2,
    double dt) {
  // Compute derivatives from trajectory using finite differences
  const TrajectoryDerivatives derivatives = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, dt);

  // Convert SE2d to SE3d (planar motion: z=0, no roll/pitch)
  // SE2: [x, y, theta]
  // SE3: [x, y, 0], [0, 0, theta] (rotation around Z axis only)
  const Eigen::Vector3d translation_3d(pose_t0.translation().x(), pose_t0.translation().y(), 0.0);
  const double yaw = pose_t0.so2().log();
  const Sophus::SO3d rotation_3d = Sophus::SO3d::rotZ(yaw);
  const Sophus::SE3d T_iv(rotation_3d, translation_3d);

  // Transform angular velocities and accelerations from inertial to vehicle frame
  // omega_vi_v = C_vi * omega_vi_i
  const Sophus::SO3d C_vi = rotation_3d.inverse();
  const Eigen::Vector3d omega_vi_v = C_vi * derivatives.angular_velocity;
  const Eigen::Vector3d omega_dot_vi_v = C_vi * derivatives.angular_acceleration;

  // Populate and return sampling inputs
  ImuSamplingInputs inputs;
  inputs.T_iv = T_iv;
  inputs.r_ddot_vi_i = derivatives.linear_acceleration;
  inputs.omega_vi_v = omega_vi_v;
  inputs.omega_dot_vi_v = omega_dot_vi_v;

  return inputs;
}

}  // namespace svdsim_server
