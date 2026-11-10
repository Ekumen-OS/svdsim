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

#ifndef SVDSIM_SERVER_UTILS_IMU_SENSOR_UTILS_HPP_
#define SVDSIM_SERVER_UTILS_IMU_SENSOR_UTILS_HPP_

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include "svdsim_server/models/imu_sensor_model.hpp"

namespace svdsim_server {

/**
 * @brief Result of computing derivatives from trajectory poses
 */
struct TrajectoryDerivatives {
  /// Linear acceleration in inertial frame [m/s^2]
  Eigen::Vector3d linear_acceleration;
  /// Angular velocity in inertial frame [rad/s]
  Eigen::Vector3d angular_velocity;
  /// Angular acceleration in inertial frame [rad/s^2]
  Eigen::Vector3d angular_acceleration;
};

/**
 * @brief Inputs required for IMU sensor sampling
 *
 * Contains all the kinematic quantities and transformations needed to call
 * the IMU sensor model sampling functions.
 */
struct ImuSamplingInputs {
  /// Vehicle pose in inertial frame (transforms from vehicle to inertial)
  Sophus::SE3d T_iv;
  /// Linear acceleration of vehicle origin in inertial frame [m/s^2]
  Eigen::Vector3d r_ddot_vi_i;
  /// Angular velocity of vehicle relative to inertial, expressed in vehicle frame [rad/s]
  Eigen::Vector3d omega_vi_v;
  /// Angular acceleration of vehicle relative to inertial, expressed in vehicle frame [rad/s^2]
  Eigen::Vector3d omega_dot_vi_v;
};

/**
 * @brief Compute kinematic derivatives from three consecutive trajectory poses using finite differences
 *
 * This function computes linear acceleration, angular velocity, and angular acceleration
 * using 3-point backward finite differences. For angular quantities, it uses proper SO2
 * group operations to handle angle wrap-around correctly.
 *
 * The time indexing is:
 * - pose_t0: Current time (t)
 * - pose_t1: One timestep ago (t - dt)
 * - pose_t2: Two timesteps ago (t - 2*dt)
 *
 * @param pose_t0 Most recent pose (current time)
 * @param pose_t1 Previous pose (one timestep ago)
 * @param pose_t2 Oldest pose (two timesteps ago)
 * @param dt Time step between consecutive poses [s]
 * @return TrajectoryDerivatives containing linear_acceleration, angular_velocity, and angular_acceleration
 */
TrajectoryDerivatives compute_derivatives_from_trajectory(
    const Sophus::SE2d& pose_t0,
    const Sophus::SE2d& pose_t1,
    const Sophus::SE2d& pose_t2,
    double dt);

/**
 * @brief Prepare inputs for IMU sensor sampling from three trajectory poses
 *
 * This function takes three consecutive poses and computes all the kinematic
 * quantities needed to call the IMU sensor model sampling functions. It performs:
 * - Finite difference computation of linear acceleration and angular velocity/acceleration
 * - Coordinate transformations from inertial to vehicle frame
 * - SE2d to SE3d pose conversion for 3D sensor model
 *
 * The caller should use the returned inputs to call:
 * - sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb)
 * - sample_gyro(omega_vi_v, C_vb)
 * - sample_magnetometer(T_iv, C_vb)
 *
 * @param pose_t0 Most recent pose (current time)
 * @param pose_t1 Previous pose (one timestep ago)
 * @param pose_t2 Oldest pose (two timesteps ago)
 * @param dt Time step between consecutive poses [s]
 * @return ImuSamplingInputs containing all arguments needed for IMU sensor sampling
 */
ImuSamplingInputs prepare_imu_sampling_inputs(
    const Sophus::SE2d& pose_t0,
    const Sophus::SE2d& pose_t1,
    const Sophus::SE2d& pose_t2,
    double dt);

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_UTILS_IMU_SENSOR_UTILS_HPP_
