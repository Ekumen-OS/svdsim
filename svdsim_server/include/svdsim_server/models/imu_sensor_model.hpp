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

#ifndef MONACOF1_SIM_SERVER_MODELS_IMU_SENSOR_MODEL_HPP_
#define MONACOF1_SIM_SERVER_MODELS_IMU_SENSOR_MODEL_HPP_

#include <random>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace svdsim_server {

/**
 * @brief IMU noise parameters
 *
 * Contains noise characteristics for IMU sensor simulation.
 */
struct IMUNoiseParameters {
  /// IMU sampling period [s]
  double imu_sampling_period = 1.0;

  /// Accelerometer noise density [m/s^2/\sqrt{Hz}]
  double accelerometer_noise_density = 0.0;

  /// Accelerometer random walk [m/s^3/\sqrt{Hz}]
  double accelerometer_random_walk = 0.0;

  /// Initial accelerometer bias [m/s^2]
  Eigen::Vector3d initial_accelerometer_bias = Eigen::Vector3d::Zero();

  /// Accelerometer bias autocorrelation time [s]
  double accelerometer_bias_autocorrelation_time = 1e10;

  /// Gyroscope noise density [rad/s/\sqrt{Hz}]
  double gyroscope_noise_density = 0.0;

  /// Gyroscope random walk [rad/s^2/\sqrt{Hz}]
  double gyroscope_random_walk = 0.0;

  /// Initial gyroscope bias [rad/s]
  Eigen::Vector3d initial_gyroscope_bias = Eigen::Vector3d::Zero();

  /// Gyroscope bias autocorrelation time [s]
  double gyroscope_bias_autocorrelation_time = 1e10;

  /// Magnetometer noise density [T/\sqrt{Hz}]
  double magnetometer_noise_density = 0.0;

  /// Magnetometer random walk [T/s/\sqrt{Hz}]
  double magnetometer_random_walk = 0.0;

  /// Initial magnetometer bias [T]
  Eigen::Vector3d initial_magnetometer_bias = Eigen::Vector3d::Zero();

  /// Magnetometer bias autocorrelation time [s]
  double magnetometer_bias_autocorrelation_time = 1e10;
};

/**
 * @brief Model for IMU sensor simulation
 *
 * Simulates a 6-DOF IMU sensor (accelerometer and gyroscope) with optional magnetometer.
 * The model accounts for the IMU mounting position and orientation within the vehicle body frame,
 * as well as body orientation variations due to suspension dynamics.
 */
class IMUSensorModel {
 public:
  /**
   * @brief Construct a new IMU Sensor Model
   *
   * @param imu_pose Pose of the IMU sensor in the vehicle body frame (SE3)
   * @param gravity Gravity magnitude [m/s^2]
   * @param noise_params IMU noise parameters
   */
  IMUSensorModel(const Sophus::SE3d& imu_pose, double gravity, const IMUNoiseParameters& noise_params);

  /**
   * @brief Sample the accelerometer
   *
   * Computes the 3D specific force measured by the IMU in the IMU frame
   * following the derivation from "State Estimation for Robotics" by Barfoot.
   *
   * Uses Barfoot notation where C_ba transforms a vector from frame a to frame b.
   * The accelerometer measures specific force (Barfoot Eq. 6.152):
   * y = C_si * (r_ddot_si_i - g_i)
   *
   * where r_ddot_si_i is the acceleration of the sensor origin in inertial frame
   * and g_i is the gravity vector in inertial frame [0, 0, -g].
   *
   * @param T_iv Vehicle pose in inertial frame (transforms from vehicle to inertial)
   * @param r_ddot_vi_i Linear acceleration of vehicle origin in inertial frame [m/s^2]
   * @param omega_vi_v Angular velocity of vehicle relative to inertial, expressed in vehicle frame [rad/s]
   * @param omega_dot_vi_v Angular acceleration of vehicle relative to inertial, expressed in vehicle frame [rad/s^2]
   * @param C_vb Rotation from body frame to vehicle frame (models suspension-induced orientation)
   * @return 3D specific force vector measured in IMU/sensor frame [m/s^2] (XYZ)
   */
  [[nodiscard]] Eigen::Vector3d sample_accelerometer(
      const Sophus::SE3d& T_iv,
      const Eigen::Vector3d& r_ddot_vi_i,
      const Eigen::Vector3d& omega_vi_v,
      const Eigen::Vector3d& omega_dot_vi_v,
      const Sophus::SO3d& C_vb);

  /**
   * @brief Sample the gyroscope
   *
   * Computes the 3D angular velocity measured by the IMU in the IMU frame
   * following the derivation from "State Estimation for Robotics" by Barfoot.
   *
   * Uses Barfoot notation where C_ba transforms a vector from frame a to frame b.
   * The gyroscope measures (Barfoot Eq. 6.156):
   * y_gyro = omega_is_s
   *
   * where omega_is_s is the angular velocity of the sensor relative to inertial,
   * expressed in the sensor frame.
   *
   * Assumptions:
   * - Body is static relative to vehicle: omega_bv = 0
   * - Sensor is rigidly mounted to body: omega_sb = 0
   * Therefore: omega_is = omega_iv, and we directly transform from vehicle to sensor frame.
   *
   * @param omega_vi_v Angular velocity of vehicle relative to inertial, expressed in vehicle frame [rad/s]
   * @param C_vb Rotation from body frame to vehicle frame (static)
   * @return 3D angular velocity vector in IMU/sensor frame [rad/s] (roll rate, pitch rate, yaw rate)
   */
  [[nodiscard]] Eigen::Vector3d sample_gyro(const Eigen::Vector3d& omega_vi_v, const Sophus::SO3d& C_vb);

  /**
   * @brief Sample the magnetometer
   *
   * Computes the magnetic field measured by the IMU in the IMU frame.
   * Assumes magnetic north is aligned with the X axis of the world/inertial frame.
   *
   * Uses Barfoot notation where C_ba transforms a vector from frame a to frame b.
   * The magnetometer measures the earth's magnetic field projected onto the sensor axes:
   * y_mag = C_si * B_i
   *
   * where B_i is the magnetic field vector in the inertial frame and C_si transforms
   * from inertial to sensor frame.
   *
   * @param T_iv Vehicle pose in inertial frame (transforms from vehicle to inertial)
   * @param C_vb Rotation from body frame to vehicle frame
   * @return 3D magnetic field vector measured in IMU/sensor frame [T] (XYZ)
   */
  [[nodiscard]] Eigen::Vector3d sample_magnetometer(const Sophus::SE3d& T_iv, const Sophus::SO3d& C_vb);

  /**
   * @brief Get the IMU pose in body frame
   * @return IMU pose SE3d transformation
   */
  [[nodiscard]] const Sophus::SE3d& imu_pose() const { return imu_pose_in_body; }

  /**
   * @brief Get the gravity magnitude
   * @return Gravity magnitude [m/s^2]
   */
  [[nodiscard]] double gravity() const { return gravity_; }

  /**
   * @brief Reset the IMU sensor state
   *
   * Initializes bias accumulators to their initial values.
   */
  void reset();

 private:
  /**
   * @brief Sample accelerometer noise
   * @return 3D accelerometer noise vector [m/s^2]
   */
  [[nodiscard]] Eigen::Vector3d sample_accelerometer_noise();

  /**
   * @brief Sample gyroscope noise
   * @return 3D gyroscope noise vector [rad/s]
   */
  [[nodiscard]] Eigen::Vector3d sample_gyroscope_noise();

  /**
   * @brief Sample magnetometer noise
   * @return 3D magnetometer noise vector [T]
   */
  [[nodiscard]] Eigen::Vector3d sample_magnetometer_noise();

  /**
   * @brief Sample from a unit Gaussian (standard normal distribution)
   * @return Random value from N(0,1)
   */
  [[nodiscard]] double sample_unit_gaussian();

  /// Pose of the IMU sensor in the body frame
  Sophus::SE3d imu_pose_in_body;

  /// Gravity magnitude [m/s^2]
  double gravity_;

  /// IMU noise parameters
  IMUNoiseParameters noise_params_;

  /// Current gyroscope bias [rad/s]
  Eigen::Vector3d gyroscope_bias_;

  /// Current accelerometer bias [m/s^2]
  Eigen::Vector3d accelerometer_bias_;

  /// Current magnetometer bias [T]
  Eigen::Vector3d magnetometer_bias_;

  /// Random number generator (mutable to allow const sampling methods)
  mutable std::mt19937 rng_;

  /// Standard normal distribution for noise generation
  mutable std::normal_distribution<double> normal_dist_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_IMU_SENSOR_MODEL_HPP_
