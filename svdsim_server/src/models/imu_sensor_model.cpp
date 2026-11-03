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

#include "svdsim_server/models/imu_sensor_model.hpp"

#include <random>

namespace svdsim_server {

IMUSensorModel::IMUSensorModel(const Sophus::SE3d& imu_pose, double gravity, const IMUNoiseParameters& noise_params)
    : imu_pose_in_body(imu_pose),
      gravity_(gravity),
      noise_params_(noise_params),
      rng_(std::random_device{}()),
      normal_dist_(0.0, 1.0) {
  reset();
}

Eigen::Vector3d IMUSensorModel::sample_accelerometer(
    const Sophus::SE3d& T_iv,               // Vehicle pose: Transforms v to i
    const Eigen::Vector3d& r_ddot_vi_i,     // Linear acceleration of vehicle in inertial frame
    const Eigen::Vector3d& omega_vi_v,      // Angular velocity of vehicle relative to i, in v frame
    const Eigen::Vector3d& omega_dot_vi_v,  // Angular acceleration of vehicle relative to i, in v frame
    const Sophus::SO3d& C_vb                // Rotation from body to vehicle
) {
  // Implementation based on Barfoot, "State Estimation for Robotics" (2017)

  // 1. Setup Rotations (Barfoot notation: C_ba transforms a vector from a to b)
  const Sophus::SO3d C_vi = T_iv.so3().inverse();              // From Inertial to Vehicle
  const Sophus::SO3d C_bs = imu_pose_in_body.so3().inverse();  // From Sensor to Body
  const Sophus::SO3d C_si = C_bs * C_vb.inverse() * C_vi;      // From Inertial to Sensor

  // 2. Define Lever Arm: r_sv_v (Vector from v to s, expressed in v)
  // Assumes imu_pose_in_body.translation() is the offset in the body frame
  const Eigen::Vector3d r_sv_v = C_vb * imu_pose_in_body.translation();

  // 3. Compute Lever-Arm Acceleration in the Vehicle Frame
  // This captures the tangential (Euler) and centripetal effects
  // Equation: omega_dot x r + omega x (omega x r)
  const Eigen::Vector3d euler_accel_v = omega_dot_vi_v.cross(r_sv_v);
  const Eigen::Vector3d centripetal_accel_v = omega_vi_v.cross(omega_vi_v.cross(r_sv_v));
  const Eigen::Vector3d lever_accel_v = euler_accel_v + centripetal_accel_v;

  // 4. Transform Vehicle-frame effects to Inertial Frame
  // This allows us to add the global vehicle acceleration and gravity
  const Sophus::SO3d C_iv = C_vi.inverse();
  const Eigen::Vector3d r_ddot_si_i = r_ddot_vi_i + (C_iv * lever_accel_v);

  // 5. Apply Gravity and Rotate to Sensor Frame (Barfoot Eq 6.152)
  // Measurement y = C_si * (r_ddot_si_i - g_i)
  // Note: If g_i is [0, 0, -9.81], then -g_i is [0, 0, +9.81]
  const Eigen::Vector3d g_i(0.0, 0.0, -gravity_);

  // Force explicit evaluation to avoid any expression template aliasing
  const Eigen::Vector3d specific_force_i = r_ddot_si_i - g_i;

  const Eigen::Vector3d y_acc = C_si * specific_force_i;

  return y_acc + sample_accelerometer_noise();
}

Eigen::Vector3d IMUSensorModel::sample_gyro(
    const Eigen::Vector3d& omega_vi_v,  // Angular velocity of vehicle relative to i, in v frame
    const Sophus::SO3d& C_vb            // Rotation from body to vehicle (static)
) {
  // Implementation based on Barfoot, "State Estimation for Robotics" (2017)

  // Gyroscope measurement model (Barfoot Eq. 6.156):
  // y_gyro = omega_is_s
  //
  // where omega_is_s is the angular velocity of the sensor frame relative to
  // the inertial frame, expressed in the sensor frame.
  //
  // Assumptions:
  // - Body is very slowly varying relative to vehicle: omega_bv = 0
  // - Sensor is rigidly mounted to body: omega_sb = 0
  // Therefore: omega_is = omega_iv, and we directly transform from vehicle to sensor frame.

  // 1. Setup Rotations (Barfoot notation: C_ba transforms a vector from a to b)
  const Sophus::SO3d C_bv = C_vb.inverse();                    // From Vehicle to Body
  const Sophus::SO3d C_sb = imu_pose_in_body.so3().inverse();  // From Body to Sensor
  const Sophus::SO3d C_sv = C_sb * C_bv;                       // From Vehicle to Sensor

  // 2. Transform to sensor frame (Barfoot measurement model)
  const Eigen::Vector3d y_gyro = C_sv * omega_vi_v;

  return y_gyro + sample_gyroscope_noise();
}

Eigen::Vector3d IMUSensorModel::sample_magnetometer(
    const Sophus::SE3d& T_iv,  // Vehicle pose: Transforms v to i
    const Sophus::SO3d& C_vb   // Rotation from body to vehicle
) {
  // Magnetometer measurement model:
  // y_mag = C_si * B_i
  //
  // where B_i is the magnetic field in the inertial frame.
  // We assume magnetic north is aligned with the X axis of the world/inertial frame,
  // so B_i = [B_mag, 0, 0] where B_mag is the magnitude of the magnetic field.

  // 1. Setup Rotations (Barfoot notation: C_ba transforms a vector from a to b)
  const Sophus::SO3d C_vi = T_iv.so3().inverse();              // From Inertial to Vehicle
  const Sophus::SO3d C_bs = imu_pose_in_body.so3().inverse();  // From Sensor to Body
  const Sophus::SO3d C_si = C_bs * C_vb.inverse() * C_vi;      // From Inertial to Sensor

  // 2. Define magnetic field in inertial frame (aligned with X axis)
  // Earth's magnetic field magnitude: 45 microTeslas
  const Eigen::Vector3d B_i(45e-6, 0.0, 0.0);

  // 3. Transform magnetic field to sensor frame
  const Eigen::Vector3d y_mag = C_si * B_i;

  return y_mag + sample_magnetometer_noise();
}

Eigen::Vector3d IMUSensorModel::sample_accelerometer_noise() {
  /// we use the Kalibr noise model for IMU noise
  // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model .
  // We augment the model with an autocorrelation time for the bias,
  // to be able to model the process as a First-Order Gauss-Markov process
  // to avoid infinite variance growth over time.

  const auto sqrt_dt = std::sqrt(noise_params_.imu_sampling_period);

  const auto sigma_a = noise_params_.accelerometer_noise_density;  // noise density [m/s^2/\sqrt{Hz}]
  const auto sigma_ad = sigma_a / sqrt_dt;                         // discrete noise [m/s^2]
  const auto sigma_ba = noise_params_.accelerometer_random_walk;   // random walk [m/s^2/\sqrt{Hz}]
  const auto sigma_bad = sigma_ba * sqrt_dt;                       // discrete random walk [m/s^2]

  auto& b_d = accelerometer_bias_;

  // Apply exponential decay to bias (first-order Markov process)
  const double dt = noise_params_.imu_sampling_period;
  const double tau = noise_params_.accelerometer_bias_autocorrelation_time;
  const double decay_factor = std::exp(-dt / tau);

  // discrete white noise
  Eigen::Vector3d n_d = Eigen::Vector3d{
      sigma_ad * sample_unit_gaussian(),  //
      sigma_ad * sample_unit_gaussian(),  //
      sigma_ad * sample_unit_gaussian()   //
  };

  // discrete bias random walk
  b_d = decay_factor * b_d + Eigen::Vector3d{
                                 sigma_bad * sample_unit_gaussian(),  //
                                 sigma_bad * sample_unit_gaussian(),  //
                                 sigma_bad * sample_unit_gaussian()   //
                             };

  return n_d + b_d;
}

Eigen::Vector3d IMUSensorModel::sample_gyroscope_noise() {
  /// we use the Kalibr noise model for IMU noise
  // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model .
  // We augment the model with an autocorrelation time for the bias,
  // to be able to model the process as a First-Order Gauss-Markov process
  // to avoid infinite variance growth over time.

  const auto sqrt_dt = std::sqrt(noise_params_.imu_sampling_period);

  const auto sigma_g = noise_params_.gyroscope_noise_density;  // noise density [rad/s/\sqrt{Hz}]
  const auto sigma_gd = sigma_g / sqrt_dt;                     // discrete noise [rad/s]
  const auto sigma_bg = noise_params_.gyroscope_random_walk;   // random walk [rad/s/\sqrt{Hz}]
  const auto sigma_bgd = sigma_bg * sqrt_dt;                   // discrete random walk [rad/s]
  auto& b_d = gyroscope_bias_;

  // Apply exponential decay to bias (first-order Markov process)
  const double dt = noise_params_.imu_sampling_period;
  const double tau = noise_params_.gyroscope_bias_autocorrelation_time;
  const double decay_factor = std::exp(-dt / tau);

  // discrete white noise
  Eigen::Vector3d n_d = Eigen::Vector3d{
      sigma_gd * sample_unit_gaussian(),  //
      sigma_gd * sample_unit_gaussian(),  //
      sigma_gd * sample_unit_gaussian()   //
  };

  // discrete bias random walk
  b_d = decay_factor * b_d + Eigen::Vector3d{
                                 sigma_bgd * sample_unit_gaussian(),  //
                                 sigma_bgd * sample_unit_gaussian(),  //
                                 sigma_bgd * sample_unit_gaussian()   //
                             };

  return n_d + b_d;
}

Eigen::Vector3d IMUSensorModel::sample_magnetometer_noise() {
  /// Kalibr does not define a noise mode for the magnetometer, but
  /// we can apply the same model with a shorter autocorrelation time
  /// to model local magnetic disturbances.

  const auto sqrt_dt = std::sqrt(noise_params_.imu_sampling_period);

  const auto sigma_m = noise_params_.magnetometer_noise_density;  // noise density [T/\sqrt{Hz}]
  const auto sigma_md = sigma_m / sqrt_dt;                        // discrete noise [T]
  const auto sigma_bm = noise_params_.magnetometer_random_walk;   // random walk [T/s/\sqrt{Hz}]
  const auto sigma_bmd = sigma_bm * sqrt_dt;                      // discrete random walk [T]

  auto& b_d = magnetometer_bias_;

  // Apply exponential decay to bias (first-order Markov process)
  const double dt = noise_params_.imu_sampling_period;
  const double tau = noise_params_.magnetometer_bias_autocorrelation_time;
  const double decay_factor = std::exp(-dt / tau);

  // discrete white noise
  Eigen::Vector3d n_d = Eigen::Vector3d{
      sigma_md * sample_unit_gaussian(),  //
      sigma_md * sample_unit_gaussian(),  //
      sigma_md * sample_unit_gaussian()   //
  };

  // discrete bias random walk
  b_d = decay_factor * b_d + Eigen::Vector3d{
                                 sigma_bmd * sample_unit_gaussian(),  //
                                 sigma_bmd * sample_unit_gaussian(),  //
                                 sigma_bmd * sample_unit_gaussian()   //
                             };

  return n_d + b_d;
}

double IMUSensorModel::sample_unit_gaussian() {
  return normal_dist_(rng_);
}

void IMUSensorModel::reset() {
  gyroscope_bias_ = noise_params_.initial_gyroscope_bias;
  accelerometer_bias_ = noise_params_.initial_accelerometer_bias;
  magnetometer_bias_ = noise_params_.initial_magnetometer_bias;
}

}  // namespace svdsim_server
