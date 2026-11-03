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

#include <gtest/gtest.h>

#include <cmath>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include "svdsim_server/utils/imu_sensor_utils.hpp"

namespace svdsim_server {
namespace {

constexpr double kTolerance = 1e-6;
constexpr double kDt = 0.01;  // 10 ms timestep

// Test compute_derivatives_from_trajectory with constant velocity motion (zero acceleration)
TEST(ImuSensorUtilsTest, ComputeDerivativesConstantVelocity) {
  // Setup: Moving at constant velocity (1 m/s) in X direction, no rotation
  const double vx = 1.0;
  const Sophus::SE2d pose_t0(Sophus::SO2d(0.0), Eigen::Vector2d(2.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t1(Sophus::SO2d(0.0), Eigen::Vector2d(1.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // Expect zero acceleration for constant velocity
  EXPECT_NEAR(result.linear_acceleration.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.linear_acceleration.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.linear_acceleration.z(), 0.0, kTolerance);

  // Expect zero angular velocity and acceleration
  EXPECT_NEAR(result.angular_velocity.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_velocity.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_velocity.z(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_acceleration.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_acceleration.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_acceleration.z(), 0.0, kTolerance);
}

// Test compute_derivatives_from_trajectory with constant acceleration
TEST(ImuSensorUtilsTest, ComputeDerivativesConstantAcceleration) {
  // Setup: Constant acceleration of 2 m/s^2 in X direction
  // Using kinematic equation: x = x0 + v0*t + 0.5*a*t^2
  // With v0 = 0, a = 2 m/s^2
  const double a = 2.0;
  const double x_t0 = 0.5 * a * (2.0 * kDt) * (2.0 * kDt);  // t = 2*dt
  const double x_t1 = 0.5 * a * (1.0 * kDt) * (1.0 * kDt);  // t = dt
  const double x_t2 = 0.0;                                  // t = 0

  const Sophus::SE2d pose_t0(Sophus::SO2d(0.0), Eigen::Vector2d(x_t0, 0.0));
  const Sophus::SE2d pose_t1(Sophus::SO2d(0.0), Eigen::Vector2d(x_t1, 0.0));
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d(x_t2, 0.0));

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // Expect acceleration of 2 m/s^2 in X direction
  EXPECT_NEAR(result.linear_acceleration.x(), a, kTolerance);
  EXPECT_NEAR(result.linear_acceleration.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.linear_acceleration.z(), 0.0, kTolerance);
}

// Test compute_derivatives_from_trajectory with constant angular velocity
TEST(ImuSensorUtilsTest, ComputeDerivativesConstantAngularVelocity) {
  // Setup: Rotating at constant angular velocity (1 rad/s)
  const double omega_z = 1.0;  // rad/s
  const Sophus::SE2d pose_t0(Sophus::SO2d(2.0 * omega_z * kDt), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t1(Sophus::SO2d(1.0 * omega_z * kDt), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // Expect constant angular velocity
  EXPECT_NEAR(result.angular_velocity.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_velocity.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_velocity.z(), omega_z, kTolerance);

  // Expect zero angular acceleration for constant angular velocity
  EXPECT_NEAR(result.angular_acceleration.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_acceleration.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.angular_acceleration.z(), 0.0, kTolerance);
}

// Test compute_derivatives_from_trajectory with angular wrap-around (crossing +/-pi)
TEST(ImuSensorUtilsTest, ComputeDerivativesAngularWrapAround) {
  // Setup: Rotating through the +/-pi discontinuity
  // Going from +pi - 0.1 to -pi + 0.1 should be correctly handled by SO2
  const double angle_t2 = M_PI - 0.15;
  const double angle_t1 = M_PI - 0.05;
  const double angle_t0 = -M_PI + 0.05;  // Wrapped around

  const Sophus::SE2d pose_t0(Sophus::SO2d(angle_t0), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t1(Sophus::SO2d(angle_t1), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t2(Sophus::SO2d(angle_t2), Eigen::Vector2d::Zero());

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // The angular velocity should be positive (rotating counterclockwise)
  // Delta angle should be ~0.1 rad, not ~2*pi
  const double expected_omega_z = 0.1 / kDt;                        // ~10 rad/s
  EXPECT_NEAR(result.angular_velocity.z(), expected_omega_z, 0.1);  // Allow some tolerance
  EXPECT_GT(result.angular_velocity.z(), 0.0);                      // Should be positive
}

// Test compute_derivatives_from_trajectory with combined motion
TEST(ImuSensorUtilsTest, ComputeDerivativesCombinedMotion) {
  // Setup: Moving in X direction with rotation
  const double vx = 2.0;       // m/s
  const double omega_z = 0.5;  // rad/s

  const Sophus::SE2d pose_t0(Sophus::SO2d(2.0 * omega_z * kDt), Eigen::Vector2d(2.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t1(Sophus::SO2d(1.0 * omega_z * kDt), Eigen::Vector2d(1.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // Expect zero linear acceleration (constant velocity)
  EXPECT_NEAR(result.linear_acceleration.x(), 0.0, kTolerance);
  EXPECT_NEAR(result.linear_acceleration.y(), 0.0, kTolerance);

  // Expect constant angular velocity
  EXPECT_NEAR(result.angular_velocity.z(), omega_z, kTolerance);

  // Expect zero angular acceleration
  EXPECT_NEAR(result.angular_acceleration.z(), 0.0, kTolerance);
}

// Test compute_derivatives_from_trajectory with circular motion
TEST(ImuSensorUtilsTest, ComputeDerivativesCircularMotion) {
  // Setup: Moving in a circle with radius R at constant angular velocity omega
  const double R = 1.0;      // radius in meters
  const double omega = 1.0;  // angular velocity in rad/s

  // Positions at different times
  const double theta_t0 = 2.0 * omega * kDt;
  const double theta_t1 = 1.0 * omega * kDt;
  const double theta_t2 = 0.0;

  const Eigen::Vector2d pos_t0(R * std::cos(theta_t0), R * std::sin(theta_t0));
  const Eigen::Vector2d pos_t1(R * std::cos(theta_t1), R * std::sin(theta_t1));
  const Eigen::Vector2d pos_t2(R * std::cos(theta_t2), R * std::sin(theta_t2));

  // For circular motion, vehicle orientation is tangent to circle (perpendicular to radius)
  const Sophus::SE2d pose_t0(Sophus::SO2d(theta_t0 + M_PI / 2), pos_t0);
  const Sophus::SE2d pose_t1(Sophus::SO2d(theta_t1 + M_PI / 2), pos_t1);
  const Sophus::SE2d pose_t2(Sophus::SO2d(theta_t2 + M_PI / 2), pos_t2);

  const auto result = compute_derivatives_from_trajectory(pose_t0, pose_t1, pose_t2, kDt);

  // For circular motion, centripetal acceleration = omega^2 * R
  const double expected_centripetal_accel = omega * omega * R;

  // Linear acceleration should have magnitude close to centripetal acceleration
  const double accel_magnitude = std::sqrt(
      result.linear_acceleration.x() * result.linear_acceleration.x() +
      result.linear_acceleration.y() * result.linear_acceleration.y());
  EXPECT_NEAR(accel_magnitude, expected_centripetal_accel, 0.1);  // Allow some numerical error

  // Angular velocity should be constant
  EXPECT_NEAR(result.angular_velocity.z(), omega, kTolerance);

  // Angular acceleration should be zero (constant angular velocity)
  EXPECT_NEAR(result.angular_acceleration.z(), 0.0, 0.1);  // Allow some numerical error
}

// Test prepare_imu_sampling_inputs with stationary vehicle
TEST(ImuSensorUtilsTest, PrepareInputsStationary) {
  // Stationary poses at a specific location
  const Sophus::SE2d stationary_pose(Sophus::SO2d(0.0), Eigen::Vector2d(1.0, 2.0));

  const auto inputs = prepare_imu_sampling_inputs(stationary_pose, stationary_pose, stationary_pose, kDt);

  // For stationary vehicle, linear acceleration should be zero
  EXPECT_NEAR(inputs.r_ddot_vi_i.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.r_ddot_vi_i.y(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.r_ddot_vi_i.z(), 0.0, kTolerance);

  // Angular velocity and acceleration should be zero
  EXPECT_NEAR(inputs.omega_vi_v.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_vi_v.y(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_vi_v.z(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_dot_vi_v.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_dot_vi_v.y(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_dot_vi_v.z(), 0.0, kTolerance);

  // Pose should be correctly converted to SE3d
  EXPECT_NEAR(inputs.T_iv.translation().x(), 1.0, kTolerance);
  EXPECT_NEAR(inputs.T_iv.translation().y(), 2.0, kTolerance);
  EXPECT_NEAR(inputs.T_iv.translation().z(), 0.0, kTolerance);
}

// Test prepare_imu_sampling_inputs with constant velocity
TEST(ImuSensorUtilsTest, PrepareInputsConstantVelocity) {
  // Constant velocity motion
  const double vx = 5.0;  // 5 m/s
  const Sophus::SE2d pose_t0(Sophus::SO2d(0.0), Eigen::Vector2d(2.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t1(Sophus::SO2d(0.0), Eigen::Vector2d(1.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));

  const auto inputs = prepare_imu_sampling_inputs(pose_t0, pose_t1, pose_t2, kDt);

  // For constant velocity, linear acceleration should be zero
  EXPECT_NEAR(inputs.r_ddot_vi_i.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.r_ddot_vi_i.y(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.r_ddot_vi_i.z(), 0.0, kTolerance);

  // No rotation, so angular velocity should be zero
  EXPECT_NEAR(inputs.omega_vi_v.z(), 0.0, kTolerance);
}

// Test prepare_imu_sampling_inputs with rotation
TEST(ImuSensorUtilsTest, PrepareInputsRotation) {
  // Rotating at constant angular velocity
  const double omega = 2.0;  // 2 rad/s
  const Sophus::SE2d pose_t0(Sophus::SO2d(2.0 * omega * kDt), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t1(Sophus::SO2d(1.0 * omega * kDt), Eigen::Vector2d::Zero());
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());

  const auto inputs = prepare_imu_sampling_inputs(pose_t0, pose_t1, pose_t2, kDt);

  // Should have angular velocity in vehicle frame
  EXPECT_NEAR(inputs.omega_vi_v.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_vi_v.y(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.omega_vi_v.z(), omega, 0.01);

  // Angular acceleration should be zero (constant angular velocity)
  EXPECT_NEAR(inputs.omega_dot_vi_v.z(), 0.0, 0.01);
}

// Test prepare_imu_sampling_inputs with combined motion
TEST(ImuSensorUtilsTest, PrepareInputsCombinedMotion) {
  // Moving and rotating
  const double vx = 3.0;     // m/s
  const double omega = 1.0;  // rad/s

  const Sophus::SE2d pose_t0(Sophus::SO2d(2.0 * omega * kDt), Eigen::Vector2d(2.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t1(Sophus::SO2d(1.0 * omega * kDt), Eigen::Vector2d(1.0 * vx * kDt, 0.0));
  const Sophus::SE2d pose_t2(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));

  const auto inputs = prepare_imu_sampling_inputs(pose_t0, pose_t1, pose_t2, kDt);

  // Constant velocity, so zero acceleration
  EXPECT_NEAR(inputs.r_ddot_vi_i.x(), 0.0, kTolerance);
  EXPECT_NEAR(inputs.r_ddot_vi_i.y(), 0.0, kTolerance);

  // Should have angular velocity
  EXPECT_NEAR(inputs.omega_vi_v.z(), omega, 0.01);

  // Pose should be at current position
  EXPECT_NEAR(inputs.T_iv.translation().x(), 2.0 * vx * kDt, kTolerance);
}

}  // namespace
}  // namespace svdsim_server
