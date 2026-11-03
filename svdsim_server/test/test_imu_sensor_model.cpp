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

// Non-ROS dependencies
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>

// Project headers
#include "svdsim_server/models/imu_sensor_model.hpp"

namespace svdsim_server {

class IMUSensorModelTest : public ::testing::Test {
 protected:
  void SetUp() override { gravity_ = 9.81; }

  double gravity_;
};

// Test sample_magnetometer with identity transforms
TEST_F(IMUSensorModelTest, SampleMagnetometerIdentityTransform) {
  // Identity transform for sensor in body
  const auto imu_pose_in_body = Sophus::SE3d();

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with no rotation
  const auto T_iv = Sophus::SE3d();
  const auto C_vb = Sophus::SO3d();

  const auto mag_reading = imu_sensor->sample_magnetometer(T_iv, C_vb);

  // With all identity transforms, magnetometer should read [45e-6, 0, 0] (aligned with world X)
  const Eigen::Vector3d expected(45e-6, 0.0, 0.0);
  EXPECT_NEAR(mag_reading.x(), expected.x(), 1e-10);
  EXPECT_NEAR(mag_reading.y(), expected.y(), 1e-10);
  EXPECT_NEAR(mag_reading.z(), expected.z(), 1e-10);
}

// Test sample_magnetometer with 90 deg sensor rotation around Z axis
TEST_F(IMUSensorModelTest, SampleMagnetometerSensor90DegRotation) {
  // 90 degree rotation around Z axis (sensor rotated left)
  const auto rotation = Sophus::SO3d::rotZ(M_PI / 2.0);
  const Eigen::Vector3d translation(1.0, 2.0, 0.5);
  const auto imu_pose_in_body = Sophus::SE3d(rotation, translation);

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with no rotation
  const auto T_iv = Sophus::SE3d();
  const auto C_vb = Sophus::SO3d();

  const auto mag_reading = imu_sensor->sample_magnetometer(T_iv, C_vb);

  // Sensor rotated 90 deg CCW, so world X axis appears as sensor -Y axis
  const Eigen::Vector3d expected(0.0, -45e-6, 0.0);
  EXPECT_NEAR(mag_reading.x(), expected.x(), 1e-10);
  EXPECT_NEAR(mag_reading.y(), expected.y(), 1e-10);
  EXPECT_NEAR(mag_reading.z(), expected.z(), 1e-10);
}

// Test sample_magnetometer with vehicle heading 90 deg east
TEST_F(IMUSensorModelTest, SampleMagnetometerVehicle90DegHeading) {
  // Identity sensor pose
  const auto imu_pose_in_body = Sophus::SE3d();

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle rotated 90 deg around Z (heading east)
  const Eigen::Vector3d vehicle_position(10.0, 5.0, 0.0);
  const auto T_iv = Sophus::SE3d(Sophus::SO3d::rotZ(M_PI / 2.0), vehicle_position);
  const auto C_vb = Sophus::SO3d();

  const auto mag_reading = imu_sensor->sample_magnetometer(T_iv, C_vb);

  // Vehicle heading 90 deg east, so north (world X) appears as vehicle/sensor -Y
  const Eigen::Vector3d expected(0.0, -45e-6, 0.0);
  EXPECT_NEAR(mag_reading.x(), expected.x(), 1e-10);
  EXPECT_NEAR(mag_reading.y(), expected.y(), 1e-10);
  EXPECT_NEAR(mag_reading.z(), expected.z(), 1e-10);
}

// Fixture for testing sample_gyro
class IMUSensorModelGyroTest : public ::testing::Test {
 protected:
  void SetUp() override { gravity_ = 9.81; }

  double gravity_;
};

// Test sample_gyro with identity sensor to body transform
TEST_F(IMUSensorModelGyroTest, SampleGyroIdentityTransform) {
  // Identity transform for sensor in body
  const auto imu_pose_in_body = Sophus::SE3d();
  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Angular velocity in vehicle frame around X axis
  const Eigen::Vector3d omega_vi_v(1.0, 0.0, 0.0);
  const auto C_vb = Sophus::SO3d();

  const auto omega_measured = imu_sensor->sample_gyro(omega_vi_v, C_vb);

  const auto expected_omega_sv_v = Eigen::Vector3d(1.0, 0.0, 0.0);
  EXPECT_TRUE(omega_measured.isApprox(expected_omega_sv_v, 1e-10));
}

// Test sample_gyro with 90 degree sensor rotation
TEST_F(IMUSensorModelGyroTest, SampleGyro90DegSensorRotation) {
  // 90 degree rotation around Z axis for sensor in body
  const auto sensor_rotation = Sophus::SO3d::rotZ(-M_PI / 2.0);
  const Eigen::Vector3d translation(0.5, 0.0, 0.1);
  const auto imu_pose_in_body = Sophus::SE3d(sensor_rotation, translation);

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Angular velocity in vehicle frame around X axis
  const Eigen::Vector3d omega_vi_v(1.0, 0.0, 0.0);
  const auto C_vb = Sophus::SO3d();

  const auto omega_measured = imu_sensor->sample_gyro(omega_vi_v, C_vb);

  const Eigen::Vector3d expected(0.0, 1.0, 0.0);
  EXPECT_TRUE(omega_measured.isApprox(expected, 1e-10));
}

// Test sample_gyro with both body and sensor rotations
TEST_F(IMUSensorModelGyroTest, SampleGyroBothRotations) {
  // 90 degree rotation around Z axis for sensor in body
  const auto sensor_rotation = Sophus::SO3d::rotZ(M_PI / 2.0);
  const auto imu_pose_in_body = Sophus::SE3d(sensor_rotation, Eigen::Vector3d::Zero());

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Angular velocity in vehicle frame around X
  const Eigen::Vector3d omega_vi_v(1.0, 0.0, 0.0);
  const auto C_vb = Sophus::SO3d::rotZ(M_PI / 2.0);

  const auto omega_measured = imu_sensor->sample_gyro(omega_vi_v, C_vb);

  const Eigen::Vector3d expected(-1.0, 0.0, 0.0);
  EXPECT_TRUE(omega_measured.isApprox(expected, 1e-10));
}

// Fixture for testing sample_accelerometer
class IMUSensorModelAccelerometerTest : public ::testing::Test {
 protected:
  void SetUp() override { gravity_ = 9.81; }

  double gravity_;
};

// Test sample_accelerometer with no motion (stationary vehicle)
TEST_F(IMUSensorModelAccelerometerTest, SampleAccelerometerNoMotion) {
  // Identity transform for sensor in body (co-located, no rotation)
  const auto imu_pose_in_body = Sophus::SE3d();

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with no rotation
  const auto T_iv = Sophus::SE3d();
  const Eigen::Vector3d r_ddot_vi_i = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();
  const auto C_vb = Sophus::SO3d();

  const auto accel_measured = imu_sensor->sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);

  // Expected: only gravity measured (specific force in free fall = 0, stationary = g)
  const Eigen::Vector3d expected(0.0, 0.0, gravity_);

  EXPECT_NEAR(accel_measured.x(), expected.x(), 1e-10);
  EXPECT_NEAR(accel_measured.y(), expected.y(), 1e-10);
  EXPECT_NEAR(accel_measured.z(), expected.z(), 1e-10);
}

// Test sample_accelerometer with 45 degree rotated sensor, co-located with vehicle
TEST_F(IMUSensorModelAccelerometerTest, SampleAccelerometer45DegRotation) {
  // Sensor rotated 45 degrees to the left (counter-clockwise around Z) and co-located (zero translation)
  const auto sensor_rotation = Sophus::SO3d::rotZ(M_PI / 4.0);  // 45 degrees
  const auto imu_pose_in_body = Sophus::SE3d(sensor_rotation, Eigen::Vector3d::Zero());

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with pure linear acceleration along X and Y
  const auto T_iv = Sophus::SE3d();
  const Eigen::Vector3d r_ddot_vi_i(1.0, 1.0, 0.0);
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();
  const auto C_vb = Sophus::SO3d();

  const auto accel_measured = imu_sensor->sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);

  // Sensor X-axis is rotated 45 deg from vehicle X, aligned with [1,1,0] direction
  const double expected_x = std::sqrt(2.0);
  const double expected_y = 0.0;
  const double expected_z = gravity_;

  EXPECT_NEAR(accel_measured.x(), expected_x, 1e-10);
  EXPECT_NEAR(accel_measured.y(), expected_y, 1e-10);
  EXPECT_NEAR(accel_measured.z(), expected_z, 1e-10);
}

// Test sample_accelerometer with centripetal acceleration only
TEST_F(IMUSensorModelAccelerometerTest, SampleAccelerometerCentripetalOnly) {
  // IMU located 1m in front of the vehicle frame (along X axis)
  const Eigen::Vector3d imu_translation(1.0, 0.0, 0.0);
  const Sophus::SE3d imu_pose_in_body(Sophus::SO3d(), imu_translation);

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with pure angular velocity
  const Sophus::SE3d T_iv;
  const Eigen::Vector3d r_ddot_vi_i = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_vi_v(0.0, 0.0, 2.0);
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();
  const Sophus::SO3d C_vb;

  const Eigen::Vector3d accel_measured =
      imu_sensor->sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);

  const double expected_x = -4.0;
  const double expected_y = 0.0;
  const double expected_z = gravity_;

  EXPECT_NEAR(accel_measured.x(), expected_x, 1e-10);
  EXPECT_NEAR(accel_measured.y(), expected_y, 1e-10);
  EXPECT_NEAR(accel_measured.z(), expected_z, 1e-10);
}

// Test sample_accelerometer with Euler (tangential) acceleration only
TEST_F(IMUSensorModelAccelerometerTest, SampleAccelerometerEulerOnly) {
  // IMU rotated 90 degrees left and positioned 0.5m in front of vehicle
  const Sophus::SO3d sensor_rotation = Sophus::SO3d::rotZ(M_PI / 2.0);  // 90 degrees left
  const Eigen::Vector3d imu_translation(0.5, 0.0, 0.0);
  const Sophus::SE3d imu_pose_in_body(sensor_rotation, imu_translation);

  auto imu_sensor = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, IMUNoiseParameters{});

  // Vehicle at origin with pure angular acceleration
  const Sophus::SE3d T_iv;
  const Eigen::Vector3d r_ddot_vi_i = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_dot_vi_v(0.0, 0.0, 3.0);
  const Sophus::SO3d C_vb;

  const Eigen::Vector3d accel_measured =
      imu_sensor->sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);

  const double expected_x = 1.5;
  const double expected_y = 0.0;
  const double expected_z = gravity_;

  EXPECT_NEAR(accel_measured.x(), expected_x, 1e-10);
  EXPECT_NEAR(accel_measured.y(), expected_y, 1e-10);
  EXPECT_NEAR(accel_measured.z(), expected_z, 1e-10);
}

// Base fixture for testing IMU noise generation
class IMUSensorModelNoiseTestBase : public ::testing::Test {
 protected:
  void SetUp() override {
    gravity_ = 9.81;

    // Setup noise parameters such that discrete sigma = sqrt(2) for both components
    sampling_period_ = 0.01;
    discrete_sigma_ = std::sqrt(2.0);
    random_walk_steps_ = 10;

    // Get noise parameters from derived class
    IMUNoiseParameters noise_params = get_noise_parameters();

    // Create IMU sensor with noise
    const auto imu_pose_in_body = Sophus::SE3d();
    imu_sensor_ = std::make_unique<IMUSensorModel>(imu_pose_in_body, gravity_, noise_params);

    // Test constants
    num_samples_ = 10000;
    expected_variance_ = 4.0;
    mean_tolerance_ = 0.15;
    variance_tolerance_ = 0.3;
  }

  // Virtual function to be implemented by derived classes
  virtual IMUNoiseParameters get_noise_parameters() = 0;

  double gravity_;
  double sampling_period_;
  double discrete_sigma_;
  std::size_t random_walk_steps_;
  std::unique_ptr<IMUSensorModel> imu_sensor_;

  // Test constants
  std::size_t num_samples_;
  double expected_variance_;
  double mean_tolerance_;
  double variance_tolerance_;

 protected:
  // Helper function to sample accelerometer and compute statistics
  std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_accelerometer_statistics(IMUSensorModel& imu_sensor) {
    const auto T_iv = Sophus::SE3d();
    const Eigen::Vector3d r_ddot_vi_i = Eigen::Vector3d::Zero();
    const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
    const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();
    const auto C_vb = Sophus::SO3d();

    std::vector<Eigen::Vector3d> samples;
    samples.reserve(num_samples_);

    for (std::size_t i = 0; i < num_samples_; ++i) {
      imu_sensor.reset();
      for (std::size_t j = 0; j < random_walk_steps_ - 1; ++j) {
        [[maybe_unused]] auto discarded_measurement =
            imu_sensor.sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);
      }
      const auto measurement = imu_sensor.sample_accelerometer(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v, C_vb);
      // Remove gravity component to isolate noise
      samples.push_back(Eigen::Vector3d(measurement.x(), measurement.y(), measurement.z() - gravity_));
    }

    return compute_mean_and_variance(samples);
  }

  // Helper function to sample gyroscope and compute statistics
  std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_gyroscope_statistics(IMUSensorModel& imu_sensor) {
    const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
    const auto C_vb = Sophus::SO3d();

    std::vector<Eigen::Vector3d> samples;
    samples.reserve(num_samples_);

    for (std::size_t i = 0; i < num_samples_; ++i) {
      imu_sensor.reset();
      for (std::size_t j = 0; j < random_walk_steps_ - 1; ++j) {
        [[maybe_unused]] auto discarded_measurement = imu_sensor.sample_gyro(omega_vi_v, C_vb);
      }
      const auto measurement = imu_sensor.sample_gyro(omega_vi_v, C_vb);
      samples.push_back(measurement);
    }

    return compute_mean_and_variance(samples);
  }

  // Helper function to sample magnetometer and compute statistics
  std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_magnetometer_statistics(IMUSensorModel& imu_sensor) {
    // Setup identity scenario to isolate noise
    const auto T_iv = Sophus::SE3d();
    const auto C_vb = Sophus::SO3d();

    std::vector<Eigen::Vector3d> samples;
    samples.reserve(num_samples_);

    for (std::size_t i = 0; i < num_samples_; ++i) {
      imu_sensor.reset();
      for (std::size_t j = 0; j < random_walk_steps_ - 1; ++j) {
        [[maybe_unused]] auto discarded_measurement = imu_sensor.sample_magnetometer(T_iv, C_vb);
      }
      const auto measurement = imu_sensor.sample_magnetometer(T_iv, C_vb);
      // Remove constant magnetic field to isolate noise
      samples.push_back(Eigen::Vector3d(measurement.x() - 45e-6, measurement.y(), measurement.z()));
    }

    return compute_mean_and_variance(samples);
  }

 private:
  // Helper function to compute mean and variance from samples
  std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_mean_and_variance(const std::vector<Eigen::Vector3d>& samples) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& sample : samples) {
      mean += sample;
    }
    mean /= samples.size();

    Eigen::Vector3d variance = Eigen::Vector3d::Zero();
    for (const auto& sample : samples) {
      Eigen::Vector3d diff = sample - mean;
      variance += diff.cwiseProduct(diff);
    }
    variance /= (samples.size() - 1);  // Unbiased estimator

    return {mean, variance};
  }
};

// Fixture for testing IMU noise with long autocorrelation time
class IMUSensorModelNoiseTest : public IMUSensorModelNoiseTestBase {
 protected:
  IMUNoiseParameters get_noise_parameters() override {
    // calculate the random walk standard deviation so that after
    // random_walk_steps_ both white noise and bias terms have variance = 2.0
    // when the autocorrelation time is very long

    IMUNoiseParameters noise_params;
    noise_params.accelerometer_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.accelerometer_random_walk = discrete_sigma_ / std::sqrt(random_walk_steps_ * sampling_period_);
    noise_params.gyroscope_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.gyroscope_random_walk = discrete_sigma_ / std::sqrt(random_walk_steps_ * sampling_period_);
    noise_params.imu_sampling_period = sampling_period_;
    noise_params.initial_accelerometer_bias = Eigen::Vector3d(1.0, 2.0, 3.0);
    noise_params.initial_gyroscope_bias = Eigen::Vector3d(2.0, 3.0, 1.0);
    noise_params.initial_magnetometer_bias = Eigen::Vector3d(3.0, 1.0, 2.0);
    // Use very long autocorrelation times (effectively no decay during test)
    noise_params.accelerometer_bias_autocorrelation_time = 1e10;
    noise_params.gyroscope_bias_autocorrelation_time = 1e10;
    noise_params.magnetometer_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.magnetometer_random_walk = discrete_sigma_ / std::sqrt(random_walk_steps_ * sampling_period_);
    noise_params.magnetometer_bias_autocorrelation_time = 1e10;
    return noise_params;
  }
};

// Test accelerometer noise statistics
TEST_F(IMUSensorModelNoiseTest, AccelerometerNoiseStatistics) {
  auto [mean, variance] = compute_accelerometer_statistics(*imu_sensor_);

  // Expected: mean ≈ initial_bias, variance ≈ 4 (with some tolerance due to finite sampling)
  const Eigen::Vector3d expected_mean(1.0, 2.0, 3.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

// Test gyroscope noise statistics
TEST_F(IMUSensorModelNoiseTest, GyroscopeNoiseStatistics) {
  auto [mean, variance] = compute_gyroscope_statistics(*imu_sensor_);

  // Expected: mean ≈ initial_bias, variance ≈ 4 (with some tolerance due to finite sampling)
  const Eigen::Vector3d expected_mean(2.0, 3.0, 1.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

// Test magnetometer noise statistics
TEST_F(IMUSensorModelNoiseTest, MagnetometerNoiseStatistics) {
  auto [mean, variance] = compute_magnetometer_statistics(*imu_sensor_);

  // Expected: mean ≈ initial_bias, variance ≈ 4 (with some tolerance due to finite sampling)
  const Eigen::Vector3d expected_mean(3.0, 1.0, 2.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

// Fixture for testing IMU noise with finite autocorrelation time
class IMUSensorModelNoiseAutocorrelationTest : public IMUSensorModelNoiseTestBase {
 protected:
  IMUNoiseParameters get_noise_parameters() override {
    // calculate the random walk standard deviation so that after
    // random_walk_steps_ both white noise and bias terms have variance = 2.0
    // when the autocorrelation time is much shorter than the
    // random_walk_steps_ * sampling_period_

    const double autocorrelation_time = sampling_period_;
    const auto averaging_factor = std::sqrt(1.0 - std::exp(-2.0 * sampling_period_ / autocorrelation_time));

    IMUNoiseParameters noise_params;
    noise_params.accelerometer_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.accelerometer_random_walk = discrete_sigma_ * averaging_factor / std::sqrt(sampling_period_);
    noise_params.gyroscope_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.gyroscope_random_walk = discrete_sigma_ * averaging_factor / std::sqrt(sampling_period_);
    noise_params.imu_sampling_period = sampling_period_;
    noise_params.initial_accelerometer_bias = Eigen::Vector3d(1.0, 2.0, 3.0);
    noise_params.initial_gyroscope_bias = Eigen::Vector3d(2.0, 3.0, 1.0);
    noise_params.initial_magnetometer_bias = Eigen::Vector3d(3.0, 1.0, 2.0);
    noise_params.accelerometer_bias_autocorrelation_time = autocorrelation_time;
    noise_params.gyroscope_bias_autocorrelation_time = autocorrelation_time;
    noise_params.magnetometer_noise_density = discrete_sigma_ * std::sqrt(sampling_period_);
    noise_params.magnetometer_random_walk = discrete_sigma_ * averaging_factor / std::sqrt(sampling_period_);
    noise_params.magnetometer_bias_autocorrelation_time = autocorrelation_time;
    return noise_params;
  }
};

// Test accelerometer noise statistics with finite autocorrelation time
TEST_F(IMUSensorModelNoiseAutocorrelationTest, AccelerometerNoiseStatistics) {
  auto [mean, variance] = compute_accelerometer_statistics(*imu_sensor_);

  const Eigen::Vector3d expected_mean(0.0, 0.0, 0.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

// Test gyroscope noise statistics with finite autocorrelation time
TEST_F(IMUSensorModelNoiseAutocorrelationTest, GyroscopeNoiseStatistics) {
  auto [mean, variance] = compute_gyroscope_statistics(*imu_sensor_);

  const Eigen::Vector3d expected_mean(0.0, 0.0, 0.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

// Test magnetometer noise statistics with finite autocorrelation time
TEST_F(IMUSensorModelNoiseAutocorrelationTest, MagnetometerNoiseStatistics) {
  auto [mean, variance] = compute_magnetometer_statistics(*imu_sensor_);

  const Eigen::Vector3d expected_mean(0.0, 0.0, 0.0);

  EXPECT_NEAR(mean.x(), expected_mean.x(), mean_tolerance_);
  EXPECT_NEAR(mean.y(), expected_mean.y(), mean_tolerance_);
  EXPECT_NEAR(mean.z(), expected_mean.z(), mean_tolerance_);

  EXPECT_NEAR(variance.x(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.y(), expected_variance_, variance_tolerance_);
  EXPECT_NEAR(variance.z(), expected_variance_, variance_tolerance_);
}

}  // namespace svdsim_server
