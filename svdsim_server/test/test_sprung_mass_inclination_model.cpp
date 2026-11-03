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

// Project headers
#include "svdsim_server/models/sprung_mass_inclination_model.hpp"

namespace svdsim_server {

class SprungMassInclinationModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Vehicle parameters from monacof1_system_configuration/config/vehicle_params.yaml
    params_.wheelbase = 0.3300;                // m
    params_.front_track_width = 0.2360;        // m (front_axle_width)
    params_.rear_track_width = 0.2360;         // m (rear_axle_width)
    params_.sprung_mass = 3.47;                // kg (chassis_mass, excludes tire mass)
    params_.max_inclination_angle = 0.174533;  // radians

    // Center of mass position from vehicle_params.yaml
    // Note: This is relative to chassis origin (rear axle)
    params_.center_of_mass_forward_distance = 0.15532;  // m (forward from rear axle)
    params_.roll_center_height = 0.0325;                // m (at wheel center, default value)
    // CoG height is specified relative to roll center in vehicle_params, so add them
    params_.center_of_mass_height = params_.roll_center_height + 0.01434;  // m (above ground)

    // Calculate spring stiffness to produce target pitch at target deceleration
    // From equilibrium: $\theta = (h_{eff} \cdot m \cdot a_x) / (k_s \cdot \sum((r_i \cdot u_F)^2))$
    // where h_eff = h_cg - h_rc (effective lever arm from roll center to CoG)
    // Solving for $k_s$: $k_s = (h_{eff} \cdot m \cdot a_x) / (\theta \cdot \sum((r_i \cdot u_F)^2))$

    const double h_cg = params_.center_of_mass_height;
    const double h_rc = params_.roll_center_height;
    const double h_eff = h_cg - h_rc;
    const double m = params_.sprung_mass;
    const double a_x = 5.0 * 9.81;                   // m/s^2
    const double target_theta = 7.5 * M_PI / 180.0;  // radians

    // For longitudinal force ($u_F = [1, 0]$), $\sum((r_i \cdot u_F)^2) = \sum(x_i^2)$
    // where $x_i$ is the longitudinal distance of each corner from CoG
    const double cg_x = params_.center_of_mass_forward_distance;
    const double x_front = params_.wheelbase - cg_x;  // Front corners
    const double x_rear = -cg_x;                      // Rear corners

    // Sum of squared longitudinal distances (2 front + 2 rear corners)
    const double sum_x_squared = 2.0 * x_front * x_front + 2.0 * x_rear * x_rear;

    // Calculate spring stiffness using effective lever arm
    params_.spring_stiffness = (h_eff * m * a_x) / (target_theta * sum_x_squared);

    model_ = std::make_unique<SprungMassInclinationModel>(params_);
  }

  SprungMassInclinationParameters params_;
  std::unique_ptr<SprungMassInclinationModel> model_;
};

// Test: No acceleration results in no inclination
TEST_F(SprungMassInclinationModelTest, NoAccelerationNoInclination) {
  // Vehicle at rest with no acceleration
  const Sophus::SE3d T_iv = Sophus::SE3d(
      Sophus::SO3d(),                 // No rotation
      Eigen::Vector3d(0.0, 0.0, 0.0)  // At origin
  );
  const Eigen::Vector3d r_ddot_vi_i = Eigen::Vector3d::Zero();     // No linear acceleration
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();      // No angular velocity
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();  // No angular acceleration

  const Sophus::SO3d body_orientation = model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);

  // Expected: Identity rotation (no inclination)
  const Sophus::SO3d expected_orientation = Sophus::SO3d();

  // Compare rotation matrices
  EXPECT_TRUE(body_orientation.matrix().isApprox(expected_orientation.matrix(), 1e-6));
}

// Test: 5g deceleration along X produces 7.5 degrees pitch
TEST_F(SprungMassInclinationModelTest, FiveGDecelerationProducesSevenPointFiveDegreesPitch) {
  // Vehicle at rest with 5g deceleration along negative X (braking)
  const Sophus::SE3d T_iv = Sophus::SE3d(
      Sophus::SO3d(),                 // No rotation
      Eigen::Vector3d(0.0, 0.0, 0.0)  // At origin
  );
  const double deceleration = -5.0 * 9.81;                         // m/s^2
  const Eigen::Vector3d r_ddot_vi_i(deceleration, 0.0, 0.0);       // Longitudinal deceleration
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();      // No angular velocity
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();  // No angular acceleration

  const Sophus::SO3d body_orientation = model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);

  // Expected: Positive pitch angle (nose down from deceleration)
  const double expected_pitch = 0.130900;
  const Sophus::SO3d expected_orientation = Sophus::SO3d::rotY(expected_pitch);

  // Compare rotation matrices with reasonable tolerance
  EXPECT_TRUE(body_orientation.matrix().isApprox(expected_orientation.matrix(), 1e-3));

  // Also verify the pitch angle directly
  const Eigen::Vector3d log_rot = body_orientation.log();
  const double actual_pitch = log_rot.y();
  EXPECT_GT(actual_pitch, 0.0);  // Deceleration produces positive pitch
  EXPECT_NEAR(actual_pitch, expected_pitch, 1e-3);
}

// Test: Lateral acceleration along Y produces correct roll
TEST_F(SprungMassInclinationModelTest, LateralAccelerationProducesCorrectRoll) {
  // Vehicle with lateral acceleration (e.g., cornering)
  const Sophus::SE3d T_iv = Sophus::SE3d(
      Sophus::SO3d(),                 // No rotation
      Eigen::Vector3d(0.0, 0.0, 0.0)  // At origin
  );
  const double lateral_accel = 1.0 * 9.81;                         // m/s^2 (rightward, +Y)
  const Eigen::Vector3d r_ddot_vi_i(0.0, lateral_accel, 0.0);      // Lateral acceleration
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();      // No angular velocity
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();  // No angular acceleration

  const Sophus::SO3d body_orientation = model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);

  // Calculate expected roll angle from equilibrium equation
  // $\theta_{roll} = (h_{eff} \cdot m \cdot a_y) / (k_s \cdot \sum(y_i^2))$
  // where h_eff = h_cg - h_rc (effective lever arm from roll center to CoG)
  const double h_cg = params_.center_of_mass_height;
  const double h_rc = params_.roll_center_height;
  const double h_eff = h_cg - h_rc;
  const double m = params_.sprung_mass;
  const double cg_y = 0.0;  // CoG on centerline

  // For lateral force ($u_F = [0, 1]$), $\sum((r_i \cdot u_F)^2) = \sum(y_i^2)$
  // where $y_i$ is the lateral distance of each corner from CoG
  const double y_left = params_.front_track_width / 2.0 - cg_y;    // Left corners
  const double y_right = -params_.front_track_width / 2.0 - cg_y;  // Right corners

  // Sum of squared lateral distances (2 front left + 2 front right + 2 rear left + 2 rear right)
  // Since CoG is centered (y=0), and front/rear track widths are equal
  const double sum_y_squared = 2.0 * y_left * y_left + 2.0 * y_right * y_right;

  // Expected roll angle using effective lever arm
  const double expected_roll = (h_eff * m * lateral_accel) / (params_.spring_stiffness * sum_y_squared);

  // Also verify the roll angle directly
  const Eigen::Vector3d log_rot = body_orientation.log();
  const double actual_roll = log_rot.x();
  EXPECT_GT(actual_roll, 0.0);  // leftward acceleration produces positive roll
  EXPECT_NEAR(actual_roll, expected_roll, 1e-3);
}

// Test: Large accelerations in all directions are bounded by max inclination
TEST_F(SprungMassInclinationModelTest, LargeAccelerationsAreBounded) {
  const Sophus::SE3d T_iv = Sophus::SE3d(
      Sophus::SO3d(),                 // No rotation
      Eigen::Vector3d(0.0, 0.0, 0.0)  // At origin
  );
  const Eigen::Vector3d omega_vi_v = Eigen::Vector3d::Zero();
  const Eigen::Vector3d omega_dot_vi_v = Eigen::Vector3d::Zero();

  // Test large acceleration in +X direction (forward)
  {
    const double large_accel = 20.0 * 9.81;  // m/s^2
    const Eigen::Vector3d r_ddot_vi_i(large_accel, 0.0, 0.0);
    const Sophus::SO3d body_orientation =
        model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);
    const Eigen::Vector3d log_rot = body_orientation.log();
    // Should rotate around Y axis (pitch), negative angle (nose up from forward acceleration)
    EXPECT_NEAR(std::abs(log_rot.y()), params_.max_inclination_angle, 1e-6);
    EXPECT_LT(log_rot.y(), 0.0);
    EXPECT_NEAR(log_rot.x(), 0.0, 1e-6);
    EXPECT_NEAR(log_rot.z(), 0.0, 1e-6);
  }

  // Test large acceleration in -X direction (braking)
  {
    const double large_accel = -20.0 * 9.81;  // m/s^2
    const Eigen::Vector3d r_ddot_vi_i(large_accel, 0.0, 0.0);
    const Sophus::SO3d body_orientation =
        model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);
    const Eigen::Vector3d log_rot = body_orientation.log();
    // Should rotate around Y axis (pitch), positive angle (nose down from braking)
    EXPECT_NEAR(std::abs(log_rot.y()), params_.max_inclination_angle, 1e-6);
    EXPECT_GT(log_rot.y(), 0.0);
    EXPECT_NEAR(log_rot.x(), 0.0, 1e-6);
    EXPECT_NEAR(log_rot.z(), 0.0, 1e-6);
  }

  // Test large acceleration in +Y direction (right turn)
  {
    const double large_accel = 20.0 * 9.81;  // m/s^2
    const Eigen::Vector3d r_ddot_vi_i(0.0, large_accel, 0.0);
    const Sophus::SO3d body_orientation =
        model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);
    const Eigen::Vector3d log_rot = body_orientation.log();
    // Should rotate around X axis (roll), negative angle (lean left from right turn)
    EXPECT_NEAR(std::abs(log_rot.x()), params_.max_inclination_angle, 1e-6);
    EXPECT_GT(log_rot.x(), 0.0);
    EXPECT_NEAR(log_rot.y(), 0.0, 1e-6);
    EXPECT_NEAR(log_rot.z(), 0.0, 1e-6);
  }

  // Test large acceleration in -Y direction (left turn)
  {
    const double large_accel = -20.0 * 9.81;  // m/s^2
    const Eigen::Vector3d r_ddot_vi_i(0.0, large_accel, 0.0);
    const Sophus::SO3d body_orientation =
        model_->compute_body_orientation(T_iv, r_ddot_vi_i, omega_vi_v, omega_dot_vi_v);
    const Eigen::Vector3d log_rot = body_orientation.log();
    // Should rotate around X axis (roll), positive angle (lean right from left turn)
    EXPECT_NEAR(std::abs(log_rot.x()), params_.max_inclination_angle, 1e-6);
    EXPECT_LT(log_rot.x(), 0.0);
    EXPECT_NEAR(log_rot.y(), 0.0, 1e-6);
    EXPECT_NEAR(log_rot.z(), 0.0, 1e-6);
  }
}

}  // namespace svdsim_server
