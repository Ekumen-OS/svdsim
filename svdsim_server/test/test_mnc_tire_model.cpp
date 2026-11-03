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
#include <optional>
#include <tuple>

// Project headers
#include "svdsim_server/models/mnc_tire_model.hpp"

namespace svdsim_server {

class TireModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    lateral_params_ = PacejkaParams{
        0.5,   // Dn - Normalized peak factor for lateral
        0.04,  // B - Stiffness factor
        2.5,   // C - Shape factor
        0.97   // E - Curvature factor
    };

    longitudinal_params_ = PacejkaParams{
        0.5,   // Dn - Normalized peak factor for longitudinal
        16.0,  // B - Stiffness factor
        1.65,  // C - Shape factor
        0.50   // E - Curvature factor
    };

    const double effective_wheel_radius = 0.05;

    // Create tire model without combined slip
    tire_model_pure_ = std::make_unique<MNCTireModel>(TireModelParams{
        longitudinal_params_,    // longitudinal
        lateral_params_,         // lateral
        effective_wheel_radius,  // effective_wheel_radius
        false                    // combined_slip - Pure slip mode
    });

    // Create tire model with combined slip
    tire_model_combined_ = std::make_unique<MNCTireModel>(TireModelParams{
        longitudinal_params_,    // longitudinal
        lateral_params_,         // lateral
        effective_wheel_radius,  // effective_wheel_radius
        true                     // combined_slip - Combined slip mode
    });

    vertical_load_ = 2.0;  // N
    wheel_radius_ = effective_wheel_radius;
  }

  std::tuple<double, double> calculate_velocities_from_slip(double vx, double slip_ratio, double slip_angle_rad) const {
    // Calculate lateral velocity from slip angle
    // alpha = atan(|vy| / |vx|) => vy = vx * tan(alpha)
    const double vy = -vx * std::tan(slip_angle_rad);

    const double v_wheel = vx / (1.0 - slip_ratio);
    const double w = v_wheel / wheel_radius_;

    return {vy, w};
  }

  double evaluate_pacejka(const PacejkaParams& params, double x, double Fz) const {
    const double D = params.Dn * Fz;
    const double B = params.B;
    const double C = params.C;
    const double E = params.E;

    const double Bx = B * x;
    const double atan_Bx = std::atan(Bx);
    const double phi = C * std::atan(Bx - E * (Bx - atan_Bx));

    return D * std::sin(phi);
  }

  PacejkaParams lateral_params_;
  PacejkaParams longitudinal_params_;
  std::unique_ptr<MNCTireModel> tire_model_pure_;
  std::unique_ptr<MNCTireModel> tire_model_combined_;
  double vertical_load_;
  double wheel_radius_;
};

// Test lateral forces from 0 to 90 degrees (every 5 degrees) against locally calculated Pacejka values
TEST_F(TireModelTest, LateralForceFromZeroToNinetyDegrees) {
  const double vx = 5.0;  // m/s forward velocity

  // Test from 0 to 90 degrees in 5 degree increments
  for (double slip_angle_deg = 0.0; slip_angle_deg <= 90.0; slip_angle_deg += 1.0) {
    const double slip_angle_rad = slip_angle_deg * M_PI / 180.0;
    const auto [vy, w] = calculate_velocities_from_slip(vx, 0.0, slip_angle_rad);

    auto forces_pure = tire_model_pure_->compute_forces(vx, vy, w, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, vy, w, vertical_load_);

    // Calculate expected force locally
    const double expected_force = evaluate_pacejka(lateral_params_, slip_angle_deg, vertical_load_);

    ASSERT_NEAR(forces_pure.F_y, expected_force, 1e-6)
        << "Pure model lateral force at " << slip_angle_deg << " degrees should match Pacejka calculation";
    ASSERT_NEAR(forces_combined.F_y, expected_force, 1e-6)
        << "Combined model lateral force at " << slip_angle_deg << " degrees should match Pacejka calculation";
  }
}

// Test longitudinal forces from 0 to 1 (every 0.1) against locally calculated Pacejka values
TEST_F(TireModelTest, LongitudinalForceFromZeroToOne) {
  const double vx = 5.0;  // m/s forward velocity

  // Test from 0 to 1 in 0.1 increments
  for (double slip_ratio = 0.0; slip_ratio <= 1.0; slip_ratio += 0.1) {
    const auto [vy, w] = calculate_velocities_from_slip(vx, slip_ratio, 0.0);

    auto forces_pure = tire_model_pure_->compute_forces(vx, vy, w, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, vy, w, vertical_load_);

    // Calculate expected force locally
    const double expected_force = evaluate_pacejka(longitudinal_params_, slip_ratio, vertical_load_);

    ASSERT_NEAR(forces_pure.F_x, expected_force, 1e-6);
    ASSERT_NEAR(forces_combined.F_x, expected_force, 1e-6);
  }
}

// Test that lateral force decreases with increasing longitudinal slip (combined slip effect)
TEST_F(TireModelTest, LateralForceDecreasesWithLongitudinalSlip) {
  const double vx = 5.0;  // m/s forward velocity

  // in the outer loop skip the 0 slip to avoid having the calculation be
  // done in the pure slip regime, which would make the combined slip forces
  // equal to the pure slip forces and thus not test the reduction effect.
  for (double slip_angle_deg = 0.01; slip_angle_deg <= 45.0; slip_angle_deg += 1.0) {
    const double slip_angle_rad = slip_angle_deg * M_PI / 180.0;

    double prev_combined_fy = std::numeric_limits<double>::max();
    std::optional<double> first_pure_fy;

    for (double slip_ratio = 0.0; slip_ratio <= 0.1; slip_ratio += 0.01) {
      const auto [vy, w] = calculate_velocities_from_slip(vx, slip_ratio, slip_angle_rad);

      auto forces_pure = tire_model_pure_->compute_forces(vx, vy, w, vertical_load_);
      auto forces_combined = tire_model_combined_->compute_forces(vx, vy, w, vertical_load_);

      if (!first_pure_fy.has_value()) {
        first_pure_fy = forces_pure.F_y;
      }

      ASSERT_NEAR(forces_pure.F_y, first_pure_fy.value(), 1e-6);

      ASSERT_LE(std::abs(forces_combined.F_y), std::abs(forces_pure.F_y));

      if (slip_ratio > 0.0) {
        ASSERT_LT(std::abs(forces_combined.F_y), std::abs(prev_combined_fy));
      } else {
        ASSERT_EQ(forces_pure.F_x, forces_combined.F_x);
        ASSERT_EQ(forces_pure.F_y, forces_combined.F_y);
      }

      prev_combined_fy = forces_combined.F_y;
    }
  }
}

// Test that longitudinal force decreases with increasing lateral slip (combined slip effect)
TEST_F(TireModelTest, LongitudinalForceDecreasesWithLateralSlip) {
  const double vx = 5.0;  // m/s forward velocity

  // in the outer loop skip the 0 slip to avoid having the calculation be
  // done in the pure slip regime, which would make the combined slip forces
  // equal to the pure slip forces and thus not test the reduction effect.
  for (double slip_ratio = 0.01; slip_ratio <= 0.1; slip_ratio += 0.01) {
    double prev_combined_fx = std::numeric_limits<double>::max();
    std::optional<double> first_pure_fx;

    for (double slip_angle_deg = 0.0; slip_angle_deg <= 45.0; slip_angle_deg += 1.0) {
      const double slip_angle_rad = slip_angle_deg * M_PI / 180.0;
      const auto [vy, w] = calculate_velocities_from_slip(vx, slip_ratio, slip_angle_rad);

      auto forces_pure = tire_model_pure_->compute_forces(vx, vy, w, vertical_load_);
      auto forces_combined = tire_model_combined_->compute_forces(vx, vy, w, vertical_load_);

      if (!first_pure_fx.has_value()) {
        first_pure_fx = forces_pure.F_x;
      }

      ASSERT_NEAR(forces_pure.F_x, first_pure_fx.value(), 1e-6);

      ASSERT_LE(std::abs(forces_combined.F_x), std::abs(forces_pure.F_x));

      if (slip_angle_deg > 0.0) {
        ASSERT_LT(std::abs(forces_combined.F_x), std::abs(prev_combined_fx));
      } else {
        ASSERT_EQ(forces_pure.F_x, forces_combined.F_x);
        ASSERT_EQ(forces_pure.F_y, forces_combined.F_y);
      }

      prev_combined_fx = forces_combined.F_x;
    }
  }
}

// Test that force signs match the wheel frame of reference
TEST_F(TireModelTest, ForceSigns) {
  const double vx = 5.0;                             // m/s forward velocity
  const double slip_angle_rad = 5.0 * M_PI / 180.0;  // 5 degree slip angle

  // Test 1: Wheel faster than ground (acceleration/traction) should give positive Fx
  {
    const double w_fast = (vx * 1.2) / wheel_radius_;  // Wheel 20% faster than ground speed

    auto forces_pure = tire_model_pure_->compute_forces(vx, 0.0, w_fast, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, 0.0, w_fast, vertical_load_);

    ASSERT_GT(forces_pure.F_x, 0.0) << "Pure model: Fx should be positive when wheel is faster than ground (traction)";
    ASSERT_GT(forces_combined.F_x, 0.0)
        << "Combined model: Fx should be positive when wheel is faster than ground (traction)";
  }

  // Test 2: Wheel slower than ground (braking) should give negative Fx
  {
    const double w_slow = (vx * 0.8) / wheel_radius_;  // Wheel 20% slower than ground speed

    auto forces_pure = tire_model_pure_->compute_forces(vx, 0.0, w_slow, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, 0.0, w_slow, vertical_load_);

    ASSERT_LT(forces_pure.F_x, 0.0) << "Pure model: Fx should be negative when wheel is slower than ground (braking)";
    ASSERT_LT(forces_combined.F_x, 0.0)
        << "Combined model: Fx should be negative when wheel is slower than ground (braking)";
  }

  // Test 3: Positive lateral velocity should give negative Fy (opposing the motion)
  {
    const double w = vx / wheel_radius_;                       // No longitudinal slip
    const double vy_positive = vx * std::tan(slip_angle_rad);  // Positive lateral velocity

    auto forces_pure = tire_model_pure_->compute_forces(vx, vy_positive, w, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, vy_positive, w, vertical_load_);

    ASSERT_LT(forces_pure.F_y, 0.0)
        << "Pure model: Fy should be negative when lateral velocity is positive (opposing motion)";
    ASSERT_LT(forces_combined.F_y, 0.0)
        << "Combined model: Fy should be negative when lateral velocity is positive (opposing motion)";
  }

  // Test 4: Negative lateral velocity should give positive Fy (opposing the motion)
  {
    const double w = vx / wheel_radius_;                        // No longitudinal slip
    const double vy_negative = -vx * std::tan(slip_angle_rad);  // Negative lateral velocity

    auto forces_pure = tire_model_pure_->compute_forces(vx, vy_negative, w, vertical_load_);
    auto forces_combined = tire_model_combined_->compute_forces(vx, vy_negative, w, vertical_load_);

    ASSERT_GT(forces_pure.F_y, 0.0)
        << "Pure model: Fy should be positive when lateral velocity is negative (opposing motion)";
    ASSERT_GT(forces_combined.F_y, 0.0)
        << "Combined model: Fy should be positive when lateral velocity is negative (opposing motion)";
  }

  // Test 5: Symmetry - equal magnitude forces for opposite velocities
  {
    const double w = vx / wheel_radius_;
    const double vy_pos = vx * std::tan(slip_angle_rad);
    const double vy_neg = -vy_pos;

    auto forces_pos = tire_model_pure_->compute_forces(vx, vy_pos, w, vertical_load_);
    auto forces_neg = tire_model_pure_->compute_forces(vx, vy_neg, w, vertical_load_);

    ASSERT_NEAR(std::abs(forces_pos.F_y), std::abs(forces_neg.F_y), 1e-10)
        << "Pure model: Fy magnitude should be equal for opposite lateral velocities";

    ASSERT_NEAR(forces_pos.F_y, -forces_neg.F_y, 1e-10)
        << "Pure model: Fy should have opposite signs for opposite lateral velocities";
  }
}

}  // namespace svdsim_server
