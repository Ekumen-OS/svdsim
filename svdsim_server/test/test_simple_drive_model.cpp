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

// Standard library
#include <cmath>

// Non-ROS dependencies
#include <gtest/gtest.h>

// Project headers
#include "svdsim_server/models/simple_drive_model.hpp"

namespace svdsim_server {

class SimpleDriveModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set a long time to stationarity to ensure the velocity
    // reference filter is fully settled in tests that require it.
    // Notice that given the limitations to the acceleration, this may be
    // longer than just the pure time constant of the filter.
    seconds_to_steady_state_ = 5.0;  // Time in seconds to reach steady state for the velocity reference filter

    // Typical drive model parameters
    max_steering_velocity_ = 1.0;  // rad/s
    max_steering_angle_ = 0.5;     // rad
    max_velocity_ = 5.0;           // m/s
    min_velocity_ = -2.0;          // m/s
    max_acceleration_ = 2.0;       // m/s^2
    braking_deceleration_ = 3.0;   // m/s^2
    dt_ = 0.01;                    // s (100 Hz)
    velocity_deadband_ = 0.2;      // m/s

    drive_model_ = BuildModel(BuildDefaultParams());
  }

  SimpleDriveModelParameters BuildDefaultParams() const {
    SimpleDriveModelParameters params;
    params.steering_velocity_max = max_steering_velocity_;
    params.steering_angle_max = max_steering_angle_;
    params.steering_hysteresis_gap = 0.0;
    params.velocity_max = max_velocity_;
    params.velocity_min = min_velocity_;
    params.drive_motor_acceleration = max_acceleration_;
    params.drive_braking_deceleration = braking_deceleration_;
    params.motor_drive_rise_time = 0.100;  // 100 ms
    params.velocity_deadband = velocity_deadband_;
    params.dt = dt_;
    return params;
  }

  std::unique_ptr<SimpleDriveModel> BuildModel(const SimpleDriveModelParameters& params) const {
    return std::make_unique<SimpleDriveModel>(params);
  }

  double seconds_to_steady_state_;

  double max_steering_velocity_;
  double max_steering_angle_;
  double max_velocity_;
  double min_velocity_;
  double max_acceleration_;
  double braking_deceleration_;
  double velocity_deadband_;
  double dt_;
  std::unique_ptr<SimpleDriveModel> drive_model_;

  // Helper function to bring the velocity reference filter to steady state
  void settle_velocity_filter_at(double velocity) {
    const int n_settling_steps = static_cast<int>(seconds_to_steady_state_ / dt_);
    for (int i = 0; i < n_settling_steps; ++i) {
      (void)drive_model_->update_acceleration(velocity, velocity);
    }
  }

  // Integrate steering velocity toward a constant reference for enough steps to traverse the
  // full steering range at max rate (an upper bound for settling from any position)
  double SimulateSteeringToConvergence(SimpleDriveModel& model, double steering_ref, double steering_feedback) const {
    const int n_steps = static_cast<int>(std::ceil((2.0 * max_steering_angle_) / (max_steering_velocity_ * dt_))) + 1;
    for (int i = 0; i < n_steps; ++i) {
      const auto steering_velocity = model.update_steering_velocity(steering_ref, steering_feedback);
      steering_feedback += steering_velocity * dt_;
    }
    return steering_feedback;
  }
};

// Steering velocity tests

TEST_F(SimpleDriveModelTest, SteeringVelocityPositiveError) {
  // Steering error is positive, should steer at max velocity
  const double steering_ref = 0.3;
  const double steering_feedback = 0.0;

  const auto steering_velocity = drive_model_->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, SteeringVelocityNegativeError) {
  // Steering error is negative, should steer at negative max velocity
  const double steering_ref = -0.2;
  const double steering_feedback = 0.1;

  const auto steering_velocity = drive_model_->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, -max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, SteeringVelocityLimitedByMaxAngle) {
  // Near max angle, steering velocity should be limited to avoid exceeding limit
  const double steering_ref = 0.5;
  const double steering_feedback = 0.499;  // Very close to max

  const auto steering_velocity = drive_model_->update_steering_velocity(steering_ref, steering_feedback);

  // Should be limited to (max_angle - feedback) / dt
  const auto expected_velocity = (max_steering_angle_ - steering_feedback) / dt_;
  EXPECT_DOUBLE_EQ(steering_velocity, expected_velocity);
  EXPECT_LT(steering_velocity, max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, SteeringVelocityLimitedByMinAngle) {
  // Near min angle, steering velocity should be limited to avoid exceeding limit
  const double steering_ref = -0.5;
  const double steering_feedback = -0.499;  // Very close to min

  const auto steering_velocity = drive_model_->update_steering_velocity(steering_ref, steering_feedback);

  // Should be limited to (min_angle - feedback) / dt
  const auto expected_velocity = (-max_steering_angle_ - steering_feedback) / dt_;
  EXPECT_DOUBLE_EQ(steering_velocity, expected_velocity);
  EXPECT_GT(steering_velocity, -max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, SteeringReferenceClampedToRange) {
  // Reference beyond max should be clamped
  const double steering_ref = 1.0;  // Beyond max_steering_angle_ (0.5)
  const double steering_feedback = 0.0;

  const auto steering_velocity = drive_model_->update_steering_velocity(steering_ref, steering_feedback);

  // Should still work, reference is clamped internally
  EXPECT_DOUBLE_EQ(steering_velocity, max_steering_velocity_);
}

// Acceleration tests

TEST_F(SimpleDriveModelTest, ForwardAcceleration) {
  // Forward motion with positive velocity error: accelerate
  const double velocity_feedback = 1.0;
  const double velocity_ref = 3.0;

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  EXPECT_DOUBLE_EQ(acceleration, max_acceleration_);
}

TEST_F(SimpleDriveModelTest, ForwardBraking) {
  // Forward motion with negative velocity error: brake
  const double velocity_feedback = 2.0;
  const double velocity_ref = 1.0;

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  EXPECT_DOUBLE_EQ(acceleration, -braking_deceleration_);
}

TEST_F(SimpleDriveModelTest, ReverseAcceleration) {
  // Reverse motion with negative velocity error: accelerate backwards
  const double velocity_feedback = -1.0;
  const double velocity_ref = -2.0;

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  EXPECT_DOUBLE_EQ(acceleration, -max_acceleration_);
}

TEST_F(SimpleDriveModelTest, ReverseBraking) {
  // Reverse motion with positive velocity error: brake
  const double velocity_feedback = -2.0;
  const double velocity_ref = -1.0;

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  EXPECT_DOUBLE_EQ(acceleration, braking_deceleration_);
}

TEST_F(SimpleDriveModelTest, VelocityReferenceClampedToRange) {
  // Reference beyond max should be clamped, and the closed-loop should settle at max velocity
  const double velocity_ref = 10.0;  // Beyond max_velocity_ (5.0)
  double velocity_feedback = 0.0;

  // Run closed-loop simulation for enough time to reach max velocity
  // Time to reach 5.0 m/s at 2.0 m/s^2 is 2.5s, plus filter settling time
  const int n_steps = static_cast<int>(seconds_to_steady_state_ / dt_);
  for (int i = 0; i < n_steps; ++i) {
    const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);
    velocity_feedback += acceleration * dt_;
  }

  // Should settle at max velocity without overshooting
  EXPECT_NEAR(velocity_feedback, max_velocity_, 0.01);
  EXPECT_LE(velocity_feedback, max_velocity_);
}

TEST_F(SimpleDriveModelTest, ZeroVelocityErrorForwardMotion) {
  // At target velocity, acceleration should be near zero
  const double velocity_feedback = 2.0;
  const double velocity_ref = 2.0;

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  // Filter numerical precision may cause very small error, but should be close to zero
  EXPECT_NEAR(acceleration, 0.0, 0.01);
}

// Velocity dead band tests

TEST_F(SimpleDriveModelTest, DeadbandZeroReferenceKeepsVelocityZero) {
  // A reference below the dead band should not command motion when the vehicle is at rest
  const double velocity_feedback = 0.0;
  const double velocity_ref = 0.1;  // Below velocity_deadband_ (0.2)

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  // The effective reference is zero, so no acceleration is commanded
  EXPECT_NEAR(acceleration, 0.0, 0.01);
}

TEST_F(SimpleDriveModelTest, DeadbandBrakesMovingVehicleToZero) {
  // A moving vehicle with a reference below the dead band should brake to rest
  const double velocity_ref = 0.05;  // Well below velocity_deadband_ (0.2)
  double velocity_feedback = 2.0;

  // Run closed-loop simulation until steady state
  const int n_steps = static_cast<int>(seconds_to_steady_state_ / dt_);
  for (int i = 0; i < n_steps; ++i) {
    const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);
    velocity_feedback += acceleration * dt_;
  }

  // Should settle at zero velocity (dead band reference treated as zero)
  EXPECT_NEAR(velocity_feedback, 0.0, 0.01);
}

TEST_F(SimpleDriveModelTest, DeadbandAboveThresholdUnaffected) {
  // References above the dead band are unaffected: full acceleration is commanded
  const double velocity_feedback = 1.0;
  const double velocity_ref = 3.0;  // Well above velocity_deadband_ (0.2)

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  EXPECT_DOUBLE_EQ(acceleration, max_acceleration_);
}

TEST_F(SimpleDriveModelTest, DeadbandSmallReverseReferenceTreatedAsZero) {
  // A small reverse reference below the dead band is treated as zero: no reverse creep
  const double velocity_feedback = 0.0;
  const double velocity_ref = -0.1;  // Magnitude below velocity_deadband_ (0.2)

  settle_velocity_filter_at(velocity_feedback);

  const auto acceleration = drive_model_->update_acceleration(velocity_ref, velocity_feedback);

  // The effective reference is zero, so no acceleration is commanded
  EXPECT_NEAR(acceleration, 0.0, 0.01);
}

// Steering hysteresis tests

TEST_F(SimpleDriveModelTest, HysteresisBelowHalfGapHoldsSteering) {
  // A reference within half the hysteresis gap of the current steering holds position
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  const double steering_feedback = 0.0;
  const double steering_ref = 0.04;  // Below half the gap (0.05)

  const auto steering_velocity = drive_model->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, 0.0);
}

TEST_F(SimpleDriveModelTest, HysteresisAtHalfGapHoldsSteering) {
  // A reference exactly half the gap away holds position (lag boundary)
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  const double steering_feedback = 0.0;
  const double steering_ref = 0.05;  // Exactly half the gap (0.05)

  const auto steering_velocity = drive_model->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, 0.0);
}

TEST_F(SimpleDriveModelTest, HysteresisBeyondHalfGapCommandsSteering) {
  // Saturates at max steering velocity only when the reference is far enough beyond half the gap
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  const double steering_feedback = 0.0;
  const double steering_ref = 0.3;  // Far enough beyond half the gap (0.05) to saturate

  const auto steering_velocity = drive_model->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, HysteresisNegativeBelowHalfGapHoldsSteering) {
  // A negative reference within half the gap of the current steering holds position
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  const double steering_feedback = 0.0;
  const double steering_ref = -0.04;  // Within half the gap (0.05)

  const auto steering_velocity = drive_model->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, 0.0);
}

TEST_F(SimpleDriveModelTest, HysteresisNegativeBeyondHalfGapCommandsSteering) {
  // A negative reference far enough beyond half the gap saturates at negative max steering velocity
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  const double steering_feedback = 0.0;
  const double steering_ref = -0.3;  // Far enough beyond half the gap (0.05) to saturate

  const auto steering_velocity = drive_model->update_steering_velocity(steering_ref, steering_feedback);

  EXPECT_DOUBLE_EQ(steering_velocity, -max_steering_velocity_);
}

TEST_F(SimpleDriveModelTest, HysteresisConvergesAroundCycle) {
  // Simulate the hysteresis loop at 8 references in both directions, checking the steering
  // settles at the expected lagged value (half the gap behind the reference) at every point
  auto params = BuildDefaultParams();
  params.steering_hysteresis_gap = 0.1;  // rad
  auto drive_model = BuildModel(params);

  // Evenly spaced references within the range so ref +/- half the gap stays within the limits
  const int kNumRefs = 8;
  const double refs[kNumRefs] = {-0.35, -0.25, -0.15, -0.05, 0.05, 0.15, 0.25, 0.35};
  const double half_gap = params.steering_hysteresis_gap / 2.0;

  // Ascending pass: the steering lags below the reference by half the gap
  double steering_feedback = -0.5;
  for (int i = 0; i < kNumRefs; ++i) {
    steering_feedback = SimulateSteeringToConvergence(*drive_model, refs[i], steering_feedback);
    EXPECT_NEAR(steering_feedback, refs[i] - half_gap, 1e-6);
  }

  // Move above the top reference so the descending branch is active
  steering_feedback = SimulateSteeringToConvergence(*drive_model, 0.5, steering_feedback);

  // Descending pass: the steering lags above the reference by half the gap
  for (int i = kNumRefs - 1; i >= 0; --i) {
    steering_feedback = SimulateSteeringToConvergence(*drive_model, refs[i], steering_feedback);
    EXPECT_NEAR(steering_feedback, refs[i] + half_gap, 1e-6);
  }
}

}  // namespace svdsim_server
