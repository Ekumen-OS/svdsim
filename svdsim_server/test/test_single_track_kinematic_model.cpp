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

// Non-ROS dependencies
#include <gtest/gtest.h>
#include <cmath>

// Project headers
#include "svdsim_server/models/single_track_kinematic_model.hpp"

namespace svdsim_server {

/**
 * @brief Test helper class to access protected members of SingleTrackKinematicModel
 */
class SingleTrackKinematicModelTestHelper : public SingleTrackKinematicModel {
 public:
  explicit SingleTrackKinematicModelTestHelper(const SingleTrackKinematicParams& params)
      : SingleTrackKinematicModel(params) {}

  // Expose protected members for testing
  const State& get_state() const { return state_; }

  void set_state(const State& state) { state_ = state; }
};

class SingleTrackKinematicModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Typical F1TENTH parameters
    params_.wheelbase = 0.324;              // m
    params_.rear_track_width = 0.2;         // m
    params_.effective_wheel_radius = 0.05;  // m
    params_.mass = 3.5;                     // kg

    model_ = std::make_unique<SingleTrackKinematicModelTestHelper>(params_);

    // Standard test parameters
    dt_ = 0.01;  // 10ms time step
    tolerance_ = 1e-3;
  }

  SingleTrackKinematicParams params_;
  std::unique_ptr<SingleTrackKinematicModelTestHelper> model_;
  double dt_;
  double tolerance_;
};

// Test forward constant velocity motion
TEST_F(SingleTrackKinematicModelTest, ForwardConstantVelocityLinearMotion) {
  // Initialize at origin with zero yaw
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial velocity
  auto state = model_->get_state();
  state(3) = 1.0;  // 1 m/s forward velocity
  model_->set_state(state);

  // Input: no steering, no acceleration
  ControlInput input;
  input << 0.0, 0.0;

  // Integrate for 1 second
  const double duration = 1.0;
  const int num_steps = static_cast<int>(duration / dt_);
  model_->integrate(input, dt_, num_steps);

  // Check final state
  const auto& final_state = model_->state_summary();
  const auto final_position = final_state.base_link_pose.translation();

  EXPECT_NEAR(final_position.x(), 1.0, tolerance_);                // Moved 1m forward
  EXPECT_NEAR(final_position.y(), 0.0, tolerance_);                // No lateral motion
  EXPECT_NEAR(final_state.vehicle_velocity.x(), 1.0, tolerance_);  // Constant velocity
  EXPECT_NEAR(final_state.steering_angle, 0.0, tolerance_);        // No steering

  // Test pose() method returns same pose as state_summary
  const Sophus::SE2d pose = model_->pose();
  const auto state_pose = final_state.base_link_pose;

  EXPECT_NEAR(pose.translation().x(), state_pose.translation().x(), tolerance_);
  EXPECT_NEAR(pose.translation().y(), state_pose.translation().y(), tolerance_);

  // Extract yaw from SE2 pose
  const double yaw_from_state = state_pose.so2().log();
  EXPECT_NEAR(pose.so2().log(), yaw_from_state, tolerance_);
}

// Test backward constant velocity motion
TEST_F(SingleTrackKinematicModelTest, BackwardConstantVelocityLinearMotion) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial negative velocity
  auto state = model_->get_state();
  state(3) = -0.5;  // 0.5 m/s backward velocity
  model_->set_state(state);

  // Input: no steering, no acceleration
  ControlInput input;
  input << 0.0, 0.0;

  // Integrate for 2 seconds
  const double duration = 2.0;
  const int num_steps = static_cast<int>(duration / dt_);
  model_->integrate(input, dt_, num_steps);

  // Check final state
  const auto& final_state = model_->state_summary();
  const auto final_position = final_state.base_link_pose.translation();

  EXPECT_NEAR(final_position.x(), -1.0, tolerance_);                // Moved 1m backward
  EXPECT_NEAR(final_position.y(), 0.0, tolerance_);                 // No lateral motion
  EXPECT_NEAR(final_state.vehicle_velocity.x(), -0.5, tolerance_);  // Constant velocity
}

// Test forward accelerated linear motion
TEST_F(SingleTrackKinematicModelTest, ForwardAcceleratedLinearMotion) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Input: no steering, constant acceleration
  ControlInput input;
  input << 0.0, 2.0;  // 2 m/s² acceleration

  // Integrate for 1 second
  const double duration = 1.0;
  const int num_steps = static_cast<int>(duration / dt_);
  model_->integrate(input, dt_, num_steps);

  // Check final state
  const auto& final_state = model_->state_summary();
  const auto final_position = final_state.base_link_pose.translation();

  // With constant acceleration: x = 0.5 * a * t²
  const double expected_distance = 0.5 * 2.0 * duration * duration;
  EXPECT_NEAR(final_position.x(), expected_distance, tolerance_);
  EXPECT_NEAR(final_position.y(), 0.0, tolerance_);

  // Final velocity: v = a * t
  EXPECT_NEAR(final_state.vehicle_velocity.x(), 2.0, tolerance_);
}

// Test backward accelerated linear motion (deceleration)
TEST_F(SingleTrackKinematicModelTest, BackwardAcceleratedLinearMotion) {
  // Initialize at origin with forward velocity
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial forward velocity
  auto state = model_->get_state();
  state(3) = 3.0;  // 3 m/s forward
  model_->set_state(state);

  // Input: no steering, negative acceleration (braking)
  ControlInput input;
  input << 0.0, -2.0;  // -2 m/s² deceleration

  // Integrate for 1 second
  const double duration = 1.0;
  const int num_steps = static_cast<int>(duration / dt_);
  model_->integrate(input, dt_, num_steps);

  // Check final state
  const auto& final_state = model_->state_summary();

  // Final velocity: v = v0 + a * t = 3 - 2*1 = 1 m/s
  EXPECT_NEAR(final_state.vehicle_velocity.x(), 1.0, tolerance_);
}

// Test constant velocity left turn
TEST_F(SingleTrackKinematicModelTest, ConstantVelocityLeftTurn) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial velocity
  auto state = model_->get_state();
  state(3) = 1.0;  // 1 m/s forward
  model_->set_state(state);

  // Input: constant left steering angle (positive), no acceleration
  const double target_steering = 0.3;  // rad
  ControlInput input;
  input << 1.0, 0.0;  // Positive steering velocity to reach left turn

  // First, ramp up to desired steering angle
  double ramp_time = target_steering / input(0);
  int ramp_steps = static_cast<int>(ramp_time / dt_);
  model_->integrate(input, dt_, ramp_steps);

  // Now maintain constant steering
  input << 0.0, 0.0;  // No steering change, no acceleration

  // Integrate for a quarter circle (approximate)
  const double turn_duration = 1.0;
  const int turn_steps = static_cast<int>(turn_duration / dt_);
  model_->integrate(input, dt_, turn_steps);

  // Check that vehicle has turned left (positive yaw change)
  const auto& final_state = model_->state_summary();
  const double final_yaw = final_state.base_link_pose.so2().log();

  EXPECT_GT(final_yaw, 0.0);                                       // Positive yaw = left turn
  EXPECT_NEAR(final_state.vehicle_velocity.x(), 1.0, tolerance_);  // Constant velocity
  EXPECT_NEAR(final_state.steering_angle, target_steering, tolerance_);
}

// Test constant velocity right turn
TEST_F(SingleTrackKinematicModelTest, ConstantVelocityRightTurn) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial velocity
  auto state = model_->get_state();
  state(3) = 1.0;  // 1 m/s forward
  model_->set_state(state);

  // Input: constant right steering angle (negative)
  const double target_steering = -0.3;  // rad (negative = right)
  ControlInput input;
  input << -1.0, 0.0;  // Negative steering velocity for right turn

  // Ramp up to desired steering angle
  double ramp_time = std::abs(target_steering / input(0));
  int ramp_steps = static_cast<int>(ramp_time / dt_);
  model_->integrate(input, dt_, ramp_steps);

  // Maintain constant steering
  input << 0.0, 0.0;

  const double turn_duration = 1.0;
  const int turn_steps = static_cast<int>(turn_duration / dt_);
  model_->integrate(input, dt_, turn_steps);

  // Check that vehicle has turned right (negative yaw change)
  const auto& final_state = model_->state_summary();
  const double final_yaw = final_state.base_link_pose.so2().log();

  EXPECT_LT(final_yaw, 0.0);                                       // Negative yaw = right turn
  EXPECT_NEAR(final_state.vehicle_velocity.x(), 1.0, tolerance_);  // Constant velocity
  EXPECT_NEAR(final_state.steering_angle, target_steering, tolerance_);
}

// Test accelerated left turn
TEST_F(SingleTrackKinematicModelTest, AcceleratedLeftTurn) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial velocity
  auto state = model_->get_state();
  state(3) = 0.5;  // 0.5 m/s initial velocity
  model_->set_state(state);

  // Input: left steering and positive acceleration
  const double target_steering = 0.4;
  ControlInput input;
  input << 1.0, 1.5;  // Steering velocity and acceleration

  // Ramp steering
  double ramp_time = target_steering / input(0);
  int ramp_steps = static_cast<int>(ramp_time / dt_);
  model_->integrate(input, dt_, ramp_steps);

  // Continue with acceleration while turning
  input << 0.0, 1.5;
  const double turn_duration = 0.5;
  const int turn_steps = static_cast<int>(turn_duration / dt_);
  model_->integrate(input, dt_, turn_steps);

  const auto& final_state = model_->state_summary();
  const double final_yaw = final_state.base_link_pose.so2().log();

  // Should have turned left (positive yaw)
  EXPECT_GT(final_yaw, 0.0);

  // Should have accelerated
  EXPECT_GT(final_state.vehicle_velocity.x(), 0.5);
}

// Test accelerated right turn
TEST_F(SingleTrackKinematicModelTest, AcceleratedRightTurn) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set initial velocity
  auto state = model_->get_state();
  state(3) = 1.0;  // 1 m/s initial velocity
  model_->set_state(state);

  // Input: right steering and positive acceleration
  const double target_steering = -0.35;
  ControlInput input;
  input << -1.0, 2.0;  // Negative steering velocity (right) and acceleration

  // Ramp steering
  double ramp_time = std::abs(target_steering / input(0));
  int ramp_steps = static_cast<int>(ramp_time / dt_);
  model_->integrate(input, dt_, ramp_steps);

  // Continue with acceleration while turning
  input << 0.0, 2.0;
  const double turn_duration = 0.5;
  const int turn_steps = static_cast<int>(turn_duration / dt_);
  model_->integrate(input, dt_, turn_steps);

  const auto& final_state = model_->state_summary();
  const double final_yaw = final_state.base_link_pose.so2().log();

  // Should have turned right (negative yaw)
  EXPECT_LT(final_yaw, 0.0);

  // Should have accelerated significantly
  EXPECT_GT(final_state.vehicle_velocity.x(), 1.0);
}

// Test that wheel angles are properly tracked
TEST_F(SingleTrackKinematicModelTest, WheelAngleTracking) {
  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d::Zero());
  model_->reset(initial_pose);

  // Set velocity and perform a turn
  auto state = model_->get_state();
  state(3) = 2.0;  // 2 m/s
  state(2) = 0.2;  // Some steering angle
  model_->set_state(state);

  // No input, just let it roll
  ControlInput input = ControlInput::Zero();

  const double duration = 1.0;
  const int num_steps = static_cast<int>(duration / dt_);
  model_->integrate(input, dt_, num_steps);

  const auto& final_state = model_->state_summary();

  // Wheels should have rotated (angles should be non-zero)
  EXPECT_NE(final_state.rear_left_wheel_angle, 0.0);
  EXPECT_NE(final_state.rear_right_wheel_angle, 0.0);

  // Due to differential in turning, wheel angles should be different
  EXPECT_NE(final_state.rear_left_wheel_angle, final_state.rear_right_wheel_angle);
}

}  // namespace svdsim_server

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
