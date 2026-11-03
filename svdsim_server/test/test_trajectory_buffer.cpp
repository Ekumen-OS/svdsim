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
#include "svdsim_server/trajectory_buffer.hpp"

namespace svdsim_server {

class TrajectoryBufferTest : public ::testing::Test {
 protected:
  void SetUp() override { tolerance_ = 1e-6; }

  double tolerance_;
};

TEST_F(TrajectoryBufferTest, ConstructionAndReset) {
  TrajectoryBuffer buffer(5);

  // Create initial pose at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, initial_pose);

  // for all deltas from 0 to infinity, should return initial pose
  for (double delta = 0.0; delta <= 10.0; delta += 1.0) {
    auto result = buffer.interpolate(delta);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 0.0, tolerance_);
    EXPECT_NEAR(result->translation().y(), 0.0, tolerance_);
    EXPECT_NEAR(result->so2().log(), 0.0, tolerance_);
  }
}

TEST_F(TrajectoryBufferTest, IntegerOffsetsInRectilinearMotion) {
  // Create a 5-element buffer
  TrajectoryBuffer buffer(5);

  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, initial_pose);

  // Add poses along x-axis: (0,0), (1,0), (2,0), (3,0), (4,0)
  // Each pose is 1 meter apart, representing motion at 1 m/s with 1s time step
  for (int i = 1; i <= 5; ++i) {
    Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(static_cast<double>(i), 0.0));
    buffer.add(i, pose);
  }

  // Test retrieval of exact poses
  // delta_t = 0 should return most recent pose (5, 0)
  auto result_0 = buffer.interpolate(0.0);
  ASSERT_TRUE(result_0.has_value());
  EXPECT_NEAR(result_0->translation().x(), 5.0, tolerance_);
  EXPECT_NEAR(result_0->translation().y(), 0.0, tolerance_);

  // delta_t = 1.0 should return pose 1 step back (4, 0)
  auto result_1 = buffer.interpolate(1.0);
  ASSERT_TRUE(result_1.has_value());
  EXPECT_NEAR(result_1->translation().x(), 4.0, tolerance_);
  EXPECT_NEAR(result_1->translation().y(), 0.0, tolerance_);

  // delta_t = 2.0 should return pose 2 steps back (3, 0)
  auto result_2 = buffer.interpolate(2.0);
  ASSERT_TRUE(result_2.has_value());
  EXPECT_NEAR(result_2->translation().x(), 3.0, tolerance_);
  EXPECT_NEAR(result_2->translation().y(), 0.0, tolerance_);

  // delta_t = 3.0 should return pose 3 steps back (2, 0)
  auto result_3 = buffer.interpolate(3.0);
  ASSERT_TRUE(result_3.has_value());
  EXPECT_NEAR(result_3->translation().x(), 2.0, tolerance_);
  EXPECT_NEAR(result_3->translation().y(), 0.0, tolerance_);

  // for deltas beyond 4.0, should return oldest pose (1, 0)
  for (double delta = 4.0; delta <= 10.0; delta += 1.0) {
    auto result = buffer.interpolate(delta);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 1.0, tolerance_);
    EXPECT_NEAR(result->translation().y(), 0.0, tolerance_);
  }
}

TEST_F(TrajectoryBufferTest, InterpolationInRectilinearMotion) {
  // Create a 5-element buffer
  TrajectoryBuffer buffer(5);

  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, initial_pose);

  // Add poses along x-axis with 1 meter spacing
  for (int i = 1; i <= 5; ++i) {
    Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(static_cast<double>(i), 0.0));
    buffer.add(i, pose);
  }

  // Test interpolation at delta_t = 0.5 (halfway between poses at x=5 and x=4)
  auto result_0_5 = buffer.interpolate(0.5);
  ASSERT_TRUE(result_0_5.has_value());
  EXPECT_NEAR(result_0_5->translation().x(), 4.5, tolerance_);
  EXPECT_NEAR(result_0_5->translation().y(), 0.0, tolerance_);

  // Test interpolation at delta_t = 1.5 (halfway between poses at x=4 and x=3)
  auto result_1_5 = buffer.interpolate(1.5);
  ASSERT_TRUE(result_1_5.has_value());
  EXPECT_NEAR(result_1_5->translation().x(), 3.5, tolerance_);
  EXPECT_NEAR(result_1_5->translation().y(), 0.0, tolerance_);

  // Test interpolation at delta_t = 2.75 (3/4 of the way from x=3 to x=2)
  auto result_2_75 = buffer.interpolate(2.75);
  ASSERT_TRUE(result_2_75.has_value());
  EXPECT_NEAR(result_2_75->translation().x(), 2.75, tolerance_);
  EXPECT_NEAR(result_2_75->translation().y(), 0.0, tolerance_);
}

TEST_F(TrajectoryBufferTest, OverflownBufferCirclesBack) {
  // Create a 5-element buffer
  TrajectoryBuffer buffer(5);

  // Initialize at origin
  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, initial_pose);

  // Add more than 5 poses to test circular behavior
  // Add poses at x = 1, 2, 3, 4, 5, 6, 7, 8
  for (int i = 1; i <= 8; ++i) {
    Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(static_cast<double>(i), 0.0));
    buffer.add(i, pose);
  }

  // After adding 8 poses, buffer should contain the last 5: [4, 5, 6, 7, 8]
  // delta_t = 0 should return most recent (8, 0)
  auto result_0 = buffer.interpolate(0.0);
  ASSERT_TRUE(result_0.has_value());
  EXPECT_NEAR(result_0->translation().x(), 8.0, tolerance_);

  // delta_t = 1.0 should return (7, 0)
  auto result_1 = buffer.interpolate(1.0);
  ASSERT_TRUE(result_1.has_value());
  EXPECT_NEAR(result_1->translation().x(), 7.0, tolerance_);

  // delta_t = 4.0 should return oldest pose in buffer (4, 0)
  auto result_4 = buffer.interpolate(4.0);
  ASSERT_TRUE(result_4.has_value());
  EXPECT_NEAR(result_4->translation().x(), 4.0, tolerance_);

  // delta_t >= 5.0 should also return oldest pose (saturate at buffer limit)
  for (double delta = 5.0; delta <= 10.0; delta += 0.2) {
    auto result = buffer.interpolate(delta);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 4.0, tolerance_);
    EXPECT_NEAR(result->translation().y(), 0.0, tolerance_);
  }
}

TEST_F(TrajectoryBufferTest, NegativeDeltaT) {
  TrajectoryBuffer buffer(5);

  Sophus::SE2d initial_pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, initial_pose);

  // Negative delta_t should return nullopt
  auto result = buffer.interpolate(-1.0);
  EXPECT_FALSE(result.has_value());
}

TEST_F(TrajectoryBufferTest, RotationInterpolatesAlongTheShortArc) {
  // Create a 2-element buffer
  TrajectoryBuffer buffer(2);

  // Create two poses at the same position but rotated 90 degrees apart
  // First pose: 0 degrees at origin
  Sophus::SE2d pose_0(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  buffer.reset(0, pose_0);

  // Second pose: 90 degrees (π/2) at origin
  Sophus::SE2d pose_90(Sophus::SO2d(M_PI / 2.0), Eigen::Vector2d(0.0, 0.0));
  buffer.add(1, pose_90);

  // Test exact poses
  // delta_t = 0.0 should return most recent pose (90 degrees)
  auto result_0 = buffer.interpolate(0.0);
  ASSERT_TRUE(result_0.has_value());
  EXPECT_NEAR(result_0->translation().x(), 0.0, tolerance_);
  EXPECT_NEAR(result_0->translation().y(), 0.0, tolerance_);
  EXPECT_NEAR(result_0->so2().log(), M_PI / 2.0, tolerance_);

  // delta_t = 1.0 should return previous pose (0 degrees)
  auto result_1 = buffer.interpolate(1.0);
  ASSERT_TRUE(result_1.has_value());
  EXPECT_NEAR(result_1->translation().x(), 0.0, tolerance_);
  EXPECT_NEAR(result_1->translation().y(), 0.0, tolerance_);
  EXPECT_NEAR(result_1->so2().log(), 0.0, tolerance_);

  // delta_t = 0.5 should interpolate to 45 degrees (π/4)
  // Midway between 0 and 90 degrees
  auto result_0_5 = buffer.interpolate(0.5);
  ASSERT_TRUE(result_0_5.has_value());
  EXPECT_NEAR(result_0_5->translation().x(), 0.0, tolerance_);
  EXPECT_NEAR(result_0_5->translation().y(), 0.0, tolerance_);
  EXPECT_NEAR(result_0_5->so2().log(), M_PI / 4.0, tolerance_);  // 45 degrees
}

}  // namespace svdsim_server
