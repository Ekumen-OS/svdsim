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

#include "svdsim_server/utils/pure_pursuit_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <sophus/se2.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace svdsim_server {
namespace {

// Helper function to create a straight line trajectory
std::vector<TrajectoryPoint> create_straight_trajectory(double length, double spacing) {
  std::vector<TrajectoryPoint> trajectory;
  for (double x = 0.0; x <= length; x += spacing) {
    trajectory.emplace_back(x, 0.0);
  }
  return trajectory;
}

// Helper function to create a circular trajectory
std::vector<TrajectoryPoint> create_circular_trajectory(double radius, size_t num_points) {
  std::vector<TrajectoryPoint> trajectory;
  for (size_t i = 0; i < num_points; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
    trajectory.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
  }
  return trajectory;
}

TEST(PurePursuitControllerTest, ConstructorValid) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;

  EXPECT_NO_THROW(PurePursuitController controller(trajectory, params));
}

TEST(PurePursuitControllerTest, ConstructorEmptyTrajectory) {
  std::vector<TrajectoryPoint> empty_trajectory;
  PurePursuitParams params;

  EXPECT_THROW(PurePursuitController controller(empty_trajectory, params), std::invalid_argument);
}

TEST(PurePursuitControllerTest, ConstructorInvalidWheelbase) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = -0.1;

  EXPECT_THROW(PurePursuitController controller(trajectory, params), std::invalid_argument);
}

TEST(PurePursuitControllerTest, ConstructorInvalidLookahead) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.lookahead_distance = 0.0;

  EXPECT_THROW(PurePursuitController controller(trajectory, params), std::invalid_argument);
}

TEST(PurePursuitControllerTest, StraightLineNoSteering) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = 0.324;
  params.lookahead_distance = 1.0;

  PurePursuitController controller(trajectory, params);

  // Vehicle at origin, aligned with trajectory
  Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  double steering = controller.compute_steering(pose);

  // On a straight path, steering should be near zero
  EXPECT_NEAR(steering, 0.0, 0.01);
}

TEST(PurePursuitControllerTest, OffsetFromStraightLine) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = 0.324;
  params.lookahead_distance = 1.0;

  PurePursuitController controller(trajectory, params);

  // Vehicle offset to the left of the trajectory
  Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 1.0));
  double steering = controller.compute_steering(pose);

  // Controller should steer right (negative) to return to path
  EXPECT_LT(steering, 0.0);
}

TEST(PurePursuitControllerTest, CircularTrajectory) {
  auto trajectory = create_circular_trajectory(5.0, 100);
  PurePursuitParams params;
  params.wheelbase = 0.324;
  params.lookahead_distance = 1.0;

  PurePursuitController controller(trajectory, params);

  // Vehicle at the first trajectory point
  Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(trajectory[0].x, trajectory[0].y));
  double steering = controller.compute_steering(pose);

  // On a circular path, there should be non-zero steering
  EXPECT_NE(steering, 0.0);
}

TEST(PurePursuitControllerTest, SteeringClamping) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = 0.324;
  params.lookahead_distance = 0.1;  // Very short lookahead
  params.max_steering = 0.3;        // Low max steering

  PurePursuitController controller(trajectory, params);

  // Vehicle with large lateral offset (requires high steering)
  Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 5.0));
  double steering = controller.compute_steering(pose);

  // Steering should be clamped to max
  EXPECT_LE(std::abs(steering), params.max_steering + 1e-6);
}

TEST(PurePursuitControllerTest, TargetPointVisualization) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = 0.324;
  params.lookahead_distance = 2.0;

  PurePursuitController controller(trajectory, params);

  Sophus::SE2d pose(Sophus::SO2d(0.0), Eigen::Vector2d(0.0, 0.0));
  [[maybe_unused]] auto steering = controller.compute_steering(pose);

  // Get target point for visualization
  const auto& target_point = controller.get_last_target_point();

  // Target point should be ahead of current position
  EXPECT_GT(target_point.x(), pose.translation().x());
}

TEST(PurePursuitControllerTest, GetTrajectory) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;

  PurePursuitController controller(trajectory, params);

  const auto& retrieved_trajectory = controller.get_trajectory();
  EXPECT_EQ(retrieved_trajectory.size(), trajectory.size());
  EXPECT_DOUBLE_EQ(retrieved_trajectory[0].x, trajectory[0].x);
}

TEST(PurePursuitControllerTest, GetParams) {
  auto trajectory = create_straight_trajectory(10.0, 0.5);
  PurePursuitParams params;
  params.wheelbase = 0.5;
  params.lookahead_distance = 1.5;

  PurePursuitController controller(trajectory, params);

  const auto& retrieved_params = controller.get_params();
  EXPECT_DOUBLE_EQ(retrieved_params.wheelbase, 0.5);
  EXPECT_DOUBLE_EQ(retrieved_params.lookahead_distance, 1.5);
}

}  // namespace
}  // namespace svdsim_server
