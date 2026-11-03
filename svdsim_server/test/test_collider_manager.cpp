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

#include <gtest/gtest.h>

#include <memory>

#include <sophus/se2.hpp>

#include "svdsim_server/collision/collider_manager.hpp"

namespace svdsim_server {
namespace {

class ColliderManagerTest : public ::testing::Test {
 protected:
  void SetUp() override { manager_ = std::make_unique<ColliderManager>(); }

  std::unique_ptr<ColliderManager> manager_;
};

TEST_F(ColliderManagerTest, RaycastDetectsObjectsInFourDirections) {
  // Create objects with 0.1 by 0.1 footprints centered on their positions
  // front_left must have x > rear_right.x and y > rear_right.y
  const std::tuple<float, float> front_left{0.05f, 0.05f};
  const std::tuple<float, float> rear_right{-0.05f, -0.05f};

  // Create two cars
  auto car1 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(car1, nullptr);

  auto car2 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 2);
  ASSERT_NE(car2, nullptr);

  // Create obstacle
  auto obstacle = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(-6.2f, 0.0f)), BodyCategory::OBSTACLE, front_left, rear_right,
      3);
  ASSERT_NE(obstacle, nullptr);

  // Create walls
  auto wall1 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 3.8f)), BodyCategory::WALL, front_left, rear_right, 4);
  ASSERT_NE(wall1, nullptr);

  auto wall2 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, -20.0f)), BodyCategory::WALL, front_left, rear_right, 5);
  ASSERT_NE(wall2, nullptr);

  const float max_distance = 10.0f;

  /// car1 will not be registered because we start the raycast from within it

  // Test raycast in +X direction (should hit car2 at ~4.5m)
  {
    Sophus::SE2d pose(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f));
    double distance = manager_->raycast(pose, max_distance);
    EXPECT_LT(distance, max_distance);
    EXPECT_NEAR(distance, 4.5, 0.1);  // car2 is at x=4.5
  }

  // Test raycast in -X direction (should hit obstacle at ~6.2m)
  {
    Sophus::SE2d pose(Sophus::SO2d(M_PI), Eigen::Vector2d(0.0f, 0.0f));
    double distance = manager_->raycast(pose, max_distance);
    EXPECT_LT(distance, max_distance);
    EXPECT_NEAR(distance, 6.2, 0.1);  // obstacle is at x=-6.2
  }

  // Test raycast in +Y direction (should hit wall1 at ~3.8m)
  {
    Sophus::SE2d pose(Sophus::SO2d(M_PI / 2.0f), Eigen::Vector2d(0.0f, 0.0f));
    double distance = manager_->raycast(pose, max_distance);
    EXPECT_LT(distance, max_distance);
    EXPECT_NEAR(distance, 3.8, 0.1);  // wall1 is at y=3.8
  }

  // Test raycast in -Y direction (should not hit anything, wall2 is beyond 10m)
  {
    Sophus::SE2d pose(Sophus::SO2d(-M_PI / 2.0f), Eigen::Vector2d(0.0f, 0.0f));
    double distance = manager_->raycast(pose, max_distance);
    EXPECT_DOUBLE_EQ(distance, max_distance);  // No hit, returns max_distance
  }
}

TEST_F(ColliderManagerTest, RaycastReturnsClosestHit) {
  // Create objects in a line along X axis at 1m intervals
  // front_left must have x > rear_right.x and y > rear_right.y
  const std::tuple<float, float> front_left{0.05f, 0.05f};
  const std::tuple<float, float> rear_right{-0.05f, -0.05f};

  auto car1 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(1.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(car1, nullptr);

  auto car2 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(2.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 2);
  ASSERT_NE(car2, nullptr);

  auto obstacle = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(3.0f, 0.0f)), BodyCategory::OBSTACLE, front_left, rear_right, 3);
  ASSERT_NE(obstacle, nullptr);

  auto wall = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(4.0f, 0.0f)), BodyCategory::WALL, front_left, rear_right, 4);
  ASSERT_NE(wall, nullptr);

  const float max_distance = 10.0f;
  Sophus::SE2d pose(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f));

  // Test that raycast returns distance to closest hit (car1 at ~1m)
  {
    double distance = manager_->raycast(pose, max_distance);
    EXPECT_LT(distance, max_distance);
    EXPECT_NEAR(distance, 1.0, 0.1);  // car1 is closest at x=1.0
  }

  // Test raycast from beyond all objects in -X direction (should hit wall at ~1m)
  {
    Sophus::SE2d reverse_pose(Sophus::SO2d(M_PI), Eigen::Vector2d(5.0f, 0.0f));
    double distance = manager_->raycast(reverse_pose, max_distance);
    EXPECT_LT(distance, max_distance);
    EXPECT_NEAR(distance, 1.0, 0.1);  // wall is at x=4.0, so distance from x=5.0 is ~1.0
  }
}

TEST_F(ColliderManagerTest, RaycastDetectsNonCenteredFootprint) {
  // Create a body with a non-centered footprint
  // Footprint extends from (0.0, -0.2) to (0.4, 0.2) relative to body pose
  const std::tuple<float, float> front_left{0.4f, 0.2f};
  const std::tuple<float, float> rear_right{0.0f, -0.2f};

  auto body = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(5.0f, 5.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(body, nullptr);

  // With body at (5.0, 5.0), footprint edges are:
  // Left edge: x = 5.0, Bottom edge: y = 4.8, Right edge: x = 5.4, Top edge: y = 5.2

  const float max_distance = 1.0f;

  // Test left edge (x = 5.0)
  {
    // Ray just outside (x = 4.95) starting from below box, pointing +Y - should miss
    Sophus::SE2d pose_outside(Sophus::SO2d(M_PI / 2.0f), Eigen::Vector2d(4.95f, 4.5f));
    double distance = manager_->raycast(pose_outside, max_distance);
    EXPECT_DOUBLE_EQ(distance, max_distance);  // No hit

    // Ray just inside (x = 5.05) starting from below box, pointing +Y - should hit
    Sophus::SE2d pose_inside(Sophus::SO2d(M_PI / 2.0f), Eigen::Vector2d(5.05f, 4.5f));
    distance = manager_->raycast(pose_inside, max_distance);
    EXPECT_LT(distance, max_distance);  // Hit detected
  }

  // Test right edge (x = 5.4)
  {
    // Ray just outside (x = 5.45) starting from below box, pointing +Y - should miss
    Sophus::SE2d pose_outside(Sophus::SO2d(M_PI / 2.0f), Eigen::Vector2d(5.45f, 4.5f));
    double distance = manager_->raycast(pose_outside, max_distance);
    EXPECT_DOUBLE_EQ(distance, max_distance);  // No hit

    // Ray just inside (x = 5.35) starting from below box, pointing +Y - should hit
    Sophus::SE2d pose_inside(Sophus::SO2d(M_PI / 2.0f), Eigen::Vector2d(5.35f, 4.5f));
    distance = manager_->raycast(pose_inside, max_distance);
    EXPECT_LT(distance, max_distance);  // Hit detected
  }

  // Test bottom edge (y = 4.8)
  {
    // Ray just outside (y = 4.75) starting from left of box, pointing +X - should miss
    Sophus::SE2d pose_outside(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 4.75f));
    double distance = manager_->raycast(pose_outside, max_distance);
    EXPECT_DOUBLE_EQ(distance, max_distance);  // No hit

    // Ray just inside (y = 4.85) starting from left of box, pointing +X - should hit
    Sophus::SE2d pose_inside(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 4.85f));
    distance = manager_->raycast(pose_inside, max_distance);
    EXPECT_LT(distance, max_distance);  // Hit detected
  }

  // Test top edge (y = 5.2)
  {
    // Ray just outside (y = 5.25) starting from left of box, pointing +X - should miss
    Sophus::SE2d pose_outside(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 5.25f));
    double distance = manager_->raycast(pose_outside, max_distance);
    EXPECT_DOUBLE_EQ(distance, max_distance);  // No hit

    // Ray just inside (y = 5.15) starting from left of box, pointing +X - should hit
    Sophus::SE2d pose_inside(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 5.15f));
    distance = manager_->raycast(pose_inside, max_distance);
    EXPECT_LT(distance, max_distance);  // Hit detected
  }
}

TEST_F(ColliderManagerTest, CheckCollisionsDetectsOverlappingBodies) {
  const std::tuple<float, float> front_left{0.5f, 0.5f};
  const std::tuple<float, float> rear_right{-0.5f, -0.5f};

  // Create a reference body at origin
  auto reference = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(reference, nullptr);

  // Create non-overlapping body (separated by 2m in X direction)
  auto non_overlapping = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(2.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 2);
  ASSERT_NE(non_overlapping, nullptr);

  // Create overlapping body (slight overlap)
  auto overlapping1 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.8f, 0.0f)), BodyCategory::OBSTACLE, front_left, rear_right, 3);
  ASSERT_NE(overlapping1, nullptr);

  // Create another overlapping body
  auto overlapping2 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.8f)), BodyCategory::WALL, front_left, rear_right, 4);
  ASSERT_NE(overlapping2, nullptr);

  // Check collisions for reference body
  std::vector<std::size_t> colliding_ids;
  manager_->check_collisions(*reference, [&](const BodyHandle& handle) { colliding_ids.push_back(handle.id()); });

  // Should detect exactly 2 collisions (overlapping1 and overlapping2)
  ASSERT_EQ(colliding_ids.size(), 2);
  EXPECT_TRUE(std::find(colliding_ids.begin(), colliding_ids.end(), overlapping1->id()) != colliding_ids.end());
  EXPECT_TRUE(std::find(colliding_ids.begin(), colliding_ids.end(), overlapping2->id()) != colliding_ids.end());
  EXPECT_FALSE(std::find(colliding_ids.begin(), colliding_ids.end(), non_overlapping->id()) != colliding_ids.end());
}

TEST_F(ColliderManagerTest, CheckCollisionsDetectsCornerOverlap) {
  const std::tuple<float, float> front_left{0.5f, 0.5f};
  const std::tuple<float, float> rear_right{-0.5f, -0.5f};

  // Create reference body at origin
  auto reference = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(reference, nullptr);

  // Create body overlapping by just one corner (positioned diagonally)
  auto corner_overlap = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.95f, 0.95f)), BodyCategory::OBSTACLE, front_left, rear_right,
      2);
  ASSERT_NE(corner_overlap, nullptr);

  // Check collisions - should detect corner overlap
  int collision_count = 0;
  manager_->check_collisions(*reference, [&](const BodyHandle& handle) {
    collision_count++;
    EXPECT_EQ(handle.id(), corner_overlap->id());
  });

  EXPECT_EQ(collision_count, 1);
}

TEST_F(ColliderManagerTest, CheckCollisionsAccountsForRotation) {
  // Create two boxes that would overlap if aligned, but don't overlap when rotated
  const std::tuple<float, float> front_left{0.5f, 0.5f};
  const std::tuple<float, float> rear_right{-0.5f, -0.5f};

  // First box at 45 degrees
  auto box1 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(M_PI / 4.0f), Eigen::Vector2d(0.0f, 0.0f)), BodyCategory::VEHICLE, front_left,
      rear_right, 1);
  ASSERT_NE(box1, nullptr);

  // Second box at 45 degrees, positioned to not overlap
  // At 45 degrees, a 1x1 box has diagonal extent of sqrt(2) ≈ 1.414
  // Position at distance 1.5 along X-axis to ensure no overlap
  auto box2 = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(M_PI / 4.0f), Eigen::Vector2d(1.5f, 0.0f)), BodyCategory::OBSTACLE, front_left,
      rear_right, 2);
  ASSERT_NE(box2, nullptr);

  // Check collisions - should not detect any
  int collision_count = 0;
  manager_->check_collisions(*box1, [&]([[maybe_unused]] const BodyHandle& handle) { collision_count++; });

  EXPECT_EQ(collision_count, 0);

  // Now rotate box2 so a corner comes into contact
  // Rotate to 0 degrees and move closer to create overlap
  box2->set_pose(Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.9f, 0.0f)));

  // Check collisions again - should now detect overlap
  collision_count = 0;
  manager_->check_collisions(*box1, [&](const BodyHandle& handle) {
    collision_count++;
    EXPECT_EQ(handle.id(), box2->id());
  });

  EXPECT_EQ(collision_count, 1);
}

TEST_F(ColliderManagerTest, SetPoseMovesBodyCorrectly) {
  const std::tuple<float, float> front_left{0.5f, 0.5f};
  const std::tuple<float, float> rear_right{-0.5f, -0.5f};

  // Create a body at origin
  auto body = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f)), BodyCategory::VEHICLE, front_left, rear_right, 1);
  ASSERT_NE(body, nullptr);

  // Create a stationary obstacle at (5, 0)
  auto obstacle = manager_->add_rectangular_body(
      Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(5.0f, 0.0f)), BodyCategory::OBSTACLE, front_left, rear_right, 2);
  ASSERT_NE(obstacle, nullptr);

  // Initially, body at origin should not collide with obstacle at (5, 0)
  {
    int collision_count = 0;
    manager_->check_collisions(*body, [&]([[maybe_unused]] const BodyHandle& handle) { collision_count++; });
    EXPECT_EQ(collision_count, 0);
  }

  // Move body to (4.5, 0) - should now overlap with obstacle
  body->set_pose(Sophus::SE2d(Sophus::SO2d(0.0f), Eigen::Vector2d(4.5f, 0.0f)));

  // After moving, body should collide with obstacle
  {
    int collision_count = 0;
    std::size_t colliding_id = 0;
    manager_->check_collisions(*body, [&](const BodyHandle& handle) {
      collision_count++;
      colliding_id = handle.id();
    });
    EXPECT_EQ(collision_count, 1);
    EXPECT_EQ(colliding_id, obstacle->id());
  }

  // Test raycast to verify position change
  {
    Sophus::SE2d raycast_pose(Sophus::SO2d(0.0f), Eigen::Vector2d(0.0f, 0.0f));
    double distance = manager_->raycast(raycast_pose, 10.0f);

    // Ray from origin in +X direction should now hit body at (4.5, 0) first
    EXPECT_LT(distance, 10.0f);
    EXPECT_NEAR(distance, 4.0, 0.1);
  }

  // Test rotation with set_pose
  body->set_pose(Sophus::SE2d(Sophus::SO2d(M_PI / 4.0f), Eigen::Vector2d(0.0f, 0.0f)));

  // Verify rotation doesn't cause false collision at origin
  {
    int collision_count = 0;
    manager_->check_collisions(*body, [&]([[maybe_unused]] const BodyHandle& handle) { collision_count++; });
    EXPECT_EQ(collision_count, 0);
  }
}

}  // namespace
}  // namespace svdsim_server
