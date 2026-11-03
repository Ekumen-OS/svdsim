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

#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/collision/static_map_handler.hpp"
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {
namespace {

class StaticMapHandlerTest : public ::testing::Test {
 protected:
  void SetUp() override { manager_ = std::make_shared<ColliderManager>(); }

  // Helper function to create a map from a string representation
  auto create_map_from_string(const char* str_map, std::size_t width, std::size_t height, double resolution) {
    StaticMap map;
    map.width = width;
    map.height = height;
    map.resolution = resolution;
    map.origin = Sophus::SE2d(Sophus::SO2d(0.0), Eigen::Vector2d(10.0, 10.0));
    map.occupancy_grid.resize(width * height, false);
    for (std::size_t y = 0; y < height; ++y) {
      for (std::size_t x = 0; x < width; ++x) {
        const auto idx_in_str = y * width + x;
        map.occupancy_grid[idx_in_str] = (str_map[idx_in_str] == 'X');
      }
    }

    return map;
  }

  // Helper function to test if there's an obstacle at a given position
  bool has_obstacle_at(const Eigen::Vector2d& position, double probe_size = 0.01) {
    // Create a small probe body at the test position
    const auto half_size = static_cast<float>(probe_size / 2.0);
    const std::tuple<float, float> front_left{half_size, half_size};
    const std::tuple<float, float> rear_right{-half_size, -half_size};

    auto probe = manager_->add_rectangular_body(
        Sophus::SE2d(Sophus::SO2d(0.0), position), BodyCategory::VEHICLE, front_left, rear_right, 999);

    if (!probe) {
      return false;
    }

    // Check for collisions
    bool collision_found = false;
    manager_->check_collisions(*probe, [&](const BodyHandle& handle) {
      // Ignore collisions with other probe bodies (category VEHICLE)
      if (handle.category() == BodyCategory::OBSTACLE) {
        collision_found = true;
      }
    });

    return collision_found;
  }

  std::shared_ptr<ColliderManager> manager_;
};

TEST_F(StaticMapHandlerTest, DetectsEdgeCellsInSparseMap) {
  // Test that coordinate conversions work correctly by placing isolated obstacle cells
  // at known positions and verifying their world coordinates.
  // Map layout (X = occupied, . = free):
  //   0 1 2 3 4 5 6 7 8

  const char* str_map =  // Rows (y from top to bottom)
      "X.......X"        // y = 0.65
      "........."        // y = 0.55
      "........."        // y = 0.45
      "........."        // y = 0.35
      "........."        // y = 0.25
      "........."        // y = 0.15
      "X........";       // y = 0.05
  // Columns
  //   ||||||||>      x = 0.85
  //   |||||||>       x = 0.75
  //   ||||||>        x = 0.65
  //   |||||>         x = 0.55
  //   ||||>          x = 0.45
  //   |||>           x = 0.35
  //   ||>            x = 0.25
  //   |>             x = 0.15
  //   >              x = 0.05

  constexpr std::size_t width = 9;
  constexpr std::size_t height = 7;
  constexpr double resolution = 0.1;  // 0.1 meter per cell

  auto map = create_map_from_string(str_map, width, height, resolution);

  // Create StaticMapHandler - all occupied cells are edges since they're isolated
  StaticMapHandler map_handler(map, manager_);

  // Point outside the map should have no obstacle
  EXPECT_FALSE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(-0.05, -0.05)));

  // Verify coordinates of placed obstacles match expected world positions
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.05, 0.05)));
  EXPECT_FALSE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.85, 0.05)));

  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.05, 0.65)));
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.85, 0.65)));
}

TEST_F(StaticMapHandlerTest, IgnoresInternalCellsInComplexMap) {
  // Test that hidden obstacles (cells surrounded by other obstacles) do not generate
  // unnecessary collision bodies that will never be hit. Only cells with at least one
  // free neighbor in any of the 8 directions should create collision bodies.
  // Map layout (X = occupied, . = free):
  //   0 1 2 3 4 5 6 7 8

  const char* str_map =  // Rows (y from top to bottom)
      "XX....X.."        // y = 0.65
      "XX...XXX."        // y = 0.55
      "......X.."        // y = 0.45
      "..XXX...."        // y = 0.35
      "..XXX...."        // y = 0.25
      "..XXX...."        // y = 0.15
      "X........";       // y = 0.05
  // Columns
  //   ||||||||>      x = 0.85
  //   |||||||>       x = 0.75
  //   ||||||>        x = 0.65
  //   |||||>         x = 0.55
  //   ||||>          x = 0.45
  //   |||>           x = 0.35
  //   ||>            x = 0.25
  //   |>             x = 0.15
  //   >              x = 0.05

  constexpr std::size_t width = 9;
  constexpr std::size_t height = 7;
  constexpr double resolution = 0.1;  // 0.1 meter per cell

  auto map = create_map_from_string(str_map, width, height, resolution);

  // Create StaticMapHandler - should only create bodies for edge cells
  StaticMapHandler map_handler(map, manager_);

  // Internal cells completely surrounded by obstacles should not create collision bodies
  EXPECT_FALSE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.05, 0.65)));
  EXPECT_FALSE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.35, 0.25)));
  EXPECT_FALSE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.05, 0.65)));

  // This cell has a diagonal free neighbor so it should be an edge and have a body
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.65, 0.55)));

  // Edge cells with free neighbors should have collision bodies
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.05, 0.05)));
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.15, 0.55)));
  EXPECT_TRUE(has_obstacle_at(map.origin.translation() + Eigen::Vector2d(0.75, 0.55)));
}

}  // namespace
}  // namespace svdsim_server
