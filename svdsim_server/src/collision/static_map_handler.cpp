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

#include "svdsim_server/collision/static_map_handler.hpp"

#include <array>
#include <cstddef>

namespace svdsim_server {

StaticMapHandler::StaticMapHandler(const StaticMap& map, std::shared_ptr<ColliderManager> manager) {
  StaticMap edges_map;
  edges_map.resolution = map.resolution;
  edges_map.origin = map.origin;
  edges_map.width = map.width;
  edges_map.height = map.height;
  edges_map.occupancy_grid = create_edge_buffer(map);

  obstacle_handles_ = create_obstacle_bodies(edges_map, manager);
}

std::vector<bool> StaticMapHandler::create_edge_buffer(const StaticMap& edges_map) {
  const auto width = edges_map.width;
  const auto height = edges_map.height;

  // Create obstacle skin buffer - only cells at the edge of obstacles
  std::vector<bool> obstacle_edge_cells(width * height, false);

  // 8-neighbor offsets (dx, dy)
  constexpr std::array<std::pair<int, int>, 8> neighbors = {
      {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};

  // Identify obstacle edge cells
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const auto idx = y * width + x;

      // Skip if cell is not occupied
      if (!edges_map.occupancy_grid[idx]) {
        continue;
      }

      // Check if any of the 8 neighbors is free
      const bool has_free_neighbor =
          std::any_of(neighbors.begin(), neighbors.end(), [x, y, width, height, &edges_map](const auto& offset) {
            const int nx = static_cast<int>(x) + offset.first;
            const int ny = static_cast<int>(y) + offset.second;

            // Out of bounds check
            if (nx < 0 || nx >= static_cast<int>(width) || ny < 0 || ny >= static_cast<int>(height)) {
              return false;
            }

            const auto neighbor_idx = static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
            return !edges_map.occupancy_grid[neighbor_idx];
          });

      // Mark as obstacle skin if it has at least one free neighbor
      if (has_free_neighbor) {
        obstacle_edge_cells[idx] = true;
      }
    }
  }

  return obstacle_edge_cells;
}

std::vector<std::shared_ptr<BodyHandle>> StaticMapHandler::create_obstacle_bodies(
    const StaticMap& map,
    std::shared_ptr<ColliderManager> manager) {
  const auto width = map.width;
  const auto height = map.height;
  const auto resolution = map.resolution;

  // count the number of obstacle edge cells and preserve space
  const auto obstacle_edge_count = std::count(map.occupancy_grid.begin(), map.occupancy_grid.end(), true);
  std::vector<std::shared_ptr<BodyHandle>> handles;
  handles.reserve(obstacle_edge_count);

  // Create collision bodies for obstacle skin cells
  // Cell corners relative to cell center (front_left and rear_right)
  const auto half_cell = resolution / 2.0;
  const auto front_left = std::tuple<float, float>{static_cast<float>(half_cell), static_cast<float>(half_cell)};
  const auto rear_right = std::tuple<float, float>{static_cast<float>(-half_cell), static_cast<float>(-half_cell)};

  // calculate the pose of the center of the cell relative to the cell corner
  const Sophus::SE2d center_pose_in_cell(Sophus::SO2d(0.0), Eigen::Vector2d(half_cell, half_cell));

  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      // the origin of the map is the map coordinate of the lower-left corner of the map buffer
      // therefore, to get the correct index, we need to flip the y coordinate
      const auto bottom_y = height - 1 - y;
      const auto idx = bottom_y * width + x;

      if (!map.occupancy_grid[idx]) {
        continue;
      }

      // calculate the pose of the cell from the map origin
      const Sophus::SE2d cell_pose_in_map(Sophus::SO2d(0.0), Eigen::Vector2d(x * resolution, y * resolution));

      // calculate cell center in world coordinates
      const Sophus::SE2d cell_center_in_world = map.origin * cell_pose_in_map * center_pose_in_cell;

      // Create rectangular body for this obstacle cell
      auto handle = manager->add_rectangular_body(
          cell_center_in_world, BodyCategory::OBSTACLE, front_left, rear_right,
          idx  // Use grid index as userdata
      );

      if (handle) {
        handles.push_back(std::move(handle));
      }
    }
  }

  return handles;
}

}  // namespace svdsim_server
