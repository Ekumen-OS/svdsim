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

#ifndef SVDSIM_SERVER_COLLISION_STATIC_MAP_HANDLER_HPP_
#define SVDSIM_SERVER_COLLISION_STATIC_MAP_HANDLER_HPP_

#include <memory>
#include <vector>

#include "svdsim_server/collision/body_handle.hpp"
#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

/// @brief Manages static map obstacles in the collision system.
///
/// This class takes an occupancy map and creates collision bodies only for obstacle
/// edge cells (skin), significantly reducing the number of collision objects while
/// maintaining collision detection accuracy. An obstacle cell is considered an edge
/// cell if it is occupied and has at least one free neighbor among its 8-neighbors.
class StaticMapHandler {
 public:
  /// @brief Constructs a StaticMapHandler from an occupancy map.
  /// @param map The static map containing obstacle data.
  /// @param manager The collider manager used to create collision bodies.
  StaticMapHandler(const StaticMap& map, std::shared_ptr<ColliderManager> manager);

  /// Deleted copy constructor to prevent copying.
  StaticMapHandler(const StaticMapHandler&) = delete;

  /// Deleted copy assignment operator to prevent copying.
  StaticMapHandler& operator=(const StaticMapHandler&) = delete;

 private:
  /// Creates a buffer marking obstacle edge cells.
  /// @param map The static map to process.
  /// @return Boolean buffer where true indicates an obstacle edge cell.
  static std::vector<bool> create_edge_buffer(const StaticMap& map);

  /// Creates collision bodies for obstacle edge cells.
  /// @param map The static map containing map metadata.
  /// @param edge_buffer Buffer indicating which cells are obstacle edges.
  /// @param manager The collider manager to create bodies with.
  /// @return Vector of body handles for created obstacles.
  static std::vector<std::shared_ptr<BodyHandle>> create_obstacle_bodies(
      const StaticMap& map,
      std::shared_ptr<ColliderManager> manager);

  /// Stored handles to collision bodies for lifecycle management
  std::vector<std::shared_ptr<BodyHandle>> obstacle_handles_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_COLLISION_STATIC_MAP_HANDLER_HPP_
