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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_COLLISION_COUNTER_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_COLLISION_COUNTER_COMPONENT_HPP_

#include <cstdint>

namespace svdsim_server {

/**
 * @brief Component that tracks collision counts for a vehicle entity.
 *
 * This component maintains separate counts for obstacle collisions (walls, static obstacles)
 * and vehicle collisions (other cars). It tracks both the count of collision events
 * (discrete transitions from no-collision to collision) and the previous collision states.
 * Used by CollisionCountSystem to detect state transitions and increment counters only
 * when an entity transitions from not colliding to colliding.
 */
struct CollisionCounterComponent {
  /// @brief Default constructor: no previous collisions, counts start at 0
  CollisionCounterComponent()
      : previous_in_collision_with_obstacle(false),
        previous_in_collision_with_vehicle(false),
        obstacle_collision_count(0),
        vehicle_collision_count(0) {}

  /// @brief Explicit constructor
  /// @param prev_obstacle_collision The previous obstacle collision state
  /// @param prev_vehicle_collision The previous vehicle collision state
  /// @param obstacle_count The current obstacle collision count
  /// @param vehicle_count The current vehicle collision count
  explicit CollisionCounterComponent(
      bool prev_obstacle_collision,
      bool prev_vehicle_collision,
      std::uint64_t obstacle_count = 0,
      std::uint64_t vehicle_count = 0)
      : previous_in_collision_with_obstacle(prev_obstacle_collision),
        previous_in_collision_with_vehicle(prev_vehicle_collision),
        obstacle_collision_count(obstacle_count),
        vehicle_collision_count(vehicle_count) {}

  /// @brief Previous obstacle collision state (from the last update cycle)
  bool previous_in_collision_with_obstacle;

  /// @brief Previous vehicle collision state (from the last update cycle)
  bool previous_in_collision_with_vehicle;

  /// @brief Total number of obstacle collision transitions (false -> true)
  std::uint64_t obstacle_collision_count;

  /// @brief Total number of vehicle collision transitions (false -> true)
  std::uint64_t vehicle_collision_count;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_COLLISION_COUNTER_COMPONENT_HPP_
