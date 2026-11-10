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

#ifndef SVDSIM_SERVER_ECS_SYSTEMS_COLLISION_COUNT_SYSTEM_HPP_
#define SVDSIM_SERVER_ECS_SYSTEMS_COLLISION_COUNT_SYSTEM_HPP_

// Standard library
#include <map>

// Non-ROS dependencies
#include <entt/entt.hpp>

// Project headers
#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/events.hpp"

namespace svdsim_server {

/**
 * @brief System that detects and counts vehicle collisions.
 *
 * Each activation cycle, iterates all vehicle entities that have a
 * VehicleColliderComponent and queries the ColliderManager for overlapping
 * bodies. Separately tracks collisions with obstacles (OBSTACLE category) and
 * other vehicles (VEHICLE category, excluding self).
 *
 * For each vehicle entity, this system:
 * - Tracks collision state transitions via CollisionCounterComponent
 * - Increments obstacle counter on no-collision -> collision transitions
 * - Increments vehicle counter on no-collision -> collision transitions
 * - Dispatches CollisionStateUpdateEvent with counts and current states (every cycle)
 *
 * Events are dispatched every activation cycle for all vehicles, enabling
 * continuous monitoring and per-vehicle collision tracking through the
 * RemoteControlSystem -> Bridge -> ROS topic pipeline.
 */
class CollisionCountSystem : public SystemInterface {
 public:
  /**
   * @brief Construct a CollisionCountSystem.
   *
   * @param registry            Reference to the EnTT entity registry.
   * @param dispatcher          Reference to the EnTT event dispatcher.
   * @param collider_manager    Shared pointer to the Box2D collision manager.
   * @param period              Activation period in simulation ticks.
   */
  CollisionCountSystem(
      entt::registry& registry,
      entt::dispatcher& dispatcher,
      std::shared_ptr<ColliderManager> collider_manager,
      std::size_t period);

  /**
   * @brief Run one activation cycle.
   *
   * Checks each vehicle's collider against all bodies in the Box2D world and
   * dispatches CollisionStateUpdateEvent per vehicle entity with separate counts and
   * states for obstacle and vehicle collisions.
   *
   * @return DiscreteTime of the next desired activation (current tick + period).
   */
  DiscreteTime update() override;

 private:
  /// @brief Reference to the entity registry.
  entt::registry& registry_;

  /// @brief Reference to the event dispatcher.
  entt::dispatcher& dispatcher_;

  /// @brief Shared collision manager (owns the Box2D world).
  std::shared_ptr<ColliderManager> collider_manager_;

  /// @brief Activation period in simulation ticks.
  std::size_t period_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_SYSTEMS_COLLISION_COUNT_SYSTEM_HPP_
