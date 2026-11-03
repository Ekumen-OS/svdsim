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

#include "svdsim_server/ecs/systems/collision_count_system.hpp"

#include "svdsim_server/collision/body_handle.hpp"
#include "svdsim_server/ecs/components/collision_counter_component.hpp"
#include "svdsim_server/ecs/components/vehicle_collider_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

namespace svdsim_server {

CollisionCountSystem::CollisionCountSystem(
    entt::registry& registry,
    entt::dispatcher& dispatcher,
    std::shared_ptr<ColliderManager> collider_manager,
    std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), collider_manager_(std::move(collider_manager)), period_(period) {}

DiscreteTime CollisionCountSystem::update() {
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  auto view = registry_.view<VehicleColliderComponent>();

  for (auto entity : view) {
    auto& collider_component = view.get<VehicleColliderComponent>(entity);

    bool in_collision_with_obstacle = false;
    bool in_collision_with_vehicle = false;
    const auto our_id = collider_component.collider().id();

    collider_manager_->check_collisions(collider_component.collider(), [&](const BodyHandle& hit) {
      if (hit.category() == BodyCategory::OBSTACLE) {
        in_collision_with_obstacle = true;
      } else if (hit.category() == BodyCategory::VEHICLE && hit.id() != our_id) {
        in_collision_with_vehicle = true;
      }
    });

    bool updated_state = false;

    // if there is no CollisionCounterComponent, set update_state to true so that we create one and dispatch an event
    if (!registry_.all_of<CollisionCounterComponent>(entity)) {
      updated_state = true;
    }

    // so that the next time a collision occurs, we can detect the transition
    // Get or create the collision counter component for this entity
    auto& counter = registry_.get_or_emplace<CollisionCounterComponent>(entity);

    // Detect state transitions and increment counters
    if (!counter.previous_in_collision_with_obstacle && in_collision_with_obstacle) {
      counter.obstacle_collision_count++;
    }
    if (!counter.previous_in_collision_with_vehicle && in_collision_with_vehicle) {
      counter.vehicle_collision_count++;
    }

    updated_state = updated_state                                                                   //
                    || (counter.previous_in_collision_with_obstacle != in_collision_with_obstacle)  //
                    || (counter.previous_in_collision_with_vehicle != in_collision_with_vehicle);

    // Update previous collision states for next cycle
    counter.previous_in_collision_with_obstacle = in_collision_with_obstacle;
    counter.previous_in_collision_with_vehicle = in_collision_with_vehicle;

    // if there were no changes, do not dispatch an event
    if (!updated_state) {
      continue;
    }

    // Dispatch collision event with all collision data
    dispatcher_.trigger(CollisionStateUpdateEvent{
        entity, counter.obstacle_collision_count, counter.vehicle_collision_count, in_collision_with_obstacle,
        in_collision_with_vehicle});
  }

  return current_time + period_;
}

}  // namespace svdsim_server
