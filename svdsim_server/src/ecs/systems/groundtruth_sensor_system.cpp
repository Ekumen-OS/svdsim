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

#include "svdsim_server/ecs/systems/groundtruth_sensor_system.hpp"

#include "svdsim_server/ecs/components/groundtruth_sensor_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

namespace svdsim_server {

GroundtruthSensorSystem::GroundtruthSensorSystem(
    entt::registry& registry,
    entt::dispatcher& dispatcher,
    std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {
  dispatcher_.sink<VehicleStateSummaryEvent>().connect<&GroundtruthSensorSystem::on_vehicle_state_summary>(*this);
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&GroundtruthSensorSystem::on_reset_vehicle_state>(*this);
}

void GroundtruthSensorSystem::on_vehicle_state_summary(const VehicleStateSummaryEvent& event) {
  // Store state for entities with ground truth sensor component
  if (registry_.all_of<GroundtruthSensorComponent>(event.entity_id)) {
    vehicle_states_[event.entity_id] = event.state;
  }
}

void GroundtruthSensorSystem::on_reset_vehicle_state(const ResetVehicleStateEvent& event) {
  // Clear stored vehicle state for the entity
  vehicle_states_.erase(event.entity_id);
}

DiscreteTime GroundtruthSensorSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  // Process ground truth sensors
  auto groundtruth_view = registry_.view<GroundtruthSensorComponent>();
  for (auto entity : groundtruth_view) {
    auto state_it = vehicle_states_.find(entity);
    if (state_it == vehicle_states_.end()) {
      continue;
    }

    const auto& state = state_it->second;

    // Trigger ground truth update event
    auto event = GroundtruthUpdateEvent{};
    event.entity_id = entity;
    event.pose = state.base_link_pose;
    event.vx = state.vehicle_velocity.x();
    event.vy = state.vehicle_velocity.y();
    event.yaw_rate = state.vehicle_angular_velocity;
    dispatcher_.trigger(event);
  }

  // Return next activation time (current + period)
  return current_time + period_;
}

}  // namespace svdsim_server
