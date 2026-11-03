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

#include "svdsim_server/ecs/systems/sparring_control_system.hpp"

#include "svdsim_server/ecs/components/single_track_kinematic_model_component.hpp"
#include "svdsim_server/ecs/components/sparring_controller_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"
#include "svdsim_server/events.hpp"

namespace svdsim_server {

SparringControlSystem::SparringControlSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {}

DiscreteTime SparringControlSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  // Iterate over all entities with SparringControllerComponent
  auto view = registry_.view<SparringControllerComponent, SingleTrackKinematicModelComponent>();

  for (auto entity : view) {
    auto& sparring = view.get<SparringControllerComponent>(entity);
    auto& vehicle_model = view.get<SingleTrackKinematicModelComponent>(entity);

    // Get current vehicle pose from the kinematic model
    const auto& pose_2d = vehicle_model.model().pose();

    // Emit pose visualization event
    dispatcher_.trigger(SparringPoseMarkerEvent{entity, pose_2d});

    // Compute steering command using pure pursuit
    double steering = sparring.controller->compute_steering(pose_2d);

    // Emit velocity command event
    dispatcher_.trigger(VelocityCommandEvent{entity, sparring.constant_velocity});

    // Emit steering command event
    dispatcher_.trigger(SteeringCommandEvent{entity, steering});

    // Emit lookahead visualization event
    const auto& target_point = sparring.controller->get_last_target_point();
    dispatcher_.trigger(SparringLookaheadMarkerEvent{entity, target_point.x(), target_point.y()});
  }

  // Return next activation time (current + period)
  return current_time + period_;
}

}  // namespace svdsim_server
