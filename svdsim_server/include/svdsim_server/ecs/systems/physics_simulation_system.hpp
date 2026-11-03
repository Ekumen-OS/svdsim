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

#ifndef MONACOF1_SIM_SERVER_ECS_SYSTEMS_PHYSICS_SIMULATION_SYSTEM_HPP_
#define MONACOF1_SIM_SERVER_ECS_SYSTEMS_PHYSICS_SIMULATION_SYSTEM_HPP_

// Standard library
#include <map>

// Non-ROS dependencies
#include <entt/entt.hpp>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/components/single_track_kinematic_model_component.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/events.hpp"

namespace svdsim_server {

/**
 * @brief System that handles traction model updates
 *
 * Subscribes to axle steering and acceleration events and uses them to
 * update the kinematic models by assembling control inputs.
 */
class PhysicsSimulationSystem {
 public:
  /**
   * @brief Construct a new Traction Model System
   *
   * @param registry Reference to the EnTT registry
   * @param dispatcher Reference to the EnTT dispatcher for event subscription
   * @param dt Time step for integration [s]
   */
  explicit PhysicsSimulationSystem(entt::registry& registry, entt::dispatcher& dispatcher, double dt);

  /**
   * @brief Update the system for the current frame
   *
   * Processes all entities with kinematic models and integrates them forward in time.
   *
   * @param num_steps Number of integration steps to perform
   */
  void update(std::size_t num_steps);

 private:
  /// @brief Reference to the entity registry
  entt::registry& registry_;

  /// @brief Reference to the event dispatcher
  entt::dispatcher& dispatcher_;

  /// @brief Time step for integration [s]
  double dt_;

  /// @brief Map of pending velocity reference commands per entity
  std::map<EntityId, VelocityCommandEvent> velocity_commands_;

  /// @brief Map of pending steering reference commands per entity
  std::map<EntityId, SteeringCommandEvent> steering_commands_;

  /// @brief Handle velocity reference command event
  /// @param event
  void on_velocity_command(const VelocityCommandEvent& event);

  /// @brief Handle steering reference command event
  /// @param event
  void on_steering_command(const SteeringCommandEvent& event);

  /// @brief Handle reset vehicle state event
  /// @param event
  void on_reset_vehicle_state(const ResetVehicleStateEvent& event);
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_ECS_SYSTEMS_PHYSICS_SIMULATION_SYSTEM_HPP_
