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

#ifndef MONACOF1_SIM_SERVER_ECS_SYSTEMS_GROUNDTRUTH_SENSOR_SYSTEM_HPP_
#define MONACOF1_SIM_SERVER_ECS_SYSTEMS_GROUNDTRUTH_SENSOR_SYSTEM_HPP_

// Standard library
#include <map>

// Non-ROS dependencies
#include <entt/entt.hpp>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/events.hpp"

namespace svdsim_server {

/**
 * @brief System that publishes ground truth data from simulated vehicles
 *
 * Subscribes to vehicle state summary events and publishes ground truth events
 * for entities with ground truth sensor components (GroundtruthSensorComponent).
 */
class GroundtruthSensorSystem : public SystemInterface {
 public:
  /**
   * @brief Construct a new Ground Truth Sensor System
   *
   * @param registry Reference to the EnTT registry
   * @param dispatcher Reference to the EnTT dispatcher for event subscription
   * @param period Distance in discrete time steps between activations
   */
  explicit GroundtruthSensorSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period);

  /**
   * @brief Update the system for the current frame
   *
   * Iterates over entities with ground truth sensor components, retrieves vehicle states,
   * and publishes corresponding ground truth update events.
   *
   * @return DiscreteTime representing the timestamp of the next desired activation
   */
  DiscreteTime update() override;

 private:
  /**
   * @brief Callback for vehicle state summary events
   *
   * Stores the latest vehicle state for each entity.
   *
   * @param event The vehicle state summary event
   */
  void on_vehicle_state_summary(const VehicleStateSummaryEvent& event);

  /// @brief Callback for reset vehicle state events
  ///
  /// Clears stored vehicle states for the entity.
  ///
  /// @param event The reset vehicle state event
  void on_reset_vehicle_state(const ResetVehicleStateEvent& event);

  /// @brief Reference to the entity registry
  entt::registry& registry_;

  /// @brief Reference to the event dispatcher
  entt::dispatcher& dispatcher_;

  /// @brief Period between activations in discrete time steps
  std::size_t period_;

  /// @brief Map of latest vehicle states per entity
  std::map<EntityId, VehicleStateSummary> vehicle_states_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_ECS_SYSTEMS_GROUNDTRUTH_SENSOR_SYSTEM_HPP_
