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

#ifndef MONACOF1_SIM_SERVER_ECS_SYSTEMS_LIDAR_SENSOR_SYSTEM_HPP_
#define MONACOF1_SIM_SERVER_ECS_SYSTEMS_LIDAR_SENSOR_SYSTEM_HPP_

// Standard library
#include <map>
#include <memory>

// Non-ROS dependencies
#include <entt/entt.hpp>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/events.hpp"
#include "svdsim_server/trajectory_buffer.hpp"

namespace svdsim_server {

/**
 * @brief System that handles lidar sensor simulation
 *
 * Subscribes to physics step pose update events and maintains a trajectory buffer
 * for each entity to support motion-compensated lidar scanning.
 */
class LidarSensorSystem : public SystemInterface {
 public:
  /**
   * @brief Construct a new Lidar Sensor System
   *
   * @param registry Reference to the EnTT registry
   * @param dispatcher Reference to the EnTT dispatcher for event subscription
   * @param period Distance in discrete time steps between activations
   */
  explicit LidarSensorSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period);

  /**
   * @brief Update the system for the current frame
   *
   * Performs lidar ray casting and updates sensor components.
   *
   * @return DiscreteTime representing the timestamp of the next desired activation
   */
  DiscreteTime update() override;

 private:
  /**
   * @brief Callback for physics step pose update events
   *
   * Creates trajectory buffer if needed and adds the new pose.
   *
   * @param event The physics step pose update event
   */
  void on_physics_step_pose_update(const PhysicsStepPoseUpdateEvent& event);

  /// @brief Callback for reset vehicle state events
  ///
  /// Clears the trajectory buffer for the entity.
  ///
  /// @param event The reset vehicle state event
  void on_reset_vehicle_state(const ResetVehicleStateEvent& event);

  /// @brief Reference to the entity registry
  entt::registry& registry_;

  /// @brief Reference to the event dispatcher
  entt::dispatcher& dispatcher_;

  /// @brief Period between activations in discrete time steps
  std::size_t period_;

  /// @brief Map of trajectory buffers per entity
  std::map<EntityId, std::shared_ptr<TrajectoryBuffer>> trajectory_buffers_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_ECS_SYSTEMS_LIDAR_SENSOR_SYSTEM_HPP_
