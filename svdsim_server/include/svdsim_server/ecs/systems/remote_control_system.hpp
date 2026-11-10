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

#ifndef SVDSIM_SERVER_ECS_SYSTEMS_REMOTE_CONTROL_SYSTEM_HPP_
#define SVDSIM_SERVER_ECS_SYSTEMS_REMOTE_CONTROL_SYSTEM_HPP_

// Standard library
#include <map>
#include <set>

// Non-ROS dependencies
#include <entt/entt.hpp>

// Project headers
#include "svdsim_server/datatypes.hpp"
#include "svdsim_server/ecs/components/drive_model_component.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"
#include "svdsim_server/events.hpp"

#include "sim_messages.pb.h"

namespace svdsim_server {

/**
 * @brief System that handles driver control commands
 *
 * Subscribes to control command events and processes them to update
 * vehicle control components.
 */
class RemoteControlSystem : public SystemInterface {
 public:
  /**
   * @brief Construct a new Driver Control System
   *
   * @param registry Reference to the EnTT registry
   * @param dispatcher Reference to the EnTT dispatcher for event subscription
   * @param period Distance in discrete time steps between activations
   */
  explicit RemoteControlSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period);

  /**
   * @brief Update the system for the current frame
   *
   * Processes all pending control commands.
   *
   * @return DiscreteTime representing the timestamp of the next desired activation
   */
  DiscreteTime update() override;

 private:
  /// @brief Reference to the entity registry
  entt::registry& registry_;

  /// @brief Reference to the event dispatcher
  entt::dispatcher& dispatcher_;

  /// @brief Period between activations in discrete time steps
  std::size_t period_;

  /// @brief Map of pending velocity reference commands per entity
  std::map<EntityId, VelocityCommandEvent> velocity_commands_;

  /// @brief Map of pending steering reference commands per entity
  std::map<EntityId, SteeringCommandEvent> steering_commands_;

  /// @brief Callbacks that have already been configured
  std::set<EntityId> configured_callbacks_;

  /// @brief Callback for processing incoming control messages
  void callback(EntityId entity_id, const SimDataMessage& msg);

  /// @brief Handler for wheel encoder events
  void on_wheel_encoder_event(const WheelEncoderEvent& event);

  /// @brief Handler for lidar scan events
  void on_lidar_scan_event(const LidarScanEvent& event);

  /// @brief Handler for IMU sample events
  void on_imu_sample_event(const ImuSampleEvent& event);

  /// @brief Handler for steering servo sensor events
  void on_steering_feedback_event(const SteeringFeedbackEvent& event);

  /// @brief Handler for ping received events
  void on_ping_received(const PingCommandReceived& event);

  /// @brief Handler for reset vehicle state events
  void on_reset_vehicle_state(const ResetVehicleStateEvent& event);

  /// @brief Handler for ground truth update events
  void on_groundtruth_update_event(const GroundtruthUpdateEvent& event);

  /// @brief Handler for sparring pose marker events
  void on_sparring_pose_marker_event(const SparringPoseMarkerEvent& event);

  /// @brief Handler for sparring lookahead marker events
  void on_sparring_lookahead_marker_event(const SparringLookaheadMarkerEvent& event);

  /// @brief Handler for collision count events
  void on_collision_count_event(const CollisionStateUpdateEvent& event);

  /// @brief Send broadcast timestamp message to an entity
  void broadcast_timestamp(EntityId entity_id);
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_SYSTEMS_REMOTE_CONTROL_SYSTEM_HPP_
