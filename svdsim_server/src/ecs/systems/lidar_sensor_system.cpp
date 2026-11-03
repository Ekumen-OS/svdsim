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

#include "svdsim_server/ecs/systems/lidar_sensor_system.hpp"

#include "svdsim_server/ecs/components/lidar_sensor_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

namespace svdsim_server {

LidarSensorSystem::LidarSensorSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {
  // Subscribe to physics step pose update events
  dispatcher_.sink<PhysicsStepPoseUpdateEvent>().connect<&LidarSensorSystem::on_physics_step_pose_update>(*this);
  // Subscribe to reset vehicle state events
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&LidarSensorSystem::on_reset_vehicle_state>(*this);
}

void LidarSensorSystem::on_physics_step_pose_update(const PhysicsStepPoseUpdateEvent& event) {
  // Check if entity has a LidarSensorComponent
  if (!registry_.all_of<LidarSensorComponent>(event.entity_id)) {
    return;
  }

  // Check if trajectory buffer exists for this entity
  auto it = trajectory_buffers_.find(event.entity_id);
  if (it == trajectory_buffers_.end()) {
    // Create a new trajectory buffer
    auto buffer = std::make_shared<TrajectoryBuffer>(1000);
    // Initialize with the current pose
    buffer->reset(0, event.vehicle_pose);
    trajectory_buffers_[event.entity_id] = buffer;
  } else {
    // Add the new pose to the existing buffer
    it->second->add(0, event.vehicle_pose);
  }
}

void LidarSensorSystem::on_reset_vehicle_state(const ResetVehicleStateEvent& event) {
  // Clear trajectory buffer for the entity
  trajectory_buffers_.erase(event.entity_id);
}

DiscreteTime LidarSensorSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  auto view = registry_.view<LidarSensorComponent>();

  for (auto entity : view) {
    auto buffer_it = trajectory_buffers_.find(entity);
    if (buffer_it == trajectory_buffers_.end()) {
      continue;
    }

    auto& lidar_component = view.get<LidarSensorComponent>(entity);
    const auto& sensor = lidar_component.sensor();

    auto event = LidarScanEvent{};
    event.entity_id = entity;
    event.min_range = sensor.min_range();
    event.max_range = sensor.max_range();
    event.horizontal_fov = sensor.horizontal_fov();
    event.points_per_scan = sensor.points_per_scan();
    event.scan_rate = sensor.scan_rate();

    event.ranges = sensor.scan(*buffer_it->second);

    dispatcher_.trigger(event);
  }

  // Return next activation time (current + period)
  return current_time + period_;
}

}  // namespace svdsim_server
