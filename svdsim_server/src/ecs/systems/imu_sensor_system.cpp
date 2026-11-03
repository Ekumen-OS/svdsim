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

#include "svdsim_server/ecs/systems/imu_sensor_system.hpp"

#include "svdsim_server/ecs/components/imu_sensor_component.hpp"
#include "svdsim_server/ecs/components/suspension_simulation_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"
#include "svdsim_server/utils/imu_sensor_utils.hpp"

namespace svdsim_server {

ImuSensorSystem::ImuSensorSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {
  // Subscribe to physics step pose update events
  dispatcher_.sink<PhysicsStepPoseUpdateEvent>().connect<&ImuSensorSystem::on_physics_step_pose_update>(*this);
  // Subscribe to reset vehicle state events
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&ImuSensorSystem::on_reset_vehicle_state>(*this);
}

void ImuSensorSystem::on_physics_step_pose_update(const PhysicsStepPoseUpdateEvent& event) {
  // Check if entity has an IMUSensorComponent
  if (!registry_.all_of<IMUSensorComponent>(event.entity_id)) {
    return;
  }

  // Check if trajectory buffer exists for this entity
  auto it = trajectory_buffers_.find(event.entity_id);
  if (it == trajectory_buffers_.end()) {
    // Create a new trajectory buffer with enough capacity for derivatives
    // We need at least 3 samples for finite differences, but store more for robustness
    auto buffer = std::make_shared<TrajectoryBuffer>(100);
    // Initialize with the current pose
    buffer->reset(0, event.vehicle_pose);
    trajectory_buffers_[event.entity_id] = buffer;
  } else {
    // Add the new pose to the existing buffer
    it->second->add(0, event.vehicle_pose);
  }
}

void ImuSensorSystem::on_reset_vehicle_state(const ResetVehicleStateEvent& event) {
  // Clear trajectory buffer for the entity
  trajectory_buffers_.erase(event.entity_id);

  // Reset the IMU sensor state (biases)
  if (registry_.all_of<IMUSensorComponent>(event.entity_id)) {
    auto& imu_component = registry_.get<IMUSensorComponent>(event.entity_id);
    imu_component.sensor().reset();
  }
}

DiscreteTime ImuSensorSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();
  const double dt = clock.time_step();

  auto view = registry_.view<IMUSensorComponent, SuspensionSimulationComponent>();

  for (auto entity : view) {
    // Check if we have a trajectory buffer for this entity
    auto buffer_it = trajectory_buffers_.find(entity);
    if (buffer_it == trajectory_buffers_.end()) {
      continue;
    }

    const auto& buffer = *buffer_it->second;
    auto& imu_component = view.get<IMUSensorComponent>(entity);
    auto& suspension_component = view.get<SuspensionSimulationComponent>(entity);

    // Extract the three most recent poses from the buffer
    const auto pose_t0_opt = buffer.interpolate(0.0);  // Most recent (0 steps back)
    const auto pose_t1_opt = buffer.interpolate(1.0);  // 1 step back
    const auto pose_t2_opt = buffer.interpolate(2.0);  // 2 steps back

    // Prepare inputs for IMU sampling
    const ImuSamplingInputs inputs = prepare_imu_sampling_inputs(*pose_t0_opt, *pose_t1_opt, *pose_t2_opt, dt);

    // Compute body orientation from suspension dynamics
    const Sophus::SO3d C_vb = suspension_component.model().compute_body_orientation(
        inputs.T_iv, inputs.r_ddot_vi_i, inputs.omega_vi_v, inputs.omega_dot_vi_v);

    // Sample IMU sensors
    const Eigen::Vector3d accelerometer = imu_component.sensor().sample_accelerometer(
        inputs.T_iv, inputs.r_ddot_vi_i, inputs.omega_vi_v, inputs.omega_dot_vi_v, C_vb);

    const Eigen::Vector3d gyroscope = imu_component.sensor().sample_gyro(inputs.omega_vi_v, C_vb);

    const Eigen::Vector3d magnetometer = imu_component.sensor().sample_magnetometer(inputs.T_iv, C_vb);

    // Create and emit IMU sample event
    ImuSampleEvent event;
    event.entity_id = entity;
    event.accelerometer = accelerometer;
    event.gyroscope = gyroscope;
    event.magnetometer = magnetometer;

    dispatcher_.trigger(event);
  }

  // Schedule next update based on period
  return current_time + period_;
}

}  // namespace svdsim_server
