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

#include "svdsim_server/ecs/systems/odometry_sensor_system.hpp"

#include "svdsim_server/ecs/components/emulated_steering_servo_sensor_component.hpp"
#include "svdsim_server/ecs/components/true_steering_servo_sensor_component.hpp"
#include "svdsim_server/ecs/components/wheel_encoder_sensor_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

namespace svdsim_server {

OdometrySensorSystem::OdometrySensorSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {
  dispatcher_.sink<VehicleStateSummaryEvent>().connect<&OdometrySensorSystem::on_vehicle_state_summary>(*this);
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&OdometrySensorSystem::on_reset_vehicle_state>(*this);
  dispatcher_.sink<SteeringCommandEvent>().connect<&OdometrySensorSystem::on_steering_command>(*this);
}

void OdometrySensorSystem::on_vehicle_state_summary(const VehicleStateSummaryEvent& event) {
  // Store state for entities with wheel encoder or steering servo sensors
  if (registry_.all_of<WheelEncoderSensorComponent>(event.entity_id) ||
      registry_.all_of<TrueSteeringServoSensorComponent>(event.entity_id)) {
    vehicle_states_[event.entity_id] = event.state;
  }
}

void OdometrySensorSystem::on_reset_vehicle_state(const ResetVehicleStateEvent& event) {
  // Clear stored vehicle state for the entity
  vehicle_states_.erase(event.entity_id);
  steering_commands_.erase(event.entity_id);
}

void OdometrySensorSystem::on_steering_command(const SteeringCommandEvent& event) {
  // Only emulated steering feedback entities need the commanded value stored
  if (registry_.all_of<WheelEncoderSensorComponent, EmulatedSteeringServoSensorComponent>(event.entity_id)) {
    steering_commands_[event.entity_id] = event;
  }
}

DiscreteTime OdometrySensorSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  // Process wheel encoder sensors
  auto encoder_view = registry_.view<WheelEncoderSensorComponent>();
  for (auto entity : encoder_view) {
    auto state_it = vehicle_states_.find(entity);
    if (state_it == vehicle_states_.end()) {
      continue;
    }

    const auto& encoder_component = encoder_view.get<WheelEncoderSensorComponent>(entity);
    const auto& state = state_it->second;

    const int32_t rear_left_pulse_count = encoder_component.rear_left_encoder().calculate_encoder_output(
        state.rear_left_wheel_cycles, state.rear_left_wheel_angle);

    const int32_t rear_right_pulse_count = encoder_component.rear_right_encoder().calculate_encoder_output(
        state.rear_right_wheel_cycles, state.rear_right_wheel_angle);

    auto event = WheelEncoderEvent{};
    event.entity_id = entity;
    event.rear_left_pulse_count = rear_left_pulse_count;
    event.rear_right_pulse_count = rear_right_pulse_count;
    dispatcher_.trigger(event);
  }

  // Process steering servo sensors
  auto steering_view = registry_.view<TrueSteeringServoSensorComponent>();
  for (auto entity : steering_view) {
    auto state_it = vehicle_states_.find(entity);
    if (state_it == vehicle_states_.end()) {
      continue;
    }

    const auto& state = state_it->second;

    auto event = SteeringFeedbackEvent{};
    event.entity_id = entity;
    event.value = state.steering_angle;
    dispatcher_.trigger(event);
  }

  // Process emulated steering servo sensors, reporting the last commanded
  // steering value as feedback instead of the true simulated angle
  auto emulated_steering_view = registry_.view<EmulatedSteeringServoSensorComponent>();
  for (auto entity : emulated_steering_view) {
    auto command_it = steering_commands_.find(entity);
    if (command_it == steering_commands_.end()) {
      continue;
    }

    auto event = SteeringFeedbackEvent{};
    event.entity_id = entity;
    event.value = command_it->second.value;
    dispatcher_.trigger(event);
  }

  // Return next activation time (current + period)
  return current_time + period_;
}

}  // namespace svdsim_server
