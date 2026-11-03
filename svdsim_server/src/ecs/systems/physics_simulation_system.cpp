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

#include "svdsim_server/ecs/systems/physics_simulation_system.hpp"
#include "svdsim_server/ecs/components/drive_model_component.hpp"
#include "svdsim_server/ecs/components/single_track_kinematic_model_component.hpp"
#include "svdsim_server/ecs/components/vehicle_collider_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

#include <rclcpp/rclcpp.hpp>

namespace svdsim_server {

PhysicsSimulationSystem::PhysicsSimulationSystem(entt::registry& registry, entt::dispatcher& dispatcher, double dt)
    : registry_(registry), dispatcher_(dispatcher), dt_(dt) {
  dispatcher_.sink<VelocityCommandEvent>().connect<&PhysicsSimulationSystem::on_velocity_command>(*this);
  dispatcher_.sink<SteeringCommandEvent>().connect<&PhysicsSimulationSystem::on_steering_command>(*this);
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&PhysicsSimulationSystem::on_reset_vehicle_state>(*this);
}

void PhysicsSimulationSystem::on_velocity_command(const VelocityCommandEvent& event) {
  if (!registry_.all_of<DriveModelComponent, SingleTrackKinematicModelComponent>(event.entity_id)) {
    return;
  }
  velocity_commands_[event.entity_id] = event;
}

void PhysicsSimulationSystem::on_steering_command(const SteeringCommandEvent& event) {
  if (!registry_.all_of<DriveModelComponent, SingleTrackKinematicModelComponent>(event.entity_id)) {
    return;
  }
  steering_commands_[event.entity_id] = event;
}

void PhysicsSimulationSystem::on_reset_vehicle_state(const ResetVehicleStateEvent& event) {
  // Clear any pending commands for the entity
  velocity_commands_.erase(event.entity_id);
  steering_commands_.erase(event.entity_id);

  // Reset the drive model if the entity has one
  if (registry_.all_of<DriveModelComponent>(event.entity_id)) {
    auto& drive_component = registry_.get<DriveModelComponent>(event.entity_id);
    auto& drive_model = drive_component.model();
    drive_model.reset();
  }

  // Reset the kinematic model state if the entity has one
  if (registry_.all_of<SingleTrackKinematicModelComponent>(event.entity_id)) {
    auto& kinematic_component = registry_.get<SingleTrackKinematicModelComponent>(event.entity_id);
    auto& kinematic_model = kinematic_component.model();
    kinematic_model.reset(event.reset_pose);
  }

  // Reset the pose of the collider if the entity has one
  if (registry_.all_of<VehicleColliderComponent>(event.entity_id)) {
    auto& collider_component = registry_.get<VehicleColliderComponent>(event.entity_id);
    collider_component.collider().set_pose(event.reset_pose);
  }
}

void PhysicsSimulationSystem::update(std::size_t num_steps) {
  // Step through simulation time, advancing all entities one step at a time
  {
    auto view = registry_.view<DriveModelComponent, SingleTrackKinematicModelComponent, VehicleColliderComponent>();

    for (std::size_t i = 0; i < num_steps; ++i) {
      for (auto entity : view) {
        auto& drive_component = view.get<DriveModelComponent>(entity);
        auto& traction_component = view.get<SingleTrackKinematicModelComponent>(entity);
        auto& collider_component = view.get<VehicleColliderComponent>(entity);

        auto& drive_model = drive_component.model();
        auto& traction_model = traction_component.model();

        const auto steering_reference = steering_commands_[entity].value;
        const auto velocity_reference = velocity_commands_[entity].value;

        //
        // update drive model

        const auto drive_steering_rate =
            drive_model.update_steering_velocity(steering_reference, traction_model.steering());
        const auto drive_acceleration =
            drive_model.update_acceleration(velocity_reference, traction_model.virtual_encoder_velocity());

        // publish the drive output in case there are any consumers
        dispatcher_.trigger(SteeringRateCommandEvent{entity, drive_steering_rate});
        dispatcher_.trigger(AccelerationCommandEvent{entity, drive_acceleration});

        //
        // update traction model

        // u0: steering velocity [rad/s] - for now using steering angle directly
        // u1: longitudinal acceleration [m/s²]
        ControlInput input;
        input(0) = drive_steering_rate;  // Steering velocity
        input(1) = drive_acceleration;   // Acceleration

        traction_model.integrate(input, dt_);

        // publish physics step pose update event
        dispatcher_.trigger(PhysicsStepPoseUpdateEvent{entity, traction_model.pose()});

        //
        // update collider pose

        collider_component.collider().set_pose(traction_model.pose());
      }

      // Advance simulation clock after all entities have been updated for this step
      auto& sim_clock = registry_.ctx().get<SimulationClock>();
      sim_clock.tick();
    }
  }

  // Publish vehicle state summary events after all steps complete
  {
    auto view = registry_.view<DriveModelComponent, SingleTrackKinematicModelComponent>();
    for (auto entity : view) {
      auto& traction_component = view.get<SingleTrackKinematicModelComponent>(entity);
      auto& traction_model = traction_component.model();
      const auto state = traction_model.state_summary();
      dispatcher_.trigger(VehicleStateSummaryEvent{entity, state});
    }
  }
}

}  // namespace svdsim_server
