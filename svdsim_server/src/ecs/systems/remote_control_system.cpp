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

#include "svdsim_server/ecs/systems/remote_control_system.hpp"
#include "svdsim_server/ecs/components/remote_control_server_component.hpp"
#include "svdsim_server/ecs/services/simulation_clock.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
namespace svdsim_server {

RemoteControlSystem::RemoteControlSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period)
    : registry_(registry), dispatcher_(dispatcher), period_(period) {
  // Subscribe to wheel encoder events
  dispatcher_.sink<WheelEncoderEvent>().connect<&RemoteControlSystem::on_wheel_encoder_event>(*this);

  // Subscribe to lidar scan events
  dispatcher_.sink<LidarScanEvent>().connect<&RemoteControlSystem::on_lidar_scan_event>(*this);

  // Subscribe to IMU sample events
  dispatcher_.sink<ImuSampleEvent>().connect<&RemoteControlSystem::on_imu_sample_event>(*this);

  // Subscribe to steering servo sensor events
  dispatcher_.sink<SteeringFeedbackEvent>().connect<&RemoteControlSystem::on_steering_feedback_event>(*this);

  // Subscribe to ping received events
  dispatcher_.sink<PingCommandReceived>().connect<&RemoteControlSystem::on_ping_received>(*this);

  // Subscribe to reset vehicle state events
  dispatcher_.sink<ResetVehicleStateEvent>().connect<&RemoteControlSystem::on_reset_vehicle_state>(*this);

  // Subscribe to ground truth update events
  dispatcher_.sink<GroundtruthUpdateEvent>().connect<&RemoteControlSystem::on_groundtruth_update_event>(*this);

  // Subscribe to sparring visualization events
  dispatcher_.sink<SparringPoseMarkerEvent>().connect<&RemoteControlSystem::on_sparring_pose_marker_event>(*this);
  dispatcher_.sink<SparringLookaheadMarkerEvent>().connect<&RemoteControlSystem::on_sparring_lookahead_marker_event>(
      *this);

  // Subscribe to collision events
  dispatcher_.sink<CollisionStateUpdateEvent>().connect<&RemoteControlSystem::on_collision_count_event>(*this);
}

DiscreteTime RemoteControlSystem::update() {
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_ticks();

  auto view = registry_.view<RemoteControlServerComponent>();

  for (auto entity : view) {
    auto& remote_control_server_component = view.get<RemoteControlServerComponent>(entity);
    auto& remote_control_server = remote_control_server_component.server();

    // configure the callback only the first time for each entity
    if (configured_callbacks_.find(entity) == configured_callbacks_.end()) {
      remote_control_server.set_callback([this, entity](const SimDataMessage& msg) { callback(entity, msg); });
      configured_callbacks_.insert(entity);
    }

    // non-blocking poll and receive
    while (remote_control_server.poll_and_receive()) {
      // Process all incoming messages
    }

    // Broadcast timestamp message
    broadcast_timestamp(entity);
  }

  // Return next activation time (current + period)
  return current_time + period_;
}

void RemoteControlSystem::callback(EntityId entity_id, const SimDataMessage& msg) {
  // Check if this is a server inbound message
  if (msg.has_server_inbound()) {
    const auto& inbound = msg.server_inbound();

    // Process throttle command
    if (inbound.has_velocity_command()) {
      const auto velocity_value = inbound.velocity_command().value();
      dispatcher_.trigger(VelocityCommandEvent{entity_id, velocity_value});
    }

    // Process steering command
    if (inbound.has_steering_command()) {
      const auto steering_value = inbound.steering_command().value();
      dispatcher_.trigger(SteeringCommandEvent{entity_id, steering_value});
    }

    // Process pong message
    if (inbound.has_ping()) {
      const auto sequence_number = inbound.ping().sequence_number();
      dispatcher_.trigger(PingCommandReceived{entity_id, sequence_number});
    }

    // Process reset vehicle state command
    if (inbound.has_reset_vehicle_state()) {
      const auto& reset_msg = inbound.reset_vehicle_state();
      const double x = reset_msg.x();
      const double y = reset_msg.y();
      const double yaw = reset_msg.yaw();

      // Create SE2d pose from x, y, yaw
      const Sophus::SE2d reset_pose(Sophus::SO2d(yaw), Eigen::Vector2d(x, y));

      dispatcher_.trigger(ResetVehicleStateEvent{entity_id, reset_pose});
    }
  }
}

void RemoteControlSystem::on_wheel_encoder_event(const WheelEncoderEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish wheel encoder data message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* encoder_data = outbound->mutable_wheel_encoder_data();
  encoder_data->set_rear_left_pulse_count(event.rear_left_pulse_count);
  encoder_data->set_rear_right_pulse_count(event.rear_right_pulse_count);
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_lidar_scan_event(const LidarScanEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish lidar scan data message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* lidar_data = outbound->mutable_lidar_scan_data();
  lidar_data->set_min_range(event.min_range);
  lidar_data->set_max_range(event.max_range);
  lidar_data->set_horizontal_fov(event.horizontal_fov);
  lidar_data->set_points_per_scan(event.points_per_scan);
  lidar_data->set_scan_rate(event.scan_rate);
  for (const auto& range : event.ranges) {
    lidar_data->add_ranges(range);
  }
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_imu_sample_event(const ImuSampleEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish IMU sample data message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* imu_data = outbound->mutable_imu_sample_data();
  imu_data->set_accel_x(event.accelerometer.x());
  imu_data->set_accel_y(event.accelerometer.y());
  imu_data->set_accel_z(event.accelerometer.z());
  imu_data->set_gyro_x(event.gyroscope.x());
  imu_data->set_gyro_y(event.gyroscope.y());
  imu_data->set_gyro_z(event.gyroscope.z());
  imu_data->set_mag_x(event.magnetometer.x());
  imu_data->set_mag_y(event.magnetometer.y());
  imu_data->set_mag_z(event.magnetometer.z());
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_steering_feedback_event(const SteeringFeedbackEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish steering status message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* steering_feedback = outbound->mutable_steering_feedback();
  steering_feedback->set_value(event.value);
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_ping_received(const PingCommandReceived& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish pong message with the same sequence number
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* pong = outbound->mutable_pong();
  pong->set_sequence_number(event.sequence_number);
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_reset_vehicle_state([[maybe_unused]] const ResetVehicleStateEvent& event) {
  // No stateful components to reset in RemoteControlSystem
  // The configured callbacks set is intentionally not cleared as callbacks remain valid
}

void RemoteControlSystem::broadcast_timestamp(EntityId entity_id) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish broadcast timestamp message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  outbound->mutable_broadcast_timestamp();
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_groundtruth_update_event(const GroundtruthUpdateEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish ground truth data message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* ground_truth = outbound->mutable_ground_truth_data();
  ground_truth->set_x(event.pose.translation().x());
  ground_truth->set_y(event.pose.translation().y());
  ground_truth->set_yaw(event.pose.so2().log());
  ground_truth->set_vx(event.vx);
  ground_truth->set_vy(event.vy);
  ground_truth->set_yaw_rate(event.yaw_rate);
  (void)remote_control_server.publish(msg);
}

void RemoteControlSystem::on_sparring_pose_marker_event(const SparringPoseMarkerEvent& event) {
  // Broadcast sparring pose marker to all connected clients
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish pose marker message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* pose_marker = outbound->mutable_sparring_pose_marker();
  pose_marker->set_x(event.pose.translation().x());
  pose_marker->set_y(event.pose.translation().y());
  pose_marker->set_yaw(event.pose.so2().log());
  pose_marker->set_entity_id(static_cast<uint32_t>(event.entity_id));

  // Publish to all clients with RemoteControlServerComponent
  auto view = registry_.view<RemoteControlServerComponent>();
  for (auto entity : view) {
    auto& remote_control_server_component = view.get<RemoteControlServerComponent>(entity);
    auto& remote_control_server = remote_control_server_component.server();
    (void)remote_control_server.publish(msg);
  }
}

void RemoteControlSystem::on_sparring_lookahead_marker_event(const SparringLookaheadMarkerEvent& event) {
  // Broadcast sparring lookahead marker to all connected clients
  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish lookahead marker message
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* lookahead_marker = outbound->mutable_sparring_lookahead_marker();
  lookahead_marker->set_target_x(event.target_x);
  lookahead_marker->set_target_y(event.target_y);
  lookahead_marker->set_entity_id(static_cast<uint32_t>(event.entity_id));

  // Publish to all clients with RemoteControlServerComponent
  auto view = registry_.view<RemoteControlServerComponent>();
  for (auto entity : view) {
    auto& remote_control_server_component = view.get<RemoteControlServerComponent>(entity);
    auto& remote_control_server = remote_control_server_component.server();
    (void)remote_control_server.publish(msg);
  }
}

void RemoteControlSystem::on_collision_count_event(const CollisionStateUpdateEvent& event) {
  // Check if the entity has a RemoteControlServerComponent
  if (!registry_.all_of<RemoteControlServerComponent>(event.entity_id)) {
    return;
  }

  auto& remote_control_server_component = registry_.get<RemoteControlServerComponent>(event.entity_id);
  auto& remote_control_server = remote_control_server_component.server();

  // Get current simulation time from clock service
  const auto& clock = registry_.ctx().get<SimulationClock>();
  const auto current_time = clock.get_time();

  // Create and publish collision count data message with all fields
  SimDataMessage msg;
  auto* timestamp = msg.mutable_timestamp();
  timestamp->set_seconds(current_time.seconds);
  timestamp->set_fraction(current_time.fractional);
  auto* outbound = msg.mutable_server_outbound();
  auto* collision_count = outbound->mutable_collision_count_data();
  collision_count->set_obstacle_collision_count(event.obstacle_collision_count);
  collision_count->set_vehicle_collision_count(event.vehicle_collision_count);
  collision_count->set_in_collision_with_obstacle(event.in_collision_with_obstacle);
  collision_count->set_in_collision_with_vehicle(event.in_collision_with_vehicle);
  (void)remote_control_server.publish(msg);
}

}  // namespace svdsim_server
