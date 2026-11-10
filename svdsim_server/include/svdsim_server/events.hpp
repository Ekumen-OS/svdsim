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

#ifndef SVDSIM_SERVER_EVENTS_HPP_
#define SVDSIM_SERVER_EVENTS_HPP_

// Non-ROS dependencies
#include <memory>

#include <eigen3/Eigen/Core>
#include <sophus/se3.hpp>

// Project headers
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

// Forward declaration
class TrajectoryBuffer;

/**
 * @brief Velocity reference command event
 *
 * Event emitted when a velocity reference command is issued to a vehicle.
 */
struct VelocityCommandEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Velocity reference value
  double value;
};

/**
 * @brief Steering reference command event
 *
 * Event emitted when a steering reference command is issued to a vehicle.
 */
struct SteeringCommandEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Steering reference value
  double value;
};

/**
 * @brief Ping received event
 *
 * Event emitted when a pong message is received from a vehicle.
 */
struct PingCommandReceived {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Sequence number from ping message
  uint64_t sequence_number;
};

/**
 * @brief Axle acceleration event
 *
 * Event emitted when axle acceleration is computed for the vehicular model.
 */
struct AccelerationCommandEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Acceleration value
  double value;
};

/**
 * @brief Steering rate command event
 *
 * Event emitted when a steering rate command is issued to the vehicular model.
 */
struct SteeringRateCommandEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Steering rate value
  double value;
};

/**
 * @brief Vehicle state summary event
 *
 * Event emitted when a vehicle's state is updated.
 */
struct VehicleStateSummaryEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Vehicle state summary
  VehicleStateSummary state;
};

/**
 * @brief Physics step pose update event
 *
 * Emitted after each physics step to update vehicle poses.
 */
struct PhysicsStepPoseUpdateEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Pose2D
  Sophus::SE2d vehicle_pose;
};

/**
 * @brief Wheel encoder event
 *
 * Event emitted when wheel encoder values are calculated.
 */
struct WheelEncoderEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Rear left wheel encoder pulse count
  int32_t rear_left_pulse_count;
  /// @brief Rear right wheel encoder pulse count
  int32_t rear_right_pulse_count;
};

/**
 * @brief Lidar sensor scan event
 *
 * Event emitted when a lidar scan is performed.
 */
struct LidarScanEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Minimum detection range [m]
  double min_range;
  /// @brief Maximum detection range [m]
  double max_range;
  /// @brief Horizontal field of view [rad]
  double horizontal_fov;
  /// @brief Number of points per scan
  int points_per_scan;
  /// @brief Lidar scan rate [Hz]
  double scan_rate;
  /// @brief Vector of range measurements [m], one per scan point
  std::vector<double> ranges;
};

/**
 * @brief IMU sensor sample event
 *
 * Event emitted when an IMU sensor is sampled.
 */
struct ImuSampleEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Accelerometer measurement in IMU frame [m/s^2]
  Eigen::Vector3d accelerometer;
  /// @brief Gyroscope measurement in IMU frame [rad/s]
  Eigen::Vector3d gyroscope;
  /// @brief Magnetometer measurement in IMU frame [T]
  Eigen::Vector3d magnetometer;
};

/**
 * @brief Steering servo sensor event
 *
 * Event emitted when steering servo sensor feedback is measured.
 */
struct SteeringFeedbackEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Steering angle value [rad]
  double value;
};

/**
 * @brief Ground truth vehicle state update event
 *
 * Event emitted when the ground truth state of a vehicle is available.
 */
struct GroundtruthUpdateEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Pose of base_link in the world frame.
  /// base_link is located at the midpoint of the rear axle at ground level.
  Sophus::SE2d pose;
  /// @brief Linear velocity of base_link expressed in the vehicle frame [m/s]
  double vx;
  double vy;
  /// @brief Angular velocity of base_link expressed in the vehicle frame [rad/s]
  double yaw_rate;
};

/**
 * @brief Reset vehicle state event
 *
 * Event emitted when a vehicle's state should be reset to initial conditions.
 * Systems with stateful components should reinitialize them upon receiving this event.
 */
struct ResetVehicleStateEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Reset pose (X, Y, Yaw) for the vehicle
  Sophus::SE2d reset_pose;
};

/**
 * @brief Sparring vehicle pose marker event
 *
 * Event emitted when sparring vehicle pose visualization data is available.
 */
struct SparringPoseMarkerEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Vehicle pose in world frame
  Sophus::SE2d pose;
};

/**
 * @brief Sparring vehicle lookahead target marker event
 *
 * Event emitted when sparring vehicle lookahead target visualization data is available.
 */
struct SparringLookaheadMarkerEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Target point position in world frame [m]
  double target_x;
  double target_y;
};

/**
 * @brief Collision count event
 *
 * Event emitted each physics cycle reporting the cumulative counts of collision
 * transitions (no-collision -> collision) for a vehicle, separated by collision type.
 * Also reports current collision states. Published on the ROS topic /collision_count
 * as a custom message with all fields.
 */
struct CollisionStateUpdateEvent {
  /// @brief target entity
  EntityId entity_id;
  /// @brief Total count of obstacle collision transitions (false -> true)
  uint64_t obstacle_collision_count;
  /// @brief Total count of vehicle collision transitions (false -> true)
  uint64_t vehicle_collision_count;
  /// @brief True when the vehicle collider overlaps any OBSTACLE body
  bool in_collision_with_obstacle;
  /// @brief True when the vehicle collider overlaps any other VEHICLE body
  bool in_collision_with_vehicle;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_EVENTS_HPP_
