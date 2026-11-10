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

#ifndef SVDSIM_SERVER_DATATYPES_HPP_
#define SVDSIM_SERVER_DATATYPES_HPP_

// Standard library
#include <cstddef>

// Non-ROS dependencies
#include <Eigen/Core>
#include <entt/entt.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

// Project headers
#include "svdsim_server/discrete_time.hpp"

namespace svdsim_server {

/**
 * @brief Type alias for entity identifiers in the ECS
 *
 * Entity IDs are defined by the EnTT library.
 */
using EntityId = entt::entity;

/**
 * @brief Type alias for 3D vehicle velocities
 *
 * Represents linear velocity in the vehicle frame [vx, vy, vz] in m/s.
 */
using VehicleVelocity3D = Eigen::Vector3d;

/**
 * @brief Type alias for 3D linear accelerations
 *
 * Represents linear acceleration [ax, ay, az] in m/s².
 */
using LinearAcceleration3D = Eigen::Vector3d;

/**
 * @brief Type alias for 3D vehicle angular velocities
 *
 * Represents angular velocity [ωx, ωy, ωz] in rad/s.
 */
using VehicleAngularVelocity3D = Eigen::Vector3d;

/**
 * @brief Type alias for control input vector (2D)
 *
 * Represents control inputs for vehicle dynamics:
 * - u0: steering velocity [rad/s]
 * - u1: longitudinal acceleration [m/s²]
 */
using ControlInput = Eigen::Matrix<double, 2, 1>;

struct StaticMap {
  // resolution of the map, in meters per cell
  double resolution;

  // origin of the map in world coordinates
  Sophus::SE2d origin;

  // map width in number of cells
  std::size_t width;
  // map height in number of cells
  std::size_t height;

  // occupancy grid data: true = occupied, false = free. Buffer is width * height elements.
  // The origin of the map is at the lower-left corner (same as ROS convention), with the
  // Y axis pointing upwards and X axis pointing to the right.
  std::vector<bool> occupancy_grid;
};

/**
 * @brief Nominal vehicle state representation
 *
 * Contains all relevant state variables of the vehicle, referenced to the
 * base_link frame. The base_link is defined at the midpoint of the rear axle at
 * ground level.
 *
 * All kinematic quantities (velocities, accelerations, angular velocities) are
 * expressed in the vehicle frame with origin at base_link.
 */
struct VehicleStateSummary {
  /// @brief Pose of base_link in the global reference frame
  Sophus::SE2d base_link_pose;

  /// @brief Linear velocity of base_link expressed in vehicle frame [m/s]
  Eigen::Vector2d vehicle_velocity;

  /// @brief Angular velocity of base_link expressed in vehicle frame [rad/s]
  double vehicle_angular_velocity;

  /// @brief Steering angle of front wheels [rad]
  double steering_angle;

  /// @brief Angular position of the front wheel [rad]
  double rear_left_wheel_angle;

  /// @brief Cumulative full rotations of the front wheel
  int32_t rear_left_wheel_cycles;

  /// @brief Angular position of the rear wheel [rad]
  double rear_right_wheel_angle;

  /// @brief Cumulative full rotations of the rear wheel
  int32_t rear_right_wheel_cycles;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_DATATYPES_HPP_
