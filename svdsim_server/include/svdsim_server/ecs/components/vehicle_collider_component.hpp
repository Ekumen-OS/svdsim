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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_VEHICLE_COLLIDER_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_VEHICLE_COLLIDER_COMPONENT_HPP_

#include <memory>

#include "svdsim_server/collision/body_handle.hpp"

namespace svdsim_server {

/**
 * @brief Component that holds a vehicle's collision body handle
 *
 * This component associates a vehicle entity with its collision representation
 * in the physics/collision system.
 */
struct VehicleColliderComponent {
  /**
   * @brief Construct a new Vehicle Collider Component
   *
   * @param collider_handle Shared pointer to the body handle managing the vehicle's collider
   */
  explicit VehicleColliderComponent(std::shared_ptr<BodyHandle> collider_handle)
      : collider_handle(std::move(collider_handle)) {}

  BodyHandle& collider() { return *collider_handle; }

  /// Handle to the vehicle's collision body
  std::shared_ptr<BodyHandle> collider_handle;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_VEHICLE_COLLIDER_COMPONENT_HPP_
