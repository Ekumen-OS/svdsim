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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_SPARRING_CONTROLLER_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_SPARRING_CONTROLLER_COMPONENT_HPP_

#include <memory>

#include <svdsim_server/utils/pure_pursuit_controller.hpp>

namespace svdsim_server {

/**
 * @brief Component holding the sparring vehicle controller
 *
 * This component manages the autonomous path following controller for
 * sparring vehicles.
 */
struct SparringControllerComponent {
  /// Pure pursuit controller for path following
  std::unique_ptr<svdsim_server::PurePursuitController> controller;

  /// Constant velocity for the sparring vehicle [m/s]
  double constant_velocity;

  /**
   * @brief Construct a new Sparring Controller Component
   *
   * @param ctrl Pure pursuit controller
   * @param velocity Constant velocity [m/s]
   */
  SparringControllerComponent(std::unique_ptr<svdsim_server::PurePursuitController> ctrl, double velocity)
      : controller(std::move(ctrl)), constant_velocity(velocity) {}
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_SPARRING_CONTROLLER_COMPONENT_HPP_
