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

#ifndef SVDSIM_SERVER_ECS_SYSTEMS_SPARRING_CONTROL_SYSTEM_HPP_
#define SVDSIM_SERVER_ECS_SYSTEMS_SPARRING_CONTROL_SYSTEM_HPP_

#include <entt/entt.hpp>

#include "svdsim_server/discrete_time.hpp"
#include "svdsim_server/ecs/systems/system_interface.hpp"

namespace svdsim_server {

/**
 * @brief System that controls sparring vehicles
 *
 * This system updates all entities with SparringControllerComponent,
 * computing steering and velocity commands based on the pure pursuit
 * path following algorithm. Emits visualization events for pose and
 * lookahead targets.
 */
class SparringControlSystem : public SystemInterface {
 public:
  /**
   * @brief Construct a new Sparring Control System
   *
   * @param registry The entity registry
   * @param dispatcher The event dispatcher
   * @param period System update period in simulation ticks
   */
  SparringControlSystem(entt::registry& registry, entt::dispatcher& dispatcher, std::size_t period);

  /**
   * @brief Update the system for the current frame
   *
   * @return DiscreteTime Next desired activation time
   */
  DiscreteTime update() override;

 private:
  entt::registry& registry_;
  entt::dispatcher& dispatcher_;
  std::size_t period_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_SYSTEMS_SPARRING_CONTROL_SYSTEM_HPP_
