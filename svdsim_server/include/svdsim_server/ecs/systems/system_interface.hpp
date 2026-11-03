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

#ifndef MONACOF1_SIM_SERVER_ECS_SYSTEMS_SYSTEM_INTERFACE_HPP_
#define MONACOF1_SIM_SERVER_ECS_SYSTEMS_SYSTEM_INTERFACE_HPP_

#include "svdsim_server/discrete_time.hpp"

namespace svdsim_server {

/**
 * @brief Interface for ECS systems
 *
 * Defines the common interface that all systems must implement.
 * Systems operate on entities with specific components and are called
 * each simulation frame.
 */
class SystemInterface {
 public:
  /// @brief Virtual destructor
  virtual ~SystemInterface() = default;

  /**
   * @brief Update the system for the current frame
   *
   * Called once per simulation step to process all relevant entities.
   *
   * @return DiscreteTime representing the timestamp of the next desired activation
   */
  virtual DiscreteTime update() = 0;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_ECS_SYSTEMS_SYSTEM_INTERFACE_HPP_
