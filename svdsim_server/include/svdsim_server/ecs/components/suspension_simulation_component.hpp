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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_SUSPENSION_SIMULATION_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_SUSPENSION_SIMULATION_COMPONENT_HPP_

#include <memory>

#include "svdsim_server/models/sprung_mass_inclination_model.hpp"

namespace svdsim_server {

/**
 * @brief Component representing suspension simulation for a vehicle
 *
 * This component holds the sprung mass inclination model instance for computing
 * vehicle body orientation based on acceleration-induced suspension compression.
 */
class SuspensionSimulationComponent {
 public:
  /**
   * @brief Construct a new Suspension Simulation Component
   *
   * @param model Unique pointer to the sprung mass inclination model instance
   */
  explicit SuspensionSimulationComponent(std::unique_ptr<SprungMassInclinationModel> model);

  /**
   * @brief Get the suspension model instance
   * @return Const reference to the suspension model
   */
  [[nodiscard]] const SprungMassInclinationModel& model() const { return *model_; }

  /**
   * @brief Get mutable access to the suspension model instance
   * @return Reference to the suspension model
   */
  [[nodiscard]] SprungMassInclinationModel& model() { return *model_; }

 private:
  /// Suspension model instance
  std::unique_ptr<SprungMassInclinationModel> model_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_SUSPENSION_SIMULATION_COMPONENT_HPP_
