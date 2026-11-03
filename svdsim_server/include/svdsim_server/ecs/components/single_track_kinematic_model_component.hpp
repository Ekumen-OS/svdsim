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

#ifndef MONACOF1_SIM_SERVER_COMPONENTS_SINGLE_TRACK_KINEMATIC_MODEL_COMPONENT_HPP_
#define MONACOF1_SIM_SERVER_COMPONENTS_SINGLE_TRACK_KINEMATIC_MODEL_COMPONENT_HPP_

#include <memory>

#include "svdsim_server/models/single_track_model_interface.hpp"
#include "svdsim_server/trajectory_buffer.hpp"

namespace svdsim_server {

/**
 * @brief Component wrapper for single-track vehicle model
 *
 * Provides a component-based interface to the single-track dynamics model.
 * Supports both kinematic and drift models through the interface.
 */
class SingleTrackKinematicModelComponent {
 public:
  /**
   * @brief Construct a new Single Track Model Component
   *
   * @param model Unique pointer to single-track model interface (takes ownership)
   * @param trajectory_buffer Shared pointer to trajectory log
   */
  explicit SingleTrackKinematicModelComponent(std::unique_ptr<SingleTrackModelInterface> model);

  /**
   * @brief Get reference to the model
   *
   * @return SingleTrackModelInterface& Reference to the model
   */
  [[nodiscard]] SingleTrackModelInterface& model();

  /**
   * @brief Get shared pointer to the trajectory log
   *
   * @return std::shared_ptr<TrajectoryBuffer> Shared pointer to the trajectory log
   */
  [[nodiscard]] std::shared_ptr<TrajectoryBuffer> trajectory_buffer() const;

 private:
  std::unique_ptr<SingleTrackModelInterface> model_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_COMPONENTS_SINGLE_TRACK_KINEMATIC_MODEL_COMPONENT_HPP_
