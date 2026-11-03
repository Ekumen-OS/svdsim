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

#ifndef MONACOF1_SIM_SERVER__COMPONENTS__MOTION__DRIVECONTROLCOMPONENT_HPP_
#define MONACOF1_SIM_SERVER__COMPONENTS__MOTION__DRIVECONTROLCOMPONENT_HPP_

#include <memory>

#include "svdsim_server/models/simple_drive_model.hpp"

namespace svdsim_server {

/// @brief Drive control component managing drive model
class DriveModelComponent {
 public:
  /// @brief Constructor
  /// @param drive_model Unique pointer to the drive model
  explicit DriveModelComponent(std::unique_ptr<SimpleDriveModel> drive_model);

  /// @brief Get const reference to the drive model
  /// @return Const reference to SimpleDriveModel
  [[nodiscard]] const SimpleDriveModel& model() const { return *drive_model_; }

  /// @brief Get mutable reference to the drive model
  /// @return Mutable reference to SimpleDriveModel
  [[nodiscard]] SimpleDriveModel& model() { return *drive_model_; }

 private:
  std::unique_ptr<SimpleDriveModel> drive_model_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER__COMPONENTS__MOTION__DRIVECONTROLCOMPONENT_HPP_
