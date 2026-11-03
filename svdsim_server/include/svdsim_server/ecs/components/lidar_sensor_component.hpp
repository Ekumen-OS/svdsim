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

#ifndef MONACOF1_SIM_SERVER_COMPONENTS_LIDAR_SENSOR_COMPONENT_HPP_
#define MONACOF1_SIM_SERVER_COMPONENTS_LIDAR_SENSOR_COMPONENT_HPP_

#include <memory>
#include <vector>

#include <sophus/se2.hpp>

#include "svdsim_server/models/lidar_sensor_model.hpp"
namespace svdsim_server {

/**
 * @brief Component representing a lidar sensor attached to a vehicle
 *
 * This component stores a lidar sensor instance and the current scan data.
 */
class LidarSensorComponent {
 public:
  /**
   * @brief Construct a new Lidar Sensor Component
   *
   * @param sensor Unique pointer to the lidar sensor model instance
   */
  explicit LidarSensorComponent(std::unique_ptr<LidarSensorModel> sensor);

  /**
   * @brief Get the lidar sensor instance
   * @return Reference to the lidar sensor
   */
  [[nodiscard]] const LidarSensorModel& sensor() const { return *sensor_; }

  /**
   * @brief Get mutable access to the lidar sensor instance
   * @return Reference to the lidar sensor
   */
  [[nodiscard]] LidarSensorModel& sensor() { return *sensor_; }

 private:
  /// Lidar sensor instance
  std::unique_ptr<LidarSensorModel> sensor_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_COMPONENTS_LIDAR_SENSOR_COMPONENT_HPP_
