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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_IMU_SENSOR_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_IMU_SENSOR_COMPONENT_HPP_

#include <memory>

#include "svdsim_server/models/imu_sensor_model.hpp"

namespace svdsim_server {

/**
 * @brief Component representing an IMU sensor attached to a vehicle
 *
 * This component holds the IMU sensor model instance for simulating
 * accelerometer, gyroscope, and magnetometer measurements.
 */
class IMUSensorComponent {
 public:
  /**
   * @brief Construct a new IMU Sensor Component
   *
   * @param sensor Unique pointer to the IMU sensor model instance
   */
  explicit IMUSensorComponent(std::unique_ptr<IMUSensorModel> sensor);

  /**
   * @brief Get the IMU sensor instance
   * @return Reference to the IMU sensor
   */
  [[nodiscard]] const IMUSensorModel& sensor() const { return *sensor_; }

  /**
   * @brief Get mutable access to the IMU sensor instance
   * @return Reference to the IMU sensor
   */
  [[nodiscard]] IMUSensorModel& sensor() { return *sensor_; }

 private:
  /// IMU sensor instance
  std::unique_ptr<IMUSensorModel> sensor_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_IMU_SENSOR_COMPONENT_HPP_
