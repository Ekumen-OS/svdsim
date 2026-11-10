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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_WHEEL_ENCODER_SENSOR_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_WHEEL_ENCODER_SENSOR_COMPONENT_HPP_

#include <memory>

#include "svdsim_server/models/pulse_encoder_model.hpp"

namespace svdsim_server {

/**
 * @brief Component representing wheel encoder sensors
 *
 * This component holds the encoder models for the rear left and rear right
 * wheel encoders of the vehicle.
 */
class WheelEncoderSensorComponent {
 public:
  /**
   * @brief Construct a new Wheel Encoder Sensor Component
   *
   * @param rear_left_encoder Unique pointer to the rear left encoder model
   * @param rear_right_encoder Unique pointer to the rear right encoder model
   */
  WheelEncoderSensorComponent(
      std::unique_ptr<PulseEncoderModel> rear_left_encoder,
      std::unique_ptr<PulseEncoderModel> rear_right_encoder);

  /**
   * @brief Get the rear left encoder model
   * @return Reference to the rear left encoder model
   */
  [[nodiscard]] const PulseEncoderModel& rear_left_encoder() const { return *rear_left_encoder_; }

  /**
   * @brief Get the rear right encoder model
   * @return Reference to the rear right encoder model
   */
  [[nodiscard]] const PulseEncoderModel& rear_right_encoder() const { return *rear_right_encoder_; }

 private:
  /// Rear left wheel encoder model
  std::unique_ptr<PulseEncoderModel> rear_left_encoder_;

  /// Rear right wheel encoder model
  std::unique_ptr<PulseEncoderModel> rear_right_encoder_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_WHEEL_ENCODER_SENSOR_COMPONENT_HPP_
