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

#ifndef SVDSIM_SERVER_MODELS_PULSE_ENCODER_MODEL_HPP_
#define SVDSIM_SERVER_MODELS_PULSE_ENCODER_MODEL_HPP_

#include <cstdint>

namespace svdsim_server {

/**
 * @brief Model for pulse encoder behavior and calculations
 *
 * This class encapsulates the configuration and conversion logic for wheel
 * pulse encoders. It handles the relationship between wheel rotation and
 * encoder pulses through the conversion ratio and pulses per revolution.
 */
class PulseEncoderModel {
 public:
  /**
   * @brief Construct a pulse encoder model with specified parameters
   *
   * @param conversion_ratio Gear ratio or conversion factor from motor to wheel
   * @param pulses_per_revolution Number of encoder pulses per wheel revolution
   */
  PulseEncoderModel(double conversion_ratio, double pulses_per_revolution);

  /**
   * @brief Calculate encoder ticks from wheel rotation
   *
   * Computes the encoder output in ticks based on the number of full wheel
   * rotations and the current angular position within a rotation.
   *
   * @param full_cycles Number of complete wheel rotations (integer cycles)
   * @param wheel_angle Current angular position within rotation [rad], range [0, 2π)
   * @return Encoder tick count as integer
   */
  [[nodiscard]] int32_t calculate_encoder_output(int32_t full_cycles, double wheel_angle) const;

  /**
   * @brief Get the conversion ratio
   * @return Conversion ratio from motor to wheel
   */
  [[nodiscard]] double conversion_ratio() const { return conversion_ratio_; }

  /**
   * @brief Get the pulses per revolution
   * @return Number of encoder pulses per wheel revolution
   */
  [[nodiscard]] double pulses_per_revolution() const { return pulses_per_revolution_; }

 private:
  /// Conversion ratio from motor to wheel (e.g., gear ratio)
  double conversion_ratio_;

  /// Number of encoder pulses per wheel revolution
  double pulses_per_revolution_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_MODELS_PULSE_ENCODER_MODEL_HPP_
