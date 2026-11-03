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

#include "svdsim_server/models/pulse_encoder_model.hpp"

#include <cmath>
#include <numbers>

namespace svdsim_server {

PulseEncoderModel::PulseEncoderModel(double conversion_ratio, double pulses_per_revolution)
    : conversion_ratio_(conversion_ratio), pulses_per_revolution_(pulses_per_revolution) {}

int32_t PulseEncoderModel::calculate_encoder_output(int32_t full_cycles, double wheel_angle) const {
  // Calculate total rotations: integer cycles + fractional part from angle
  const double total_rotations = static_cast<double>(full_cycles) + wheel_angle / (2.0 * M_PI);

  // Convert to encoder ticks: rotations * conversion_ratio * pulses_per_revolution
  const double ticks = total_rotations * conversion_ratio_ * pulses_per_revolution_;

  // Round to nearest integer
  return static_cast<int32_t>(std::round(ticks));
}

}  // namespace svdsim_server
