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

#include "svdsim_server/ecs/components/wheel_encoder_sensor_component.hpp"

namespace svdsim_server {

WheelEncoderSensorComponent::WheelEncoderSensorComponent(
    std::unique_ptr<PulseEncoderModel> rear_left_encoder,
    std::unique_ptr<PulseEncoderModel> rear_right_encoder)
    : rear_left_encoder_(std::move(rear_left_encoder)), rear_right_encoder_(std::move(rear_right_encoder)) {}

}  // namespace svdsim_server
