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

#include "svdsim_server/ecs/components/lidar_sensor_component.hpp"

#include "svdsim_server/models/lidar_sensor_model.hpp"

namespace svdsim_server {

LidarSensorComponent::LidarSensorComponent(std::unique_ptr<LidarSensorModel> sensor) : sensor_(std::move(sensor)) {}

}  // namespace svdsim_server
