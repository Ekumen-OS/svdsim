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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_TRUE_STEERING_SERVO_SENSOR_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_TRUE_STEERING_SERVO_SENSOR_COMPONENT_HPP_

namespace svdsim_server {

/**
 * @brief Component that marks an entity as having steering servo sensor feedback
 *
 * Entities with this component will publish steering status events through
 * the OdometrySensorSystem.
 */
struct TrueSteeringServoSensorComponent {};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_TRUE_STEERING_SERVO_SENSOR_COMPONENT_HPP_
