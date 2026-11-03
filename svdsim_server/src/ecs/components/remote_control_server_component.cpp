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

#include "svdsim_server/ecs/components/remote_control_server_component.hpp"

namespace svdsim_server {

RemoteControlServerComponent::RemoteControlServerComponent(std::unique_ptr<svdsim_proto::VehicleControlServer> server)
    : server_(std::move(server)) {}

}  // namespace svdsim_server
