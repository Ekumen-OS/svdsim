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

#ifndef SVDSIM_SERVER_ECS_COMPONENTS_REMOTE_CONTROL_SERVER_COMPONENT_HPP_
#define SVDSIM_SERVER_ECS_COMPONENTS_REMOTE_CONTROL_SERVER_COMPONENT_HPP_

#include <chrono>
#include <memory>

#include "sim_messages.pb.h"
#include "svdsim_proto/vehicle_control_server.hpp"

namespace svdsim_server {

/// @brief Vehicle control component for processing commands via ZeroMQ communication
class RemoteControlServerComponent {
 public:
  /// @brief Constructor
  /// @param server Unique pointer to VehicleControlServer instance
  explicit RemoteControlServerComponent(std::unique_ptr<svdsim_proto::VehicleControlServer> server);

  /// @brief Get const reference to the server
  /// @return Const reference to VehicleControlServer
  [[nodiscard]] const svdsim_proto::VehicleControlServer& server() const { return *server_; }

  /// @brief Get mutable reference to the server
  /// @return Mutable reference to VehicleControlServer
  [[nodiscard]] svdsim_proto::VehicleControlServer& server() { return *server_; }

 private:
  std::unique_ptr<svdsim_proto::VehicleControlServer> server_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_ECS_COMPONENTS_REMOTE_CONTROL_SERVER_COMPONENT_HPP_
