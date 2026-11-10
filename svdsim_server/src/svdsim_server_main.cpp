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

// Standard library
#include <memory>

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "svdsim_server/svdsim_server_node.hpp"

int main(int argc, char* argv[]) {
  // Initialize ROS subsystems
  rclcpp::init(argc, argv);

  try {
    // Instantiate the SvdsimServerNode
    auto node = std::make_shared<svdsim_server::SvdsimServerNode>();

    // Spin the node to process callbacks (simulation runs via timer)
    rclcpp::spin(node);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("svdsim_server"), "Failed to initialize or run simulation server: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Shutdown ROS subsystems
  rclcpp::shutdown();
  return 0;
}
