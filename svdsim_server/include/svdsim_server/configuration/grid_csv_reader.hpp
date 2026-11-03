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

#ifndef MONACOF1_SIM_SERVER_CONFIGURATION_GRID_CSV_READER_HPP_
#define MONACOF1_SIM_SERVER_CONFIGURATION_GRID_CSV_READER_HPP_

#include <istream>
#include <string>
#include <vector>

namespace svdsim_server {

/**
 * @brief Vehicle type enumeration
 */
enum class VehicleType {
  REMOTE,   ///< Remote-controlled vehicle
  SPARRING  ///< Autonomous path following sparring vehicle
};

/**
 * @brief Structure representing a vehicle configuration entry from CSV
 */
struct VehicleGridEntry {
  /// Unique identifier for the vehicle
  std::string id;

  /// Initial X position in world coordinates [m]
  double init_x;

  /// Initial Y position in world coordinates [m]
  double init_y;

  /// Initial yaw angle [rad]
  double init_yaw;

  /// Network port number for vehicle communication
  int port;

  /// Vehicle type (remote or sparring)
  VehicleType type;
};

/**
 * @brief CSV reader for vehicle grid configuration files
 *
 * Reads CSV files containing vehicle spawn configuration with columns:
 * id, initx, inity, inityaw, port, type
 *
 * The 'type' column specifies vehicle type: 'remote' or 'sparring'.
 *
 * Example usage:
 * @code
 * std::ifstream file("vehicles.csv");
 * GridCsvReader reader;
 * auto vehicles = reader << file;
 * @endcode
 */
class GridCsvReader {
 public:
  /**
   * @brief Construct a new Grid Csv Reader
   */
  GridCsvReader() = default;

  /**
   * @brief Stream insertion operator for reading CSV data
   * @param is Input stream containing CSV data
   * @return Vector of parsed vehicle entries
   * @throws std::runtime_error if CSV parsing or validation fails
   */
  std::vector<VehicleGridEntry> operator<<(std::istream& is);
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_CONFIGURATION_GRID_CSV_READER_HPP_
