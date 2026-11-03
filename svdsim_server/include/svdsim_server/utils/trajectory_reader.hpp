// Copyright 2026 Ekumen
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

#ifndef SVDSIM_SERVER_UTILS_TRAJECTORY_READER_HPP_
#define SVDSIM_SERVER_UTILS_TRAJECTORY_READER_HPP_

#include <Eigen/Core>
#include <istream>
#include <string>
#include <vector>

namespace svdsim_server {

/**
 * @brief 2D trajectory point
 *
 * Represents a waypoint in a 2D trajectory with x and y coordinates.
 */
struct TrajectoryPoint {
  double x;  ///< X coordinate in world frame [m]
  double y;  ///< Y coordinate in world frame [m]

  TrajectoryPoint() : x(0.0), y(0.0) {}
  TrajectoryPoint(double x_val, double y_val) : x(x_val), y(y_val) {}

  /**
   * @brief Convert to Eigen vector
   */
  [[nodiscard]] Eigen::Vector2d to_vector() const { return Eigen::Vector2d(x, y); }
};

/**
 * @brief CSV reader for trajectory files
 *
 * Reads CSV files containing trajectory waypoints with x and y columns.
 * The CSV file must have a header row with 'x' and 'y' fields.
 *
 * Example CSV format:
 * @code
 * x,y
 * 0.0,0.0
 * 1.0,0.5
 * 2.0,1.0
 * @endcode
 *
 * Example usage:
 * @code
 * TrajectoryReader reader;
 * auto trajectory = reader.read_from_file("path/to/trajectory.csv");
 * @endcode
 */
class TrajectoryReader {
 public:
  /**
   * @brief Construct a new Trajectory Reader
   */
  TrajectoryReader() = default;

  /**
   * @brief Read trajectory from a file
   *
   * @param filepath Path to the CSV file
   * @return Vector of trajectory points
   * @throws std::runtime_error if file cannot be opened or parsing fails
   */
  [[nodiscard]] std::vector<TrajectoryPoint> read_from_file(const std::string& filepath);

  /**
   * @brief Read trajectory from an input stream
   *
   * @param is Input stream containing CSV data
   * @return Vector of trajectory points
   * @throws std::runtime_error if CSV parsing or validation fails
   */
  [[nodiscard]] std::vector<TrajectoryPoint> read_from_stream(std::istream& is);

 private:
  static constexpr size_t kMinimumTrajectoryPoints = 3;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_UTILS_TRAJECTORY_READER_HPP_
