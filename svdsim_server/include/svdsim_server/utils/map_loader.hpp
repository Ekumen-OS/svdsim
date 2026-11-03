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

#ifndef MONACOF1_SIM_SERVER_UTILS_MAP_LOADER_HPP_
#define MONACOF1_SIM_SERVER_UTILS_MAP_LOADER_HPP_

// Standard library
#include <string>

// Project headers
#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {
namespace utils {

/**
 * @brief Load a ROS map from YAML and image files
 *
 * Reads a ROS map specification from a YAML file and loads the corresponding
 * occupancy grid image (PNG or PGM format). The function follows the ROS map_server
 * specification for map files.
 *
 * YAML file format:
 * - image: path to the image file (relative to YAML file location)
 * - resolution: map resolution in meters/pixel
 * - origin: [x, y, theta] pose of the (0,0) pixel in the map
 * - occupied_thresh: pixels with occupancy > threshold are considered occupied
 * - free_thresh: pixels with occupancy < threshold are considered free
 * - negate: whether to invert the color interpretation
 *
 * @param yaml_filepath Path to the YAML map specification file
 * @return StaticMap structure containing the loaded map data
 * @throws std::runtime_error if file cannot be read or format is invalid
 */
StaticMap load_map_from_yaml(const std::string& yaml_filepath);

}  // namespace utils
}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_UTILS_MAP_LOADER_HPP_
