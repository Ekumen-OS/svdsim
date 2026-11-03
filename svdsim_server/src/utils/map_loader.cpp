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

#include "svdsim_server/utils/map_loader.hpp"

// Standard library
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

// Non-ROS dependencies
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <sophus/se2.hpp>

namespace svdsim_server {
namespace utils {

StaticMap load_map_from_yaml(const std::string& yaml_filepath) {
  if (!std::filesystem::exists(yaml_filepath)) {
    throw std::runtime_error("Map YAML file does not exist: " + yaml_filepath);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_filepath);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to parse YAML file '" + yaml_filepath + "': " + e.what());
  }
  const auto& const_config = config;

  if (!const_config["image"]) {
    throw std::runtime_error("Missing 'image' field in map YAML: " + yaml_filepath);
  }
  if (!const_config["resolution"]) {
    throw std::runtime_error("Missing 'resolution' field in map YAML: " + yaml_filepath);
  }
  if (!const_config["origin"]) {
    throw std::runtime_error("Missing 'origin' field in map YAML: " + yaml_filepath);
  }

  const auto image_filename = const_config["image"].as<std::string>();
  const auto resolution = const_config["resolution"].as<double>();
  const auto origin_vec = const_config["origin"].as<std::vector<double>>();

  if (origin_vec.size() != 3) {
    throw std::runtime_error("Origin must have 3 values [x, y, theta], got " + std::to_string(origin_vec.size()));
  }

  const auto occupied_thresh = const_config["occupied_thresh"] ? const_config["occupied_thresh"].as<double>() : 0.65;
  const auto free_thresh = const_config["free_thresh"] ? const_config["free_thresh"].as<double>() : 0.196;

  // Handle negate field - can be integer (0/1) or boolean (true/false)
  bool negate = false;
  if (const_config["negate"]) {
    try {
      // Try loading as integer first (0 or 1)
      negate = (const_config["negate"].as<int>() != 0);
    } catch (const YAML::Exception&) {
      // If that fails, try loading as boolean
      try {
        negate = const_config["negate"].as<bool>();
      } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            "Invalid 'negate' value in YAML (must be 0/1 or true/false): " + std::string(e.what()));
      }
    }
  }

  const auto yaml_path = std::filesystem::path(yaml_filepath);
  const auto image_path = yaml_path.parent_path() / image_filename;

  if (!std::filesystem::exists(image_path)) {
    throw std::runtime_error("Map image file does not exist: " + image_path.string());
  }

  const auto image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    throw std::runtime_error("Failed to load map image: " + image_path.string());
  }

  if (image.cols <= 0 || image.rows <= 0) {
    throw std::runtime_error(
        "Invalid image dimensions: " + std::to_string(image.cols) + "x" + std::to_string(image.rows));
  }

  StaticMap map;
  map.resolution = resolution;
  map.width = static_cast<std::size_t>(image.cols);
  map.height = static_cast<std::size_t>(image.rows);

  const Sophus::SO2d rotation(origin_vec[2]);
  const Eigen::Vector2d translation(origin_vec[0], origin_vec[1]);
  map.origin = Sophus::SE2d(rotation, translation);

  map.occupancy_grid.resize(map.width * map.height);

  for (std::size_t y = 0; y < map.height; ++y) {
    for (std::size_t x = 0; x < map.width; ++x) {
      const auto pixel_value = image.at<uint8_t>(static_cast<int>(y), static_cast<int>(x));

      // Convert pixel value to occupancy probability [0, 1]
      // Apply negate if specified (invert black/white interpretation)
      // Notice that because in the image, 0 = black (occupied), 255 = white (free),
      // we need to invert if negate is false to turn it into occupancy probability.
      const auto occupancy =
          !negate ? (1.0 - static_cast<double>(pixel_value) / 255.0) : (static_cast<double>(pixel_value) / 255.0);

      // Determine if cell is occupied based on thresholds
      // In ROS convention: 0 = free, 100 = occupied, -1 = unknown
      // We use: false = free, true = occupied
      const auto is_occupied = (occupancy >= occupied_thresh) ? true
                               : (occupancy <= free_thresh)   ? false
                                                              : false;  // Unknown cells treated as free

      // we keep the ROS convention that the origin of the map is the lower-left corner of the image
      const auto index = y * map.width + x;
      map.occupancy_grid[index] = is_occupied;
    }
  }

  return map;
}

}  // namespace utils
}  // namespace svdsim_server
