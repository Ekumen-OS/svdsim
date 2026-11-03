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

#include "svdsim_server/utils/trajectory_reader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace svdsim_server {

namespace {

/**
 * @brief Trim whitespace from both ends of a string
 */
std::string trim(const std::string& str) {
  const auto start = std::find_if(str.begin(), str.end(), [](unsigned char ch) { return !std::isspace(ch); });
  const auto end = std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base();
  return (start < end) ? std::string(start, end) : std::string();
}

/**
 * @brief Split a CSV line into fields
 */
std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool in_quotes = false;

  for (char ch : line) {
    if (ch == '"') {
      in_quotes = !in_quotes;
    } else if (ch == ',' && !in_quotes) {
      fields.push_back(trim(field));
      field.clear();
    } else {
      field += ch;
    }
  }
  fields.push_back(trim(field));

  return fields;
}

/**
 * @brief Parse a double value with validation
 */
double parse_double(const std::string& str, const std::string& field_name, size_t line_number) {
  if (str.empty()) {
    throw std::runtime_error("Line " + std::to_string(line_number) + ": Field '" + field_name + "' is empty");
  }

  try {
    size_t pos = 0;
    double value = std::stod(str, &pos);

    if (pos != str.size()) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Field '" + field_name + "' contains invalid characters: '" + str +
          "'");
    }

    if (!std::isfinite(value)) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Field '" + field_name + "' must be a finite number, got: '" + str +
          "'");
    }

    return value;
  } catch (const std::invalid_argument&) {
    throw std::runtime_error(
        "Line " + std::to_string(line_number) + ": Field '" + field_name + "' is not a valid number: '" + str + "'");
  } catch (const std::out_of_range&) {
    throw std::runtime_error(
        "Line " + std::to_string(line_number) + ": Field '" + field_name + "' is out of range: '" + str + "'");
  }
}

/**
 * @brief Find column index by name
 */
int find_column_index(const std::vector<std::string>& header, const std::string& column_name) {
  auto it = std::find(header.begin(), header.end(), column_name);
  if (it == header.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(header.begin(), it));
}

}  // namespace

std::vector<TrajectoryPoint> TrajectoryReader::read_from_file(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open trajectory file: " + filepath);
  }

  try {
    return read_from_stream(file);
  } catch (const std::exception& e) {
    throw std::runtime_error("Error reading trajectory from '" + filepath + "': " + e.what());
  }
}

std::vector<TrajectoryPoint> TrajectoryReader::read_from_stream(std::istream& is) {
  std::vector<TrajectoryPoint> trajectory;
  std::string line;
  size_t line_number = 0;

  // Read header line
  if (!std::getline(is, line)) {
    throw std::runtime_error("Failed to read header line from trajectory file");
  }
  line_number++;

  line = trim(line);
  if (line.empty()) {
    throw std::runtime_error("Header line is empty");
  }

  std::vector<std::string> header = split_csv_line(line);

  // Find x and y columns
  int x_index = find_column_index(header, "x");
  int y_index = find_column_index(header, "y");

  if (x_index < 0) {
    throw std::runtime_error("Header line does not contain required 'x' column");
  }
  if (y_index < 0) {
    throw std::runtime_error("Header line does not contain required 'y' column");
  }

  // Read data lines
  while (std::getline(is, line)) {
    line_number++;
    line = trim(line);

    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    // Skip comment lines
    if (line[0] == '#') {
      continue;
    }

    std::vector<std::string> fields = split_csv_line(line);

    if (fields.size() < header.size()) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Expected at least " + std::to_string(header.size()) +
          " fields, got " + std::to_string(fields.size()));
    }

    TrajectoryPoint point;
    point.x = parse_double(fields[x_index], "x", line_number);
    point.y = parse_double(fields[y_index], "y", line_number);

    trajectory.push_back(point);
  }

  if (trajectory.size() < kMinimumTrajectoryPoints) {
    throw std::runtime_error(
        "Trajectory must contain at least " + std::to_string(kMinimumTrajectoryPoints) + " points, got " +
        std::to_string(trajectory.size()));
  }

  return trajectory;
}

}  // namespace svdsim_server
