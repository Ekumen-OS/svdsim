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

#include "svdsim_server/configuration/grid_csv_reader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
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
 * @brief Split a CSV line into fields, respecting quoted fields
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

    // Check if entire string was parsed
    if (pos != str.size()) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Field '" + field_name + "' contains invalid characters: '" + str +
          "'");
    }

    // Check for NaN or infinity
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
 * @brief Parse an integer value with validation
 */
int parse_int(const std::string& str, const std::string& field_name, size_t line_number) {
  if (str.empty()) {
    throw std::runtime_error("Line " + std::to_string(line_number) + ": Field '" + field_name + "' is empty");
  }

  try {
    size_t pos = 0;
    int value = std::stoi(str, &pos);

    // Check if entire string was parsed
    if (pos != str.size()) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Field '" + field_name + "' contains invalid characters: '" + str +
          "'");
    }

    // Validate port range
    if (field_name == "port") {
      if (value < 1 || value > 65535) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) +
            ": Port must be between 1 and 65535, got: " + std::to_string(value));
      }
    }

    return value;
  } catch (const std::invalid_argument&) {
    throw std::runtime_error(
        "Line " + std::to_string(line_number) + ": Field '" + field_name + "' is not a valid integer: '" + str + "'");
  } catch (const std::out_of_range&) {
    throw std::runtime_error(
        "Line " + std::to_string(line_number) + ": Field '" + field_name + "' is out of range: '" + str + "'");
  }
}

/**
 * @brief Validate vehicle ID
 */
void validate_id(const std::string& id, size_t line_number) {
  if (id.empty()) {
    throw std::runtime_error("Line " + std::to_string(line_number) + ": Vehicle ID cannot be empty");
  }

  // Check for valid characters (alphanumeric, underscore, hyphen)
  for (char ch : id) {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-') {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Vehicle ID contains invalid character '" + std::string(1, ch) +
          "'. Only alphanumeric, underscore, and hyphen allowed: '" + id + "'");
    }
  }
}

/**
 * @brief Parse vehicle type from string
 */
VehicleType parse_vehicle_type(const std::string& str, size_t line_number) {
  std::string type_lower = str;
  std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);

  if (type_lower == "remote") {
    return VehicleType::REMOTE;
  } else if (type_lower == "sparring") {
    return VehicleType::SPARRING;
  } else {
    throw std::runtime_error(
        "Line " + std::to_string(line_number) + ": Invalid vehicle type '" + str + "'. Must be 'remote' or 'sparring'");
  }
}

}  // namespace

std::vector<VehicleGridEntry> GridCsvReader::operator<<(std::istream& is) {
  std::vector<VehicleGridEntry> entries;

  std::string line;
  size_t line_number = 0;
  bool header_read = false;

  while (std::getline(is, line)) {
    ++line_number;

    // Skip empty lines
    std::string trimmed = trim(line);
    if (trimmed.empty()) {
      continue;
    }

    // Skip comment lines
    if (trimmed[0] == '#') {
      continue;
    }

    // Parse CSV fields
    auto fields = split_csv_line(line);

    // Handle header line (expect 6 columns: id, initx, inity, inityaw, port, type)
    if (!header_read) {
      if (fields.size() != 6) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) +
            ": Header must have 6 columns (id, initx, inity, inityaw, port, type), got " +
            std::to_string(fields.size()));
      }

      // Validate header column names (case-insensitive)
      std::vector<std::string> expected_headers = {"id", "initx", "inity", "inityaw", "port", "type"};
      for (size_t i = 0; i < expected_headers.size(); ++i) {
        std::string field_lower = fields[i];
        std::transform(field_lower.begin(), field_lower.end(), field_lower.begin(), ::tolower);
        if (field_lower != expected_headers[i]) {
          throw std::runtime_error(
              "Line " + std::to_string(line_number) + ": Expected header column '" + expected_headers[i] +
              "' at position " + std::to_string(i + 1) + ", got '" + fields[i] + "'");
        }
      }

      header_read = true;
      continue;
    }

    // Parse data lines
    if (fields.size() != 6) {
      throw std::runtime_error(
          "Line " + std::to_string(line_number) + ": Expected 6 fields, got " + std::to_string(fields.size()));
    }

    VehicleGridEntry entry;

    // Parse and validate each field
    entry.id = fields[0];
    validate_id(entry.id, line_number);

    entry.init_x = parse_double(fields[1], "initx", line_number);
    entry.init_y = parse_double(fields[2], "inity", line_number);
    entry.init_yaw = parse_double(fields[3], "inityaw", line_number);
    entry.port = parse_int(fields[4], "port", line_number);
    entry.type = parse_vehicle_type(fields[5], line_number);

    // Check for duplicate IDs
    for (const auto& existing_entry : entries) {
      if (existing_entry.id == entry.id) {
        throw std::runtime_error("Line " + std::to_string(line_number) + ": Duplicate vehicle ID '" + entry.id + "'");
      }
    }

    // Check for duplicate ports
    for (const auto& existing_entry : entries) {
      if (existing_entry.port == entry.port) {
        throw std::runtime_error(
            "Line " + std::to_string(line_number) + ": Duplicate port number " + std::to_string(entry.port) +
            " (already used by vehicle '" + existing_entry.id + "')");
      }
    }

    entries.push_back(entry);
  }

  if (!header_read) {
    throw std::runtime_error("CSV file is empty or contains no valid header");
  }

  if (entries.empty()) {
    throw std::runtime_error("CSV file contains no vehicle entries");
  }

  return entries;
}

}  // namespace svdsim_server
