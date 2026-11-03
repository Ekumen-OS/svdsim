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

#ifndef MONACOF1_SIM_SERVER_WALL_TIME_HPP_
#define MONACOF1_SIM_SERVER_WALL_TIME_HPP_

#include <cstdint>
#include <limits>
#include <ostream>

namespace svdsim_server {

/**
 * @brief Wall time with seconds and fractional components
 *
 * Represents a point in simulated wall time with integer seconds and
 * fractional seconds as a floating-point value.
 */
struct WallTime {
  /// @brief Type used for real-valued time representation
  using RealType = double;

  /// @brief Integer seconds component
  int64_t seconds;

  /// @brief Fractional seconds component [0.0, 1.0)
  RealType fractional;

  /// @brief Default epsilon for floating-point comparison
  static constexpr RealType kDefaultEpsilon = std::numeric_limits<RealType>::epsilon();

  /**
   * @brief Construct a WallTime from total seconds
   * @param total_seconds Total time in seconds
   * @return WallTime with split seconds and fractional components
   */
  static WallTime from_seconds(RealType total_seconds);

  /**
   * @brief Convert to total seconds as RealType
   * @return Total time in seconds
   */
  [[nodiscard]] RealType to_seconds() const;

  /**
   * @brief Equality comparison with epsilon tolerance for fractional part
   * @param other The wall time to compare with
   */
  bool operator==(const WallTime& other) const;

  /**
   * @brief Inequality comparison with epsilon tolerance for fractional part
   * @param other The wall time to compare with
   */
  bool operator!=(const WallTime& other) const;

  /**
   * @brief Less than comparison
   * @param other The wall time to compare with
   */
  bool operator<(const WallTime& other) const;

  /**
   * @brief Less than or equal comparison
   * @param other The wall time to compare with
   */
  bool operator<=(const WallTime& other) const;

  /**
   * @brief Greater than comparison
   * @param other The wall time to compare with
   */
  bool operator>(const WallTime& other) const;

  /**
   * @brief Greater than or equal comparison
   * @param other The wall time to compare with
   */
  bool operator>=(const WallTime& other) const;

  /**
   * @brief Addition operator
   * @param other The wall time to add
   * @return Result of the addition
   */
  WallTime operator+(const WallTime& other) const;

  /**
   * @brief Subtraction operator
   * @param other The wall time to subtract
   * @return Result of the subtraction
   */
  WallTime operator-(const WallTime& other) const;

  /**
   * @brief Addition assignment operator
   * @param other The wall time to add
   * @return Reference to this object after addition
   */
  WallTime& operator+=(const WallTime& other);

  /**
   * @brief Subtraction assignment operator
   * @param other The wall time to subtract
   * @return Reference to this object after subtraction
   */
  WallTime& operator-=(const WallTime& other);

  /**
   * @brief Check if wall times are approximately equal within given tolerance
   * @param other The wall time to compare with
   * @param epsilon Tolerance for floating-point comparison
   */
  [[nodiscard]] bool is_approx_equal(const WallTime& other, RealType epsilon = kDefaultEpsilon) const;
};

/**
 * @brief Stream output operator for WallTime
 */
std::ostream& operator<<(std::ostream& os, const WallTime& ts);

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_WALL_TIME_HPP_
