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

#ifndef MONACOF1_SIM_SERVER_DISCRETE_TIME_HPP_
#define MONACOF1_SIM_SERVER_DISCRETE_TIME_HPP_

#include <cstddef>
#include <ostream>
#include <string>

namespace svdsim_server {

/**
 * @brief Wrapper class for discrete simulation time
 *
 * Provides type-safe discrete time representation for simulation ticks.
 * Discrete time is represented as unsigned integers for precise discrete
 * time simulation.
 */
class DiscreteTime {
 public:
  /// @brief Underlying counter type
  using ValueType = int32_t;

  /**
   * @brief Default constructor, initializes to zero
   */
  constexpr DiscreteTime() : value_(0) {}

  /**
   * @brief Implicit conversion constructor from int32_t
   * @param value The tick count value
   */
  constexpr DiscreteTime(ValueType value) : value_(value) {}  // NOLINT(google-explicit-constructor)

  /**
   * @brief Get the underlying tick count value
   * @return The tick count
   */
  [[nodiscard]] constexpr ValueType value() const { return value_; }

  /**
   * @brief Equality comparison
   */
  constexpr bool operator==(const DiscreteTime& other) const { return value_ == other.value_; }

  /**
   * @brief Inequality comparison
   */
  constexpr bool operator!=(const DiscreteTime& other) const { return value_ != other.value_; }

  /**
   * @brief Less than comparison
   */
  constexpr bool operator<(const DiscreteTime& other) const { return value_ < other.value_; }

  /**
   * @brief Less than or equal comparison
   */
  constexpr bool operator<=(const DiscreteTime& other) const { return value_ <= other.value_; }

  /**
   * @brief Greater than comparison
   */
  constexpr bool operator>(const DiscreteTime& other) const { return value_ > other.value_; }

  /**
   * @brief Greater than or equal comparison
   */
  constexpr bool operator>=(const DiscreteTime& other) const { return value_ >= other.value_; }

  /**
   * @brief Pre-increment operator
   */
  DiscreteTime& operator++() {
    ++value_;
    return *this;
  }

  /**
   * @brief Post-increment operator
   */
  DiscreteTime operator++(int) {
    DiscreteTime temp = *this;
    ++value_;
    return temp;
  }

  /**
   * @brief Pre-decrement operator
   */
  DiscreteTime& operator--() {
    --value_;
    return *this;
  }

  /**
   * @brief Post-decrement operator
   */
  DiscreteTime operator--(int) {
    DiscreteTime temp = *this;
    --value_;
    return temp;
  }

  /**
   * @brief Addition operator with integer
   */
  constexpr DiscreteTime operator+(ValueType delta) const { return DiscreteTime(value_ + delta); }

  /**
   * @brief Subtraction operator with integer
   */
  constexpr DiscreteTime operator-(ValueType delta) const { return DiscreteTime(value_ - delta); }

  /**
   * @brief Subtraction operator between two DiscreteTime values
   * @return The difference as an integer (number of ticks)
   */
  constexpr ValueType operator-(const DiscreteTime& other) const { return value_ - other.value_; }

  /**
   * @brief Compound addition operator with integer
   */
  DiscreteTime& operator+=(ValueType delta) {
    value_ += delta;
    return *this;
  }

  /**
   * @brief Compound subtraction operator with integer
   */
  DiscreteTime& operator-=(ValueType delta) {
    value_ -= delta;
    return *this;
  }

 private:
  ValueType value_;
};

}  // namespace svdsim_server

namespace std {
/**
 * @brief Stream output operator
 */
inline std::ostream& operator<<(std::ostream& os, const svdsim_server::DiscreteTime& ts) {
  os << ts.value();
  return os;
}

/**
 * @brief Convert DiscreteTime to string
 * @param ts The DiscreteTime to convert
 * @return String representation of the tick count
 *
 * This function enables std::to_string() usage with DiscreteTime via ADL.
 */
inline std::string to_string(const svdsim_server::DiscreteTime& ts) {
  return std::to_string(ts.value());
}
}  // namespace std

#endif  // MONACOF1_SIM_SERVER_DISCRETE_TIME_HPP_
