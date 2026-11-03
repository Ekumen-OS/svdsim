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

#include "svdsim_server/wall_time.hpp"

#include <cmath>

namespace svdsim_server {

WallTime WallTime::from_seconds(RealType total_seconds) {
  WallTime result;
  result.seconds = static_cast<int64_t>(std::floor(total_seconds));
  result.fractional = total_seconds - static_cast<RealType>(result.seconds);
  return result;
}

WallTime::RealType WallTime::to_seconds() const {
  return static_cast<RealType>(seconds) + fractional;
}

bool WallTime::operator==(const WallTime& other) const {
  return is_approx_equal(other, kDefaultEpsilon);
}

bool WallTime::operator!=(const WallTime& other) const {
  return !(*this == other);
}

bool WallTime::operator<(const WallTime& other) const {
  if (seconds != other.seconds) {
    return seconds < other.seconds;
  }
  return fractional < other.fractional;
}

bool WallTime::operator<=(const WallTime& other) const {
  return *this < other || *this == other;
}

bool WallTime::operator>(const WallTime& other) const {
  return !(*this <= other);
}

bool WallTime::operator>=(const WallTime& other) const {
  return !(*this < other);
}

WallTime WallTime::operator+(const WallTime& other) const {
  return from_seconds(to_seconds() + other.to_seconds());
}

WallTime WallTime::operator-(const WallTime& other) const {
  return from_seconds(to_seconds() - other.to_seconds());
}

WallTime& WallTime::operator+=(const WallTime& other) {
  *this = *this + other;
  return *this;
}

WallTime& WallTime::operator-=(const WallTime& other) {
  *this = *this - other;
  return *this;
}

bool WallTime::is_approx_equal(const WallTime& other, RealType epsilon) const {
  return seconds == other.seconds && std::abs(fractional - other.fractional) < epsilon;
}

std::ostream& operator<<(std::ostream& os, const WallTime& ts) {
  os << ts.to_seconds();
  return os;
}

}  // namespace svdsim_server
