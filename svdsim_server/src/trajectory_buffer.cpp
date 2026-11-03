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

#include "svdsim_server/trajectory_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace svdsim_server {

TrajectoryBuffer::TrajectoryBuffer(std::size_t max_length)
    : max_length_(max_length), latest_sample_timestamp_(DiscreteTime(0)), write_index_(0) {
  buffer_.resize(max_length_);
}

void TrajectoryBuffer::reset(DiscreteTime timestamp, const Sophus::SE2d& pose) {
  // Fill all buffer entries with the initial pose (as if vehicle was static)
  std::fill(buffer_.begin(), buffer_.end(), pose);
  latest_sample_timestamp_ = timestamp;
  write_index_ = 0;
}

void TrajectoryBuffer::add(DiscreteTime timestamp, const Sophus::SE2d& pose) {
  buffer_[write_index_] = pose;
  latest_sample_timestamp_ = timestamp;
  write_index_ = (write_index_ + 1) % max_length_;
}

std::optional<Sophus::SE2d> TrajectoryBuffer::interpolate(double delta_t) const {
  // Buffer is always full after reset

  // If delta is negative, return nullopt
  if (delta_t < 0.0) {
    return std::nullopt;
  }

  // If delta_t exceeds buffer history, return oldest pose
  // Note: delta_t is measured in samples, not seconds
  if (delta_t >= static_cast<double>(max_length_ - 1)) {
    // Oldest element is at write_index_
    return buffer_[write_index_];
  }

  // Find the two poses that surround the target sample
  // delta_t represents how many samples into the past (0.0 = current, 1.0 = one sample back, etc.)
  const std::size_t older_steps = static_cast<std::size_t>(std::ceil(delta_t));
  const std::size_t newer_steps = static_cast<std::size_t>(std::floor(delta_t));

  // If indices are the same (delta_t is an integer), return exact pose
  if (newer_steps == older_steps) {
    // Most recent is at (write_index_ - 1 + max_length_) % max_length_
    const std::size_t idx = (write_index_ + max_length_ - 1 - newer_steps) % max_length_;
    return buffer_[idx];
  }

  // Compute interpolation factor (0 = older, 1 = newer)
  const double alpha = delta_t - std::floor(delta_t);

  // Get buffer indices (newer is more recent, older is further back)
  const std::size_t newer_idx = (write_index_ + max_length_ - 1 - newer_steps) % max_length_;
  const std::size_t older_idx = (write_index_ + max_length_ - 1 - older_steps) % max_length_;

  // Get the two poses
  const Sophus::SE2d& older = buffer_[older_idx];
  const Sophus::SE2d& newer = buffer_[newer_idx];

  // Interpolate using SE2 Lie group: pose = older * exp(alpha * log(older^-1 * newer))
  const Sophus::SE2d relative = older.inverse() * newer;
  const Sophus::SE2d::Tangent tangent = relative.log();
  const Sophus::SE2d interpolated = older * Sophus::SE2d::exp(alpha * tangent);

  return interpolated;
}

}  // namespace svdsim_server
