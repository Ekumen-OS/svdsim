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

#ifndef MONACOF1_SIM_SERVER_TRAJECTORY_LOG_HPP_
#define MONACOF1_SIM_SERVER_TRAJECTORY_LOG_HPP_

#include <deque>
#include <optional>

#include <sophus/se2.hpp>

#include "svdsim_server/datatypes.hpp"

namespace svdsim_server {

/**
 * @brief Circular buffer for storing SE2 pose trajectory with timestamps
 *
 * Maintains a fixed-size history of poses with interpolation support.
 */
class TrajectoryBuffer {
 public:
  /**
   * @brief Construct a new Trajectory Log
   *
   * @param max_length Maximum number of poses to store in the buffer
   */
  explicit TrajectoryBuffer(std::size_t max_length);

  /**
   * @brief Reset the trajectory log with a single initial pose
   *
   * Clears the buffer and initializes it with one pose.
   *
   * @param timestamp Initial timestamp
   * @param pose Initial SE2 pose
   */
  void reset(DiscreteTime timestamp, const Sophus::SE2d& pose);

  /**
   * @brief Add a new pose to the trajectory log
   *
   * Appends the pose to the end of the circular buffer. If the buffer is full,
   * the oldest pose is removed.
   *
   * @param timestamp Timestamp of the pose
   * @param pose SE2 pose to add
   */
  void add(DiscreteTime timestamp, const Sophus::SE2d& pose);

  /**
   * @brief Interpolate a pose at a given time in the past
   *
   * Retrieves or interpolates a pose from delta_t samples ago.
   * If delta_t exceeds the buffer history, returns the oldest available pose.
   * Otherwise, performs SE2 interpolation using Lie group algebra.
   *
   * @param delta_t Number of samples into the past (0.0 = most recent, 1.0 = one sample back, etc.)
   * @return Interpolated SE2 pose, or std::nullopt if buffer is empty
   */
  [[nodiscard]] std::optional<Sophus::SE2d> interpolate(double delta_t) const;

 private:
  std::size_t max_length_;
  DiscreteTime latest_sample_timestamp_;
  std::vector<Sophus::SE2d> buffer_;
  std::size_t write_index_;  // Next position to write
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_TRAJECTORY_LOG_HPP_
