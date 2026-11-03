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

#ifndef MONACOF1_SIM_SERVER_MODELS_LIDAR_SENSOR_MODEL_HPP_
#define MONACOF1_SIM_SERVER_MODELS_LIDAR_SENSOR_MODEL_HPP_

#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <sophus/se2.hpp>

#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/trajectory_buffer.hpp"

namespace svdsim_server {

/**
 * @brief Model for lidar sensor simulation using collision detection
 *
 * Simulates a 2D lidar sensor by performing raycasts through the collision manager.
 */
class LidarSensorModel {
 public:
  /**
   * @brief Construct a new Lidar Sensor Model
   *
   * @param collider_manager Shared pointer to the collision manager for raycasting
   * @param lidar_pose Pose of the lidar sensor in the vehicle reference frame
   * @param min_range Minimum detection range [m]
   * @param max_range Maximum detection range [m]
   * @param horizontal_fov Horizontal field of view [rad]
   * @param points_per_scan Number of points per scan
   * @param range_noise_stddev Standard deviation of range measurement noise [m]
   * @param physics_rate Physics simulation update rate [Hz]
   * @param scan_rate Lidar scan rate [Hz]
   * @param enable_lidar_skew Enable motion compensation for lidar skew
   */
  LidarSensorModel(
      std::shared_ptr<ColliderManager> collider_manager,
      const Sophus::SE2d& lidar_pose,
      double min_range,
      double max_range,
      double horizontal_fov,
      int points_per_scan,
      double range_noise_stddev,
      double physics_rate,
      double scan_rate,
      bool enable_lidar_skew);

  /**
   * @brief Perform a lidar scan using the trajectory buffer for motion compensation
   *
   * @param trajectory_buffer Trajectory buffer containing vehicle pose history
   * @return Vector of range measurements [m], one per scan point
   */
  [[nodiscard]] std::vector<double> scan(const TrajectoryBuffer& trajectory_buffer) const;

  /**
   * @brief Get the lidar pose in vehicle frame
   * @return Lidar pose SE2d transformation
   */
  [[nodiscard]] const Sophus::SE2d& lidar_pose() const { return lidar_pose_; }

  /**
   * @brief Get the minimum detection range
   * @return Minimum range [m]
   */
  [[nodiscard]] double min_range() const { return min_range_; }

  /**
   * @brief Get the maximum detection range
   * @return Maximum range [m]
   */
  [[nodiscard]] double max_range() const { return max_range_; }

  /**
   * @brief Get the horizontal field of view
   * @return Horizontal FOV [rad]
   */
  [[nodiscard]] double horizontal_fov() const { return horizontal_fov_; }

  /**
   * @brief Get the number of points per scan
   * @return Points per scan
   */
  [[nodiscard]] int points_per_scan() const { return points_per_scan_; }

  /**
   * @brief Get the range noise standard deviation
   * @return Range noise stddev [m]
   */
  [[nodiscard]] double range_noise_stddev() const { return range_noise_stddev_; }

  /**
   * @brief Get the physics simulation rate
   * @return Physics rate [Hz]
   */
  [[nodiscard]] double physics_rate() const { return physics_rate_; }

  /**
   * @brief Get the lidar scan rate
   * @return Scan rate [Hz]
   */
  [[nodiscard]] double scan_rate() const { return scan_rate_; }

  /**
   * @brief Get whether lidar skew compensation is enabled
   * @return True if skew compensation is enabled
   */
  [[nodiscard]] bool enable_lidar_skew() const { return enable_lidar_skew_; }

 private:
  std::shared_ptr<ColliderManager> collider_manager_;
  Sophus::SE2d lidar_pose_;
  double min_range_;
  double max_range_;
  double horizontal_fov_;
  int points_per_scan_;
  double range_noise_stddev_;
  double physics_rate_;
  double scan_rate_;
  bool enable_lidar_skew_;
  mutable std::mt19937 random_generator_;
  mutable std::normal_distribution<double> noise_distribution_;
};

}  // namespace svdsim_server

#endif  // MONACOF1_SIM_SERVER_MODELS_LIDAR_SENSOR_MODEL_HPP_
