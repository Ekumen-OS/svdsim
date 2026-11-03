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

#ifndef SVDSIM_SERVER_UTILS_PURE_PURSUIT_CONTROLLER_HPP_
#define SVDSIM_SERVER_UTILS_PURE_PURSUIT_CONTROLLER_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include <sophus/se2.hpp>

#include "svdsim_server/utils/trajectory_reader.hpp"

namespace svdsim_server {

/**
 * @brief Configuration parameters for pure pursuit controller
 */
struct PurePursuitParams {
  double wheelbase;           ///< Vehicle wheelbase [m]
  double lookahead_distance;  ///< Lookahead distance for target point selection [m]
  double max_steering;        ///< Maximum absolute steering angle [rad]

  PurePursuitParams() : wheelbase(0.324), lookahead_distance(0.8), max_steering(0.6) {}
};

/**
 * @brief Pure pursuit path following controller
 *
 * Implements the pure pursuit algorithm for path following. The controller
 * computes steering commands to follow a reference trajectory by pursuing
 * a lookahead point on the path.
 *
 * The algorithm:
 * 1. Finds the closest point on the trajectory to the current vehicle position
 * 2. Selects a lookahead point along the trajectory at the specified lookahead distance
 * 3. Computes the curvature needed to reach the lookahead point
 * 4. Converts curvature to steering angle using the vehicle wheelbase
 *
 * Example usage:
 * @code
 * PurePursuitParams params;
 * params.wheelbase = 0.324;
 * params.lookahead_distance = 0.8;
 *
 * PurePursuitController controller(trajectory, params);
 *
 * Sophus::SE2d current_pose(Sophus::SO2d(yaw), Eigen::Vector2d(x, y));
 * double steering = controller.compute_steering(current_pose);
 * // Apply steering command
 * @endcode
 */
class PurePursuitController {
 public:
  /**
   * @brief Construct a new Pure Pursuit Controller
   *
   * @param trajectory Reference trajectory to follow
   * @param params Controller parameters
   * @throws std::invalid_argument if trajectory is empty or parameters are invalid
   */
  PurePursuitController(const std::vector<TrajectoryPoint>& trajectory, const PurePursuitParams& params);

  /**
   * @brief Compute steering command for current pose
   *
   * @param current_pose Current vehicle pose (Sophus SE2)
   * @return Computed steering angle [rad]
   */
  [[nodiscard]] double compute_steering(const Sophus::SE2d& current_pose);

  /**
   * @brief Get the reference trajectory
   */
  [[nodiscard]] const std::vector<TrajectoryPoint>& get_trajectory() const { return trajectory_; }

  /**
   * @brief Get the controller parameters
   */
  [[nodiscard]] const PurePursuitParams& get_params() const { return params_; }

  /**
   * @brief Get the last computed target point (for visualization)
   */
  [[nodiscard]] const Eigen::Vector2d& get_last_target_point() const { return last_target_point_; }

 private:
  /**
   * @brief Result of closest point search
   */
  struct ClosestPointResult {
    Eigen::Vector2d point;     ///< Interpolated closest point on trajectory
    size_t segment_end_index;  ///< Index of the trajectory point at the end of the segment
  };

  /**
   * @brief Find the closest point on the trajectory (interpolated)
   *
   * Finds the closest trajectory point, checks the two adjacent segments,
   * and returns the interpolated closest point on the closer segment.
   *
   * @param pose Current vehicle pose (Sophus SE2)
   * @return Closest point result with interpolated point and segment end index
   */
  [[nodiscard]] ClosestPointResult find_closest_point(const Sophus::SE2d& pose) const;

  /**
   * @brief Find the lookahead target point (interpolated)
   *
   * Walks along the trajectory from the closest point, accumulating distance
   * until reaching the lookahead distance, then interpolates to get the exact point.
   *
   * @param closest_point The closest point on the trajectory
   * @param segment_end_index Index of the trajectory point at the end of the segment containing closest_point
   * @return Lookahead target point (interpolated)
   */
  [[nodiscard]] Eigen::Vector2d find_lookahead_point(const Eigen::Vector2d& closest_point, size_t segment_end_index)
      const;

  /**
   * @brief Project a point onto a line segment
   *
   * @param point Point to project
   * @param seg_start Start of line segment
   * @param seg_end End of line segment
   * @return Projected point on the segment (clamped to segment bounds)
   */
  [[nodiscard]] static Eigen::Vector2d project_point_onto_segment(
      const Eigen::Vector2d& point,
      const Eigen::Vector2d& seg_start,
      const Eigen::Vector2d& seg_end);

  /**
   * @brief Clamp a value to a range
   */
  template <typename T>
  [[nodiscard]] static T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : ((value > max_val) ? max_val : value);
  }

  std::vector<TrajectoryPoint> trajectory_;
  PurePursuitParams params_;

  // Last computed target point (for visualization)
  Eigen::Vector2d last_target_point_;
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_UTILS_PURE_PURSUIT_CONTROLLER_HPP_
