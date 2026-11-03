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

#include "svdsim_server/utils/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace svdsim_server {

PurePursuitController::PurePursuitController(
    const std::vector<TrajectoryPoint>& trajectory,
    const PurePursuitParams& params)
    : trajectory_(trajectory), params_(params), last_target_point_(0.0, 0.0) {
  if (trajectory_.empty()) {
    throw std::invalid_argument("Trajectory cannot be empty");
  }

  if (params_.wheelbase <= 0.0) {
    throw std::invalid_argument("Wheelbase must be positive");
  }

  if (params_.lookahead_distance <= 0.0) {
    throw std::invalid_argument("Lookahead distance must be positive");
  }

  if (params_.max_steering <= 0.0) {
    throw std::invalid_argument("Maximum steering must be positive");
  }
}

double PurePursuitController::compute_steering(const Sophus::SE2d& current_pose) {
  // Find the closest point on the trajectory (interpolated)
  auto closest_result = find_closest_point(current_pose);

  // Find the lookahead target point (interpolated)
  Eigen::Vector2d target = find_lookahead_point(closest_result.point, closest_result.segment_end_index);

  // Transform target point into vehicle frame
  const Eigen::Vector2d target_in_vehicle_frame = current_pose.inverse() * target;

  double distance_squared = target_in_vehicle_frame.squaredNorm();

  // Handle edge case: vehicle is very close to target
  // In this case, continue straight ahead
  if (distance_squared < 1e-6) {
    last_target_point_ = target;
    return 0.0;
  }

  // Pure pursuit curvature formula: kappa = 2 * y / L^2
  // where y is the lateral offset in vehicle frame and L is the distance
  double curvature = 2.0 * target_in_vehicle_frame.y() / distance_squared;

  // Convert curvature to steering angle: delta = atan(wheelbase * kappa)
  double steering = std::atan(params_.wheelbase * curvature);

  // Clamp to maximum steering angle
  steering = clamp(steering, -params_.max_steering, params_.max_steering);

  // Store last target point for visualization
  last_target_point_ = target;

  return steering;
}

PurePursuitController::ClosestPointResult PurePursuitController::find_closest_point(const Sophus::SE2d& pose) const {
  const Eigen::Vector2d vehicle_pos = pose.translation();

  // Find the closest trajectory point using std::min_element
  auto closest_iter = std::min_element(
      trajectory_.begin(), trajectory_.end(), [&vehicle_pos](const TrajectoryPoint& a, const TrajectoryPoint& b) {
        const Eigen::Vector2d pa(a.x, a.y);
        const Eigen::Vector2d pb(b.x, b.y);
        return (pa - vehicle_pos).squaredNorm() < (pb - vehicle_pos).squaredNorm();
      });
  const size_t closest_index = std::distance(trajectory_.begin(), closest_iter);

  // Now check the two segments adjacent to this point
  const size_t point_count = trajectory_.size();
  const size_t prev_index = (closest_index == 0) ? point_count - 1 : closest_index - 1;
  const size_t next_index = (closest_index + 1) % point_count;

  // Segment 1: prev_index -> closest_index
  const Eigen::Vector2d seg1_start(trajectory_[prev_index].x, trajectory_[prev_index].y);
  const Eigen::Vector2d seg1_end(trajectory_[closest_index].x, trajectory_[closest_index].y);
  Eigen::Vector2d proj1 = project_point_onto_segment(vehicle_pos, seg1_start, seg1_end);
  double dist1_sq = (proj1 - vehicle_pos).squaredNorm();

  // Segment 2: closest_index -> next_index
  const Eigen::Vector2d seg2_start(trajectory_[closest_index].x, trajectory_[closest_index].y);
  const Eigen::Vector2d seg2_end(trajectory_[next_index].x, trajectory_[next_index].y);
  Eigen::Vector2d proj2 = project_point_onto_segment(vehicle_pos, seg2_start, seg2_end);
  double dist2_sq = (proj2 - vehicle_pos).squaredNorm();

  // Return the closer projection
  if (dist1_sq < dist2_sq) {
    return {proj1, closest_index};
  } else {
    return {proj2, next_index};
  }
}

Eigen::Vector2d PurePursuitController::find_lookahead_point(
    const Eigen::Vector2d& closest_point,
    size_t segment_end_index) const {
  const size_t point_count = trajectory_.size();

  // Walk along the trajectory from the closest point
  // Start from the segment end index (the trajectory point after the closest point)
  double accumulated_distance = 0.0;
  size_t index = segment_end_index;

  // Start from the closest point, not from the trajectory point
  Eigen::Vector2d current_pos = closest_point;

  for (size_t i = 0; i < point_count; ++i) {
    const Eigen::Vector2d next_point(trajectory_[index].x, trajectory_[index].y);

    Eigen::Vector2d segment = next_point - current_pos;
    double segment_length = segment.norm();

    if (accumulated_distance + segment_length >= params_.lookahead_distance) {
      // Interpolate within this segment
      double remaining = params_.lookahead_distance - accumulated_distance;
      double t = remaining / segment_length;

      return current_pos + t * segment;
    }

    accumulated_distance += segment_length;
    current_pos = next_point;
    index = (index + 1) % point_count;
  }

  // If we traversed the entire trajectory, return the furthest point
  return current_pos;
}

Eigen::Vector2d PurePursuitController::project_point_onto_segment(
    const Eigen::Vector2d& point,
    const Eigen::Vector2d& seg_start,
    const Eigen::Vector2d& seg_end) {
  const Eigen::Vector2d segment = seg_end - seg_start;
  const double segment_length_squared = segment.squaredNorm();

  // Degenerate segment (start == end)
  if (segment_length_squared < 1e-10) {
    return seg_start;
  }

  // Compute projection parameter t
  const Eigen::Vector2d point_to_start = point - seg_start;
  const double t = point_to_start.dot(segment) / segment_length_squared;

  // Clamp t to [0, 1] to stay within segment bounds
  const double t_clamped = clamp(t, 0.0, 1.0);

  // Return interpolated point
  return seg_start + t_clamped * segment;
}

}  // namespace svdsim_server
