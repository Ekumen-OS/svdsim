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

#include "svdsim_server/models/lidar_sensor_model.hpp"

#include "svdsim_server/collision/collider_manager.hpp"
#include "svdsim_server/trajectory_buffer.hpp"

namespace svdsim_server {

LidarSensorModel::LidarSensorModel(
    std::shared_ptr<ColliderManager> collider_manager,
    const Sophus::SE2d& lidar_pose,
    double min_range,
    double max_range,
    double horizontal_fov,
    int points_per_scan,
    double range_noise_stddev,
    double physics_rate,
    double scan_rate,
    bool enable_lidar_skew)
    : collider_manager_(std::move(collider_manager)),
      lidar_pose_(lidar_pose),
      min_range_(min_range),
      max_range_(max_range),
      horizontal_fov_(horizontal_fov),
      points_per_scan_(points_per_scan),
      range_noise_stddev_(range_noise_stddev),
      physics_rate_(physics_rate),
      scan_rate_(scan_rate),
      enable_lidar_skew_(enable_lidar_skew),
      random_generator_(std::random_device{}()),
      noise_distribution_(0.0, range_noise_stddev) {}

std::vector<double> LidarSensorModel::scan(const TrajectoryBuffer& trajectory_buffer) const {
  const auto timestamps_per_ray =                         //
      (physics_rate_ / scan_rate_) *                      // physics steps per full cycle
      (horizontal_fov_ / (2 * 3.14159)) *                 // fraction of the cycle spent in a scan
      (1.0 / static_cast<double>(points_per_scan_ - 1));  // fraction of the scan per ray

  std::vector<double> ranges(points_per_scan_, std::numeric_limits<double>::quiet_NaN());

  const auto raytrace_range = max_range_ + 1.0;  // extend a bit beyond max range to detect no hits and return NaN

  for (int i = 0; i < points_per_scan_; ++i) {
    // Compute angle for this scan point
    double angle = -horizontal_fov_ / 2.0 + static_cast<double>(i) * (horizontal_fov_ / (points_per_scan_ - 1));

    // compute the direction of the ray in the lidar frame
    const Sophus::SE2d ray_direction{Sophus::SO2d(angle), Eigen::Vector2d{0.0, 0.0}};

    // calculate the offset time for this ray in timestamp units
    double time_offset_in_timestamps = static_cast<double>(points_per_scan_ - 1 - i) * timestamps_per_ray;

    // determine the pose of the vehicle at the time of this ray
    auto vehicle_pose_opt = trajectory_buffer.interpolate(enable_lidar_skew_ ? time_offset_in_timestamps : 0.0);

    // calculate the pose of the ray direction in the world frame
    if (vehicle_pose_opt.has_value()) {
      const auto& vehicle_pose = vehicle_pose_opt.value();
      const Sophus::SE2d ray_pose_world = vehicle_pose * lidar_pose_ * ray_direction;

      auto range = collider_manager_->raycast(ray_pose_world, static_cast<float>(raytrace_range));

      // Apply Gaussian noise
      range += noise_distribution_(random_generator_);

      // if not in range, set to NaN
      if ((range < min_range_) || (range > max_range_)) {
        range = std::numeric_limits<double>::quiet_NaN();
      }

      ranges[i] = static_cast<double>(range);
    }
  }
  return ranges;
}

}  // namespace svdsim_server
