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

#include "svdsim_server/models/simple_drive_model.hpp"

#include <algorithm>
#include <cmath>

namespace svdsim_server {

SimpleDriveModel::SimpleDriveModel(const SimpleDriveModelParameters& params)
    : params_(params), velocity_ref_filter_(params.motor_drive_rise_time, params.dt) {}

void SimpleDriveModel::reset() {
  velocity_ref_filter_.reset();
}

double SimpleDriveModel::update_steering_velocity(double steering_ref, double steering_feedback) const {
  // Clamp the reference to the valid steering angle range
  const auto constrained_steering_ref =
      std::clamp(steering_ref, -params_.steering_angle_max, params_.steering_angle_max);

  // Make the effective reference lag the command by half the hysteresis gap.
  // Half the gap is applied on each side, so the parameter represents the full hysteresis loop
  // width and the vehicle settles at half the gap behind the reference in each direction.
  const auto steering_error_to_ref = constrained_steering_ref - steering_feedback;
  const auto hysteresis_gap_half = params_.steering_hysteresis_gap / 2.0;
  const auto hysteresis_adjusted_steering_ref =
      (steering_error_to_ref > 0.0) ? std::max(constrained_steering_ref - hysteresis_gap_half, steering_feedback)
                                    : std::min(constrained_steering_ref + hysteresis_gap_half, steering_feedback);

  // Calculate steering error and velocity needed to reach target in one time step
  const auto steering_error = hysteresis_adjusted_steering_ref - steering_feedback;
  const auto steering_velocity_to_ref = steering_error / params_.dt;

  // Clamp to maximum steering velocity
  auto steering_velocity =
      std::clamp(steering_velocity_to_ref, -params_.steering_velocity_max, params_.steering_velocity_max);

  // Calculate maximum steering velocities that won't violate angle limits in one time step
  const auto steering_velocity_to_max = (params_.steering_angle_max - steering_feedback) / params_.dt;
  const auto steering_velocity_to_min = (-params_.steering_angle_max - steering_feedback) / params_.dt;

  // Further clamp to avoid exceeding steering angle limits
  steering_velocity = std::clamp(steering_velocity, steering_velocity_to_min, steering_velocity_to_max);

  return steering_velocity;
}

double SimpleDriveModel::update_acceleration(double velocity_ref, double velocity_feedback) {
  // Apply second-order smoothing filter to reduce jitter in the velocity reference
  const auto smoothed_velocity_ref = velocity_ref_filter_.update(velocity_ref);

  // Clamp the smoothed reference to the valid velocity range to prevent filter overshoot
  // from pushing the reference out of bounds
  const auto constrained_velocity_ref = std::clamp(smoothed_velocity_ref, params_.velocity_min, params_.velocity_max);

  // Apply the velocity dead band: references with magnitude below the threshold are
  // treated as zero so the vehicle comes to (and stays at) rest.
  const auto effective_velocity_ref =
      std::abs(constrained_velocity_ref) < params_.velocity_deadband ? 0.0 : constrained_velocity_ref;

  const auto velocity_error = effective_velocity_ref - velocity_feedback;

  const auto accel_to_ref = std::abs(velocity_error) / params_.dt;

  double acceleration = 0.0;

  // The asymmetry handles forward vs reverse motion correctly
  if (velocity_feedback >= 0.0) {
    // Forward motion (or stopped moving forward)
    if (velocity_error > 0.0) {
      // Need to speed up: apply motor acceleration
      acceleration = std::min(params_.drive_motor_acceleration, accel_to_ref);
    } else {
      // Need to slow down: apply braking deceleration (negative acceleration)
      acceleration = -std::min(params_.drive_braking_deceleration, accel_to_ref);
    }
  } else {
    // Reverse motion
    if (velocity_error > 0.0) {
      // Need to speed up (toward zero or forward): apply braking deceleration
      acceleration = std::min(params_.drive_braking_deceleration, accel_to_ref);
    } else {
      // Need to speed up backward: apply motor acceleration (negative)
      acceleration = -std::min(params_.drive_motor_acceleration, accel_to_ref);
    }
  }

  return acceleration;
}

}  // namespace svdsim_server
