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

#include "svdsim_server/utils/pid_controller.hpp"

#include <cmath>

namespace svdsim_server {

PIDController::PIDController(const PIDControllerParams& params, double dt)
    : params_(params), dt_(dt), error_integral_(0.0), previous_error_(0.0), filtered_derivative_(0.0) {
  // Calculate derivative filter coefficient (low-pass filter)
  // alpha = dt / (tau + dt)
  alpha_ = dt_ / (params_.derivative_time_constant + dt_);
}

void PIDController::reset() {
  error_integral_ = 0.0;
  previous_error_ = 0.0;
  filtered_derivative_ = 0.0;
}

double PIDController::calculate(double reference, double feedback) {
  // Calculate current error
  const double error = reference - feedback;

  // Proportional term
  const double p_term = params_.kp * error;

  // Integral term with anti-windup
  error_integral_ += error * dt_;
  const double i_term = params_.ki * error_integral_;

  // Derivative term with filtering
  const double error_derivative = (error - previous_error_) / dt_;
  filtered_derivative_ = alpha_ * error_derivative + (1.0 - alpha_) * filtered_derivative_;
  const double d_term = params_.kd * filtered_derivative_;

  // Calculate control action
  const double control_action = p_term + i_term + d_term;

  // Anti-windup: prevent integral wind-up when output saturates
  // This simple implementation prevents further integration when the
  // control action magnitude is large and the error has the same sign
  if (params_.enable_anti_windup) {
    if (control_action > params_.output_max) {
      if (error > 0.0) {
        // Don't integrate further when saturating at max
        error_integral_ -= error * dt_;
      }
    } else if (control_action < params_.output_min) {
      if (error < 0.0) {
        // Don't integrate further when saturating at min
        error_integral_ -= error * dt_;
      }
    }
  }

  // Update state for next iteration
  previous_error_ = error;

  // Clamp output to limits
  if (control_action > params_.output_max) {
    return params_.output_max;
  } else if (control_action < params_.output_min) {
    return params_.output_min;
  }

  return control_action;
}

}  // namespace svdsim_server
