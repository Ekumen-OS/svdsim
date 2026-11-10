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

#ifndef SVDSIM_SERVER_UTILS_PID_CONTROLLER_HPP_
#define SVDSIM_SERVER_UTILS_PID_CONTROLLER_HPP_

#include <limits>

namespace svdsim_server {

/**
 * @brief Parameters for PID controller
 */
struct PIDControllerParams {
  /// Proportional gain
  double kp{1.0};
  /// Integral gain
  double ki{0.0};
  /// Derivative gain
  double kd{0.0};
  /// Time constant for derivative term filtering [s]
  double derivative_time_constant{0.0};
  /// Enable anti-windup for integral term
  bool enable_anti_windup{true};
  /// Minimum output value
  double output_min{std::numeric_limits<double>::lowest()};
  /// Maximum output value
  double output_max{std::numeric_limits<double>::max()};
};

/**
 * @brief PID (Proportional-Integral-Derivative) controller
 *
 * Implements a discrete-time PID controller with configurable gains,
 * derivative filtering, and optional integral anti-windup.
 */
class PIDController {
 public:
  /**
   * @brief Construct a new PID Controller
   *
   * @param params PID controller parameters
   * @param dt Sampling time [s]
   */
  PIDController(const PIDControllerParams& params, double dt);

  /**
   * @brief Reset the controller state
   *
   * Clears the error integral, previous error, and filtered derivative.
   */
  void reset();

  /**
   * @brief Calculate control action
   *
   * @param reference Reference (setpoint) value
   * @param feedback Feedback (measured) value
   * @return Control action
   */
  double calculate(double reference, double feedback);

 private:
  // Parameters
  PIDControllerParams params_;  ///< PID controller parameters

  // Derivative filter coefficient
  double alpha_;  ///< Derivative filter coefficient

  // Configuration
  double dt_;  ///< Sampling time [s]

  // State
  double error_integral_;       ///< Accumulated error integral
  double previous_error_;       ///< Previous error value
  double filtered_derivative_;  ///< Filtered derivative term
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_UTILS_PID_CONTROLLER_HPP_
