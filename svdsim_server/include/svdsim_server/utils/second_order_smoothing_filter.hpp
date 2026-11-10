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

#ifndef SVDSIM_SERVER_UTILS_SECOND_ORDER_SMOOTHING_FILTER_HPP_
#define SVDSIM_SERVER_UTILS_SECOND_ORDER_SMOOTHING_FILTER_HPP_

namespace svdsim_server {

/**
 * @brief Second-order reference smoothing filter
 *
 * Closed-loop transfer function:
 *   T(s) = (2*omega_n*s + omega_n^2) / (s^2 + 2*omega_n*s + omega_n^2)
 *
 * Step response reaches 90% of final value at rise_time_90, with ~13.5% overshoot.
 * Initial derivative for unit step: 2*omega_n ≈ 1.563 / rise_time_90.
 */
class SecondOrderSmoothingFilter {
 public:
  /**
   * @brief Construct a second-order smoothing filter
   *
   * @param rise_time_90 Rise time [s]: time to reach 90% of step final value
   * @param dt Sampling time [s]
   */
  SecondOrderSmoothingFilter(double rise_time_90, double dt);

  /**
   * @brief Reset the filter state to zero
   */
  void reset();

  /**
   * @brief Update the filter with a new reference sample
   *
   * @param reference Current value of the input reference
   * @return Smoothed reference value
   */
  [[nodiscard]] double update(double reference);

 private:
  // Precomputed difference-equation coefficients
  double a1_;  ///< Feedback coefficient for w[n-1]
  double a2_;  ///< Feedback coefficient for w[n-2]
  double b0_;  ///< Feedforward coefficient for w[n]
  double b1_;  ///< Feedforward coefficient for w[n-1]

  // Direct Form II delay-line states
  double w1_;  ///< w[n-1]
  double w2_;  ///< w[n-2]
};

}  // namespace svdsim_server

#endif  // SVDSIM_SERVER_UTILS_SECOND_ORDER_SMOOTHING_FILTER_HPP_
