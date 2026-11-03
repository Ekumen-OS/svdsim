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

#include "svdsim_server/utils/second_order_smoothing_filter.hpp"

namespace svdsim_server {

SecondOrderSmoothingFilter::SecondOrderSmoothingFilter(double rise_time_90, double dt) {
  // The denominator of T(s) matches s^2 + 2*zeta*omega_n*s + omega_n^2, giving:
  //   kp = 2 * zeta * omega_n
  //   ki = omega_n^2
  //
  // zeta = 1 (critical damping) is fixed.
  //
  // omega_n is derived from the rise time (time to 90% of step final value) via the
  // empirically determined relationship: omega_n = 0.781616 / rise_time_90.
  // This ensures that the closed-loop step response reaches y=0.9 at t=rise_time_90.
  constexpr double ALPHA = 0.781616;  // rise_time_90 * omega_n for zeta=1.0
  const double omega_n = ALPHA / rise_time_90;
  const double kp = 2.0 * omega_n;
  const double ki = omega_n * omega_n;
  const double den = 1.0 + kp * dt + ki * dt * dt;

  a1_ = (2.0 + kp * dt) / den;
  a2_ = -1.0 / den;
  b0_ = (kp * dt + ki * dt * dt) / den;
  b1_ = -(kp * dt) / den;

  w1_ = 0.0;
  w2_ = 0.0;
}

void SecondOrderSmoothingFilter::reset() {
  w1_ = 0.0;
  w2_ = 0.0;
}

double SecondOrderSmoothingFilter::update(double reference) {
  // Direct Form II biquad derived from T(s) = (kp*s + ki) / (s^2 + kp*s + ki)
  // via backward Euler s = (1 - z^{-1}) / dt:
  //
  //   w[n] = r[n]  + a1*w[n-1] + a2*w[n-2]   (recursive / feedback section)
  //   y[n] = b0*w[n] + b1*w[n-1]              (FIR / feedforward section)
  const double w = reference + a1_ * w1_ + a2_ * w2_;
  const double output = b0_ * w + b1_ * w1_;

  w2_ = w1_;
  w1_ = w;

  return output;
}

}  // namespace svdsim_server
