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

// Non-ROS dependencies
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

// Project headers
#include "svdsim_server/utils/second_order_smoothing_filter.hpp"

namespace svdsim_server {

class SecondOrderSmoothingFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rise_time_90_ = 0.050;  // 50 ms
    dt_ = 0.001;            // 1 ms
  }

  double rise_time_90_;
  double dt_;
};

static double run_step(double rise_time_90, double dt, int n_steps) {
  SecondOrderSmoothingFilter uut(rise_time_90, dt);

  double output = 0.0;
  for (int i = 0; i < n_steps; ++i) {
    output = uut.update(1.0);
  }
  return output;
}

TEST_F(SecondOrderSmoothingFilterTest, StepResponseAtMultipleTimePoints) {
  // Test the step response at three key time points:
  // - At t = rise_time_90: should reach ~89.4% (slightly below 90% due to discretization)
  // - At t = 2*rise_time_90: numerator zero causes ~11.4% overshoot
  // - At t = 100*rise_time_90: well converged to final value
  //
  // Expected values produced by compute_filter_expected_values.py.
  const std::vector<int> n_steps = {
      static_cast<int>(1.0 * rise_time_90_ / dt_),   // 50 steps
      static_cast<int>(2.0 * rise_time_90_ / dt_),   // 100 steps
      static_cast<int>(100.0 * rise_time_90_ / dt_)  // 5000 steps
  };

  const std::vector<double> expected_outputs = {
      0.89390767,  // At rise_time_90
      1.11430768,  // At 2*rise_time_90 (overshoot peak)
      1.0          // At 100*rise_time_90 (converged)
  };

  const std::vector<double> tolerances = {1e-5, 1e-5, 1e-6};

  for (size_t i = 0; i < n_steps.size(); ++i) {
    EXPECT_NEAR(run_step(rise_time_90_, dt_, n_steps[i]), expected_outputs[i], tolerances[i])
        << "Step response mismatch at n=" << n_steps[i] << " steps";
  }
}

TEST_F(SecondOrderSmoothingFilterTest, InitialDerivativeForUnitStep) {
  // By the initial value theorem, the initial derivative of the step response
  // for a unit step should be 2*omega_n.
  // Since omega_n = 0.781616 / rise_time_90, we have:
  // initial_derivative = 2 * 0.781616 / rise_time_90 ≈ 1.563 / rise_time_90
  SecondOrderSmoothingFilter uut(rise_time_90_, dt_);

  // Apply first step (from zero state)
  const double y0 = uut.update(1.0);
  // Apply second step
  const double y1 = uut.update(1.0);

  // Approximate the initial derivative using finite difference
  const double initial_derivative_approx = (y1 - y0) / dt_;

  // Expected initial derivative: 2 * omega_n = 2 * 0.781616 / rise_time_90
  constexpr double ALPHA = 0.781616;
  const double expected_initial_derivative = 2.0 * ALPHA / rise_time_90_;

  // Allow 5% tolerance due to discretization effects
  EXPECT_NEAR(initial_derivative_approx, expected_initial_derivative, 0.05 * expected_initial_derivative)
      << "Initial derivative should be approximately 2*omega_n = " << expected_initial_derivative;
}

TEST_F(SecondOrderSmoothingFilterTest, OvershootForUnitStep) {
  // The transfer function has a numerator zero at -omega_n/2, which causes
  // approximately 13.5% overshoot in the step response.
  SecondOrderSmoothingFilter uut(rise_time_90_, dt_);

  // Run the filter for enough time to capture the overshoot peak (around 2-3 times rise_time_90)
  const int n_steps = static_cast<int>(3.0 * rise_time_90_ / dt_);
  double max_output = 0.0;

  for (int i = 0; i < n_steps; ++i) {
    const double output = uut.update(1.0);
    max_output = std::max(max_output, output);
  }

  // Overshoot percentage = (max - final_value) / final_value * 100%
  // Expected overshoot ≈ 13.5%, so max ≈ 1.135
  const double expected_max = 1.135;

  // Allow 2% absolute tolerance on the overshoot (i.e., ±0.02 on max value)
  EXPECT_NEAR(max_output, expected_max, 0.02) << "Peak overshoot should be approximately 13.5% (max ≈ 1.135)";
}

TEST_F(SecondOrderSmoothingFilterTest, RampTrackingAfterTwentyTimesRiseTime) {
  // The closed-loop transfer function has two integrators in the forward path,
  // so the steady-state error to a ramp input is zero.
  const double slope = 1.0;  // 1 unit/s
  const int n_settle = static_cast<int>(20.0 * rise_time_90_ / dt_);
  const int n_window = static_cast<int>(5.0 * rise_time_90_ / dt_);
  const int total = n_settle + n_window;

  SecondOrderSmoothingFilter uut(rise_time_90_, dt_);

  double output = 0.0;
  for (int i = 0; i < total; ++i) {
    const double reference = slope * i * dt_;
    output = uut.update(reference);

    // wait until we reach steady state
    if (i >= n_settle) {
      const double error = std::abs(output - slope * i * dt_);
      EXPECT_LT(error, 1e-6) << "Ramp tracking error exceeded threshold at step " << i;
    }
  }
}

TEST_F(SecondOrderSmoothingFilterTest, ResetClearsState) {
  // After the filter has settled at a non-zero value, reset() should restore
  // the delay-line states to zero so that the next output equals the zero-state
  // response to a fresh unit step (i.e., the very first output is close to 0).
  SecondOrderSmoothingFilter uut(rise_time_90_, dt_);

  // Let the filter settle so both w1_ and w2_ are non-zero.
  const int n_settle = static_cast<int>(5.0 * rise_time_90_ / dt_);
  for (int i = 0; i < n_settle; ++i) {
    (void)uut.update(1.0);
  }

  uut.reset();

  // First sample after reset: w1_=0, w2_=0, so output = b0*r[0] (very small), not 1.0.
  // In particular the zero-state first output  must be strictly less than the pre-reset output.
  const double output_after_reset = uut.update(1.0);
  EXPECT_LT(output_after_reset, 0.1) << "First output after reset should be close to zero, not near the settled value";

  // Verify the filter re-converges correctly from the reset state.
  const int n_reconverge = static_cast<int>(100.0 * rise_time_90_ / dt_);
  double final_output = output_after_reset;
  for (int i = 0; i < n_reconverge; ++i) {
    final_output = uut.update(1.0);
  }
  EXPECT_NEAR(final_output, 1.0, 1e-6) << "Filter should converge back to 1.0 after reset";
}

}  // namespace svdsim_server
